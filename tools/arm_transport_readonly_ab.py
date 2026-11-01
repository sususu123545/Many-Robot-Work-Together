"""ON-ROBOT diagnostic only: enforce read-only UART packets, compare pacing.

Run in an isolated container only AFTER the normal driver releases /dev/rrc.
No ROS node, no motion, no stop/torque/limit/offset writes, no automatic reset.
"""
import json
import os
from pathlib import Path
import time

from servo_protocol import ServoBoard, crc8


def check_read_packet(data):
    if (len(data)!=7 or data[:4]!=b'\xaa\x55\x05\x02'
            or data[4] not in (5,7,9) or not 1<=data[5]<=6
            or crc8(data[2:-1])!=data[-1]):
        raise ValueError('Diagnostic refused a non-read UART packet')


class ReadOnlyPort:
    def __init__(self,port):self.port=port
    @property
    def in_waiting(self):return self.port.in_waiting
    def read(self,count):return self.port.read(count)
    def write(self,data):
        check_read_packet(data)
        return self.port.write(data)
    def close(self):self.port.close()


def interleaved_temperature(board,ident):
    """Diagnostic hypothesis only; same three temperature attempts, real replies.

    Between missing temperature replies query the same servo's position once.
    A failed position read is NOT ignored. No cached/synthetic temperature.
    """
    for attempt in range(3):
        try:return board.read(ident,9,'<BBbB',attempts=1)[0]
        except TimeoutError:
            if attempt==2:raise
            board._record('temperature_interleaved_retry',servo_id=ident,attempt=attempt+1)
            board.read(ident,5,'<BBbh',attempts=1)


def main():
    import serial
    cycles=int(os.environ.get('AB_CYCLES','180'))
    if not 20<=cycles<=240:raise ValueError('Bounded 20..240 cycles required')
    port=serial.Serial(None,baudrate=1000000,timeout=.01,write_timeout=.1,exclusive=True)
    port.rts=False;port.dtr=False;port.port='/dev/rrc';port.open()
    board=ServoBoard(port=ReadOnlyPort(port))
    results=[]
    first=None
    try:
        supply={}
        for ident in range(1,7):
            try:supply[ident]={'millivolts':board.voltage_mv(ident)}
            except (OSError,TimeoutError) as error:supply[ident]={'error':str(error)}
        print('VOLTAGE_READ_ONLY='+json.dumps(supply),flush=True)
        mode=os.environ.get('AB_PROFILES','timing')
        if mode not in ('timing','interleaved'):raise ValueError('Unknown diagnostic profile')
        profiles=([('paired_20ms',.02,.5),('paired_50ms',.05,.8)] if mode=='timing'
                  else [('interleaved_temperature_20ms',.02,.5)])
        for profile,gap,period in profiles:
            before=board.diagnostics()['counters'];started=time.monotonic();errors=[];reads=0
            latest={};max_drift={};temps=[]
            for cycle in range(cycles):
                cycle_start=time.monotonic();values={}
                for ident in range(1,7):
                    values[ident]={}
                    temp=(lambda ident:interleaved_temperature(board,ident)) if mode=='interleaved' else board.temperature
                    for label,command in [('position',board.position),('temperature',temp)]:
                        wait=gap-(time.monotonic()-board.last_read_tx)
                        if wait>0:time.sleep(wait)
                        try:
                            value=command(ident);values[ident][label]=value
                            if label=='temperature':
                                temps.append(value)
                                if value>=55:raise RuntimeError('Diagnostic temperature limit reached')
                        except (OSError,TimeoutError) as error:
                            errors.append({'cycle':cycle,'servo':ident,'query':label,'error':str(error),
                                           'diagnostics':board.diagnostics()})
                        reads+=1
                if first is None:first={i:v['position'] for i,v in values.items() if 'position' in v}
                for ident,v in values.items():
                    if 'position' in v and ident in first:
                        drift=abs(v['position']-first[ident]);max_drift[ident]=max(max_drift.get(ident,0),drift)
                        if drift>20:raise RuntimeError('Physical position changed >20 ticks; stop diagnosis')
                latest=values
                wait=period-(time.monotonic()-cycle_start)
                if wait>0:time.sleep(wait)
                if (cycle+1)%30==0:
                    print('AB_PROGRESS='+json.dumps({'profile':profile,'cycles':cycle+1,'errors':len(errors),
                                                    'seconds':round(time.monotonic()-started,1)}),flush=True)
            after=board.diagnostics()['counters']
            row={'profile':profile,'servo_order':[1,2,3,4,5,6],'cycles':cycles,'logical_reads':reads,'errors':errors,
                 'seconds':round(time.monotonic()-started,2),'latest':latest,'max_raw_drift':max_drift,
                 'temperature_range':[min(temps),max(temps)] if temps else None,
                 'counter_delta':{k:v-before.get(k,0) for k,v in after.items() if v!=before.get(k,0)}}
            results.append(row)
            summary={k:v for k,v in row.items() if k!='errors'}
            summary['error_count']=len(errors)
            summary['failed_queries']=[{k:v for k,v in error.items() if k!='diagnostics'} for error in errors[:6]]
            print('AB_PROFILE_RESULT='+json.dumps(summary),flush=True)
        report={'results':results,'voltage':supply,'read_only_uart_enforced':True,
                'note':'A no-error window alone is not proof of a fix or hardware health'}
        Path('/results/report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print('AB_REPORT_COMPLETE',flush=True)
    finally:board.close()


if __name__=='__main__':main()
