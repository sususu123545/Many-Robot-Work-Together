// Offline regression checks: subscription-only access and honest data freshness.
'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const html=fs.readFileSync('web_console/sensor_console.html','utf8');
const source=[...html.matchAll(/<script\b[^>]*>([\s\S]*?)<\/script>/gi)].map(m=>m[1]).join('\n');
let now=1000;
const elements=new Map(),messages=[],timeouts=new Map();
const draw=new Proxy({}, {get:(_,name)=>()=>{},set:()=>true});
function element(id){
  if(!elements.has(id))elements.set(id,{
    id,textContent:'',dataset:{},hidden:false,width:400,height:400,
    child:{textContent:''},listeners:{},classList:{toggle(){},add(){},remove(){}},
    querySelector(){return this.child;},addEventListener(event,fn){this.listeners[event]=fn;},
    removeAttribute(){},setAttribute(){},getBoundingClientRect(){return {width:480,height:480};},getContext(){return draw;}
  });
  return elements.get(id);
}
class Socket{
  static OPEN=1;
  constructor(){this.readyState=0;Socket.last=this;}
  send(message){messages.push(JSON.parse(message));}
  close(){this.readyState=3;if(this.onclose)this.onclose();}
  open(){this.readyState=1;this.onopen();}
}
const context=vm.createContext({
  document:{getElementById:element,querySelectorAll:()=>[],addEventListener(){},hidden:false},
  window:{devicePixelRatio:2,addEventListener(){}},WebSocket:Socket,performance:{now:()=>now},
  fetch:()=>new Promise(()=>{}),AbortSignal:{timeout(){}},AbortController,
  ResizeObserver:class{observe(){}},setInterval:()=>1,
  setTimeout:(fn)=>{const id=timeouts.size+1;timeouts.set(id,fn);return id;},clearTimeout:id=>timeouts.delete(id),
  URL,URLSearchParams,Image:class{},Date,console
});
vm.runInContext(source,context);
Socket.last.open();
assert.deepEqual(messages,[{op:'subscribe',id:'sensor-view-scan',topic:'/scan',type:'sensor_msgs/msg/LaserScan',throttle_rate:100,queue_length:1}]);
const scan={ranges:[null,0,-1,0.1,0.5,2,4,10],range_min:0.2,range_max:8,angle_min:-Math.PI,angle_increment:0.1};
function receive(frame){Socket.last.onmessage({data:JSON.stringify(frame)});}
receive({op:'publish',topic:'/scan',msg:scan});
assert.equal(element('pointCount').textContent,'3');
assert.equal(element('nearest').textContent,'0.50');
assert.equal(element('lidarOverlay').hidden,true);
now=1100;receive({op:'publish',topic:'/scan',msg:scan});
assert.equal(element('scanRate').textContent,'10.0');
receive({op:'publish',topic:'/scan',msg:{ranges:'bad'}});
assert.equal(element('pointCount').textContent,'3','Invalid frames must not replace good data');
now=4200;vm.runInContext('updateStatus()',context);
assert.equal(element('lidarOverlay').hidden,false,'Stale scans must be marked offline');
assert.equal(element('scanRate').textContent,'—');
assert.equal(element('nearest').textContent,'—');
now=4300;receive({op:'publish',topic:'/scan',msg:{...scan,ranges:[null,0,-1,20]}});
assert.equal(element('pointCount').textContent,'0');
assert.equal(element('nearest').textContent,'—','An empty scan must not invent a distance');
Socket.last.close();
assert.equal(element('lidarOverlay').hidden,false);
assert.equal(vm.runInContext('state.scan',context),null,'Disconnect must clear old points');
assert(timeouts.size>0,'Disconnect should schedule a bounded retry');
element('reconnect').listeners.click();Socket.last.open();
assert(messages.every(message=>message.op==='subscribe'),'Reconnection must never send movement, authorization, or service requests');
assert(!html.includes('data-velocity')&&!html.includes('set_parameters_atomically'));
vm.runInContext(`
  const jpeg=new Uint8Array([45,45,13,10,255,216,1,2,255,217,13,10]);
  const parsed=extractJpeg(jpeg);
  if(!parsed||parsed.frame.length!==6||parsed.rest.length!==2)throw new Error('MJPEG frame boundaries');
  if(extractJpeg(jpeg.slice(0,9))!==null)throw new Error('Incomplete JPEG must await next network chunk');
  const joined=new Uint8Array([...jpeg,...jpeg]);
  const first=extractJpeg(joined),second=extractJpeg(first.rest);
  if(!second||second.frame.length!==6)throw new Error('Multiple frames in one network chunk');
`,context);
console.log('Sensor console checks passed: read-only connection, range filtering, freshness, reconnection, chunked MJPEG frames.');

async function checkVision(){
  const requests=[];
  context.Blob=Blob;
  context.btoa=value=>Buffer.from(value,'binary').toString('base64');
  context.fetch=async(url,options)=>{
    requests.push({url,options});
    if(url==='/api/vision/analyze')return {ok:true,json:async()=>({job_id:'test-job'})};
    return {ok:true,json:async()=>({status:'succeeded',result:{text:'红球在左下方。',elapsed_seconds:2.1,truncated:false}})};
  };
  element('visionInstruction').value='寻找红球';now=5000;
  vm.runInContext('state.cameraAt=4990;state.modelAvailable=true;state.latestFrame=new Uint8Array([255,216,1,2,255,217]);state.frameCapturedAt=Date.now()/1000',context);
  const first=vm.runInContext('analyzeFrame({preventDefault(){}})',context);
  await vm.runInContext('analyzeFrame({preventDefault(){}})',context);
  await first;
  assert.equal(requests.filter(r=>r.url==='/api/vision/analyze').length,1,'Double-clicks must not submit duplicate model jobs');
  const body=JSON.parse(requests[0].options.body);
  assert.equal(body.instruction,'寻找红球');
  assert.equal(body.image_base64,Buffer.from([255,216,1,2,255,217]).toString('base64'));
  assert.equal(element('visionResult').textContent,'红球在左下方。');
  assert(element('visionMeta').textContent.includes('2.1'));
  assert.equal(vm.runInContext('state.analysisBusy',context),false);
  const before=requests.length;now=10000;
  await vm.runInContext('analyzeFrame({preventDefault(){}})',context);
  assert.equal(requests.length,before,'Old camera frames must not be submitted');
  assert(messages.every(message=>message.op==='subscribe'),'Model responses must never publish ROS commands');
  console.log('Vision UI checks passed: exact captured image, one job at a time, result rendering, stale-frame rejection, no ROS commands.');
}
checkVision().catch(error=>{console.error(error);process.exitCode=1;});
