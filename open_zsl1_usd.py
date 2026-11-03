from isaacsim import SimulationApp
app = SimulationApp({'headless': False, 'width': 1440, 'height': 900, 'renderer': 'RaytracedLighting'})
import omni.usd
path = '/mnt/llm/g1_sim_ws/l1_env/assets/zsl1_hifid.usd'
print('OPENING_USD', path, flush=True)
context = omni.usd.get_context()
context.open_stage(path)
for _ in range(600):
    app.update()
print('STAGE_OPENED', context.get_stage() is not None, flush=True)
while app.is_running():
    app.update()
app.close()
