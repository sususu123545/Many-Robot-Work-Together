from isaacsim import SimulationApp
app = SimulationApp({'headless': False, 'width': 1280, 'height': 720, 'renderer': 'RaytracedLighting'})
import omni.usd
omni.usd.get_context().open_stage('/home/sususu/competition_sim/assets/g1_no_hand/g1_scene_asset.usd')
for _ in range(300):
    app.update()
print('LOCAL_STAGE_OPENED', omni.usd.get_context().get_stage() is not None, flush=True)
while app.is_running():
    app.update()
app.close()
