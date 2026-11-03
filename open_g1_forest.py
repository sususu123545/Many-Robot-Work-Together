from isaacsim import SimulationApp
app = SimulationApp({'headless': False, 'width': 1440, 'height': 900, 'renderer': 'RaytracedLighting'})
import omni.usd
path = '/mnt/llm/g1_sim_ws/competition_sim/stages/gui_run1/g1_base_course.usd'
print('OPENING_SCENE', path, flush=True)
context = omni.usd.get_context()
context.open_stage(path)
for _ in range(900):
    app.update()
stage = context.get_stage()
print('SCENE_OPENED', stage is not None, flush=True)
if stage:
    for prim in stage.Traverse():
        name = prim.GetName()
        if any(s in name.lower() for s in ('g1', 'tree', 'obstacle', 'terrain', 'ground', 'ramp', 'step')):
            print('SCENE_PRIM', prim.GetPath(), flush=True)
while app.is_running():
    app.update()
app.close()
