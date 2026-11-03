"""Isaac Sim 4.5 installation check; local primitives only, no robot commands."""

import json
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp(
    {"headless": True, "width": 640, "height": 480, "renderer": "RaytracedLighting"}
)

try:
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
    world.scene.add(
        FixedCuboid(
            prim_path="/World/floor",
            name="floor",
            position=np.array([0.0, 0.0, -0.05]),
            scale=np.array([4.0, 4.0, 0.1]),
            size=1.0,
        )
    )
    cube = world.scene.add(
        DynamicCuboid(
            prim_path="/World/test_cube",
            name="test_cube",
            position=np.array([0.0, 0.0, 1.0]),
            size=0.1,
            mass=1.0,
        )
    )
    world.reset()
    for step in range(180):
        world.step(render=(step % 30 == 0))
    position, _ = cube.get_world_pose()
    velocity = cube.get_linear_velocity()
    passed = bool(
        np.isfinite(position).all()
        and 0.04 < float(position[2]) < 0.08
        and np.linalg.norm(velocity) < 0.05
    )
    result = {
        "passed": passed,
        "physics_steps": 180,
        "rendered_steps": 6,
        "cube_position": position.tolist(),
        "cube_velocity": velocity.tolist(),
        "scope": "Isaac Sim startup, primitive collision and rendering; no robot validation",
    }
    Path("smoke-test-result.json").write_text(json.dumps(result, indent=2) + "\n")
    print("ISAAC_SMOKE_TEST " + json.dumps(result), flush=True)
    if not passed:
        raise RuntimeError("The cube did not settle correctly on the test floor")
finally:
    app.close()
