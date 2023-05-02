try:
    from omni.isaac.kit import SimulationApp
except ModuleNotFoundError:
    import sys
    print(f'usage: ~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh {sys.argv[0]}')
    exit()
simulation_app = SimulationApp({'headless': False})

import math
import numpy as np
import time
from typing import Optional, List
import omni.isaac.core.tasks
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.objects import cuboid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.usd import get_context
from pxr import Usd, Sdf, Gf

def main():
    world = World(stage_units_in_meters=1.0)

    add_reference_to_stage(
        usd_path=usd_path,
        prim_path=prim_path,
    )
    XFormPrim(
        prim_path=prim_path,
        name=name,
        translation=[0, 0, 0],
        orientation=[1, 0, 0, 0],
        scale=[1, 1, 1],
    )

    world.reset()

    while simulation_app.is_running():
        world.step(render=True)
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
            
            # Do stuff

    simulation_app.close()

if __name__ == '__main__':
    main()
