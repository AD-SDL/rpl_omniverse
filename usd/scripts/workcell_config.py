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
import json

def add_rpl():
    # omni.kit.commands.execute('CreatePayload',
    #     path_to=Sdf.Path('/World/rpl'),
    #     asset_path='/home/vision/rpl_omniverse/usd/rooms/rpl/rpl.usda',
    #     usd_context=get_context()
    # )
    add_reference_to_stage(
        usd_path='/home/vision/rpl_omniverse/usd/rooms/rpl/rpl.usda',
        prim_path=f'/World/rpl',
    )
    XFormPrim(
        prim_path='/World/rpl',
        name='rpl',
        translation=np.array([0, 0, 0]),
        orientation=np.array([1, 0, 0, 0]),
        scale=np.array([1, 1, 1]),
    )

idx2t = {
    0: np.array([-0.75,  1.25, 0]),
    1: np.array([-0.75,  0.50, 0]),
    2: np.array([-0.75, -0.25, 0]),
    3: np.array([-0.75, -1.00, 0]),
    4: np.array([ 0.75,  1.25, 0]),
    5: np.array([ 0.75,  0.50, 0]),
    6: np.array([ 0.75, -0.25, 0]),
    7: np.array([ 0.75, -1.00, 0]),
}

idx2r = {
    0: np.array([0.70711, 0, 0, -0.70711]),
    1: np.array([0.70711, 0, 0, -0.70711]),
    2: np.array([0.70711, 0, 0, -0.70711]),
    3: np.array([0.70711, 0, 0, -0.70711]),
    4: np.array([0.70711, 0, 0,  0.70711]),
    5: np.array([0.70711, 0, 0,  0.70711]),
    6: np.array([0.70711, 0, 0,  0.70711]),
    7: np.array([0.70711, 0, 0,  0.70711]),
}

def add_empty_cell(name, translation, carts):
    add_reference_to_stage(
        usd_path='/home/vision/rpl_omniverse/usd/scenes/cell_empty.usda',
        prim_path=f'/World/{name}',
    )
    XFormPrim(
        prim_path=f'/World/{name}',
        name=name,
        translation=translation,
        orientation=np.array([1, 0, 0, 0]),
        scale=np.array([1, 1, 1]),
    )

    for idx, rob in carts.items():
        idx = int(idx)
        cart_translation = idx2t[idx]

        add_reference_to_stage(
            usd_path='/home/vision/rpl_omniverse/usd/objects/cart.usda',
            prim_path=f'/World/{name}_cart_{idx}',
        )
        XFormPrim(
            prim_path=f'/World/{name}_cart_{idx}',
            name=name,
            translation=translation+cart_translation,
            orientation=np.array([1, 0, 0, 0]),
            scale=np.array([1, 1, 1]),
        )

        if rob == 'None':
            continue

        add_reference_to_stage(
            usd_path=f'/home/vision/rpl_omniverse/usd/robots/{rob}.usda',
            prim_path=f'/World/{name}_cart_{idx}_robot',
        )
        XFormPrim(
            prim_path=f'/World/{name}_cart_{idx}_robot',
            name=name,
            translation=translation+cart_translation+np.array([0, 0, 1.0514]),
            orientation=idx2r[idx],
            scale=np.array([1, 1, 1]),
        )

def main():
    world = World(stage_units_in_meters=1.0)

    add_rpl()

    with open('robots.json') as f:
        robots = json.load(f)

    cell_1_x = 8
    cell_1_y = -10.4

    #for robot in robots:
    add_empty_cell(
        'cell_empty_0',
        [cell_1_x, cell_1_y, 0],
        robots['robot0']
    )
    add_empty_cell(
        'cell_empty_1',
        [cell_1_x+3.5, cell_1_y, 0],
        robots['robot1']
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
