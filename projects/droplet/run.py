try:
    from omni.isaac.kit import SimulationApp
except ModuleNotFoundError:
    print('usage:  ~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh run.py')
    exit()
simulation_app = SimulationApp({'headless': False})

import math
import numpy as np
import time
from pathlib import Path

import omni.isaac.core.tasks
import omni.isaac.manipulators.controllers
import omni.kit.commands
from omni.isaac.core import World
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.rotations import euler_angles_to_quat
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.motion_generation import ArticulationMotionPolicy
from omni.isaac.motion_generation.lula import RmpFlow
from omni.usd import get_context
from pxr import Usd, Sdf, Gf

PATH_BASE = Path(__file__).absolute().parent
TIME_MULTIPLIER = 2

# The Sequencer holds a list of steps and activates the current one on request
class Sequencer():
    def __init__(self):
        self.start_time = time.time()
        self.steps = []

    def reset(self):
        self.start_time = time.time()
        for step in self.steps:
            step.reset()

    def do(self):
        # Save start and end timestamps
        s = self.start_time
        e = time.time()

        # Find which step we are in now
        for step in self.steps:
            if s + step.dtime > e:
                break
            s += step.dtime
        else:
            # If we are past all steps, just sleep
            step = DoneStep()
            step.activated = True

        # Activate the step, and get a target pose to move to, if relevant
        return step.activate()

# Each Step defines a slice of time and what to do during that time
class SleepStep():
    def __init__(self, dtime):
        self.dtime = dtime * TIME_MULTIPLIER
        self.activated = False

    def reset(self):
        self.activated = False

    def activate(self):
        if not self.activated:
            print('Sleeping')

        self.activated = True

class MoveStep():
    def __init__(self, dtime, pos, rot):
        self.dtime = dtime * TIME_MULTIPLIER
        self.pos = pos
        self.rot = rot
        self.activated = False

    def reset(self):
        pass

    def activate(self):
        if not self.activated:
            print('Moving')

        self.activated = True

        return np.array(self.pos), np.array(self.rot)

class AttachStep():
    def __init__(self, dtime, body0, body1, pos0=None, pos1=None):
        self.dtime = dtime * TIME_MULTIPLIER
        self.body0 = body0
        self.body1 = body1
        self.pos0 = pos0
        self.pos1 = pos1
        self.activated = False

    def reset(self):
        self.activated = False

    def activate(self):
        if not self.activated:
            print('Attaching', self.body0, self.body1)

            stage = get_context().get_stage()

            # (True, Usd.Prim(</World/pipette_assembly/node_/FixedJoint>))
            omni.kit.commands.execute(
                'CreateJointCommand',
                stage=stage,
                joint_type='Fixed',
                from_prim=stage.GetPrimAtPath(self.body0),
                to_prim=stage.GetPrimAtPath(self.body1),
            )

            omni.kit.commands.execute(
                'ChangeProperty',
                prop_path=Sdf.Path(self.body1 + '.physics:rigidBodyEnabled'),
                value=True,
                prev=None,
            )

            omni.kit.commands.execute(
                'ChangeProperty',
                prop_path=Sdf.Path(self.body1 + '.physics:kinematicEnabled'),
                value=False,
                prev=None,
            )

        if self.pos0 is not None:
            omni.kit.commands.execute(
                'ChangeProperty',
                prop_path=Sdf.Path(self.body1 + '/FixedJoint.physics:localPos0'),
                value=Gf.Vec3f(*self.pos0),
                prev=Gf.Vec3f(0.0, 0.0, 0.0),
            )

        if self.pos1 is not None:
            omni.kit.commands.execute(
                'ChangeProperty',
                prop_path=Sdf.Path(self.body1 + '/FixedJoint.physics:localPos1'),
                value=Gf.Vec3f(*self.pos1),
                prev=Gf.Vec3f(0.0, 0.0, 0.0),
            )

        self.activated = True

class DetachStep():
    def __init__(self, dtime, body0, body1):
        self.dtime = dtime * TIME_MULTIPLIER
        self.body0 = body0
        self.body1 = body1
        self.activated = False

    def reset(self):
        self.activated = False

    def activate(self):
        if not self.activated:
            print('Detaching', self.body0, self.body1)

            omni.kit.commands.execute(
                'DeletePrims',
                paths=[self.body1 + '/FixedJoint'],
                destructive=False,
            )

        self.activated = True

class DoneStep():
    def __init__(self):
        self.dtime = 1
        self.activated = False

    def reset(self):
        self.activated = False

    def activate(self):
        if not self.activated:
            print('Done')

        self.activated = True

class FollowTargetStep():
    def __init__(self, target):
        self.dtime = math.inf
        self.target = target
        self.activated = False

    def reset(self):
        self.activated = False

    def activate(self):
        if not self.activated:
            print('Following', self.target)

        self.activated = True

        return self.target.get_world_pose()

def main():
    world = World(stage_units_in_meters=1.0)

    # Add droplet
    add_reference_to_stage(
        usd_path=str(PATH_BASE / 'usd/droplet.usda'),
        prim_path='/World/droplet',
    )
    XFormPrim(
        prim_path='/World/droplet',
        name='droplet',
        translation=[0, 0, 0],
        orientation=[1, 0, 0, 0],
        scale=[0.01, 0.01, 0.01],
    )

    add_reference_to_stage(
        usd_path=str(PATH_BASE / 'usd/pipette_assembly.usdc'),
        prim_path='/World/pipette_assembly',
    )
    XFormPrim(
        prim_path='/World/pipette_assembly',
        name='pipette_assembly',
        translation=[0, 0, 0],
        orientation=[0.70711, 0.70711, 0, 0],
        scale=[0.01, 0.01, 0.01],
    )

    for i in range(8):
        for j in range(12):
            add_reference_to_stage(
                usd_path=str(PATH_BASE / 'usd/tip.usd'),
                prim_path=f'/World/tip_{i}_{j}',
            )
            XFormPrim(
                prim_path=f'/World/tip_{i}_{j}',
                name=f'tip_{i}_{j}',
                translation=[0.35427 - 0.00901*j, 0.13092 - 0.00901*i, -0.16065],
                orientation=[0.70711, 0.70711, 0, 0],
                scale=[0.01, 0.01, 0.01],
            )

    add_reference_to_stage(
        usd_path=str(PATH_BASE / 'usd/ur3e.usd'),
        prim_path='/World/ur3e',
    )
    robot = world.scene.add(Robot(
        prim_path='/World/ur3e',
        name='ur3e',
        translation=[0.30497, -0.30425, -0.06094],
        orientation=[1, 0, 0, 0],
        scale=[1, 1, 1],
    ))
    robot.set_joints_default_state(positions=np.array([-0.8, -1.8, -1.2, -1.7, 1.6, 0.0]))
    articulation_controller = robot.get_articulation_controller()


    # target = cuboid.VisualCuboid(
    #     prim_path='/World/target',
    #     translation=[-0.00016, -0.0773, 0.18685],
    #     orientation=[0.0, 0.0, -1.0, 0.0],
    #     scale=[0.01, 0.01, 0.01],
    #     color=np.array([1.0, 0.0, 0.0]),
    # )


    rmpflow = RmpFlow(
        robot_description_path=str(PATH_BASE / 'ur3e_descriptor.yaml'),
        urdf_path=str(PATH_BASE / 'ur3e.urdf'),
        rmpflow_config_path=str(PATH_BASE / 'ur3e_rmpflow.yaml'),
        end_effector_frame_name='wrist_3_link',
        maximum_substep_size=.0034,
    )
    # rmpflow.visualize_collision_spheres()
    articulation_rmpflow = ArticulationMotionPolicy(robot, rmpflow, default_physics_dt=1/120*TIME_MULTIPLIER)


    rot_a = np.array([0.0, 0.0, -1.0, 0.0])
    rot_b = np.array([0.0, -0.70711, 0.70711, 0.0])

    pos_pipette = np.array([-0.00016, -0.0773, 0.18685])
    pos_tip = np.array([0.35466, 0.05438, 0.0])
    pos_vial0 = np.array([0.45630, -0.08413, 0.0])
    pos_vial1 = np.array([0.45630, -0.08413-0.02037, 0.0])
    pos_vial2 = np.array([0.45630, -0.08413-0.02037-0.02037, 0.0])
    pos_mix = np.array([0.43205, -0.08315, 0.0])
    pos_trash = np.array([0.49114, -0.006, 0.0])
    pos_distant = np.array([0.4, 0.0, 0.3])
    z = np.array([0.0, 0.0, 1.0])

    sequencer = Sequencer()
    sequencer.steps.extend([
        # Let the scene settle
        SleepStep(dtime=5),

        # Grab pipette assembly
        MoveStep(dtime=5, pos=pos_pipette + 0.1*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette, rot=rot_a),
        AttachStep(
            dtime=0.2,
            body0='/World/ur3e/tool0',
            body1='/World/pipette_assembly/node_',
            pos0=(0.001, 0.07772, 0.18494),
            pos1=(0.0, 0.0, 0.0),
        ),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),

        # Home
        MoveStep(dtime=3, pos=pos_distant, rot=rot_a),

        # Pipette tip 0,0
        MoveStep(dtime=2, pos=pos_tip + 0.07*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + 0.015*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + -0.003*z, rot=rot_a),
        AttachStep(
            dtime=0.2,
            body0='/World/pipette_assembly/node_',
            body1='/World/tip_0_0/node_',
            pos0=(0.0, 0.029, 0.0),
            pos1=(0.0, 0.0, 0.0),
        ),
        MoveStep(dtime=2, pos=pos_tip + 0.015*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + 0.07*z, rot=rot_a),
    ])

    for i in range(-180, -80, 10):
        sequencer.steps.append(
            MoveStep(dtime=0.2, pos=pos_tip + 0.07*z, rot=euler_angles_to_quat(np.array([-180, 0, i]), degrees=True)),
        )

    sequencer.steps.extend([
        # Vial 0
        MoveStep(dtime=4, pos=pos_vial0 + 0.07*z, rot=rot_b),
        MoveStep(dtime=2, pos=pos_vial0 + 0.015*z, rot=rot_b),
        SleepStep(dtime=1),
        MoveStep(dtime=2, pos=pos_vial0 + 0.07*z, rot=rot_b),

        # Mix 0,0
        MoveStep(dtime=2, pos=pos_mix + 0.07*z, rot=rot_b),
        MoveStep(dtime=2, pos=pos_mix + 0.02*z, rot=rot_b),
        SleepStep(dtime=1),
        MoveStep(dtime=2, pos=pos_mix + 0.07*z, rot=rot_b),

        # Trash
        MoveStep(dtime=2, pos=pos_trash + 0.07*z, rot=rot_b),
        MoveStep(dtime=0.5, pos=pos_trash, rot=rot_b),
        MoveStep(dtime=2, pos=pos_trash + -0.06*z, rot=rot_b),
        DetachStep(
            dtime=0.2,
            body0='/World/pipette_assembly/node_',
            body1='/World/tip_0_0/node_',
        ),
        MoveStep(dtime=0.5, pos=pos_trash, rot=rot_b),
        MoveStep(dtime=2, pos=pos_trash + 0.07*z, rot=rot_b),
    ])

    for i in range(-90, -190, -10):
        sequencer.steps.append(
            MoveStep(dtime=0.2, pos=pos_trash + 0.07*z, rot=euler_angles_to_quat(np.array([-180, 0, i]), degrees=True)),
        )

    sequencer.steps.extend([
        # Pipette tip 0,0
        MoveStep(dtime=2, pos=pos_tip + np.array([0, -0.00901, 0]) + 0.07*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + np.array([0, -0.00901, 0]) + 0.015*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + np.array([0, -0.00901, 0]) + -0.003*z, rot=rot_a),
        AttachStep(
            dtime=0.2,
            body0='/World/pipette_assembly/node_',
            body1='/World/tip_1_0/node_',
            pos0=(0.0, 0.029, 0.0),
            pos1=(0.0, 0.0, 0.0),
        ),
        MoveStep(dtime=2, pos=pos_tip + np.array([0, -0.00901, 0]) + 0.015*z, rot=rot_a),
        MoveStep(dtime=2, pos=pos_tip + np.array([0, -0.00901, 0]) + 0.07*z, rot=rot_a),
    ])

    for i in range(-180, -80, 10):
        sequencer.steps.append(
            MoveStep(dtime=0.2, pos=pos_tip + 0.07*z, rot=euler_angles_to_quat(np.array([-180, 0, i]), degrees=True)),
        )

    sequencer.steps.extend([
        # Vial 1
        MoveStep(dtime=4, pos=pos_vial1 + 0.07*z, rot=rot_b),
        MoveStep(dtime=2, pos=pos_vial1 + 0.015*z, rot=rot_b),
        SleepStep(dtime=1),
        MoveStep(dtime=2, pos=pos_vial1 + 0.07*z, rot=rot_b),

        # Mix 0,0
        MoveStep(dtime=2, pos=pos_mix + 0.07*z, rot=rot_b),
        MoveStep(dtime=2, pos=pos_mix + 0.02*z, rot=rot_b),
        SleepStep(dtime=6),
        MoveStep(dtime=2, pos=pos_mix + 0.07*z, rot=rot_b),

        # Home
        MoveStep(dtime=5, pos=pos_distant, rot=rot_a),

        # Put down pipette assembly
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette, rot=rot_a),
        DetachStep(
            dtime=0.2,
            body0='/World/ur3e/tool0',
            body1='/World/pipette_assembly/node_',
        ),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),
        SleepStep(dtime=6),

        # Grab pipette assembly
        MoveStep(dtime=2, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette, rot=rot_a),
        AttachStep(
            dtime=0.2,
            body0='/World/ur3e/tool0',
            body1='/World/pipette_assembly/node_',
            pos0=(0.001, 0.07772, 0.18494),
            pos1=(0.0, 0.0, 0.0),
        ),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),

        # Home
        MoveStep(dtime=5, pos=pos_distant, rot=rot_a),
    ])

    for i in range(-180, -80, 10):
        sequencer.steps.append(
            MoveStep(dtime=0.2, pos=pos_tip + 0.07*z, rot=euler_angles_to_quat(np.array([-180, 0, i]), degrees=True)),
        )

    sequencer.steps.extend([
        # 3rd vial
        MoveStep(dtime=4, pos=pos_vial2 + 0.07*z, rot=rot_b),
        MoveStep(dtime=2, pos=pos_vial2 + 0.015*z, rot=rot_b),
        SleepStep(dtime=5),
        MoveStep(dtime=2, pos=pos_vial2 + 0.07*z, rot=rot_b),

        # Trash
        MoveStep(dtime=2, pos=pos_trash + 0.07*z, rot=rot_b),
        MoveStep(dtime=0.5, pos=pos_trash, rot=rot_b),
        MoveStep(dtime=2, pos=pos_trash + -0.06*z, rot=rot_b),
        DetachStep(
            dtime=0.2,
            body0='/World/pipette_assembly/node_',
            body1='/World/tip_1_0/node_',
        ),
        MoveStep(dtime=0.5, pos=pos_trash, rot=rot_b),
        MoveStep(dtime=2, pos=pos_trash + 0.07*z, rot=rot_b),
    ])

    for i in range(-90, -190, -10):
        sequencer.steps.append(
            MoveStep(dtime=0.2, pos=pos_trash + 0.07*z, rot=euler_angles_to_quat(np.array([-180, 0, i]), degrees=True)),
        )

    sequencer.steps.extend([
        # Home
        MoveStep(dtime=5, pos=pos_distant, rot=rot_a),

        # Put down pipette assembly
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette, rot=rot_a),
        DetachStep(
            dtime=0.2,
            body0='/World/ur3e/tool0',
            body1='/World/pipette_assembly/node_',
        ),
        MoveStep(dtime=3, pos=pos_pipette + 0.05*z, rot=rot_a),
        MoveStep(dtime=3, pos=pos_pipette + 0.1*z, rot=rot_a),
    ])

    sequencer.steps.append(
        DoneStep(),
    )
    # sequencer.steps.append(
    #     FollowTargetStep(target),
    # )

    world.reset()

    time_prev = time.time()
    while simulation_app.is_running():
        while time.time() < time_prev + (1/120*TIME_MULTIPLIER):
            time.sleep(1/1200*TIME_MULTIPLIER)
        time_prev = time.time()

        world.step(render=True)
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                sequencer.reset()

            target_pose = sequencer.do()

            if target_pose is None:
                action = ArticulationAction(joint_positions=[None, None, None, None, None, None])
            else:
                robot_pose = robot.get_world_pose()
                rmpflow.set_end_effector_target(
                    target_position=target_pose[0] - robot_pose[0],
                    target_orientation=target_pose[1],
                )
                action = articulation_rmpflow.get_next_articulation_action()

            articulation_controller.apply_action(action)

    simulation_app.close()

if __name__ == '__main__':
    main()
