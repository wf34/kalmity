#!/usr/bin/env python3

from multiprocessing import Process
import sys
import typing
import time
import os
import string
import re
import subprocess
import random

import numpy as np

from pydot import graph_from_dot_data
from tap import Tap

from pydrake.all import (
    AbstractValue,
    LeafSystem,
    StartMeshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    Parser,
    RotationMatrix,
    RigidTransform,
    MeshcatVisualizer,
    Simulator,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    TrajectorySource,
    Quaternion,
)

from make_frustum_mesh import create_rectangular_frustum_mesh_file

SIM_DELTA_T = 1.e-3

def raise_browser_for_meshcat(browser, target_url, comm_filename):
    print(f'Meshcat is now available at {target_url}')
    extra_opts='--enable-logging=stderr'
    cmd = [browser, target_url]

    if browser in ('chrome', 'chromium'):
        cmd.insert(1, extra_opts)

    # this requires a hack at: lib/python3.13/site-packages/pydrake/share/drake/geometry/meshcat.html
    pattern = re.compile(r'meshcat-has-loaded')
    process = subprocess.Popen(cmd, stderr=subprocess.PIPE, universal_newlines=True)
    while True:
        line = process.stderr.readline()
        if not line and process.poll() is not None:
            break
        line = line.strip()
        if line:
            match = pattern.search(line)
            if match:
                print(f'Pattern matched: {line}')
                with open(comm_filename, 'w') as the_file:
                    the_file.write('1')

def detachify(func):
    '''Decorate a function so that its calls are async in a detached process.'''

    # create a process fork and run the function
    def forkify(*args, **kwargs):
        if os.fork() != 0:
            return
        func(*args, **kwargs)

    # wrapper to run the forkified function
    def wrapper(*args, **kwargs):
        proc = Process(target=lambda: forkify(*args, **kwargs))
        proc.start()
        proc.join()
        return

    return wrapper


def open_browser_link(replay_browser, meshcat_web_url):
    characters = string.ascii_letters + string.digits
    random_string = ''.join(random.choices(characters, k=20))
    comm_filename = f'/tmp/{random_string}.buf'
    detachify(raise_browser_for_meshcat)(replay_browser, meshcat_web_url, comm_filename)
    time_at_detach = time.time()
    load_finished = False
    while time.time() - time_at_detach < 20. and not load_finished:
        if os.path.exists(comm_filename):
            with open(comm_filename, 'r') as the_file:
                status = int(the_file.read().strip())
                if status == 1:
                    load_finished = True
        time.sleep(1.)


class SimArgs(Tap):
    seed: int = 34
    experiment_duration: float = 5.  # in seconds
    spheres_count: int = 50
    volume: typing.List[typing.List[float]] = [[0, 0, 0], [10, 5, 5]]
    replay_browser: str = 'chromium'
    diagram_destination: str = 'sim_diagram.png'

    def process_args(self):
        self.volume = np.array(self.volume)


def make_sphere(parser, id_, R=0.03, mass=0.1, color=[0.8, 0.2, 0.2, 1.0]):
    inertia = .4 * mass * R ** 2
    sphere_str = None
    with open(os.path.join(os.path.dirname(sys.argv[0]), 'sphere_sdf.template'), 'r') as the_file:
        sphere_str = the_file.read() \
            .replace('{{ name }}', id_) \
            .replace('{{ mass }}', str(mass)) \
            .replace('{{ inertia }}', str(inertia)) \
            .replace('{{ R }}', str(R)) \
            .replace('{{ rgba }}', ' '.join(map(str, color)))

    if sphere_str is None:
        raise Exception('unreachable')

    return parser.AddModelsFromString(sphere_str, 'sdf')[0]


def calculate_rect_frustum_inertia(mass, bottom_width, top_width, aspect_ratio, height):
    bw, bh = bottom_width, bottom_width / aspect_ratio
    tw, th = top_width, top_width / aspect_ratio

    h = height
    c = mass / 20.

    return c * (th**2 + th*th + th**2 + 3*h**2), \
           c * (bw**2 + bw*tw + tw**2 + 3*h**2), \
           c * (bw**2 + bw*tw + tw**2 + bh**2 + bh*th + th**2)


def make_frustum(parser, id_='slam_agent', color=[0.2, 0.2, 0.8, 1.0]):
    mass = 1.
    bottom_width = .2
    top_width = .02
    aspect_ratio = 1.777
    height = .3

    #obj_filename = create_rectangular_frustum_mesh_file('rect_frustum.obj', bottom_width, top_width, aspect_ratio, height)
    obj_filename = 'rect_frustum.obj'
    full_path = os.path.join(os.path.realpath(os.path.dirname(sys.argv[0])), obj_filename)
    Ixx, Iyy, Izz = calculate_rect_frustum_inertia(mass, bottom_width, top_width, aspect_ratio, height)

    frustum_str = None
    with open(os.path.join(os.path.dirname(sys.argv[0]), 'frustum_sdf.template'), 'r') as the_file:
        frustum_str = the_file.read() \
            .replace('{{ name }}', id_) \
            .replace('{{ mass }}', str(mass)) \
            .replace('{{ inertia_x }}', str(Ixx)) \
            .replace('{{ inertia_y }}', str(Iyy)) \
            .replace('{{ inertia_z }}', str(Izz)) \
            .replace('{{ rgba }}', ' '.join(map(str, color))) \
            .replace('{{ obj_path }}', full_path)

    if frustum_str is None:
        raise Exception('unreachable')

    return parser.AddModelsFromString(frustum_str, 'sdf')[0]


def make_figure8_translational_trajectory(duration: float, X_WL: RigidTransform, scale: np.array):
    num_samples = 100
    times = np.linspace(0, duration, num_samples)

    positions = []
    for t in times:
        theta = 2 * np.pi * t / duration

        # Lemniscate of Bernoulli
        x = scale[0] * np.sin(theta) / 2
        y = scale[1] * np.sin(theta) * np.cos(theta) / 2
        z = 0.

        positions.append([x, y, z, 1])

    positions_local = np.array(positions).T  # 4xN
    positions_world = X_WL.GetAsMatrix4() @ positions_local
    positions_world[:-1, :] /= positions_world[-1, :]
    positions_world = positions_world[:-1, :]

    trajectory = PiecewisePolynomial.CubicShapePreserving(
        times, positions_world, zero_end_point_derivatives=True
    )

    return trajectory

def quaternion_from_vectors(source, target):

    a = source / np.linalg.norm(source)
    b = target / np.linalg.norm(target)

    dot = np.dot(a, b)

    if dot >= 1.0:
        # Vectors are the same
        return Quaternion.Identity()
    elif dot <= -1.0:
        # Vectors are opposite
        # Find perpendicular axis
        if abs(a[0]) < 0.9:
            axis = np.cross(a, [1, 0, 0])
        else:
            axis = np.cross(a, [0, 1, 0])
        axis = axis / np.linalg.norm(axis)
        return Quaternion(w=0, x=axis[0], y=axis[1], z=axis[2])

    # General case
    cross = np.cross(a, b)
    w = 1.0 + dot
    arr = np.array([w] + cross.tolist())
    arr /= np.linalg.norm(arr)
    w, x,y,z = arr.tolist()
    return Quaternion(w=w, x=x, y=y, z=z)


def make_circular_roto_trajectory(duration: float):
    model_lookvec = np.array([0, 0, -1])

    origin_lookvec = np.array([0, 1, 0])
    half_motion_lookvec = origin_lookvec * -1
    quat_lookvec = [-1, 0, 0]
    three_quat_lookvec = [1, 0, 0]
    steps = 4
    vecs = [origin_lookvec, quat_lookvec, half_motion_lookvec, three_quat_lookvec, origin_lookvec]
    times = np.linspace(0, duration, steps +1).tolist()
    quats = list(map(lambda x: quaternion_from_vectors(model_lookvec, x), vecs))

    return PiecewiseQuaternionSlerp(times, quats)


class FrustumMover(LeafSystem):
    def __init__(self, plant, body):
        LeafSystem.__init__(self)
        self.plant = plant
        self.plant_context = None
        self.body = body

        self.DeclareVectorInputPort('commanded_translation', 3)
        self.DeclareVectorInputPort('commanded_orientation', 4)

        self.DeclareContinuousState(1)

    def Initialize(self, plant_context):
        self.plant_context = plant_context

    def DoCalcTimeDerivatives(self, context, derivatives):
        if self.plant_context is None:
            derivatives.get_mutable_vector().SetFromVector([0.0])
            return

        t = self.GetInputPort('commanded_translation').Eval(context)
        q = Quaternion(*self.GetInputPort('commanded_orientation').Eval(context))
        X_WO = RigidTransform(q, t)
        self.plant.SetFreeBodyPose(self.plant_context, self.body, X_WO)
        derivatives.get_mutable_vector().SetFromVector([1.0])


def run_sim(args: SimArgs):
    np.random.seed(args.seed)

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=SIM_DELTA_T)
    parser = Parser(plant, scene_graph)

    frustum_model = make_frustum(parser)
    # plant.set_gravity_enabled(frustum_model, False)
    frustum_body = plant.GetBodyByName('base_link', frustum_model)


    for id_ in range(args.spheres_count):
        sphere_model = make_sphere(parser, f'sphr_{id_:04d}')
        sphere_body = plant.GetBodyByName('sphere_base', sphere_model)
        position = np.random.uniform(low=args.volume[0, :], high=args.volume[1, :])
        plant.WeldFrames(
            plant.world_frame(),
            sphere_body.body_frame(),
            RigidTransform(position)
        )

    R_WL = RotationMatrix.MakeYRotation(np.radians(-15.)).multiply(RotationMatrix.MakeXRotation(np.radians(10.)))
    vol = args.volume[1, :] - args.volume[0, :]
    span = vol * 1.5
    t_L_W = vol / 2
    X_WL = RigidTransform(R_WL, t_L_W)

    t = make_figure8_translational_trajectory(args.experiment_duration, X_WL, span)
    r = make_circular_roto_trajectory(args.experiment_duration)
    tr_trajectory_source = builder.AddSystem(TrajectorySource(t))
    ro_trajectory_source = builder.AddSystem(TrajectorySource(r))

    frustum_mover = builder.AddSystem(FrustumMover(plant, frustum_body))

    plant.Finalize()

    builder.Connect(
        tr_trajectory_source.get_output_port(0),
        frustum_mover.GetInputPort('commanded_translation')
    )

    builder.Connect(
        ro_trajectory_source.get_output_port(0),
        frustum_mover.GetInputPort('commanded_orientation')
    )

    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    graph_from_dot_data(diagram.GetGraphvizString(max_depth=4))[0].write_png(args.diagram_destination)

    context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    meshcat.StartRecording(set_visualizations_while_recording=False)

    plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    frustum_mover.Initialize(plant_context)

    simulator.AdvanceTo(args.experiment_duration)
    meshcat.PublishRecording()
    open_browser_link(args.replay_browser, meshcat.web_url())


if '__main__' == __name__:
    run_sim(SimArgs().parse_args())
