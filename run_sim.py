#!/usr/bin/env python3

from multiprocessing import Process
import sys
import typing
import math
import time
import os
import string
import re
import subprocess
import random

import numpy as np

from pydot import graph_from_dot_data
from tap import Tap

import matplotlib.pyplot as plt

from pydrake.all import (
    MakeRenderEngineVtk,
    RenderEngineVtkParams,
    AbstractValue,
    LeafSystem,
    StartMeshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    Parser,
    Sphere,
    RotationMatrix,
    RigidTransform,
    MeshcatVisualizer,
    Simulator,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    Trajectory,
    TrajectorySource,
    CameraInfo,
    RgbdSensor,
    RenderCameraCore,
    ColorRenderCamera,
    Cylinder,
    DepthRenderCamera,
    DepthRange,
    ClippingRange,
    FrameId,
    AbstractValue,
    Rgba,
    RollPitchYaw,
    Quaternion,
)

from make_frustum_mesh import create_rectangular_frustum_mesh_file

SIM_DELTA_T = 3.e-2

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


def visualize_trajectory_with_cylinders(meshcat_vis, trajectory_points, line_name="trajectory"):
    """
    Visualize trajectory using thin cylinders as line segments
    """
    for i in range(len(trajectory_points) - 1):
        start_point = trajectory_points[i]
        end_point = trajectory_points[i + 1]

        # Calculate line properties
        direction = end_point - start_point
        length = np.linalg.norm(direction)

        if length < 1e-6:  # Skip very short segments
            continue

        # Calculate midpoint
        midpoint = (start_point + end_point) / 2

        # Calculate rotation to align cylinder with line direction
        z_axis = direction / length
        # Create orthogonal vectors
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross([0, 0, 1], z_axis)
        else:
            x_axis = np.cross([1, 0, 0], z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        # Create rotation matrix
        rotation = RotationMatrix(np.column_stack([x_axis, y_axis, z_axis]))

        # Create transform
        transform = RigidTransform(rotation, midpoint)

        # Create cylinder geometry (thin radius for line appearance)
        cylinder = Cylinder(radius=0.01, length=length)

        # Add to meshcat
        segment_name = f"{line_name}_segment_{i}"
        meshcat_vis.SetObject(
            segment_name,
            cylinder,
            Rgba(0.0, 0.0, 0.0, 1.0)
        )
        meshcat_vis.SetTransform(segment_name, transform)


def make_sphere_passive_vis(meshcat_vis, sphere_name, meshcat, X_WB, color=[0.8, 0.2, 0.2, 0.5], R=0.05):
    passive_sphere_name = f'passive_{sphere_name}'
    meshcat_vis.SetObject(
        passive_sphere_name,
        Sphere(R),
        Rgba(*color),
    )
    meshcat_vis.SetTransform(passive_sphere_name, X_WB)


def make_sphere(parser, id_, R=0.1, mass=0.1, color=[0.2, 0.8, 0.2, 1.0]):
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


def make_frustum(parser, id_='slam_agent', color=[0.2, 0.2, 0.8, 1.0], aspect_ratio = 1.777, hfov=None):
    if hfov is None:
        raise Exception('unreachable')
    mass = 1.
    bottom_width = .2
    top_width = .02
    height = bottom_width / (2 * math.tan(hfov / 2.))

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


def make_camera(camera_name: str, frame_id: FrameId, aspect_ratio: float, hfov: float) -> RgbdSensor:
    width = 640
    height = int(width // aspect_ratio)

    focal_x = width / (2 * math.tan(hfov / 2.))
    focal_y = focal_x
    center_x, center_y = width/2, height/2  # principal point
    
    camera_info = CameraInfo(
        width=width,
        height=height,
        focal_x=focal_x,
        focal_y=focal_y,
        center_x=center_x,
        center_y=center_y
    )
    render_camera_core = RenderCameraCore(
        camera_name,
        camera_info,
        clipping=ClippingRange(5e-3, 100.0),
        X_BS=RigidTransform(),
    )
    color_camera = ColorRenderCamera(render_camera_core)
    depth_camera = DepthRenderCamera(render_camera_core, depth_range=DepthRange(6e-3, 100.))
    R_PB = quaternion_from_vectors([0, 0, -1], [0, 0, 1])
    tB_P = [0, 0, 0.119175]  # this needs to be regenerated when frustum is redone
    return RgbdSensor(
        parent_id=frame_id,
        X_PB=RigidTransform(R_PB, tB_P),
        color_camera=color_camera,
        depth_camera=depth_camera,
    )


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

    return trajectory, positions_world.T

def quaternion_from_vectors(target, source):

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


def make_circular_roto_trajectory(duration: float, translational_t: Trajectory):
    timespan = translational_t.end_time() - translational_t.start_time()
    t_origin = translational_t.value(0)
    model_lookvec = np.array([0, 0, -1])

    origin_lookvec = [0, 1, 0]
    r_LB = quaternion_from_vectors(origin_lookvec, model_lookvec) # To local attitude from Obj-file attitude

    yaws = [0]
    steps = 8
    for resp_time in map(lambda x: timespan * x / steps, range(1, steps)):
        look_at = translational_t.value(resp_time) - t_origin
        look_at /= np.linalg.norm(look_at)
        yaws.append(np.arctan2(look_at[1], look_at[0]) + np.pi/2)
        assert look_at[2] == 0
    yaws.append(0)

    for step, v in enumerate(yaws):
        rel_time = step / steps
        print(step, rel_time, np.degrees(v))

    times = np.linspace(0, duration, steps +1).tolist()
    quats = list(map(lambda yaw: RotationMatrix(RollPitchYaw(0, 0, yaw)).ToQuaternion().multiply(r_LB), yaws))

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


def check_visibility(X_WS: RigidTransform, intrinsics: CameraInfo, p_W: np.array) -> bool:
    width, height = intrinsics.width(), intrinsics.height()
    fx, fy = intrinsics.focal_x(), intrinsics.focal_y()
    cx, cy = intrinsics.center_x(), intrinsics.center_y()
    p_S = X_WS.inverse() @ p_W
    K = intrinsics.intrinsic_matrix()
    if p_S[2] <= 0.:
        return False  # is behind
    p_px = K @ p_S
    u, v = p_px[:2]

    return 0. <= u and u <= width and \
           0. <= v and v <= height


class SphereRecolorer(LeafSystem):

    def __init__(self, meshcat, plant, sphere_models, frustum_model, camera):
        LeafSystem.__init__(self)
        self.meshcat = meshcat
        self.plant = plant
        self.sphere_models = sphere_models
        self.frustum_model = frustum_model
        self.agents_camera = camera

        self.DeclareAbstractInputPort(
            "body_poses",
            model_value=AbstractValue.Make([RigidTransform()])
        )

        self.DeclarePerStepDiscreteUpdateEvent(self.calc_recoloring)


    def initialize(self, camera_context):
        self.camera_context = camera_context
        self.color_camera = self.agents_camera.GetColorRenderCamera(self.camera_context)
        core_camera = self.color_camera.core()
        self.camera_info = core_camera.intrinsics()
        self.X_PB = self.agents_camera.GetX_PB(self.camera_context)
        self.X_BS = core_camera.sensor_pose_in_camera_body()


    def calc_recoloring(self, context, state):
        now = context.get_time()
        poses = self.GetInputPort('body_poses').Eval(context)
        agents_ind = self.plant.GetBodyByName('base_link', self.frustum_model).index()
        X_WA = poses[agents_ind]
        X_WS = X_WA @ self.X_PB @ self.X_BS
        vis_cnt = 0
        for model in self.sphere_models:
            ind = self.plant.GetBodyByName('sphere_base', model).index()
            p_WSphere = poses[ind].translation()
            vis_status = check_visibility(X_WS, self.camera_info, p_WSphere)
            model_name = self.plant.GetModelInstanceName(model)
            path = f'visualizer/{model_name}/sphere_base/{model_name}/sphere_base_vis'
            self.meshcat.SetProperty(path, "visible", vis_status, time_in_recording=context.get_time())
            vis_cnt += int(vis_status)

        print(f'{now:.4f}, visible: {vis_cnt}')

        #color_image = self.agents_camera.color_image_output_port().Eval(self.camera_context)
        #color_array = np.array(color_image.data).reshape(
        #      480  , color_image.width(), 3
        #) #color_image.height()
        #fig = plt.figure(figsize=(10, 7.5))#5.6274))
        #ax = fig.subplots(nrows=1, ncols=1)
        #ax.imshow(color_array)
        #fig.savefig(f'frames/{now:.4f}.png')

        #self.meshcat.SetObject('indicator_sphere', Sphere(0.1), rgba=Rgba(*self.get_color(vis_cnt > 0)), time=context.get_time())
        #self.meshcat.SetTransform('indicator_sphere', RigidTransform())


def run_sim(args: SimArgs):
    np.random.seed(args.seed)

    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    render_params = RenderEngineVtkParams()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=SIM_DELTA_T)
    render_engine = MakeRenderEngineVtk(render_params)
    scene_graph.AddRenderer("agents_cam", render_engine)
    parser = Parser(plant, scene_graph)

    camera_name = 'agents_cam'
    aspect_ratio = 1.777
    hfov = np.radians(80.)

    frustum_model = make_frustum(parser, aspect_ratio=aspect_ratio, hfov=hfov)
    frustum_body = plant.GetBodyByName('base_link', frustum_model)
    body_frame_id = plant.GetBodyFrameIdOrThrow(frustum_body.index())
    camera = make_camera(camera_name, body_frame_id, aspect_ratio, hfov)

    camera_system = builder.AddSystem(camera)
    builder.Connect(
        scene_graph.get_query_output_port(),
        camera.query_object_input_port()
    )

    # plant.set_gravity_enabled(frustum_model, False)

    sphere_models = []
    for id_ in range(args.spheres_count):
        sphere_name = f'sphr_{id_:04d}'
        sphere_models.append(make_sphere(parser, sphere_name))
        sphere_body = plant.GetBodyByName('sphere_base', sphere_models[-1])
        position = np.random.uniform(low=args.volume[0, :], high=args.volume[1, :])
        X_WB = RigidTransform(position)
        plant.WeldFrames(
            plant.world_frame(),
            sphere_body.body_frame(),
            X_WB,
        )
        make_sphere_passive_vis(meshcat, sphere_name, meshcat, X_WB)

    R_WL2 = RotationMatrix.MakeYRotation(np.radians(-15.)).multiply(RotationMatrix.MakeXRotation(np.radians(10.)))
    R_WL1 = RotationMatrix.Identity()
    vol = args.volume[1, :] - args.volume[0, :]
    span = vol * 1.5
    t_L_W = args.agent_origin
    X_WL1 = RigidTransform(R_WL1, t_L_W)
    X_WL2 = RigidTransform(R_WL2, t_L_W)

    t_flat, _ = make_figure8_translational_trajectory(args.experiment_duration, X_WL1, span)
    r = make_circular_roto_trajectory(args.experiment_duration, t_flat)
    t, positions = make_figure8_translational_trajectory(args.experiment_duration, X_WL2, span)
    visualize_trajectory_with_cylinders(meshcat, positions)

    tr_trajectory_source = builder.AddSystem(TrajectorySource(t))
    ro_trajectory_source = builder.AddSystem(TrajectorySource(r))

    frustum_mover = builder.AddSystem(FrustumMover(plant, frustum_body))
    recolorer = builder.AddSystem(SphereRecolorer(
        meshcat, plant, sphere_models, frustum_model, camera))

    plant.Finalize()

    builder.Connect(
        tr_trajectory_source.get_output_port(0),
        frustum_mover.GetInputPort('commanded_translation')
    )

    builder.Connect(
        ro_trajectory_source.get_output_port(0),
        frustum_mover.GetInputPort('commanded_orientation')
    )

    builder.Connect(
        plant.get_body_poses_output_port(),
        recolorer.GetInputPort('body_poses')
    )

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    graph_from_dot_data(diagram.GetGraphvizString(max_depth=4))[0].write_png(args.diagram_destination)

    context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    meshcat.StartRecording(set_visualizations_while_recording=True)

    plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    frustum_mover.Initialize(plant_context)
    camera_context = diagram.GetMutableSubsystemContext(camera_system, simulator.get_mutable_context())
    recolorer.initialize(camera_context)

    simulator.AdvanceTo(args.experiment_duration)
    meshcat.PublishRecording()
    open_browser_link(args.replay_browser, meshcat.web_url())


if '__main__' == __name__:
    run_sim(SimArgs().parse_args())
