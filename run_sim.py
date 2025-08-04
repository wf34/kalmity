#!/usr/bin/env python3

from multiprocessing import Process
import re
import sys
import typing
import math
import time
import os
import string
import re
import subprocess
import random
from collections import Counter, deque

import numpy as np
from kalmity import GtObservation, InertialMeasurement, Runtime

from pydot import graph_from_dot_data
from tap import Tap

import matplotlib.pyplot as plt

from pydrake.all import (
    # BasicVector,
    # Accelerometer,
    # Gyroscope,
    RenderEngineVtkParams,
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
    TriggerType,
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
IMU_FREQ = 500.
IMU_PERIOD = 1. / IMU_FREQ
VISUAL_PERIOD = SIM_DELTA_T


def make_sphere_name_from_id(id_: int) -> str:
    return f'sphr_{id_:04d}'


def parse_id_from_sphere_name(name: str) -> int:
    reprog = re.compile(r'^sphr_(\d{4})$')
    res = reprog.match(name)
    if not res:
        raise Exception(f'this sphere does not match: {name}')
    else:
       return int(res.group(1))


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


FRONT = 'front'
ORACLE = 'oracle'
class SimArgs(Tap):
    seed: int = 34
    experiment_duration: float = 5.  # in seconds
    spheres_count: int = 50
    outer_spheres_count: int = 30
    volume: typing.List[typing.List[float]] = [[0, 0, 1], [10, 5, 4]]
    volume_outer: typing.List[typing.List[float]] = [[-10, -5, -1], [20, 10, 6]]
    agent_span: typing.List[typing.List[float]] = [[0, 0, 1.5], [15, 7.5, 8]]
    agent_origin: typing.List[float] = [5, 2.5, 2.5]
    replay_browser: str = 'chromium'
    diagram_destination: str = 'sim_diagram.png'
    with_images: bool = False
    use_real_imu: bool = False
    slam_mode: typing.Literal[ORACLE, FRONT] = ORACLE

    def process_args(self):
        self.volume = np.array(self.volume)
        self.volume_outer = np.array(self.volume_outer)
        self.agent_span = np.array(self.agent_span)

        if self.slam_mode == FRONT:
            raise Exception('yet unsupported')


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


def make_sphere_passive_vis(meshcat_vis, sphere_name, meshcat, X_WB, color=[0.0, 0.8, 0.2, 0.2], R=0.05):
    passive_sphere_name = f'passive_{sphere_name}'
    meshcat_vis.SetObject(
        passive_sphere_name,
        Sphere(R),
        Rgba(*color),
    )
    meshcat_vis.SetTransform(passive_sphere_name, X_WB)


def make_sphere(parser, id_, R=0.1, mass=0.1, color=[0.8, 0.2, 0.2, 1.0]):
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


def make_frustum(parser, id_='slam_agent', color=[0.2, 0.2, 0.8, 1.0], aspect_ratio = 1.777, mass = 1., hfov=None):
    if hfov is None:
        raise Exception('unreachable')

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


def make_camera(scene_graph, camera_name: str, frame_id: FrameId, aspect_ratio: float, hfov: float) -> RgbdSensor:
    if not scene_graph.HasRenderer(camera_name):
        raise Exception('unreachable')

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
    color_camera = ColorRenderCamera(render_camera_core, show_window=False)  # works only with Vtk, backed=GLX
    depth_camera = DepthRenderCamera(render_camera_core, depth_range=DepthRange(6e-3, 65.))
    R_PB = quaternion_from_vectors([0, 0, -1], [0, 0, 1])
    tB_P = [0, 0, 0.119175]  # this needs to be regenerated when frustum is redone
    return RgbdSensor(
        parent_id=frame_id,
        X_PB=RigidTransform(R_PB, tB_P),
        color_camera=color_camera,
        depth_camera=depth_camera,
    )


def add_imu_sensor(args, builder, plant, agents_body, t, r):
    if args.use_real_imu:
        X_B_Imu = RigidTransform(
            RotationMatrix.Identity(),
            np.array([0.0, 0.0, 0.1])
        )

        accelerometer = builder.AddSystem(Accelerometer(
            body=body,
            X_BS=X_B_Imu,
            gravity_vector=np.array([0.0, 0.0, -9.81])
        ))
        gyroscope = builder.AddSystem(Gyroscope(
            body=body,
            X_BS=X_B_Imu
        ))

        builder.Connect(
            plant.get_state_output_port(),
            accelerometer.get_input_port()
        )
        builder.Connect(
            plant.get_state_output_port(),
            gyroscope.get_input_port()
        )
        raise Exception('isnt implemented')

    else:
        return builder.AddSystem(ImuSensor(args.use_real_imu, t=t, r=r))


def make_dummy_translational_trajectory(duration: float, X_WL: RigidTransform):
    num_samples = 100
    times = np.linspace(0, duration, num_samples)

    positions = []
    for t in times:
        positions.append([0, 0, 0, 1])

    positions_local = np.array(positions).T  # 4xN
    positions_world = X_WL.GetAsMatrix4() @ positions_local
    positions_world[:-1, :] /= positions_world[-1, :]
    positions_world = positions_world[:-1, :]

    return PiecewisePolynomial.CubicShapePreserving(
        times, positions_world, zero_end_point_derivatives=True
    )


def make_figure8_translational_trajectory(duration: float, X_WL: RigidTransform, span: np.array):
    scale = span[1, :] - span[0, :]
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


def make_dummy_roto_trajectory(duration: float):
    model_lookvec = np.array([0, 0, -1])
    origin_lookvec = [0, 1, 0]
    r_LB = quaternion_from_vectors(origin_lookvec, model_lookvec) # To local attitude from Obj-file attitude
    steps = 8
    times = np.linspace(0, duration, steps +1).tolist()
    quats = [r_LB] * len(times)
    return PiecewiseQuaternionSlerp(times, quats)


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
        yaws.append(np.arctan2(look_at[1], look_at[0]).item() + np.pi/2)
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

        self.input_port_t = self.DeclareVectorInputPort('translation', 3)
        self.input_port_q = self.DeclareVectorInputPort('orientation', 4)

        self.DeclareContinuousState(1)

    def initialize(self, plant_context):
        self.plant_context = plant_context

    def DoCalcTimeDerivatives(self, context, derivatives):
        if self.plant_context is None:
            derivatives.get_mutable_vector().SetFromVector([0.0])
            return

        t = self.input_port_t.Eval(context)
        q = Quaternion(*self.input_port_q.Eval(context))
        X_WO = RigidTransform(q, t)
        self.plant.SetFreeBodyPose(self.plant_context, self.body, X_WO)
        derivatives.get_mutable_vector().SetFromVector([1.0])


def check_visibility(X_WS: RigidTransform, intrinsics: CameraInfo, p_W: np.array) -> bool:
    width, height = intrinsics.width(), intrinsics.height()
    fx, fy = intrinsics.focal_x(), intrinsics.focal_y()
    cx, cy = intrinsics.center_x(), intrinsics.center_y()
    p_S = X_WS.inverse() @ p_W
    if p_S[2] <= 0.:
        return False, 'behind', (None, None)  # is behind

    K = intrinsics.intrinsic_matrix()
    p_px = K @ p_S
    p_px /= p_px[2]
    u, v = p_px[:2]

    status = 0. <= u and u <= width and \
             0. <= v and v <= height
    return status, 'visible' if status else 'fov', (u,v)


class VisualObserver(LeafSystem):
    def __init__(self, args, plant, sphere_names, frustum_model, camera):
        LeafSystem.__init__(self)
        self.with_images = args.with_images
        self.slam_mode = args.slam_mode
        self.plant = plant
        self.delta_t = self.plant.time_step()
        self.sphere_names = sphere_names
        self.frustum_model = frustum_model
        self.agents_camera = camera
        self.observations = []
        self.tick_counter = 0

        if ORACLE != self.slam_mode:
            raise Exception(f'{self.slam_mode} isn\'t yet supported')

        self.DeclareAbstractInputPort(
            'body_poses',
            AbstractValue.Make([RigidTransform()])
        )

        self.DeclareAbstractOutputPort(
            'gt_visual_observations',
            lambda: AbstractValue.Make([GtObservation()]),
            self.set_observations,
        )

        self.DeclarePeriodicPublishEvent(
            period_sec=self.delta_t,
            offset_sec=self.delta_t / 2,
            publish=self.evaluate_visibility)


    def initialize(self, camera_context):
        self.camera_context = camera_context
        self.color_camera = self.agents_camera.GetColorRenderCamera(self.camera_context)
        core_camera = self.color_camera.core()
        self.camera_info = core_camera.intrinsics()
        self.X_PB = self.agents_camera.GetX_PB(self.camera_context)
        self.X_BS = core_camera.sensor_pose_in_camera_body()
        self.X_PS = self.X_PB @ self.X_BS

    def set_observations(self, context, output):
        output.set_value(self.observations)

    def evaluate_visibility(self, context):
        if not hasattr(self, 'X_PS'):
            raise Exception(f'{self.__class__.__name__} was not initialized')

        now = context.get_time()
        delta = self.plant.time_step()
        poses = self.GetInputPort('body_poses').Eval(context)
        agents_ind = self.plant.GetBodyByName('base_link', self.frustum_model).index()
        X_WA = poses[agents_ind]
        X_WS = X_WA @ self.X_PS
        #print(X_WS.rotation().ToRollPitchYaw(), X_WS.translation())

        status_counts = Counter()
        self.observations = []
        for sphere_name in self.sphere_names:
            sphere_id = parse_id_from_sphere_name(sphere_name)
            model_index = self.plant.GetModelInstanceByName(sphere_name)
            ind = self.plant.GetBodyByName('sphere_base', model_index).index()
            p_WSphere = poses[ind].translation()

            vis_status, step, uv = check_visibility(X_WS, self.camera_info, p_WSphere)
            status_counts[step] += 1
            if vis_status:
                self.observations.append(GtObservation(now, sphere_id, uv))

        ordered_status_counts = list(map(lambda x: (x, status_counts[x]), sorted(status_counts.keys())))
        print(f'{now:.4f}, {ordered_status_counts}')
        self.query_image_and_store_on_drive(now)

    def query_image_and_store_on_drive(self, now: float):
        if not self.with_images:
            return

        color_image = self.agents_camera.color_image_output_port().Eval(self.camera_context)
        color_array = color_image.data[:, :, :3]
        fig = plt.figure(figsize=(10, 5.6274))
        ax = fig.subplots(nrows=1, ncols=1)
        ax.imshow(color_array)
        ax.set_title(f'{now:.4f}')
        fig.savefig(f'frames/{self.tick_counter:07d}.png')
        self.tick_counter += 1
        plt.close(fig)


class ImuSensor(LeafSystem):
    def __init__(self, use_real_imu: bool, gyroscope=None, accelerometer=None, t=None, r=None):
        LeafSystem.__init__(self)
        if use_real_imu:
            assert gyroscope is not None
            assert accelerometer is not None

            self.gyroscope = gyroscope
            self.accelerometer = accelerometer

            self.input_accel_port = self.DeclareVectorInputPort("acceleration", BasicVector(3))
            self.input_angular_port = self.DeclareVectorInputPort("angular_rate", BasicVector(3))
        else:
            assert t is not None
            assert r is not None
            self.acceleration = t.MakeDerivative(derivative_order=2)
            self.angular_rate = r.MakeDerivative()

        self.DeclareAbstractOutputPort(
            'inertial_measurement',
            lambda: AbstractValue.Make(InertialMeasurement),
            self.make_inertial_measurement,
        )

    def make_inertial_measurement(self, context, output):
        now = context.get_time()
        # print(f'imu {now:.3f}, {self.input_accel_port.HasValue(context)} {self.input_angular_port.HasValue(context)}')
        a = self.acceleration.value(now).ravel().tolist()
        a = (*a,)
        w = self.angular_rate.value(now).ravel().tolist()
        w = (*w,)
        output.set_value(InertialMeasurement(time = now, gyro=w, accel=a))

        #if not self.input_accel_port.HasValue(context) or:
        #    return

        # self.input_accel_port.Eval(context)


class FilterBasedNavigation(LeafSystem):
    def __init__(self, args):
        LeafSystem.__init__(self)
        self.pose_estimate = None, None
        self.pose_buffer = deque(maxlen=100)

        self.filter_runtime = Runtime('xx')
        self.filter_runtime.set_pose_callback(self.store_pose_estimate)

        self.vis_obs_port = self.DeclareAbstractInputPort(
            "gt_visual_observations",
            model_value=AbstractValue.Make([GtObservation()])
        )

        self.input_imu_port = self.DeclareAbstractInputPort(
            "inertial_measurement",
            model_value=AbstractValue.Make([InertialMeasurement()])
        )

        self.DeclareAbstractOutputPort(
            "pose_estimate",
            lambda: AbstractValue.Make(RigidTransform()),
            self.set_pose_estimate,
        )

        print(f'`FilterBasedNavigation` imu period is {IMU_PERIOD:.3f}, vis period: {VISUAL_PERIOD:.3f}')
        self.DeclarePeriodicPublishEvent(
            period_sec=IMU_PERIOD,
            offset_sec=0.0,
            publish=self.fetch_solutions
        )

        self.DeclarePeriodicPublishEvent(
            period_sec=VISUAL_PERIOD,
            offset_sec=0.0,
            publish=self.communicate_observations
        )

    def set_pose_estimate(self, context, output):
        last_pose_time, X_WAest = self.pose_estimate
        if last_pose_time is None or X_WAest is None:
            return
        output.set_value(X_WAest)

    def store_pose_estimate(self, timestamp_and_slam_pose_estimate: typing.List[float]):
        assert 8 == len(timestamp_and_slam_pose_estimate)
        timestamp = timestamp_and_slam_pose_estimate[0]
        slam_pose_estimate = timestamp_and_slam_pose_estimate[1:]
        q = slam_pose_estimate[:4]
        qq = Quaternion(w=q[0], x=q[1], y=q[2], z=q[3])
        t = slam_pose_estimate[-3:]
        self.pose_buffer.append((timestamp, RigidTransform(qq, t)))

    def fetch_solutions(self, context):
        # pass from the pose buffer to `self.pose_estimate`
        now = context.get_time()

        if  self.input_imu_port.HasValue(context):
            imu_mes = self.input_imu_port.Eval(context)
            if imu_mes is not None:
                self.filter_runtime.add_inertial_measurement(imu_mes)

        arg_min = None
        min_time_offset = None

        for i in range(len(self.pose_buffer)):
            ts, pose = self.pose_buffer[i]
            offset = math.fabs(ts - now)
            if min_time_offset is None or offset < min_time_offset:
                min_time_offset = offset
                arg_min = i

        if arg_min is not None:
            self.pose_estimate = self.pose_buffer[arg_min]

    def communicate_observations(self, context):
        now = context.get_time()
        if not self.vis_obs_port.HasValue(context):
            return
        observations = self.vis_obs_port.Eval(context)
        if observations is None:
            return

        for obs in observations:
            assert obs.ts < now + 1.e-6, f'observation={obs.ts:.6f} now={now:.6f}'
            self.filter_runtime.add_observation(obs)


class SphereRecolorer(LeafSystem):
    def __init__(self, meshcat, plant, sphere_names):
        LeafSystem.__init__(self)
        self.meshcat = meshcat
        self.plant = plant
        self.full_sphere_names = set(sphere_names)

        self.input_obs_port = self.DeclareAbstractInputPort(
            "gt_visual_observations",
            model_value=AbstractValue.Make([GtObservation()])
        )
        self.DeclarePerStepUnrestrictedUpdateEvent(self.calc_recoloring)

    def change_vis_property(self, time_now, status, model_name):
        path = f'visualizer/{model_name}/sphere_base/{model_name}/sphere_base_vis'
        self.meshcat.SetProperty(path, "visible", status, time_in_recording=time_now)

    def calc_recoloring(self, context, state):
        if not self.input_obs_port.HasValue(context):
            return
        observations = self.input_obs_port.Eval(context)
        if observations is None:
            return

        now = context.get_time()
        visible_spheres = set([])

        for obs in observations:
            assert obs.ts < now + 1.e-6, f'observation={obs.ts:.6f} now={now:.6f}'
            model_name = make_sphere_name_from_id(obs.id)
            visible_spheres.add(model_name)
            self.change_vis_property(now, True, model_name)

        for unseen_model_name in self.full_sphere_names-visible_spheres:
            self.change_vis_property(now, False, unseen_model_name)


class RigidTransformDemuxer(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.input_port = self.DeclareAbstractInputPort("pose_estimate", AbstractValue.Make(RigidTransform()))
        self.DeclareVectorOutputPort("estimate_quaternion", 4, self.calc_quaternion)
        self.DeclareVectorOutputPort("estimate_translation", 3, self.calc_translation)

    def calc_quaternion(self, context, output):
        if not self.input_port.HasValue(context):
            return
        X = self.input_port.Eval(context)
        if X is None:
            return
        q = X.rotation().ToQuaternion()
        output.SetFromVector([q.w(), q.x(), q.y(), q.z()])

    def calc_translation(self, context, output):
        if not self.input_port.HasValue(context):
            return
        X = self.input_port.Eval(context)
        output.SetFromVector(X.translation())


def get_outer_position(args) -> np.array:
    is_outer = False
    while not is_outer:
        tentative_pos = np.random.uniform(low=args.volume_outer[0, :], high=args.volume_outer[1, :])
        if np.all(args.volume[0, :] <= tentative_pos) and \
           np.all(tentative_pos <= args.volume[1, :]):
            continue
        else:
           is_outer = True
    return tentative_pos


def run_sim(args: SimArgs):
    np.random.seed(args.seed)

    meshcat = StartMeshcat()
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=SIM_DELTA_T)

    render_params = RenderEngineVtkParams()
    render_params.backend = 'GLX'
    render_engine = MakeRenderEngineVtk(render_params)

    camera_name = 'agents_cam'
    scene_graph.AddRenderer(camera_name, render_engine)
    parser = Parser(plant, scene_graph)

    aspect_ratio = 1.777
    hfov = np.radians(80.)

    gt_frustum_model = make_frustum(parser, id_='slam_agent', aspect_ratio=aspect_ratio, hfov=hfov, color=[0.2, 0.2, 0.8, 0.8], mass=1.)
    dummy_frustum_model = make_frustum(parser, id_='slam_est_agent', aspect_ratio=aspect_ratio, hfov=hfov, color=[0.8, 0.2, 0.2, 1.0], mass=1.e-3)

    gt_frustum_body = plant.GetBodyByName('base_link', gt_frustum_model)
    dummy_frustum_body = plant.GetBodyByName('base_link', dummy_frustum_model)

    agents_body_frame_id = plant.GetBodyFrameIdOrThrow(gt_frustum_body.index())
    camera = make_camera(scene_graph, camera_name, agents_body_frame_id, aspect_ratio, hfov)

    camera_system = builder.AddSystem(camera)
    builder.Connect(
        scene_graph.get_query_output_port(),
        camera.query_object_input_port()
    )

    plant.set_gravity_enabled(gt_frustum_model, False)
    plant.set_gravity_enabled(dummy_frustum_model, False)

    sphere_names = []
    for id_ in range(args.spheres_count + args.outer_spheres_count):
        sphere_names.append(make_sphere_name_from_id(id_))
        sphere_name = sphere_names[-1]
        sphere_model =make_sphere(parser, sphere_name)
        sphere_body = plant.GetBodyByName('sphere_base', sphere_model)
        if id_ < args.spheres_count:
            position = np.random.uniform(low=args.volume[0, :], high=args.volume[1, :])
        else:
            position = get_outer_position(args)
        X_WB = RigidTransform(position)
        plant.WeldFrames(
            plant.world_frame(),
            sphere_body.body_frame(),
            X_WB,
        )
        make_sphere_passive_vis(meshcat, sphere_name, meshcat, X_WB)

    R_WL2 = RotationMatrix.MakeYRotation(np.radians(-15.)).multiply(RotationMatrix.MakeXRotation(np.radians(10.)))
    R_WL1 = RotationMatrix.Identity()
    t_L_W = args.agent_origin
    X_WL1 = RigidTransform(R_WL1, t_L_W)
    X_WL2 = RigidTransform(R_WL2, t_L_W)

    t_flat, _ = make_figure8_translational_trajectory(args.experiment_duration, X_WL1, args.agent_span)
    r = make_circular_roto_trajectory(args.experiment_duration, t_flat)
    t, positions = make_figure8_translational_trajectory(args.experiment_duration, X_WL2, args.agent_span)
    visualize_trajectory_with_cylinders(meshcat, positions)


    imu_system = add_imu_sensor(args, builder, plant, gt_frustum_body, t, r)

    tr_trajectory_source = builder.AddSystem(TrajectorySource(t))
    ro_trajectory_source = builder.AddSystem(TrajectorySource(r))

    gt_frustum_mover = builder.AddSystem(FrustumMover(plant, gt_frustum_body))

    visual_observer = builder.AddSystem(VisualObserver(
        args, plant, sphere_names, gt_frustum_model, camera))

    navigation_system = builder.AddSystem(FilterBasedNavigation(args))
    recolorer = builder.AddSystem(SphereRecolorer(meshcat, plant, sphere_names))
    demux = builder.AddSystem(RigidTransformDemuxer())
    dummy_frustum_mover = builder.AddSystem(FrustumMover(plant, dummy_frustum_body))

    plant.Finalize()

    builder.Connect(
        tr_trajectory_source.get_output_port(0),
        gt_frustum_mover.GetInputPort('translation'),
    )

    builder.Connect(
        ro_trajectory_source.get_output_port(0),
        gt_frustum_mover.GetInputPort('orientation'),
    )

    builder.Connect(
        plant.get_body_poses_output_port(),
        visual_observer.GetInputPort('body_poses'),
    )

    builder.Connect(
        visual_observer.GetOutputPort('gt_visual_observations'),
        navigation_system.GetInputPort('gt_visual_observations'),
    )
    builder.Connect(
        visual_observer.GetOutputPort('gt_visual_observations'),
        recolorer.GetInputPort('gt_visual_observations'),
    )
    builder.Connect(
        imu_system.GetOutputPort('inertial_measurement'),
        navigation_system.GetInputPort('inertial_measurement'),
    )

    builder.Connect(
        navigation_system.GetOutputPort('pose_estimate'),
        demux.GetInputPort('pose_estimate'),
    )

    builder.Connect(
        demux.GetOutputPort('estimate_quaternion'),
        dummy_frustum_mover.GetInputPort('orientation'),
    )
    builder.Connect(
        demux.GetOutputPort('estimate_translation'),
        dummy_frustum_mover.GetInputPort('translation'),
    )

    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    graph_from_dot_data(diagram.GetGraphvizString(max_depth=4))[0].write_png(args.diagram_destination)

    context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, context)
    simulator.Initialize()
    meshcat.StartRecording(set_visualizations_while_recording=True)

    plant_context = diagram.GetMutableSubsystemContext(plant, simulator.get_mutable_context())
    gt_frustum_mover.initialize(plant_context)
    dummy_frustum_mover.initialize(plant_context)
    camera_context = diagram.GetMutableSubsystemContext(camera_system, simulator.get_mutable_context())
    visual_observer.initialize(camera_context)

    simulator.AdvanceTo(args.experiment_duration)
    meshcat.PublishRecording()
    open_browser_link(args.replay_browser, meshcat.web_url())


if '__main__' == __name__:
    run_sim(SimArgs().parse_args())
