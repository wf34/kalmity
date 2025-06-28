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
from tap import Tap
import numpy as np

from pydrake.all import (
    StartMeshcat,
    DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    Parser,
    RigidTransform,
    MeshcatVisualizer,
    Simulator,
)

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
                print(f"Pattern matched: {line}")
                with open(comm_filename, 'w') as the_file:
                    the_file.write('1')

def detachify(func):
    """Decorate a function so that its calls are async in a detached process."""

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
        print('.')
        if os.path.exists(comm_filename):
            with open(comm_filename, 'r') as the_file:
                status = int(the_file.read().strip())
                if status == 1:
                    load_finished = True
        time.sleep(1.)


class SimArgs(Tap):
    spheres_count: int = 100
    volume: typing.List[typing.List[float]] = [[0, 0, 0], [10, 5, 5]]
    replay_browser: str = 'chromium'
    seed: int = 34


def make_sphere(parser, id_, R=0.01, mass=0.1, color=[0.8, 0.2, 0.2, 1.0]):
    inertia = .4 * mass * R ** 2
    sphere_str = None
    with open(os.path.join(os.path.dirname(sys.argv[0]), 'sphere_sdf.template'), 'r') as the_file:
        sphere_template = the_file.read()
        sphere_str = sphere_template \
            .replace('{{ name }}', id_) \
            .replace('{{ mass }}', str(mass)) \
            .replace('{{ inertia }}', str(inertia)) \
            .replace('{{ R }}', str(R)) \
            .replace('{{ rgba }}', ' '.join(map(str, color))) \

    if sphere_str is None:
        raise Exception('unreachable')

    return parser.AddModelsFromString(sphere_str, "sdf")[0]


def run_sim(args: SimArgs):
    np.random.seed(args.seed)


    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=SIM_DELTA_T)
    parser = Parser(plant, scene_graph)

    for id_ in range(args.spheres_count):
        sphere_model = make_sphere(parser, f'sphr_{id_:04d}')
        sphere_body = plant.GetBodyByName("sphere_base", sphere_model)
        position = np.random.uniform(low=args.volume[0], high=args.volume[1])
        plant.WeldFrames(
            plant.world_frame(),
            sphere_body.body_frame(),
            RigidTransform(position)
        )

    plant.Finalize()

    meshcat = StartMeshcat()
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, context)
    simulator.AdvanceTo(3.0)
    open_browser_link(args.replay_browser, meshcat.web_url())


if '__main__' == __name__:
    run_sim(SimArgs().parse_args())
