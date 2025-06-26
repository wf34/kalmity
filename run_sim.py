#!/usr/bin/env python3

from multiprocessing import Process
import typing
import time
import os
import string
import re
import subprocess
import random
from tap import Tap

from pydrake.all import (
    StartMeshcat,
)

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
    replay_browser: str = 'chromium'


def run_sim(args: SimArgs):
    meshcat = StartMeshcat()
    meshcat.PublishRecording()
    open_browser_link(args.replay_browser, meshcat.web_url())

if '__main__' == __name__:
    run_sim(SimArgs().parse_args())
