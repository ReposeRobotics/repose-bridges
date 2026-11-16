#!/usr/bin/env python3
"""MuJoCo transport for the SO-ARM robot-app example.

Serves the five arm joints of the bundled SO-ARM100 URDF (named joint1..joint5).
The Copper bridge must be `repose_mujoco_sim::CuMujocoSimBridge<5>` with the same
joint count. Protocol: line-delimited JSON over TCP, one command in / one state
out (lockstep), plus an initial state line on connect. See the repose-mujoco-sim
crate docs for the wire format.

Run headless (default) or with a passive MuJoCo viewer:

    python sim/sidecar.py                 # headless on 127.0.0.1:55091
    python sim/sidecar.py --viewer        # opens a MuJoCo window (needs a display)

Requires: python -m pip install mujoco numpy
"""
import argparse
import json
import os
import socket
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

# Position-control PD gains and actuator force cap (Nm). The SO-ARM100 uses
# Feetech STS3215 servos (~3 Nm stall); gravity compensation (qfrc_bias) carries
# the static load so the PD term only corrects tracking error.
KP, KD, FORCE = 30.0, 3.0, 4.0
SUBSTEPS = 10  # 10 * 0.002 s = 0.02 s of physics per command (50 Hz lockstep)
# Reflected rotor inertia of the geared servos (kg m^2), absent from the URDF.
# The links weigh grams; without it the PD loop is unstable at this timestep.
ARMATURE = 0.02


def load_model(path, viewer):
    tree = ET.parse(path)
    if not viewer:
        # Headless: Copper owns collision checking; drop all MuJoCo geometry.
        for link in tree.getroot().iter('link'):
            for child in list(link):
                if child.tag in ('visual', 'collision'):
                    link.remove(child)
    with tempfile.NamedTemporaryFile(suffix='.urdf') as source:
        tree.write(source.name)
        spec = mujoco.MjSpec.from_file(source.name)
    spec.option.timestep = 0.002
    joints = list(spec.joints)
    expected = [f'joint{i}' for i in range(1, len(joints) + 1)]
    if not joints or [joint.name for joint in joints] != expected:
        raise ValueError('URDF must contain joint1 through jointN in order')
    for joint in joints:
        joint.armature = ARMATURE
        spec.add_actuator(name='motor_' + joint.name,
                          type=mujoco.mjtActuator.mjACTUATOR_MOTOR,
                          trntype=mujoco.mjtTrn.mjTRN_JOINT, target=joint.name,
                          gear=[1, 0, 0, 0, 0, 0], forcelimited=True,
                          forcerange=[-FORCE, FORCE])
    model = spec.compile()
    if viewer:
        # Keep the visual geometry for display but disable contact physics:
        # this is a position-controlled demo, not a contact simulation.
        model.geom_contype[:] = 0
        model.geom_conaffinity[:] = 0
    return model


def send_state(conn, model, data):
    n = model.nu
    message = {'type': 'state', 'n': n, 'q': data.qpos[:n].tolist(),
               'qd': data.qvel[:n].tolist(), 'time': float(data.time)}
    conn.sendall((json.dumps(message, allow_nan=False) + '\n').encode())


def main():
    parser = argparse.ArgumentParser()
    default_urdf = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'so_arm100.urdf')
    parser.add_argument('--urdf', default=default_urdf, help='SO-ARM URDF (joint1..jointN)')
    parser.add_argument('--port', type=int, default=55091)
    parser.add_argument('--viewer', action='store_true', help='open a passive MuJoCo viewer')
    args = parser.parse_args()
    model = load_model(args.urdf, args.viewer)
    data = mujoco.MjData(model)
    viewer = None
    if args.viewer:
        import mujoco.viewer as mujoco_viewer
        viewer = mujoco_viewer.launch_passive(model, data)
    with socket.socket() as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('127.0.0.1', args.port))
        server.listen(1)
        print(f'MuJoCo ready on 127.0.0.1:{args.port}', flush=True)
        while True:
            conn, _ = server.accept()
            try:
                serve(conn, model, data, viewer)
            except (json.JSONDecodeError, UnicodeDecodeError, KeyError) as error:
                # Not the Copper bridge (e.g. a local HTTP port probe): drop it.
                print(f'ignoring non-protocol client: {error}', flush=True)
            except ConnectionError:
                print('bridge disconnected', flush=True)


def serve(conn, model, data, viewer):
    with conn, conn.makefile('rb') as reader:
        n = model.nu
        send_state(conn, model, data)
        for line in reader:
            message = json.loads(line)
            q = np.asarray(message['q'], dtype=float)
            qd = np.asarray(message.get('qd', [0] * n), dtype=float)
            problem = None
            if message.get('n', n) != n or q.shape != (n,) or qd.shape != (n,):
                problem = (f'joint count mismatch: sidecar serves {n} joints, '
                           f'command has n={message.get("n")}, '
                           f'len(q)={q.size}, len(qd)={qd.size}')
            elif not np.isfinite(q).all() or not np.isfinite(qd).all():
                problem = 'invalid joint command: non-finite value'
            if problem is not None:
                # Keep the request/response lockstep: answer with an error line.
                print(problem, flush=True)
                conn.sendall((json.dumps({'type': 'error', 'message': problem}) + '\n').encode())
                continue
            for _ in range(SUBSTEPS):
                mujoco.mj_forward(model, data)
                data.ctrl[:] = np.clip(KP * (q - data.qpos[:n]) + KD * (qd - data.qvel[:n])
                                       + data.qfrc_bias[:n], -FORCE, FORCE)
                mujoco.mj_step(model, data)
            if viewer is not None:
                viewer.sync()
            send_state(conn, model, data)


if __name__ == '__main__':
    main()
