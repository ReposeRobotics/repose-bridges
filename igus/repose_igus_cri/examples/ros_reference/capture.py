#!/usr/bin/env python3
"""Capture exact-input FK references from an isolated ROS robot_state_publisher.

Read the active robot_description and one joint-state observation. Publish test
angles only on unique validation topics, never the hardware joint-state topic.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess
import tempfile
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import yaml


def wait_until(node, predicate, seconds, description):
    deadline = time.monotonic() + seconds
    while not predicate():
        if time.monotonic() >= deadline:
            raise RuntimeError(f"Timed out waiting for {description}")
        rclpy.spin_once(node, timeout_sec=0.05)


def chain_joints(xml, base, tip):
    root = ET.fromstring(xml)
    parents = {}
    for joint in root.findall("joint"):
        child = joint.find("child").get("link")
        if child in parents:
            raise ValueError("URDF contains duplicate child links")
        parents[child] = joint
    chain = []
    link = tip
    visited = set()
    while link != base:
        if link in visited or link not in parents:
            raise ValueError(f"No unambiguous URDF path from {base} to {tip}")
        visited.add(link)
        joint = parents[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    return list(reversed(chain))


def transform_matrix(transform):
    q = transform.rotation
    t = transform.translation
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def compose_tf(edges, base, tip):
    matrices = []
    link = tip
    visited = set()
    while link != base:
        if link in visited or link not in edges:
            return None
        visited.add(link)
        parent, matrix = edges[link]
        matrices.append(matrix)
        link = parent
    result = np.eye(4)
    for matrix in reversed(matrices):
        result = result @ matrix
    return result


def capture(args):
    rclpy.init()
    node = rclpy.create_node("repose_fk_capture_" + uuid.uuid4().hex[:8])
    process = None
    try:
        service = args.description_node.rstrip("/") + "/get_parameters"
        client = node.create_client(GetParameters, service)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f"Unavailable parameter service: {service}")
        future = client.call_async(GetParameters.Request(names=["robot_description"]))
        wait_until(node, future.done, 5, "robot_description parameter")
        response = future.result()
        if response is None or len(response.values) != 1 or not response.values[0].string_value:
            raise RuntimeError("robot_description parameter is empty")
        xml = response.values[0].string_value
        chain = chain_joints(xml, args.base, args.tip)
        movable = [j for j in chain if j.get("type") != "fixed"]
        names = [j.get("name") for j in movable]
        if names != [f"joint{i}" for i in range(1, 7)]:
            raise ValueError(f"Expected six ReBeL joints joint1…joint6, got {names}")
        observed = []

        def on_joints(message):
            values = dict(zip(message.name, message.position))
            if all(name in values and math.isfinite(values[name]) for name in names):
                observed[:] = [values[name] for name in names]

        subscription = node.create_subscription(JointState, args.joint_topic, on_joints, 10)
        wait_until(node, lambda: bool(observed), 5, "complete measured ROS joint state")
        node.destroy_subscription(subscription)
        samples = [{"name": "observed_ros", "joints": observed.copy()}]
        samples.append({"name": "zero", "joints": [0.0] * 6})
        limits = []
        for joint in movable:
            limit = joint.find("limit")
            limits.append((-math.pi, math.pi) if joint.get("type") == "continuous"
                          else (float(limit.get("lower")), float(limit.get("upper"))))
        for i, (lo, hi) in enumerate(limits):
            for direction in [-1, 1]:
                values = [0.0] * 6
                values[i] = max(lo, min(hi, direction * 0.4))
                samples.append({"name": f"joint{i + 1}_{direction:+d}", "joints": values})
        rng = random.Random(729)
        for i in range(16):
            samples.append({"name": f"synthetic_{i:02}",
                            "joints": [rng.uniform(lo * 0.8, hi * 0.8) for lo, hi in limits]})
        # No robot motion: these configurations exist only on validation topics.
        namespace = "/repose_fk_reference_" + uuid.uuid4().hex[:10]
        dynamic = {}
        static = {}
        requested_stamp = [None]
        seen_stamps = set()

        def receive_tf(message, fixed=False):
            for transform in message.transforms:
                stamp = (transform.header.stamp.sec, transform.header.stamp.nanosec)
                if not fixed:
                    seen_stamps.add(stamp)
                if fixed or stamp == requested_stamp[0]:
                    target = static if fixed else dynamic
                    target[transform.child_frame_id] = (
                        transform.header.frame_id, transform_matrix(transform.transform))

        node.create_subscription(TFMessage, namespace + "/tf", receive_tf, 10)
        node.create_subscription(TFMessage, namespace + "/tf_static",
                                 lambda msg: receive_tf(msg, True),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher = node.create_publisher(JointState, namespace + "/joint_states", 10)
        executable = Path(args.publisher)
        if not executable.is_file():
            raise RuntimeError(f"Missing robot_state_publisher executable: {executable}")
        with tempfile.TemporaryDirectory(prefix="repose-ros-reference-") as directory:
            parameters = Path(directory) / "params.yaml"
            parameters.write_text(yaml.safe_dump({"/**": {"ros__parameters": {
                "robot_description": xml, "publish_frequency": 1000.0,
                "use_sim_time": False}}}))
            log_path = Path(directory) / "publisher.log"
            with log_path.open("w+") as log:
                process = subprocess.Popen([
                    str(executable), "--ros-args", "--params-file", str(parameters),
                    "-r", "__ns:=" + namespace,
                    "-r", "/tf:=" + namespace + "/tf",
                    "-r", "/tf_static:=" + namespace + "/tf_static",
                    "-r", "/joint_states:=" + namespace + "/joint_states"],
                    stdout=log, stderr=subprocess.STDOUT)
                wait_until(node, lambda: publisher.get_subscription_count() > 0, 8,
                           "isolated reference subscriber")
                for sample in samples:
                    dynamic.clear()
                    message = JointState()
                    message.name = names
                    message.position = sample["joints"]

                    def publish_reference():
                        message.header.stamp = node.get_clock().now().to_msg()
                        requested_stamp[0] = (message.header.stamp.sec, message.header.stamp.nanosec)
                        dynamic.clear()
                        publisher.publish(message)

                    timer = node.create_timer(0.1, publish_reference)
                    publish_reference()
                    try:
                        wait_until(node, lambda: compose_tf({**static, **dynamic}, args.base, args.tip)
                                   is not None, 3, "timestamp-matched reference transforms")
                    except RuntimeError:
                        log.seek(0)
                        raise RuntimeError(f"Reference publisher failed: dynamic={list(dynamic)}, static={list(static)}, requested={requested_stamp}, seen={seen_stamps}: " + log.read()) from None
                    node.destroy_timer(timer)
                    sample["transform"] = compose_tf({**static, **dynamic}, args.base, args.tip).tolist()
                    sample["stamp"] = list(requested_stamp[0])
                process.terminate()
                process.wait(timeout=5)
                process = None
        digest = hashlib.sha256(xml.encode()).hexdigest()
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output / "robot.urdf").write_text(xml)
        # Mesh collision geometry is unsupported by Repose's startup importer.
        # Remove collision elements only; joint transforms and limits are untouched.
        root = ET.fromstring(xml)
        original_joints = [ET.tostring(joint) for joint in root.findall("joint")]
        for link in root.findall("link"):
            for collision in link.findall("collision"):
                link.remove(collision)
        if original_joints != [ET.tostring(joint) for joint in root.findall("joint")]:
            raise RuntimeError("Kinematic joints changed while removing collision geometry")
        kinematic_xml = ET.tostring(root, encoding="unicode")
        (args.output / "robot.kinematics.urdf").write_text(kinematic_xml)
        fixture = {"schema": 1, "source": service, "urdf_sha256": digest,
                   "reference": "ROS robot_state_publisher",
                   "capture_utc": datetime.now(timezone.utc).isoformat(),
                   "publisher_sha256": hashlib.sha256(executable.read_bytes()).hexdigest(),
                   "kinematic_urdf_sha256": hashlib.sha256(kinematic_xml.encode()).hexdigest(),
                   "base_frame": args.base,
                   "tip_frame": args.tip, "joint_names": names, "samples": samples}
        (args.output / "reference.json").write_text(json.dumps(fixture, indent=2) + "\n")
        print(f"Captured {len(samples)} references: {args.base} → {args.tip}")
        print(f"URDF SHA-256: {digest}")
        print(args.output / "reference.json")
    finally:
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description-node", default="/robot_state_publisher")
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--base", default="base_link")
    parser.add_argument("--tip", default="link6")
    parser.add_argument("--publisher", default="/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher")
    parser.add_argument("--output", type=Path, required=True)
    capture(parser.parse_args())


if __name__ == "__main__":
    main()
