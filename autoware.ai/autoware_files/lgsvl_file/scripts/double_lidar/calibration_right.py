#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#


import os
import math
import lgsvl
import numpy as np
from pyquaternion import Quaternion
from lgsvl.utils import transform_to_matrix


class AutowareCalibration:
    def __init__(self, agent_name="DoubleLiDAR (Autoware)"):
        self.agent_name = agent_name
        self.scene_name = "BorregasAve"
        self.sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)
        if self.sim.current_scene != self.scene_name:
            self.sim.load(self.scene_name)
        self.sim.reset()
        self.ego = self.sim.add_agent(self.agent_name, lgsvl.AgentType.EGO)
        self._load_sensors()
    
    def _load_sensors(self):
        for sensor in self.ego.get_sensors():
            print(sensor.name)
            if sensor.name == "Main Camera":
                self.sensor_camera = sensor
            if sensor.name == "LidarRight":
                self.sensor_lidar = sensor
    
    def calibrate(self):
        extrinsic_mat = self._get_extrinsic(self.sensor_lidar, self.sensor_camera)
        intrinsic_mat = self._get_intrinsic(self.sensor_camera)
        self._mat_to_str(extrinsic_mat, intrinsic_mat)
    
    def _mat_to_str(self, extrinsic_mat, intrinsic_mat):
        ext_str = ', '.join(['{:.12e}'.format(e) for e in extrinsic_mat.flatten()])
        int_str = ', '.join(['{:.12e}'.format(e) for e in intrinsic_mat.flatten()])
        yaml = '%YAML:1.0\n' \
            + '---\n' \
            + 'CameraExtrinsicMat: !!opencv-matrix\n' \
            + '   rows: 4\n' \
            + '   cols: 4\n' \
            + '   dt: d\n' \
            + '   data: [{}]\n'.format(ext_str) \
            + 'CameraMat: !!opencv-matrix\n' \
            + '   rows: 3\n' \
            + '   cols: 3\n' \
            + '   dt: d\n' \
            + '   data: [{}]\n'.format(int_str) \
            + 'DistCoeff: !!opencv-matrix\n' \
            + '   rows: 1\n' \
            + '   cols: 5\n' \
            + '   dt: d\n' \
            + '   data: [0., 0., 0., 0., 0.]\n' \
            + 'ImageSize: [{}, {}]\n'.format(self.im_width, self.im_height) \
            + 'ReprojectionError: 0.'
        
        # with open('{}.yaml'.format(self.agent_name), 'w+') as f:
        #     f.write(yaml)
        with open('right_calibration.yaml', 'w+') as f:
            f.write(yaml)

    def _get_extrinsic(self, lidar, camera):
        lid_to_ego = self._get_tf(lidar.transform)
        ego_to_lid = np.linalg.inv(lid_to_ego)
        cam_to_ego = self._get_tf(camera.transform)
        cam_to_lid = np.dot(ego_to_lid, cam_to_ego)

        quat1 = Quaternion(axis=(1, 0, 0), angle=-np.pi / 2)
        quat2 = Quaternion(axis=(0, 1, 0), angle=np.pi / 2)
        quat = quat1 * quat2
        tf = np.dot(cam_to_lid, quat.transformation_matrix)

        tmp = tf[2][3]
        tf[2][3] = tf[1][3]
        tf[1][3] = -tf[0][3]
        tf[0][3] = tmp

        return tf

    def _get_intrinsic(self, camera):
        self.im_width = camera.width
        self.im_height = camera.height
        aspect_ratio = self.im_width / self.im_height
        vertical_fov = camera.fov
        horizon_fov = 2 * math.degrees(math.atan(math.tan(math.radians(vertical_fov) / 2) * aspect_ratio))
        fx = self.im_width / (2 * math.tan(0.5 * math.radians(horizon_fov)))
        fy = self.im_height / (2 * math.tan(0.5 * math.radians(vertical_fov)))
        cx = self.im_width / 2
        cy = self.im_height / 2

        proj_mat = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1],
        ])

        return proj_mat
    
    def _get_tf(self, tr):
        tf = np.eye(4)

        px = tr.position.x
        py = tr.position.y
        pz = tr.position.z

        ax = tr.rotation.x * math.pi / 180.0
        ay = tr.rotation.y * math.pi / 180.0
        az = tr.rotation.z * math.pi / 180.0

        sx, cx = math.sin(ax), math.cos(ax)
        sy, cy = math.sin(ay), math.cos(ay)
        sz, cz = math.sin(az), math.cos(az)

        tf[:3, :3] = np.array([
            [sx * sy * sz + cy * cz, cx * sz, sx * cy * sz - sy * cz],
            [sx * sy * cz - cy * sz, cx * cz, sy * sz + sx * cy * cz],
            [cx * sy, -sx, cx * cy],
        ])
        
        tf[:3, 3] = np.transpose([px, py, pz])

        return tf


if __name__ == "__main__":
    calib = AutowareCalibration()
    calib.calibrate()
