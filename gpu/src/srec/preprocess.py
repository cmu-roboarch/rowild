#! /usr/bin/env python3

# MIT License
#
# Copyright (c) 2023 Carnegie Mellon University
#
# This file is part of RoWild.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

'''
    Initially written by Ming Hsiao in MATLAB
    Redesigned and rewritten by Wei Dong (weidong@andrew.cmu.edu)
    Modified by Mohammad Bakhshalipour (bakhshalipour@cmu.edu)
'''

import os
import argparse
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import quaternion

import transforms
import o3d_utility


def load_gt_poses(gt_filename):
    indices = []
    Ts = []

    # Camera to world
    # Dirty left 2 right coordinate transform
    # https://github.com/theNded/MeshHashing/blob/master/src/io/config_manager.cc#L88
    T_l2r = np.eye(4)
    T_l2r[1, 1] = -1

    with open(gt_filename) as f:
        content = f.readlines()
        for line in content:
            data = np.array(list(map(float, line.strip().split(' '))))
            indices.append(int(data[0]))

            data = data[1:]

            q = data[3:][[3, 0, 1, 2]]
            q = quaternion.from_float_array(q)
            R = quaternion.as_rotation_matrix(q)

            t = data[:3]
            T = np.eye(4)

            T[0:3, 0:3] = R
            T[0:3, 3] = t

            Ts.append(T_l2r @ T @ np.linalg.inv(T_l2r))

    return indices, Ts


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('path', help='path to the dataset folder containing rgb/ and depth/')
    parser.add_argument('gt', help='path to ground truth file')
    args = parser.parse_args()

    # Load intrinsics and gt poses for evaluation
    intrinsic_struct = o3d.io.read_pinhole_camera_intrinsic('intrinsics.json')
    intrinsic = np.array(intrinsic_struct.intrinsic_matrix)
    indices, gt_poses = load_gt_poses(args.gt)
    depth_scale = 5000.0

    depth_path = os.path.join(args.path, 'depth')
    normal_path = os.path.join(args.path, 'normal')
    transform_init_file = os.path.join(args.path, 'init_transform.txt')
    os.makedirs(normal_path, exist_ok=True)

    # Write down initial transformation
    tMat = '\n'.join(['\t'.join([str(cell) for cell in row]) for row in gt_poses[0]])
    with open(transform_init_file, 'w') as f: f.write(tMat);

    # Generate normal maps
    for i in indices:
        print('Preprocessing frame {:03d}'.format(i))
        depth = np.asarray(o3d.io.read_image('{}/{}.png'.format(
            depth_path, i))) / depth_scale
        vertex_map = transforms.unproject(depth, intrinsic)

        pcd = o3d_utility.make_point_cloud(vertex_map.reshape((-1, 3)))
        pcd.estimate_normals()

        normal_map = np.asarray(pcd.normals).reshape(vertex_map.shape)
        assert normal_map.shape[2] == 3

        for j in range(normal_map.shape[2]):
            np.savetxt('{}/{}-dim{}.txt'.format(normal_path, i, j+1), normal_map[:,:,j], fmt='%.2f')
