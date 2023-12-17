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

import open3d as o3d
import numpy as np

flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]


def make_point_cloud(points, normals=None, colors=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if normals is not None:
        pcd.normals = o3d.utility.Vector3dVector(normals)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def visualize_icp(source_points, target_points, T):
    pcd_source = make_point_cloud(source_points)
    pcd_source.paint_uniform_color([1, 0, 0])

    pcd_target = make_point_cloud(target_points)
    pcd_target.paint_uniform_color([0, 1, 0])

    pcd_source.transform(T)
    o3d.visualization.draw_geometries([
        pcd_source.transform(flip_transform),
        pcd_target.transform(flip_transform)
    ])


def visualize_correspondences(source_points, target_points, T):
    if len(source_points) != len(target_points):
        print(
            'Error! source points and target points has different length {} vs {}'
            .format(len(source_points), len(target_points)))
        return

    pcd_source = make_point_cloud(source_points)
    pcd_source.paint_uniform_color([1, 0, 0])
    pcd_source.transform(T)
    pcd_source.transform(flip_transform)

    pcd_target = make_point_cloud(target_points)
    pcd_target.paint_uniform_color([0, 1, 0])
    pcd_target.transform(flip_transform)

    corres = []
    for k in range(len(source_points)):
        corres.append((k, k))

    lineset = o3d.geometry.LineSet.create_from_point_cloud_correspondences(
        pcd_source, pcd_target, corres)

    o3d.visualization.draw_geometries([pcd_source, pcd_target, lineset])
