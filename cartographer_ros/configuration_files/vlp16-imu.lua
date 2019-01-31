-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.3



-- These were just my first guess: use more points for SLAMing and adapt a bit for the ranges that are bigger for cars.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 20.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 60.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

-- The submaps felt pretty big - since the car moves faster, we want them to be
-- slightly smaller. You are also slamming at 10cm - which might be aggressive
-- for cars and for the quality of the laser. I increased 'high_resolution',
-- you might need to increase 'low_resolution'. Increasing the
-- '*num_iterations' in the various optimization problems also trades
-- performance/quality.
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 4.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2


MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7


POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.optimization_problem.rotation_weight = 3e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3
--POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.5

-- This is probably the most important change: lower the matching score for
-- adding constraints.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.min_score = 0.4


--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 20.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(30.)

return options
