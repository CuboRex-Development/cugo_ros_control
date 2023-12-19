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
  tracking_frame = "base_link", -- imu_link, If you are using gazebo, use 'base_footprint' (libgazebo_ros_imu's bug)
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false, -- defalt true
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.1,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.1,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 8

TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 30.0 
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0
TRAJECTORY_BUILDER_2D.use_imu_data = false 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
--POSE_GRAPH.constraint_builder.linear_search_window = 7.0

POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
      -- > サブマップと近い poses について、考慮する距離の閾値。
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 12.
      -- > 最適なスキャンアライメントを見つけるための最小の線形探索 Window。

-- Local SLAM
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025                    -- 増やす デフォルト： 0.025
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1    -- 増やす デフォルト： 0.05

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200   -- 小さく デフォルト： 200
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.        -- 小さく デフォルト： 50.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5       -- 増やす デフォルト： 0.5
--TRAJECTORY_BUILDER_2D.max_range = 30.                              -- 小さく デフォルト： 30.
--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20                  -- 小さく デフォルト： 20


-- Global SLAM
--POSE_GRAPH.optimize_every_n_nodes = 90                             -- 小さく デフォルト： 90
--MAP_BUILDER.num_background_threads = 8                             -- 増やす(CPUコア数)デフォルト: 4
--POSE_GRAPH.global_sampling_ratio = 0.003                           -- 小さく デフォルト： 0.003
--POSE_GRAPH.constraint_builder.sampling_ratio = 0.3                 -- 小さく デフォルト： 0.3
--POSE_GRAPH.constraint_builder.min_score = 0.65                     -- 増やす デフォルト:  0.55

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200   -- 小さく デフォルト： 200     (Local SLAM 設定と同じ)
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.        -- 小さく デフォルト： 50.     (Local SLAM 設定と同じ)
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5       -- 増やす デフォルト： 0.5     (Local SLAM 設定と同じ)
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025                    -- 増やす デフォルト： 0.025   (Local SLAM 設定と同じ)
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1    -- 増やす デフォルト： 0.05    (Local SLAM 設定と同じ)
--TRAJECTORY_BUILDER_2D.submaps.num_range_data = 20                  -- 小さく デフォルト： 20      (Local SLAM 設定と同じ)

--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5.
                                                                   -- 小さく デフォルト： 5.
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
                                                                   -- 小さく デフォルト： 1.
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30.)
                                                                   -- 小さく デフォルト： math.rad(30.)
--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
                                                                   -- 小さく デフォルト： math.rad(20.)
--POSE_GRAPH.global_constraint_search_after_n_seconds = 10.          -- 増やす デフォルト:  10.
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
                                                                   -- 小さく デフォルト： 10
--POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
                                                                   -- 小さく デフォルト： 50
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
                                                                   -- 小さく デフォルト： 20



return options
