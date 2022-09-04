include "map_builder.lua"
include "trajectory_builder.lua"

robot_id = os.getenv("MKMR_ROBOT_ID")

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = robot_id .. "/map",
  tracking_frame = robot_id .. "/base_footprint",
  published_frame = robot_id .. "/odom",
  odom_frame = robot_id .. "/odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
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
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.2 --default=0
TRAJECTORY_BUILDER_2D.max_range = 10  --default=30
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5  -- default=5
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --default=1
TRAJECTORY_BUILDER_2D.voxel_filter_size=0.06-- default=0.025 


--Voxel filter
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter = {
  max_length = 2, --default = 0.5
  min_num_points = 100, --default=200
  max_range = 15., --default=50
}

--Loop Closure Adaptive Voxel filter
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter = {
  max_length = 0.9, --default = 0.9
  min_num_points = 100, --default = 100
  max_range = 50., --default = 50
}

-- correlative_scan_matcher
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true --default =false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher = {
  linear_search_window =0.1, --default=0.1 
  angular_search_window = math.rad(30), --default=math.rad(20.)
  translation_delta_cost_weight = 1e-2, --default=1e-1 
  rotation_delta_cost_weight = 1e-2, --default=1e-1 
}

-- ceres scan matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher = {
  occupied_space_weight = 100, --default=1
  translation_weight = 100, --default=10
  rotation_weight = 1e3, --default=40
  ceres_solver_options = {
    use_nonmonotonic_steps = true, --default =false
    max_num_iterations = 40, --default=20
    num_threads = 2,  --default=1
  },
}
--motion filter
TRAJECTORY_BUILDER_2D.motion_filter = {
  max_time_seconds = 1.,  --default=5
  max_distance_meters = 0.1,  --default=0.2
  max_angle_radians = math.rad(1.),  --default=math.rad(1.)
}

--Submaps
TRAJECTORY_BUILDER_2D.submaps = {
  num_range_data = 50, --default=90
  grid_options_2d = {
    grid_type = "PROBABILITY_GRID",
    resolution = 0.06, --default = 0.05
  },
  range_data_inserter = {
    range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D",
    probability_grid_range_data_inserter = {
      insert_free_space = true, --defaut=true
      hit_probability = 0.55, --default=0.55
      miss_probability = 0.49, --default=0.49
    },
    tsdf_range_data_inserter = {
      truncation_distance = 0.3, --default=0.3
      maximum_weight = 10., --default=10
      update_free_space = false, --defaut=false
      normal_estimation_options = {
        num_normal_samples = 4, --default=4
        sample_radius = 0.5, --default=0.5
      },
      project_sdf_distance_to_scan_normal = true, --defaut=true
      update_weight_range_exponent = 0, --default=0
      update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5, --default=0.5
      update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5, --default=0.5
    },
  },
}

--Pose Graph
-- Pose Graph All parameters are default on bellow 
POSE_GRAPH.optimize_every_n_nodes = 90 

POSE_GRAPH.constraint_builder = {
  sampling_ratio = 0.3,
  max_constraint_distance = 15.,
  min_score = 0.55,
  global_localization_min_score = 0.6,
  loop_closure_translation_weight = 1.1e4,
  loop_closure_rotation_weight = 1e5,
  log_matches = true,
  fast_correlative_scan_matcher = {
    linear_search_window = 7.,
    angular_search_window = math.rad(30.),
    branch_and_bound_depth = 7,
  },
  ceres_scan_matcher = {
    occupied_space_weight = 20.,
    translation_weight = 10.,
    rotation_weight = 1.,
    ceres_solver_options = {
      use_nonmonotonic_steps = true,
      max_num_iterations = 10,
      num_threads = 1,
    },
  },
  fast_correlative_scan_matcher_3d = {
    branch_and_bound_depth = 8,
    full_resolution_depth = 3,
    min_rotational_score = 0.77,
    min_low_resolution_score = 0.55,
    linear_xy_search_window = 5.,
    linear_z_search_window = 1.,
    angular_search_window = math.rad(15.),
  },
  ceres_scan_matcher_3d = {
    occupied_space_weight_0 = 5.,
    occupied_space_weight_1 = 30.,
    translation_weight = 10.,
    rotation_weight = 1.,
    only_optimize_yaw = false,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 10,
      num_threads = 1,
    },
  },
}


POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3
POSE_GRAPH.optimization_problem = {
  huber_scale = 1e1,
  acceleration_weight = 1.1e2,
  rotation_weight = 1.6e4,
  local_slam_pose_translation_weight = 1e5,
  local_slam_pose_rotation_weight = 1e5,
  odometry_translation_weight = 1e5,
  odometry_rotation_weight = 1e5,
  fixed_frame_pose_translation_weight = 1e1,
  fixed_frame_pose_rotation_weight = 1e2,
  fixed_frame_pose_use_tolerant_loss = false,
  fixed_frame_pose_tolerant_loss_param_a = 1,
  fixed_frame_pose_tolerant_loss_param_b = 1,
  log_solver_summary = false,
  use_online_imu_extrinsics_in_3d = true,
  fix_z_in_3d = false,
  ceres_solver_options = {
    use_nonmonotonic_steps = false,
    max_num_iterations = 50,
    num_threads = 7,
  }
}

POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.log_residual_histograms = true
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
return options
