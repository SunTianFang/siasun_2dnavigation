
#ifndef SLAM_MAPPING_OPTIONS_H_
#define SLAM_MAPPING_OPTIONS_H_

#include "transformproto.h"


#include "sensorproto.h"

namespace proto{



struct SensorId {
  enum SensorType {
    RANGE = 0,
    IMU = 1,
    ODOMETRY = 2,
    FIXED_FRAME_POSE = 3,
    LANDMARK = 4,
    LOCAL_SLAM_RESULT = 5,
  };

  SensorType type ;
  string id ;
};


struct InitialTrajectoryPose {

  Rigid3d relative_pose ;
  int32 to_trajectory_id ;
  int64 timestamp ;
};




struct GridOptions2D {
  enum GridType {
    INVALID_GRID = 0,
    PROBABILITY_GRID = 1,
  };

  GridType grid_type;
  float resolution ;
};



struct ProbabilityGridRangeDataInserterOptions2D {
  // Probability change for a hit (this will be converted to odds and therefore
  // must be greater than 0.5).
  double hit_probability;

  // Probability change for a miss (this will be converted to odds and therefore
  // must be less than 0.5).
  double miss_probability ;

  // If 'false', free space will not change the probabilities in the occupancy
  // grid.
  bool insert_free_space;
};
struct RangeDataInserterOptions {
  enum RangeDataInserterType {
    INVALID_INSERTER = 0,
    PROBABILITY_GRID_INSERTER_2D = 1,
  };

  RangeDataInserterType range_data_inserter_type ;
  ProbabilityGridRangeDataInserterOptions2D probability_grid_range_data_inserter_options_2d ;
};

struct SubmapsOptions2D {
  // Number of range data before adding a new submap. Each submap will get twice
  // the number of range data inserted: First for initialization without being
  // matched against, then while being matched.
  int32 num_range_data ;
  GridOptions2D grid_options_2d ;
  RangeDataInserterOptions range_data_inserter_options ;
};

struct RealTimeCorrelativeScanMatcherOptions {
  // Minimum linear search window in which the best possible scan alignment
  // will be found.
  double linear_search_window ;

  // Minimum angular search window in which the best possible scan alignment
  // will be found.
  double angular_search_window ;

  // Weights applied to each part of the score.
  double translation_delta_cost_weight ;
  double rotation_delta_cost_weight ;
  double angular_resolution;

};

struct MotionFilterOptions {
  // Threshold above which range data is inserted based on time.
  double max_time_seconds ;

  // Threshold above which range data is inserted based on linear motion.
  double max_distance_meters ;

  // Threshold above which range data is inserted based on rotational motion.
  double max_angle_radians ;
};




struct CeresSolverOptions {
  // Configure the Ceres solver. See the Ceres documentation for more
  // information: https://code.google.com/p/ceres-solver/
  bool use_nonmonotonic_steps ;
  int32 max_num_iterations ;
  int32 num_threads ;
};

struct FastCorrelativeScanMatcherOptions2D {
  // Minimum linear search window in which the best possible scan alignment
  // will be found.
  double linear_search_window ;

  // Minimum angular search window in which the best possible scan alignment
  // will be found.
  double angular_search_window ;

  // Number of precomputed grids to use.
  int32 branch_and_bound_depth ;
};


struct CeresScanMatcherOptions2D {
  // Scaling parameters for each cost functor.
  double occupied_space_weight ;
  double translation_weight ;
  double rotation_weight ;


};



struct ConstraintBuilderOptions {
  // A constraint will be added if the proportion of added constraints to
  // potential constraints drops below this number.
  double sampling_ratio ;

  // Threshold for poses to be considered near a submap.
  double max_constraint_distance ;

  // Threshold for the scan match score below which a match is not considered.
  // Low scores indicate that the scan and map do not look similar.
  double min_score ;

  // Threshold below which global localizations are not trusted.
  double global_localization_min_score ;

  // Weight used in the optimization problem for the translational component of
  // loop closure constraints.
  double loop_closure_translation_weight ;

  // Weight used in the optimization problem for the rotational component of
  // loop closure constraints.
  double loop_closure_rotation_weight ;

  // If enabled, logs information of loop-closing constraints for debugging.
  bool log_matches ;

  // Options for the internally used scan matchers.
  FastCorrelativeScanMatcherOptions2D
      fast_correlative_scan_matcher_options ;
  CeresScanMatcherOptions2D
      ceres_scan_matcher_options ;


};


struct LocalTrajectoryBuilderOptions2D {
  // Rangefinder points outside these ranges will be dropped.
  float min_range ;
  float max_range ;
  float min_z ;
  float max_z ;

  // Points beyond 'max_range' will be inserted with this length as empty space.
  float missing_data_ray_length ;

  // Number of range data to accumulate into one unwarped, combined range data
  // to use for scan matching.
  int32 num_accumulated_range_data ;

  // Voxel filter that gets applied to the range data immediately after
  // cropping.
  float voxel_filter_size ;

  // Voxel filter used to compute a sparser point cloud for matching.
  ::proto::AdaptiveVoxelFilterOptions adaptive_voxel_filter_options ;

  // Voxel filter used to compute a sparser point cloud for finding loop
  // closures.
  ::proto::AdaptiveVoxelFilterOptions loop_closure_adaptive_voxel_filter_options ;

  // Whether to solve the online scan matching first using the correlative scan
  // matcher to generate a good starting point for Ceres.
  bool use_online_correlative_scan_matching ;
  RealTimeCorrelativeScanMatcherOptions real_time_correlative_scan_matcher_options ;
  CeresScanMatcherOptions2D ceres_scan_matcher_options ;
  MotionFilterOptions motion_filter_options ;

  // Time constant in seconds for the orientation moving average based on
  // observed gravity via the IMU. It should be chosen so that the error
  // 1. from acceleration measurements not due to gravity (which gets worse when
  // the constant is reduced) and
  // 2. from integration of angular velocities (which gets worse when the
  // constant is increased) is balanced.
  double imu_gravity_time_constant ;

  SubmapsOptions2D submaps_options ;

  // True if IMU data should be expected and used.
  bool use_imu_data ;
};



struct TrajectoryBuilderOptions {
  LocalTrajectoryBuilderOptions2D trajectory_builder_2d_options ;
  //LocalTrajectoryBuilderOptions3D trajectory_builder_3d_options = 2;
  bool pure_localization ;
  InitialTrajectoryPose initial_trajectory_pose ;

  //struct OverlappingSubmapsTrimmerOptions2D {
   // int32 fresh_submaps_count ;
   // double min_covered_area ;
   // int32 min_added_submaps_count ;
  //};
  //OverlappingSubmapsTrimmerOptions2D overlapping_submaps_trimmer_2d ;
};
struct TrajectoryBuilderOptionsWithSensorIds {
  vector<SensorId> sensor_id ;
  TrajectoryBuilderOptions trajectory_builder_options ;
};


struct AllTrajectoryBuilderOptions {
  vector<TrajectoryBuilderOptionsWithSensorIds> options_with_sensor_ids ;
};

struct OptimizationProblemOptions {
  // Scaling parameter for Huber loss function.
  double huber_scale ;

  // Scaling parameter for the IMU acceleration term.
  double acceleration_weight ;

  // Scaling parameter for the IMU rotation term.
  double rotation_weight ;

  // Scaling parameter for translation between consecutive nodes based on the
  // local SLAM pose.
  double local_slam_pose_translation_weight ;

  // Scaling parameter for rotation between consecutive nodes based on the
  // local SLAM pose.
  double local_slam_pose_rotation_weight ;

  // Scaling parameter for translation between consecutive nodes based on the
  // odometry.
  double odometry_translation_weight ;

  // Scaling parameter for rotation between consecutive nodes based on the
  // odometry.
  double odometry_rotation_weight ;

  // Scaling parameter for the FixedFramePose translation.
  double fixed_frame_pose_translation_weight ;

  // Scaling parameter for the FixedFramePose rotation.
  double fixed_frame_pose_rotation_weight ;

  // 3D only: fix Z.
  bool fix_z_in_3d ;

  // If true, the Ceres solver summary will be logged for every optimization.
  bool log_solver_summary ;

  CeresSolverOptions ceres_solver_options ;
};



struct PoseGraphOptions {
  // Online loop closure: If positive, will run the loop closure while the map
  // is built.
  int32 optimize_every_n_nodes ;

  // Options for the constraint builder.
  ConstraintBuilderOptions
      constraint_builder_options ;

  // Weight used in the optimization problem for the translational component of
  // non-loop-closure scan matcher constraints.
  double matcher_translation_weight ;

  // Weight used in the optimization problem for the rotational component of
  // non-loop-closure scan matcher constraints.
  double matcher_rotation_weight ;

  // Options for the optimization problem.
  OptimizationProblemOptions
      optimization_problem_options ;

  // Number of iterations to use in 'optimization_problem_options' for the final
  // optimization.
  int32 max_num_final_iterations ;

  // Rate at which we sample a single trajectory's nodes for global
  // localization.
  double global_sampling_ratio ;

  // Whether to output histograms for the pose residuals.
  bool log_residual_histograms ;

  // If for the duration specified by this option no global contraint has been
  // added between two trajectories, loop closure searches will be performed
  // globally rather than in a smaller search window.
  double global_constraint_search_after_n_seconds;
};

struct MapBuilderOptions {


  // Number of threads to use for background computations.
  int32 num_background_threads ;
  PoseGraphOptions pose_graph_options;
  // Sort sensor input independently for each trajectory.
  bool collate_by_trajectory ;
};




}

#endif
