include "mapping.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 7,
}

POSE_GRAPH.optimize_every_n_nodes = 1
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40 -- default: 90

return options
