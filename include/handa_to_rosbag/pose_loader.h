#ifndef HANDA_TO_ROSBAG_POSE_LOADER_H_
#define HANDA_TO_ROSBAG_POSE_LOADER_H_

#include <string>

#include "handa_to_rosbag/common.h"

namespace handa_to_rosbag {

void loadHandaPose(const std::string& pose_path, Transformation* T_W_C_ptr);

void loadFreiburgPose(const std::string& pose_path, int index,
                      Transformation* T_W_C_ptr);

// Transforms a vector in a left handed coordinate-system to a right handed one
inline Eigen::Vector3d toRhc(const Eigen::Vector3d& v) {
  Eigen::Vector3d v_rhc;
  v_rhc.x() = v.x();
  v_rhc.y() = -v.y();
  v_rhc.z() = v.z();
  return v_rhc;
}

}  // namespace handa_to_rosbag

#endif /* HANDA_TO_ROSBAG_POSE_LOADER_H_ */
