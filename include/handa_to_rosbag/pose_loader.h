#ifndef HANDA_TO_ROSBAG_POSE_LOADER_H_
#define HANDA_TO_ROSBAG_POSE_LOADER_H_

#include <string>

#include "handa_to_rosbag/common.h"

namespace handa_to_rosbag {

void loadHandaPose(const std::string& pose_path, Transformation* T_W_C_ptr);

void loadFreiburgPose(const std::string& pose_path, int index,
                      Transformation* T_W_C_ptr);

}  // namespace handa_to_rosbag

#endif /* HANDA_TO_ROSBAG_POSE_LOADER_H_ */
