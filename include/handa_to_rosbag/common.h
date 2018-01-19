#ifndef HANDA_TO_ROSBAG_COMMON_H_
#define HANDA_TO_ROSBAG_COMMON_H_

#include <vector>

#include <kindr/minimal/quat-transformation.h>
#include <kindr/minimal/rotation-quaternion.h>

namespace handa_to_rosbag {

// The floating point to use with minkindr types
using FloatingPoint = double;

// Transformation typedefs
typedef kindr::minimal::QuatTransformationTemplate<FloatingPoint>
    Transformation;
typedef kindr::minimal::RotationQuaternionTemplate<FloatingPoint> Quaternion;
typedef kindr::minimal::PositionTemplate<FloatingPoint> Position;

/*// Aligned Eigen containers
template <typename Type>
using AlignedVector = std::vector<Type, Eigen::aligned_allocator<Type>>;

// Containers of transforms
typedef AlignedVector<Transformation> TransformationVector;
*/
}  // namespace handa_to_rosbag

#endif /* HANDA_TO_ROSBAG_COMMON_H_ */
