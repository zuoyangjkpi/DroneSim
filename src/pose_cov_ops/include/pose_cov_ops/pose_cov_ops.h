#ifndef POSE_COV_OPS_POSE_COV_OPS_H
#define POSE_COV_OPS_POSE_COV_OPS_H

#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace pose_cov_ops {

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;

/// Compose two poses: out = a ⊕ b
void compose(const PoseWithCovariance &a, const PoseWithCovariance &b, PoseWithCovariance &out);
void compose(const PoseWithCovariance &a, const Pose &b, PoseWithCovariance &out);

/// Compute relative pose: out = b ⊖ a (pose b expressed in frame of a)
void inverseCompose(const PoseWithCovariance &a, const PoseWithCovariance &b, PoseWithCovariance &out);
void inverseCompose(const PoseWithCovariance &a, const Pose &b, PoseWithCovariance &out);

}  // namespace pose_cov_ops

#endif  // POSE_COV_OPS_POSE_COV_OPS_H
