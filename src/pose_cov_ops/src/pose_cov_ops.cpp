#include <pose_cov_ops/pose_cov_ops.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pose_cov_ops {
namespace {

PoseWithCovariance poseToPwC(const Pose &pose) {
  PoseWithCovariance out;
  out.pose = pose;
  return out;
}

tf2::Transform toTf(const Pose &pose) {
  tf2::Transform tf;
  tf2::fromMsg(pose, tf);
  return tf;
}

Pose toPoseMsg(const tf2::Transform &tf) {
  Pose pose_msg;
  const tf2::Vector3 &o = tf.getOrigin();
  pose_msg.position.x = o.x();
  pose_msg.position.y = o.y();
  pose_msg.position.z = o.z();
  pose_msg.orientation = tf2::toMsg(tf.getRotation());
  return pose_msg;
}

void setCovariance(PoseWithCovariance &out,
                   const std::array<double, 36> &cov_a,
                   const std::array<double, 36> &cov_b) {
  // Simple combination: element-wise sum to keep non-zero uncertainty.
  for (size_t i = 0; i < out.covariance.size(); ++i) {
    out.covariance[i] = cov_a[i] + cov_b[i];
  }
}

}  // namespace

void compose(const PoseWithCovariance &a, const PoseWithCovariance &b, PoseWithCovariance &out) {
  tf2::Transform Ta = toTf(a.pose);
  tf2::Transform Tb = toTf(b.pose);
  tf2::Transform T = Ta * Tb;

  out.pose = toPoseMsg(T);
  setCovariance(out, a.covariance, b.covariance);
}

void compose(const PoseWithCovariance &a, const Pose &b, PoseWithCovariance &out) {
  compose(a, poseToPwC(b), out);
}

void inverseCompose(const PoseWithCovariance &a, const PoseWithCovariance &b, PoseWithCovariance &out) {
  tf2::Transform Ta = toTf(a.pose);
  tf2::Transform Tb = toTf(b.pose);
  tf2::Transform T = Ta.inverseTimes(Tb);

  out.pose = toPoseMsg(T);
  setCovariance(out, a.covariance, b.covariance);
}

void inverseCompose(const PoseWithCovariance &a, const Pose &b, PoseWithCovariance &out) {
  inverseCompose(a, poseToPwC(b), out);
}

}  // namespace pose_cov_ops
