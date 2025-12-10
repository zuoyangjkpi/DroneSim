#include <chrono>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace {
geometry_msgs::msg::Quaternion rpyToQuat(double roll, double pitch, double yaw) {
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}
}  // namespace

class SceneMarkersNode : public rclcpp::Node {
public:
  SceneMarkersNode() : Node("scene_markers") {
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/scene_markers", 1);
    timer_ = create_wall_timer(500ms, std::bind(&SceneMarkersNode::publishMarkers, this));
  }

private:
  void publishMarkers() {
    visualization_msgs::msg::MarkerArray array;
    const rclcpp::Time now = get_clock()->now();
    const std::string frame = "world";

    // World origin
    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = frame;
    origin.header.stamp = now;
    origin.ns = "scene";
    origin.id = 0;
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.scale.x = 0.3;
    origin.scale.y = 0.3;
    origin.scale.z = 0.3;
    origin.pose.orientation.w = 1.0;
    origin.color.r = 1.0f;
    origin.color.g = 1.0f;
    origin.color.b = 1.0f;
    origin.color.a = 0.9f;
    origin.lifetime = rclcpp::Duration(0, 0);  // persistent
    array.markers.push_back(origin);

    // Drone initial pose (lake world)
    visualization_msgs::msg::Marker drone;
    drone.header.frame_id = frame;
    drone.header.stamp = now;
    drone.ns = "scene";
    drone.id = 1;
    drone.type = visualization_msgs::msg::Marker::CUBE;
    drone.scale.x = 0.6;
    drone.scale.y = 0.6;
    drone.scale.z = 0.2;
    drone.pose.position.x = -15.0;
    drone.pose.position.y = -35.0;
    drone.pose.position.z = 5.0;
    drone.pose.orientation.w = 1.0;
    drone.color.r = 0.0f;
    drone.color.g = 1.0f;
    drone.color.b = 0.0f;
    drone.color.a = 0.8f;
    drone.lifetime = rclcpp::Duration(0, 0);
    array.markers.push_back(drone);

    // GUI camera pose (lake world default)
    visualization_msgs::msg::Marker cam;
    cam.header.frame_id = frame;
    cam.header.stamp = now;
    cam.ns = "scene";
    cam.id = 2;
    cam.type = visualization_msgs::msg::Marker::ARROW;
    cam.scale.x = 2.0;  // shaft length
    cam.scale.y = 0.2;  // shaft diameter
    cam.scale.z = 0.2;  // head diameter
    cam.pose.position.x = -23.0;
    cam.pose.position.y = -45.4;
    cam.pose.position.z = 34.5;
    cam.pose.orientation = rpyToQuat(0.0, 0.54, 1.11);
    cam.color.r = 0.0f;
    cam.color.g = 0.7f;
    cam.color.b = 1.0f;
    cam.color.a = 0.9f;
    cam.lifetime = rclcpp::Duration(0, 0);
    array.markers.push_back(cam);

    pub_->publish(array);
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SceneMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
