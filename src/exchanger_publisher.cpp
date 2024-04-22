#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <exchanger_publisher/ExchangerConfig.h>
#include <dynamic_reconfigure/server.h>

class ExchangerPublisherNode
{
public:
  ExchangerPublisherNode() : timer_(nh_.createTimer(ros::Duration(0.1), &ExchangerPublisherNode::publishTF, this))
  {
    dyn_server_ = new dynamic_reconfigure::Server<exchanger_publisher::ExchangerConfig>(ros::NodeHandle("~"));
    dyn_server_->setCallback(boost::bind(&ExchangerPublisherNode::reconfigureCB, this, _1, _2));
  }
private:
  void reconfigureCB(exchanger_publisher::ExchangerConfig& config, uint32_t level)
  {
    current_config_ = config;

  }

  void publishTF(const ros::TimerEvent& event)
  {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "exchanger";

    // Set your desired transform values here (e.g., translation and rotation)
    transformStamped.transform.translation.x = current_config_.x;
    transformStamped.transform.translation.y = current_config_.y;
    transformStamped.transform.translation.z = current_config_.z;

    tf2::Quaternion q;
    q = rpy2quat(current_config_.roll, current_config_.pitch, current_config_.yaw);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    broadcaster_.sendTransform(transformStamped);
  }

  static tf2::Quaternion rpy2quat(double roll, double pitch, double yaw) {
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    m.setRPY(roll, pitch, yaw);
    m.getRotation(q);
    return q;
  }


  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster broadcaster_;
  dynamic_reconfigure::Server<exchanger_publisher::ExchangerConfig>* dyn_server_;
  exchanger_publisher::ExchangerConfig current_config_;
  ros::Timer timer_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ExchangerPublisherNode node;
  ros::spin();
  return 0;
}