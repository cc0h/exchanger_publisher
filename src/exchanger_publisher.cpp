#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <exchanger_publisher/ExchangerConfig.h>
#include <dynamic_reconfigure/server.h>
#include "random"

#define CV_PI   3.1415926535897932384626433832795

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
    if (current_config_.gen != last_value_)
    {
      last_value_ =current_config_.gen;
      switch (current_config_.level) {
        case 1:
          current_config_.x = 0.2;
          current_config_.y = gen_rand_number(-0.185, 0.185);
          current_config_.z = 0.72;
          current_config_.roll = 0;
          current_config_.pitch = 0;
          current_config_.yaw = 0;
          break;
        case 2:
          current_config_.x = gen_rand_number(0, 0.27);
          current_config_.y = gen_rand_number(-0.255, 0.255);
          current_config_.z = gen_rand_number(0.72, 0.9);
          current_config_.roll = 0;
          current_config_.pitch = 0;
          current_config_.yaw = 0;
          break;
        case 3:
          current_config_.x = gen_rand_number(0, 0.27);
          current_config_.y = gen_rand_number(-0.255, 0.255);
          current_config_.z = gen_rand_number(0.72, 0.9);
          current_config_.roll = gen_rand_number(-45.0, 45.0);
          current_config_.pitch = gen_rand_number(0.0, 90.0);
          current_config_.yaw = 0;
          break;
        case 4:
          current_config_.x = gen_rand_number(0, 0.27);
          current_config_.y = gen_rand_number(-0.255, 0.255);
          current_config_.z = gen_rand_number(0.72, 0.9);
          current_config_.roll = gen_rand_number(-45.0, 45.0);
          current_config_.pitch = gen_rand_number(0.0, 90.0);
          current_config_.yaw = gen_rand_number(-90.0, 90.0);
          break;
        case 5:
          current_config_.x = gen_rand_number(0, 0.27);
          current_config_.y = gen_rand_number(-0.255, 0.255);
          current_config_.z = gen_rand_number(0.72, 0.9);
          current_config_.roll = gen_rand_number(-45.0, 45.0);
          current_config_.pitch = gen_rand_number(0.0, 90.0);
          current_config_.yaw = gen_rand_number(-135.0, 135.0);
          break;
      }
      dyn_server_->updateConfig(current_config_);
    }
  }

  void publishTF(const ros::TimerEvent& event)
  {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "exchanger_base";
    transformStamped.transform.translation.x = current_config_.x + 0.3;
    transformStamped.transform.translation.y = current_config_.y;
    transformStamped.transform.translation.z = current_config_.z;

    tf2::Quaternion q;

    roll_ = CV_PI / 180 * current_config_.roll;
    pitch_ = CV_PI / 180 * current_config_.pitch;
    yaw_ = CV_PI / 180 * current_config_.yaw;

    q.setRPY(roll_, pitch_, yaw_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    broadcaster_.sendTransform(transformStamped);
  }

  double gen_rand_number(double min, double max){
    gen_();
    std::uniform_real_distribution<> dis(min, max);
    return dis(gen_);
  }


  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster broadcaster_;
  dynamic_reconfigure::Server<exchanger_publisher::ExchangerConfig>* dyn_server_;
  exchanger_publisher::ExchangerConfig current_config_;
  ros::Timer timer_;
  double roll_, pitch_, yaw_;
  std::mt19937 gen_;
  bool last_value_{true};
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ExchangerPublisherNode node;
  geometry_msgs::TransformStamped static_transformstamped;

  static_transformstamped.header.stamp = ros::Time::now();
  static_transformstamped.header.frame_id = "exchanger_base";
  static_transformstamped.child_frame_id = "exchanger";
  static_transformstamped.transform.translation.x = 0;
  static_transformstamped.transform.translation.y = 0;
  static_transformstamped.transform.translation.z = 0;
  tf2::Quaternion sq;
  sq.setRPY(CV_PI, 0 , CV_PI);
  static_transformstamped.transform.rotation.x = sq.x();
  static_transformstamped.transform.rotation.y = sq.y();
  static_transformstamped.transform.rotation.z = sq.z();
  static_transformstamped.transform.rotation.w = sq.w();

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  static_broadcaster.sendTransform(static_transformstamped);
  ros::spin();
  return 0;
}