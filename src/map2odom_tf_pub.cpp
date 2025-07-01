#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h> 

class MAP2ODOM
{
public:
  MAP2ODOM(ros::NodeHandle &nh)
  : tf_pub(), static_tf_pub(), map_odom_flag(false)
  {
    pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/base_point", 1);

    //subscriber
    pointcloud_sub_.subscribe(nh, "/agent1/ouster/points", 1);
    odom_sub_.subscribe(nh, "/Odometry", 1);

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), pointcloud_sub_, odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.1));
    sync_->registerCallback(boost::bind(&MAP2ODOM::callback, this, _1, _2));
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const nav_msgs::OdometryConstPtr &odom_msg)
  {
    const ros::Time& stamp = odom_msg->header.stamp; //time_save
    //============== map2odom ======================//
    if (!map_odom_flag)
    {
    geometry_msgs::TransformStamped map2odom;
    map2odom.header.stamp    = stamp;
    map2odom.header.frame_id = "map";
    map2odom.child_frame_id  = "odom";

    map2odom.transform.translation.x = -odom_msg->pose.pose.position.x;
    map2odom.transform.translation.y = -odom_msg->pose.pose.position.y;
    map2odom.transform.translation.z = -odom_msg->pose.pose.position.z;

    tf2::Quaternion q(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w);
    map2odom.transform.rotation = tf2::toMsg(q.inverse());

    static_tf_pub.sendTransform(map2odom);
    map_odom_flag = true;
    }

    geometry_msgs::TransformStamped odom2base;
    odom2base.header.stamp    = stamp;
    odom2base.header.frame_id = "odom";
    odom2base.child_frame_id  = "base_link";
    odom2base.transform.translation.x = odom_msg->pose.pose.position.x;
    odom2base.transform.translation.y = odom_msg->pose.pose.position.y;
    odom2base.transform.translation.z = odom_msg->pose.pose.position.z;
    odom2base.transform.rotation      = odom_msg->pose.pose.orientation;
    tf_pub.sendTransform(odom2base);

    sensor_msgs::PointCloud2 base_point = *cloud_msg;
    base_point.header.frame_id = "base_link";
    base_point.header.stamp    = stamp;
    pointcloud_pub_.publish(base_point);

    }

private:
  ros::Publisher pointcloud_pub_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  
  tf2_ros::TransformBroadcaster tf_pub;
  tf2_ros::StaticTransformBroadcaster static_tf_pub;
  bool map_odom_flag;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_transformer_node");
  ros::NodeHandle nh;
  MAP2ODOM transformer(nh);
  ros::spin();
  return 0;
}
