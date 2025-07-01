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
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Dense>
#include <cmath>

class PointCloudTransformer
{
public:
  PointCloudTransformer(ros::NodeHandle &nh)
  {
    pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/transformed_point", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("/odom_path", 1);

    //publisher
    point_filter_pub = nh.advertise<sensor_msgs::PointCloud2>("/filter/point", 1);
    odom_filter_pub = nh.advertise<nav_msgs::Odometry>("/filter/odom", 1);

    //subscriber
    // pointcloud_sub_.subscribe(nh, "/ouster/points", 1);
    // pointcloud_sub_.subscribe(nh, "/velodyne/points", 1);
    pointcloud_sub_.subscribe(nh, "/agent1/ouster/points", 1);
    odom_sub_.subscribe(nh, "/Odometry", 1);

    sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), pointcloud_sub_, odom_sub_));
    sync_->setMaxIntervalDuration(ros::Duration(0.1));
    sync_->registerCallback(boost::bind(&PointCloudTransformer::callback, this, _1, _2));
    path_msg_.header.frame_id = "odom";
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg, const nav_msgs::OdometryConstPtr &odom_msg)
  {
    double odom_time = odom_msg->header.stamp.toSec();
    double point_time = cloud_msg->header.stamp.toSec();
    double time_diff = std::abs(odom_time - point_time);
    std::cout << "odom_time: " << odom_time 
              << ", point_time: " << point_time 
              << ", difference: " << time_diff << std::endl;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(odom_msg->pose.pose.position.x,
                                    odom_msg->pose.pose.position.y,
                                    odom_msg->pose.pose.position.z));
    tf::Quaternion q(odom_msg->pose.pose.orientation.x,
                     odom_msg->pose.pose.orientation.y,
                     odom_msg->pose.pose.orientation.z,
                     odom_msg->pose.pose.orientation.w);
    transform.setRotation(q);

    // static tf::TransformBroadcaster br;
    // br.sendTransform(tf::StampedTransform(transform, odom_msg->header.stamp, "odom", "base_link_1"));

    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix(transform, eigen_transform);
    sensor_msgs::PointCloud2 output;
    pcl_ros::transformPointCloud(eigen_transform, *cloud_msg, output);
    output.header = odom_msg->header;
    output.header.frame_id = "odom";
    pointcloud_pub_.publish(output);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = odom_msg->header;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose = odom_msg->pose.pose;
    path_msg_.poses.push_back(pose_stamped);
    path_msg_.header.stamp = odom_msg->header.stamp;
    path_pub_.publish(path_msg_);

    sensor_msgs::PointCloud2 filter_point;
    nav_msgs::Odometry filter_odom;

    filter_point = *cloud_msg;
    filter_point.header = odom_msg->header;
    filter_point.header.frame_id = "odom";

    filter_odom = *odom_msg;
    filter_odom.header.frame_id = "odom";
    
    point_filter_pub.publish(filter_point);
    odom_filter_pub.publish(filter_odom);

  }

private:
  ros::Publisher pointcloud_pub_;
  ros::Publisher path_pub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  nav_msgs::Path path_msg_;

  ros::Publisher odom_filter_pub;
  ros::Publisher point_filter_pub;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_transformer_node");
  ros::NodeHandle nh;
  PointCloudTransformer transformer(nh);
  ros::spin();
  return 0;
}
