#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
void point_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
   pcl::fromROSMsg(*ros_cloud,pointCloud);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "ground_truth_cloud");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 10, point_cloud_cb);
  ros::Duration(2).sleep();
  ROS_INFO_STREAM("Subcribed to the point cloud topic");
  ROS_INFO_STREAM("Extracting the ground truth cloud for pose estimation....");

  while(ros::ok()){

    ros::spinOnce();
    auto cloudData = pointCloud;

    if (cloudData.size()==0){
      ROS_WARN("Point cloud not received");
    }
    ROS_INFO_STREAM("Size of the point cloud: "<<cloudData.size()<<"\n");
    //ROS_INFO_STREAM("X: "<<pointCloud[0].x<<" Y: "<<pointCloud[0].y<<" Z: "<<pointCloud[0].z<<"\n");
    for(const auto& point : pointCloud){
      ROS_INFO_STREAM("X: "<<point.x<<" Y: "<<point.y<<" Z: "<<point.z<<"\n");
      ROS_INFO_STREAM("R: "<<point.r<<" G: "<<point.g<<" B: "<<point.b<<"\n");
    }
    ros::Duration(0.1).sleep();

  }
}