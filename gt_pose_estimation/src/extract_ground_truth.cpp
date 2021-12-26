#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tuple>

typedef pcl::PointCloud<pcl::PointXYZRGB> pclPointsWithColor;
typedef pcl::PointCloud<pcl::PointXYZ> pclPoints;

pclPointsWithColor pointCloud;


void point_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& ros_cloud)
{
   pcl::fromROSMsg(*ros_cloud,pointCloud);
}

pclPointsWithColor setSourceCloud(ros::NodeHandle &nh, std::string& cloudTopic){
  
  auto cloudData = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloudTopic,nh);
  pclPointsWithColor data;
  if (cloudData!=NULL){
    pcl::fromROSMsg(*cloudData,data);
    ROS_INFO("Received the source cloud");
  }
  else{
    ROS_ERROR("Failed to set the source cloud");
    ros::Duration(5).sleep();
  }
  return data;
}

pclPoints getPoints(pclPointsWithColor& data){
  pclPoints points;

  pcl::copyPointCloud(points,data);
  //pcl::copyPointCloud(targetPoints,targetCloud);

  //return std::make_tuple(sourcePoints,targetPoints);
  return points;

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "scan_matching_pose_estimation");
  ros::NodeHandle nh;
  std::string topic{"/camera/depth_registered/points"};
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe (topic, 10, point_cloud_cb);
  ros::Duration(2).sleep();
  ROS_INFO_STREAM("Subcribed to the point cloud topic");
  ROS_INFO_STREAM("Receiving the first point cloud data");

  auto sourceCloud = setSourceCloud(nh,topic);
  auto sourcePoints = getPoints(sourceCloud);

  while(ros::ok()){

    ros::spinOnce();
    auto targetCloud = pointCloud;
    auto targetPoints = getPoints(targetCloud);

    if (targetCloud.size()==0){
      ROS_WARN("Point cloud not received");
    }
    ROS_INFO_STREAM("Size of the point cloud: "<<targetCloud.size()<<"\n");
    //ROS_INFO_STREAM("X: "<<pointCloud[0].x<<" Y: "<<pointCloud[0].y<<" Z: "<<pointCloud[0].z<<"\n");
    for(const auto& point : targetCloud){
      ROS_INFO_STREAM("X: "<<point.x<<" Y: "<<point.y<<" Z: "<<point.z<<"\n");
      ROS_INFO_STREAM("R: "<<point.r<<" G: "<<point.g<<" B: "<<point.b<<"\n");
    }
    ros::Duration(0.1).sleep();

  }
}