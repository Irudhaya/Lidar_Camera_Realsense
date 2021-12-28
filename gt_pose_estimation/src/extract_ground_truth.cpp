#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <vector>
#include <tuple>
#include <string>

typedef sensor_msgs::PointCloud2ConstPtr rosMsgPointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> pclPointsWithColor;
typedef pcl::PointCloud<pcl::PointXYZ> pclPoints;
typedef std::vector<std::vector<cv::Point>> points2D;
typedef cv::Mat image;

//pclPointsWithColor pointCloud;
rosMsgPointCloud rosCloud;
cv_bridge::CvImageConstPtr cv_ptr;
int image_width,image_height;


void point_cloud_cb (const sensor_msgs::PointCloud2ConstPtr& msg_cloud)
{
   rosCloud = msg_cloud;
   //pcl::fromROSMsg(*ros_cloud,pointCloud);
   ROS_INFO("point cloud callback called");
}

void raw_image_cb (const sensor_msgs::ImageConstPtr& ros_image){
  ROS_INFO("image callback called");
  cv_ptr = cv_bridge::toCvShare(ros_image, sensor_msgs::image_encodings::BGR8);
  image_width = ros_image->width;
  image_height = ros_image->height;
  ROS_INFO("Source image received");
  //image rawImage(image_height,image_width,sensor_msgs::image_encodings::RGB8);
  auto rawImage = cv_ptr->image; // copy of image 
  cv::imshow("Raw Image", rawImage);
  cv::waitKey(3);
  ROS_INFO("Source image display finished");
  //   // Draw an example circle on the video stream
  //   if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
  //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  //   // Update GUI Window
  //   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //   cv::waitKey(3);

  //   // Output modified video stream
  //   image_pub_.publish(cv_ptr->toImageMsg());
  // }
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

  ROS_INFO_STREAM("Size of the point cloud while copying: "<<data.size()<<"\n");
  pclPoints points;

  pcl::copyPointCloud(points,data);
  //pcl::copyPointCloud(targetPoints,targetCloud);

  //return std::make_tuple(sourcePoints,targetPoints);
  return points;

}

pclPoints getROICloud(pclPointsWithColor& cloud, points2D& contourPoints){
  pclPoints downSampledCloud;
  ROS_INFO("Raw  Data");
  auto objectContours = contourPoints[0];
  std::for_each(objectContours.begin(),objectContours.end(),[&](auto& point){
    if(point.x<=0.70){
      downSampledCloud.push_back (pcl::PointXYZ (cloud[0].x,cloud[0].y,cloud[0].z));
    }
  });
}

points2D getContours();//implement

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "scan_matching_pose_estimation");
  ros::NodeHandle nh;
  std::string pointCloudTopic{"/camera/depth_registered/points"};
  std::string rawImageTopic{"/camera/color/image_raw"};

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber cloud_sub = nh.subscribe (pointCloudTopic, 10, point_cloud_cb);
  ros::Subscriber image_sub = nh.subscribe (rawImageTopic, 10, raw_image_cb);
  ros::Duration(2).sleep();
  ROS_INFO_STREAM("Subcribed to the point cloud topic");
  ROS_INFO_STREAM("Receiving the first point cloud data");
  ros::spinOnce(); //callback called to initialize the pointers to pointcloud and image
  auto sourceCloud = setSourceCloud(nh,pointCloudTopic);
  if (sourceCloud.size() !=0){
    ROS_INFO("Source point cloud received");
    auto sourcePoints = getPoints(sourceCloud);
  }
  

  while(ros::ok()){

    pclPointsWithColor rawPointCloud;
    pcl::fromROSMsg(*rosCloud,rawPointCloud);
    ROS_INFO_STREAM("Size of the point cloud before copying: "<<rawPointCloud.size()<<"\n");
    //auto targetCloud = pointCloud;
    pclPoints targetPoints;

    if (rawPointCloud.size()==0){
      ROS_WARN("Point cloud not received");
    }
    else {
      targetPoints = getPoints(rawPointCloud);
    }
    ROS_INFO_STREAM("Size of the point cloud after copying: "<<rawPointCloud.size()<<"\n");
    ROS_INFO_STREAM("Size of the points: "<<targetPoints.size()<<"\n");
    //ROS_INFO_STREAM("X: "<<pointCloud[0].x<<" Y: "<<pointCloud[0].y<<" Z: "<<pointCloud[0].z<<"\n");
    for(const auto& point : rawPointCloud){
      ROS_INFO_STREAM("X: "<<point.x<<" Y: "<<point.y<<" Z: "<<point.z<<"\n");
      ROS_INFO_STREAM("R: "<<point.r<<" G: "<<point.g<<" B: "<<point.b<<"\n");
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    

  }
}