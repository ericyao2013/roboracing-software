#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <math.h>
#include <rr_platform/steering.h>



double steering = 0.0;


void ultrasonicCallBack(const sensor_msgs::PointCloud2ConstPtr& obstacles) {
  pcl::PointCloud2 map = *obstacles; this is not what to do probz

  float minPoint = map(0,0);
  float minDistance = pcl::euclideanDistance(minPoint, PointXYZ(0,0,0));

  for (pcl::PointXYZ point : map) { user iterator
    float dist = pcl::euclideanDistance(point, PointXYZ(0,0,0));
    if (dist < minDistance) {
      minPoint = point;
      minDistance = dist;
    }
  }

   steering = math::atan(minPoint.x, minPoint.y); //this may need to do absolute value, then set steering - or + later


}




int main(int argc, char** argv) {
    ros::init(argc, argv, "ultrasonic_demo");
    ros:: Publisher steerPub = nh.advertise<rr_platform::steering>("/steering", 1);

    auto img_sub = nh.subscribe("ultrasonic_array", 1, ultrasonicCallBack);

    rr_platform::steering steerMsg;
    steerMsg.angle = steering;
    steerMsg.header.stamp = ros::Time::now();
    steerPub.publish(steerMsg);

    ros::spin()
    return 0;
}
