
#include <ros/package.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pose_estimation/PoseEstimation.h>
#include "cv.h"
typedef pose_estimation::PoseEstimation MsgT;

using namespace tf;
using namespace pcl_ros;

ros::ServiceClient global;

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_test");
    ros::NodeHandle n;

  //  if (!once){


        //get screen shot pose
        ROS_INFO("Subscribing to detection service for screen shot pose...");
        ros::service::waitForService("/inSceneDetector/detect");
        MsgT msgglobal01;
        msgglobal01.request.scenario = "detect_screen_shot";

            if(global.call(msgglobal01)) {
                    pcl::console::print_value("Screenshot detected!\n");
            } else pcl::console::print_error("Screenshot not detected!\n");

    //}
    //ros::spin();

return 0;
}









