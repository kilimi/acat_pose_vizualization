
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
bool once = false;
ros::ServiceClient global;


bool printResults(MsgT msgglobal){
       // if (msgglobal.response.pose_value.size() == 0) return false;
	std::cout << "number of ints:" << msgglobal.response.labels_int.size() << std::endl;
	std::cout << "number of pose_value:" << msgglobal.response.pose_value.size() << std::endl;
	for(size_t n = 0; n < msgglobal.response.pose_value.size(); n++) {
		pcl::console::print_warn("Pose goodness value: %f\n", msgglobal.response.pose_value[n]);
		pcl::console::print_warn("Pose:\n");
		std::cout << msgglobal.response.poses[n] << std::endl;
	}
}
int main(int argc, char **argv){
    ros::init(argc, argv, "run_pose_estimation_for_scene_screenshot");

    ros::NodeHandle nodeHandle = ros::NodeHandle("~");
    global = nodeHandle.serviceClient<MsgT>("/inSceneDetector/detect");

    if (!once){


        //get screen shot pose
        ROS_INFO("Subscribing to detection service for screen shot pose...");
        ros::service::waitForService("/inSceneDetector/detect");
        MsgT msgglobal01;
        msgglobal01.request.scenario = "detect_screen_shot";

            if(global.call(msgglobal01)) {
			//printResults(msgglobal01);
                    pcl::console::print_value("Screenshot detected!\n");
            } else pcl::console::print_error("Screenshot not detected!\n");

	once = true;
    }
    ros::spinOnce();

return 0;
}









