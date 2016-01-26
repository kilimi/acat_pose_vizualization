
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
#include <exception>

using namespace std;

typedef pose_estimation::PoseEstimation MsgT;

typedef pcl::PointXYZRGBA PointT;
cv::Point3f getColor(int number);

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pose estimation result"));

bool once = false;
std::vector<int> rotorcaps_idx;

using namespace tf;
using namespace pcl_ros;


void displayPose(MsgT::Response out){
//    ROS_INFO_STREAM("rotorcaps_idx.size():" <<  rotorcaps_idx.size());
    for (size_t j = 0; j < rotorcaps_idx.size(); j++){
        std::stringstream object_name;
        object_name << j;
//        ROS_INFO_STREAM("before remove" <<  j);
        viewer->removePointCloud(object_name.str());
        viewer->removeText3D(object_name.str());
//        ROS_INFO_STREAM("after remove" <<  object_name.str());
    }
    rotorcaps_idx.clear();

    ROS_INFO_STREAM("scene loading..." );
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>());

     
    //    pcl::io::loadPCDFile<PointT> ("/home/acat2/catkin_ws/src/aau_workcell/data/stereo_PC_binary.pcd", *scene);
   // pcl::io::loadPCDFile<PointT> ("/home/acat2/catkin_ws/src/aau_workcell/data/carmine_PC_binary.pcd", *scene);

    //pcl::fromROSMsg(out.scene, *scene);
    int v1(0);

   

    pcl::PointCloud<PointT>::Ptr object (new pcl::PointCloud<PointT>());

    pcl::fromROSMsg(out.object, *object);
    std::cout << "OBJECT SIZE: " << object->size() << std::endl;

    if (object->size() == 1029) //it is axle_cad
	pcl::io::loadPCDFile<PointT> ("/home/acat2/catkin_ws/src/aau_workcell/data/stereo_PC_binary.pcd", *scene);
    else  pcl::io::loadPCDFile<PointT> ("/home/acat2/catkin_ws/src/aau_workcell/data/carmine_PC_binary.pcd", *scene);


    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(scene);
    viewer->updatePointCloud<PointT> (scene, rgb, "Scene");
    ROS_INFO_STREAM("scene done..." );


    ROS_INFO_STREAM("out.pose_value.size(): " << out.pose_value.size());
    //int startPoseIdx = out.poses.size()/2;

    for (int i = 0; i < out.pose_value.size(); i++){
        std::stringstream name, object_name;
        name << i  << ", pose goodness: " << out.pose_value[i];
        pcl::PointCloud<PointT>::Ptr final(new pcl::PointCloud<PointT>());

        tf::Transform transform;
        tf::transformMsgToTF(out.vizualizerPoses[i], transform);

        Eigen::Matrix4f m_init, m;
        transformAsMatrix(transform, m_init);

        m_init(12) = m_init(12)/1000;
        m_init(13) = m_init(13)/1000;
        m_init(14) = m_init(14)/1000;

        m = m_init;
        
        object_name << i;
        pcl::transformPointCloud(*object, *final, m);
        cv::Point3f color = getColor(i);

        viewer->addPointCloud<PointT> (final, object_name.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.x, color.y, color.z, object_name.str());
        viewer->addText(name.str(), 0, 0 + (i)*30, color.x, color.y, color.z, object_name.str());
        rotorcaps_idx.push_back(i);
    }

}


cv::Point3f getColor(int number){
    switch (number){
    case 0:
        return cv::Point3f(0,1,0.63);
        break;
    case 1:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 2:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 3:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 4:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 5:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 6:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 7:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 8:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 9:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 10:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 11:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 12:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 13:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 14:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 15:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 16:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 17:
        return cv::Point3f(0,0.4,0.251);
        break;
    case 18:
        return cv::Point3f(0.1,0.2,0.3);
        break;
    case 19:
        return cv::Point3f(0.4,0.5,0.6);
        break;
    case 20:
        return cv::Point3f(0.7,0,0.8);
        break;
    }
    return cv::Point3f(0.5,1,1);
}

//////
ros::ServiceClient global;
void poseCallback(MsgT::Response msg)
{
//    ROS_INFO_STREAM("I am displaying!!\n");
    displayPose(msg);
    viewer->spinOnce (10);
} 

int main(int argc, char **argv){
    ros::init(argc, argv, "pose_vizualizer");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/inSceneDetector/vizualize", 10, poseCallback);

    try{

        if (!once){
            pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

            pcl::io::loadPCDFile<PointT> ("/home/acat2/catkin_ws/src/aau_workcell/data/stereo_PC_binary.pcd", *cloud);

            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);

            viewer->setPosition(0, 100);
            viewer->setSize(600, 600);
            viewer->addPointCloud<PointT> (cloud, rgb, "Scene", 0);
            viewer->setCameraPosition(0,0,-0.5, 0,0,1, 0,-1,0);
            once = true;

        }
    } catch (exception& e){
        cout << "Standard exception: " << e.what() << endl;
    }

    ros::spin();

    return 0;
}










