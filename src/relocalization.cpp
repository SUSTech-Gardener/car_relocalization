// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <math.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include<pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include "livox_ros_driver/CustomMsg.h"


typedef pcl::PointXYZI PointType;

int ndtNum = 0;

double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;

bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;

bool firstRelocalization = false;
bool rotation_flag = false;

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurround_corner(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_relo(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_relo(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudALLFromMap_relo(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap_relo_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap_relo_down(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr ndt_input_cloud(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr ndt_output_cloud(new pcl::PointCloud<PointType>);

std::vector< Eigen::Matrix4f > pose_map;
std::vector< Eigen::Matrix4f > add_pose_map;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes2(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudFullResColor_pcd(new pcl::PointCloud<pcl::PointXYZRGB>());

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

float transformTobeMapped[6] = {0};
float transformAftMapped[6] = {0};

// sub map for initial relocalization
pcl::PointCloud<PointType>::Ptr sub_corner_map(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr sub_surf_map(new pcl::PointCloud<PointType>());
//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtree_subcorner(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtree_subsurf(new pcl::KdTreeFLANN<PointType>());

pcl::PointCloud<PointType>::Ptr subCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr subcoeffSel(new pcl::PointCloud<PointType>());

int first_relocalized_flag;
ros::Publisher publishPointcloud,publishImu;

double rad2deg(double radians)
{
  return radians * 180.0 / M_PI;
}
double deg2rad(double degrees)
{
  return degrees * M_PI / 180.0;
}
void transformUpdate()
{
    for (int i = 0; i < 6; i++) {
        transformAftMapped[i] = transformTobeMapped[i];
    }
}
//lidar coordinate sys to world coordinate sys
void pointAssociateToMap(PointType const * const pi, PointType * const po)
{
    //rot z（transformTobeMapped[2]）
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    //rot x（transformTobeMapped[0]）
    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    //rot y（transformTobeMapped[1]）then add trans
    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    po->intensity = pi->intensity;
}

void RGBpointAssociateToMap(PointType const * const pi, pcl::PointXYZRGB * const po)
{
    float x1 = cos(transformTobeMapped[2]) * pi->x
            - sin(transformTobeMapped[2]) * pi->y;
    float y1 = sin(transformTobeMapped[2]) * pi->x
            + cos(transformTobeMapped[2]) * pi->y;
    float z1 = pi->z;

    float x2 = x1;
    float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
    float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

    po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
            + transformTobeMapped[3];
    po->y = y2 + transformTobeMapped[4];
    po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
            + transformTobeMapped[5];
    //po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - std::floor(intensity);

    int reflection_map = intensity*10000;

    if (reflection_map < 30)
    {
        int green = (reflection_map * 255 / 30);
        po->r = 0;
        po->g = green & 0xff;
        po->b = 0xff;
    }
    else if (reflection_map < 90)
    {
        int blue = (((90 - reflection_map) * 255) / 60);
        po->r = 0x0;
        po->g = 0xff;
        po->b = blue & 0xff;
    }
    else if (reflection_map < 150)
    {
        int red = ((reflection_map-90) * 255 / 60);
        po->r = red & 0xff;
        po->g = 0xff;
        po->b = 0x0;
    }
    else
    {
        int green = (((255-reflection_map) * 255) / (255-150));
        po->r = 0xff;
        po->g = green & 0xff;
        po->b = 0;
    }
}

void laserCloudHandler(const livox_ros_driver::CustomMsg::ConstPtr &laserCloudPointNow)
{
    

    livox_ros_driver::CustomMsg pubLivoxMsg = *laserCloudPointNow;

    if(first_relocalized_flag)
    {
        publishPointcloud.publish(pubLivoxMsg);
    }
    else
    {
        timeLaserCloudCornerLast = laserCloudPointNow->header.stamp.toSec();

        // CustomMsg 2 pointcloud2
        pcl::PointCloud<PointType> pcl_point; 
        for(int i = 0;i < laserCloudPointNow->point_num;++i)
        {
            PointType pt;
            pt.x = laserCloudPointNow->points[i].x;
            pt.y = laserCloudPointNow->points[i].y;
            pt.z = laserCloudPointNow->points[i].z;
            pt.intensity = laserCloudPointNow->points[i].reflectivity;
            pcl_point.push_back(pt);
        }

        laserCloudCornerLast->clear();
        laserCloudCornerLast = pcl_point.makeShared();
        pcl_point.clear();
        // pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

        pcl::VoxelGrid<PointType> downSizeFilterMap;
        downSizeFilterMap.setLeafSize(0.3, 0.3, 0.3);
        downSizeFilterMap.setInputCloud(laserCloudCornerLast);
        downSizeFilterMap.filter(*laserCloudSurfLast_down);

        newLaserCloudCornerLast = true;
    }
}

void ImuHandler(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    // timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

    // laserCloudSurfLast->clear();
    // pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

    // newLaserCloudSurfLast = true;

    sensor_msgs::Imu pubLivoxImu = *msg_in;

    if(first_relocalized_flag)
    {
        publishImu.publish(pubLivoxImu);
    }
}

void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    laserCloudFullResColor->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

    newLaserCloudFullRes = true;
}

//use icp/ndt to match
double initial_first_pose()
{
    Eigen::Matrix4f initial_pose;
    double min_score = 100;

    std::vector<double> score_judge;

    // // add rotation initial guess
    // Eigen::Matrix4f rotation_pose;
    // rotation_pose << -1,0,0,0,
    //                  0,-1,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // Eigen::Matrix4f rotation_pose1;
    // rotation_pose1 << 0,-1,0,0,
    //                  1,0,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // Eigen::Matrix4f rotation_pose2;
    // rotation_pose2 << 0,1,0,0,
    //                  -1,0,0,0,
    //                  0,0, 1,0,
    //                  0,0,0, 1;
    // for(auto kf : pose_map){
        
    //     Eigen::Matrix4f temp_kf = kf * rotation_pose;
    //     Eigen::Matrix4f temp_kf1 = kf * rotation_pose1;
    //     Eigen::Matrix4f temp_kf2 = kf * rotation_pose2;

    //     add_pose_map.push_back(kf);
    //     add_pose_map.push_back(temp_kf);
    //     add_pose_map.push_back(temp_kf1);
    //     add_pose_map.push_back(temp_kf2);
    // }

    //find the best kf pose
    for(int i = 0;i < pose_map.size();++i){
        Eigen::Matrix4f kf = pose_map[i];
        std::cout << "find first localization process : " << i << "/" << pose_map.size() << std::endl;
        //pcl::NormalDistributionsTransform<PointType,PointType> ndt;
        pcl::IterativeClosestPoint<PointType,PointType> ndt;

        //ndt_input_cloud = laserCloudSurfStack2;
        // ndt.setTransformationEpsilon(0.01);
        // ndt.setResolution(1);
        // ndt.setStepSize(0.1);
        ndt.setMaximumIterations(20); 

        //ndt.setInputCloud(ndt_input_cloud);
        ndt.setInputSource(ndt_input_cloud);
        ndt.setInputTarget(laserCloudCornerFromMap_relo_down);

        Eigen::Matrix4f init_guess;
        init_guess = kf;

        ndt.align(*ndt_output_cloud, init_guess);

        std::cout << "init_guess KF : " << std::endl << init_guess << std::endl;
        std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
        << " score: " << ndt.getFitnessScore() << std::endl;

        double score = ndt.getFitnessScore();

        if(score < min_score) {
            min_score = score;
            initial_pose = init_guess;
        }   

        if(score_judge.size() <= 3)
        {
            score_judge.push_back(score);
        }
        else
        {
            score_judge.erase(score_judge.begin());
            score_judge.push_back(score);
        }

        if(
            score_judge[1] - score_judge[0] > 0.1 &&
            score_judge[2] - score_judge[1] > 0.1
            )
        {
            break;
        }
    }

    //match and transform
    pcl::IterativeClosestPoint<PointType,PointType> ndt_final;

    ndt_final.setMaximumIterations(100); 
    ndt_final.setInputSource(ndt_input_cloud);
    ndt_final.setInputTarget(laserCloudCornerFromMap_relo_down);

    std::cout<<" DEBUG initial_pose: "<< initial_pose << "min_score: "<<min_score << std::endl;

    ndt_final.align(*ndt_output_cloud, initial_pose);
    std::cout << "Normal Distributions Transform has converged:" << ndt_final.hasConverged()
    << " score: " << ndt_final.getFitnessScore() << std::endl;

    double temp_score = ndt_final.getFitnessScore();

    pcl::transformPointCloud(*ndt_input_cloud, *ndt_output_cloud, ndt_final.getFinalTransformation());

    std::cout<<" DEBUG : "<< ndt_final.getFinalTransformation() << std::endl;

    Eigen::Matrix4f relocal_pose = ndt_final.getFinalTransformation();

    // ----------ndt end-----------

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;  //创建ICP的实例类
    icp.setInputSource(ndt_output_cloud);
    icp.setInputTarget(laserCloudCornerFromMap_relo_down);
    icp.setMaxCorrespondenceDistance(100);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100);   
    icp.align(*ndt_output_cloud);
    relocal_pose = icp.getFinalTransformation();

    Eigen::Matrix3f relocal_R = relocal_pose.block<3,3>(0,0);
    Eigen::Quaternionf relocal_Q(relocal_R);
    Eigen::Vector3f relocal_tt = relocal_pose.block<3,1>(0,3);
    Eigen::Vector3f euler_angles = relocal_R.eulerAngles(2,0,1);

    std::cout<<" DEBUG relocal_pose: "<< relocal_pose << std::endl;

    transformTobeMapped[0] = euler_angles(0);
    transformTobeMapped[1] = euler_angles(1);
    transformTobeMapped[2] = euler_angles(2);
    transformTobeMapped[3] = relocal_tt(0);
    transformTobeMapped[4] = relocal_tt(1);
    transformTobeMapped[5] = relocal_tt(2);

    transformAftMapped[0] = euler_angles(0);
    transformAftMapped[1] = euler_angles(1);
    transformAftMapped[2] = euler_angles(2);
    transformAftMapped[3] = relocal_tt(0);
    transformAftMapped[4] = relocal_tt(1);
    transformAftMapped[5] = relocal_tt(2);

    std::cout<<"DEBUG ICP END-----------------------------------------"<<std::endl;
    ndt_input_cloud->clear();

    return temp_score;
}
// void rotation_mapping() //TODO
// {
// }
int main(int argc, char** argv)
{
    ros::init(argc, argv, "relocalization_node");
    ros::NodeHandle nh;

    ros::param::get("first_relocalized_flag",first_relocalized_flag);

    // 接受lidar
    ros::Subscriber subLaserCloudCornerLast = nh.subscribe<livox_ros_driver::CustomMsg>
            ("/livox/lidar", 2, laserCloudHandler);

    // 接受imu
    ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::Imu>
            ("/livox/imu", 2, ImuHandler);

    publishPointcloud = nh.advertise<livox_ros_driver::CustomMsg>
        ("/current_pointcloud", 1);

    publishImu = nh.advertise<sensor_msgs::Imu>
        ("/current_imu", 1);

    // ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>
    //         ("/livox_cloud", 2, laserCloudFullResHandler);

    // ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/laser_cloud_surround", 1);
    // ros::Publisher pubLaserCloudSurround_corner = nh.advertise<sensor_msgs::PointCloud2>
    //         ("/laser_cloud_surround_corner", 1);

    ros::Publisher pubNDTCloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/ndt_out_put", 1);
    ros::Publisher puboldmap_allpoints = nh.advertise<sensor_msgs::PointCloud2>
            ("/oldmap_allpoints", 1);
    ros::Publisher puboldmap_corner = nh.advertise<sensor_msgs::PointCloud2>
            ("/oldmap_corner", 1);
    ros::Publisher puboldmap_surf = nh.advertise<sensor_msgs::PointCloud2>
           ("/oldmap_surf", 1); 

    // ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 1);

    // nav_msgs::Odometry odomAftMapped;
    // odomAftMapped.header.frame_id = "/camera_init";
    // odomAftMapped.child_frame_id = "/aft_mapped";

    ros::Publisher pubRelocalPose = nh.advertise<nav_msgs::Odometry> ("/re_local_pose", 1);
    nav_msgs::Odometry RelocalPose;
    RelocalPose.header.frame_id = "/camera_init";
    RelocalPose.child_frame_id = "/aft_mapped";

    // ros::Publisher pubOldMapPose = nh.advertise<nav_msgs::Odometry> ("/old_map_pose", 1);
    // nav_msgs::Odometry OldMapPose;
    // OldMapPose.header.frame_id = "/camera_init";
    // OldMapPose.child_frame_id = "/aft_mapped";

    // tf::TransformBroadcaster tfBroadcaster;
    // tf::StampedTransform aftMappedTrans;
    // aftMappedTrans.frame_id_ = "/camera_init";
    // aftMappedTrans.child_frame_id_ = "/aft_mapped";

    // ros::Publisher pubOdomInit = nh.advertise<nav_msgs::Odometry> ("/mapped_for_odo_init", 1);
    // nav_msgs::Odometry odomInit;

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_odom_path", 100);

    nav_msgs::Path laserPath;

    std::string map_file_path;
    ros::param::get("map_file_path",map_file_path);
    // bool use_map_update;
    // ros::param::get("~use_map_update",use_map_update);

    //------------------------------------load relo map -------------------------------
    if(pcl::io::loadPCDFile<PointType>
             (map_file_path + "/corner.pcd",*laserCloudCornerFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file corner_map.pcd\n");
        return(-1);
    }
    else
    {
        pcl::VoxelGrid<PointType> downSizeMap;
        downSizeMap.setLeafSize(1.0,1.0,1.0);
        downSizeMap.setInputCloud(laserCloudCornerFromMap_relo);
        downSizeMap.filter(*laserCloudCornerFromMap_relo_down);
        
    }
    if(pcl::io::loadPCDFile<PointType>
             (map_file_path + "/surf.pcd",*laserCloudSurfFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file surf_map.pcd\n");
        return(-1);
    }
    if(pcl::io::loadPCDFile<PointType>
             (map_file_path + "/all_points.pcd",*laserCloudALLFromMap_relo) == -1){
        PCL_ERROR("Couldn't read file all_points_map.pcd\n");
        return(-1);
    }    
    std::ifstream kf_map(map_file_path + "/key_frame.txt");
    
    if(kf_map){
        while(kf_map){
            Eigen::Quaternionf q;
            Eigen::Vector3f t;
            kf_map >> q.x() >> q.y() >> q.z() >> q.w()
                   >> t(0) >> t(1) >> t(2);

            Eigen::Matrix3f R = q.matrix();
            Eigen::Matrix4f T;
            T << R(0,0) , R(0,1) , R(0,2) , t(0),
                 R(1,0) , R(1,1) , R(1,2) , t(1),
                 R(2,0) , R(2,1) , R(2,2) , t(2),
                 0 , 0 , 0 , 1;
            // 关键帧地图
            pose_map.push_back(T);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header.frame_id = "/camera_init";
            laserPose.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);

            laserPose.pose.orientation.x = q.x();
            laserPose.pose.orientation.y = q.y();
            laserPose.pose.orientation.z = q.z();
            laserPose.pose.orientation.w = q.w();
            laserPose.pose.position.x = t(0);
            laserPose.pose.position.y = t(1);
            laserPose.pose.position.z = t(2);

            laserPath.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
            laserPath.poses.push_back(laserPose);
            laserPath.header.frame_id = "/camera_init";
            pubLaserPath.publish(laserPath);

        }
    }
    else{
        std::cout<<"NO key_frame_map.txt !"<<std::endl;
    }

    sensor_msgs::PointCloud2 oldmap_allpoints;
    pcl::toROSMsg(*laserCloudALLFromMap_relo, oldmap_allpoints);
    oldmap_allpoints.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_allpoints.header.frame_id = "/camera_init";
    puboldmap_allpoints.publish(oldmap_allpoints);

    sensor_msgs::PointCloud2 oldmap_corner;
    pcl::toROSMsg(*laserCloudCornerFromMap_relo, oldmap_corner);
    oldmap_corner.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_corner.header.frame_id = "/camera_init";
    puboldmap_corner.publish(oldmap_corner);

    sensor_msgs::PointCloud2 oldmap_surf;
    pcl::toROSMsg(*laserCloudSurfFromMap_relo, oldmap_surf);
    oldmap_surf.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
    oldmap_surf.header.frame_id = "/camera_init";
    puboldmap_surf.publish(oldmap_surf);


    int frameCount = 0;

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        if (newLaserCloudCornerLast 
                // && newLaserCloudSurfLast && newLaserCloudFullRes &&
                // fabs(timeLaserCloudSurfLast - timeLaserCloudCornerLast) < 0.005 &&
                // fabs(timeLaserCloudFullRes - timeLaserCloudCornerLast) < 0.005
            ) {

            newLaserCloudCornerLast = false;
            // newLaserCloudSurfLast = false;
            // newLaserCloudFullRes = false;

            frameCount++;
            //initial first pose use 10 frames
            if(!firstRelocalization){
                *ndt_input_cloud += *laserCloudSurfLast_down;
                if(frameCount >= 10){

                    double score = initial_first_pose();
                    
                    sensor_msgs::PointCloud2 ndt_cloud;
                    pcl::toROSMsg(*ndt_output_cloud, ndt_cloud);
                    ndt_cloud.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
                    ndt_cloud.header.frame_id = "/camera_init";
                    pubNDTCloud.publish(ndt_cloud);


                    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                        (transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]);
    
                    RelocalPose.header.stamp = ros::Time().fromSec(timeLaserCloudCornerLast);
                    RelocalPose.pose.pose.orientation.x = geoQuat.y;
                    RelocalPose.pose.pose.orientation.y = geoQuat.z;
                    RelocalPose.pose.pose.orientation.z = geoQuat.x;
                    RelocalPose.pose.pose.orientation.w = geoQuat.w; 
                    RelocalPose.pose.pose.position.x = transformTobeMapped[3];
                    RelocalPose.pose.pose.position.y = transformTobeMapped[4];
                    RelocalPose.pose.pose.position.z = transformTobeMapped[5];
                    pubRelocalPose.publish(RelocalPose);

                    first_relocalized_flag = 1;
                    std::cout << "first localization success ! " << std::endl;

                    nh.setParam("first_relocalizated",1);

                    if(true){

                    }
                    else{
                        rotation_flag = true;
                        ndt_input_cloud->clear();
                        continue;
                    }


                }else{
                    continue;
                }
            }
        }

        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

