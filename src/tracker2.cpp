//========================================
// 360 degree TRACKER
// TMS-TRACKER V 2.0
// Edwin Babaians , Sahand Sharifzadeh
// TUM - Germany
//========================================

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/console/parse.h>

//PCL/COMMONS
#include <pcl/common/common_headers.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

//PCL/SAMPLE
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

//PCL/TRACKER
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/tracking/impl/particle_filter.hpp>
#include <pcl/tracking/impl/particle_filter_omp.hpp>
#include <pcl/tracking/impl/tracker.hpp>
#include <pcl/tracking/impl/tracking.hpp>
#include <pcl/tracking/kld_adaptive_particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>

//PCL/VISUALIZER
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

//PCL/SEARCH
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/pcl_search.h>

//PCL/RAGEIMAGE
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image.h>

//PCL/REGISTRATION
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>

//PCL/VISUALIZATION
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

//PCL/SAMPLE
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

//PCL/SEGMENTATION
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>

//PCL/KDTREE
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/kdtree.h>

//PCL/FEATURES
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/don.h>
#include <pcl/features/normal_3d_omp.h>

//PCL/KEYPOINTS
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>

//PCL/RECOGNITION
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>

//PCL/SURFACE
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>

//PCL/FILTERS
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

//STD
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <dirent.h>
#include <limits>
#include <vector>
#include <string>
#include <vector>

//PCL/IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/io.h>

//BOOST
#include <boost/lexical_cast.hpp>

//OPENCV
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//EIGEN
#include <Eigen/Core>

//MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "std_msgs/String.h"
#include <moveit_msgs/DisplayTrajectory.h>

#include <pcl/console/print.h>
#include <pcl/registration/sample_consensus_prerejective.h>

//Global
bool is_new_stream_ready = false;
double downsampling_grid_size = 0.001; //1mm
bool is_vis_setting_done = false;
boost::mutex mutex_1;
boost::mutex mutex_2;
bool is_show_particle = true;
bool is_stream_points_ready = false;
bool is_stream_callback_busy = false;
float z_limit = 1.5; //meter
int particles = 600;
image_transport::Publisher pub_image;
ros::Publisher pub_cloud;
ros::Publisher pub_info;
ros::Publisher pub_pf_point;
pcl::PolygonMesh::Ptr mri_mesh(new pcl::PolygonMesh());
pcl::PolygonMesh::Ptr ground_truth_mesh(new pcl::PolygonMesh());

//TYPEDEF

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;


//XYZRGB
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudTC;
typedef pcl::PointXYZRGB PointTC;
typedef PointCloudTC::Ptr PointCloudTCPtr;
typedef PointCloudTC::ConstPtr PointCloudTCConstPtr;
//XYZ
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef pcl::PointXYZ PointT;
typedef PointCloudT::Ptr PointCloudTPtr;
typedef PointCloudT::ConstPtr PointCloudTConstPtr;

//Clouds
PointCloudTCPtr mri_cloud(new PointCloudTC);
PointCloudTCPtr mri_template_cloud(new PointCloudTC);

PointCloudTCPtr ground_truth_cloud(new PointCloudTC);
PointCloudTCPtr stream_cloud(new PointCloudTC);
PointCloudTCPtr stream_pre1_cloud(new PointCloudTC);
PointCloudTCPtr stream_pre2_cloud(new PointCloudTC);
PointCloudTCPtr result_cloud (new PointCloudTC);
PointCloudTCPtr result_cloud_temp (new PointCloudTC);
PointCloudTCPtr ground_truth_cloud_temp(new PointCloudTC);

PointCloudTCPtr result_cloud_GT (new PointCloudTC);
PointCloudTCPtr result_cloud_Forest (new PointCloudTC);
PointCloudTCPtr result_cloud_CLM (new PointCloudTC);
PointCloudTCPtr result_cloud_Fused (new PointCloudTC);

float forest_x = 0,forest_y = 0,forest_z = 0,forest_roll = 0,forest_pitch = 0,forest_yaw = 0;
float clm_x = 0,clm_y = 0,clm_z = 0,clm_roll = 0,clm_pitch = 0,clm_yaw = 0;

//TF
Eigen::Matrix4f ground_truth_tf = Eigen::Matrix4f::Identity();

//TRACKER
typedef pcl::tracking::ParticleXYZRPY ParticleT;
typedef pcl::tracking::ParticleFilterTracker<PointTC, ParticleT> ParticleFilter;
boost::shared_ptr<ParticleFilter> main_tracker; //KLD Adaptive PF

//CONFIG
bool is_show_ground_truth = true;
bool is_capture_mode = false;
bool is_point_cloud_shown = false;
bool is_live = false;
std::string main_template_path = "/home/edwin/outhand2.ply";
std::string main_mri_path = "/home/edwin/129.ply";
std::string camera_point_topic = "/camera/depth_registered/points";
std::string save_log_path = "/home/edwin/report/log.txt";

bool tms_only_icp = false;
bool is_first_align = true;
bool is_show_result = true;

Eigen::Affine3f transformation;
Eigen::Affine3f transformationg;
Eigen::Matrix4f result_Matrix_ICP;
Eigen::Matrix4f result_Matrix_PF;
Eigen::Matrix4f result_Matrix_RANSAC;
Eigen::Matrix4f result_Matrix_Forest;
Eigen::Matrix4f result_Matrix_GT;
Eigen::Matrix4f result_Matrix_CLM;
Eigen::Matrix4f result_Matrix;
Eigen::Matrix4f result_Matrix_kalman;

//CONFIG/BENCHMARK
int benchmark_max_people_count = 17;
int benchmark_person_index = 0;
std::string benchmark_base_path = "/home/edwin/hpdb/";

int frame_counter = 0;
int fps = 0;
int fps_counter = 0;

bool forest_ready = false;
bool clm_ready = false;

class KalmanFilter
{
    double pre_action;
    double prediction;
    double teta_main;
    double kalman_gain;
    double var_prediction;
    double var_main;
    double var_base;
    double var_follow;

public:
    KalmanFilter()
    {
        pre_action = 0;
        prediction = 0;

        teta_main = 0;
        kalman_gain = 0;
        var_prediction = 0;
        var_main = 0;
    }

    void setVars(double base,double follow)
    {
        var_base = base;
        var_follow = follow;
    }

    void KalmanUpdate(double sensor_base,double sensor_follow,double var_base,double var_follow)
    {
        double delta_action = ( sensor_follow - pre_action );
        prediction = teta_main + delta_action;
        teta_main = prediction + kalman_gain * (sensor_base - prediction);

        var_prediction = var_main + var_follow;
        var_main = var_prediction - (kalman_gain * kalman_gain) * (var_prediction + var_base);
        kalman_gain = var_prediction / (var_prediction + var_base);
        pre_action = sensor_follow;
    }

    double tick(double base,double follow)
    {
        KalmanUpdate(base, follow,var_base,var_follow);
        return teta_main;
    }

    ~KalmanFilter()
    {

    }
};

KalmanFilter kfilter1;
KalmanFilter kfilter2;
KalmanFilter kfilter3;
KalmanFilter kfilter4;
KalmanFilter kfilter5;
KalmanFilter kfilter6;
//moveit::planning_interface::MoveGroup group("arm_t");
//moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

enum TrackingModes
{
    PF,
    ICP,
    PFICP
};

TrackingModes tracking_mode = PF;

void save_log(std::string method,float rot_error,float dis_error,float x,float y,float z,float roll,float pitch,float yaw,float frame,float person,float fps)
{
    std::ofstream outfile;
    outfile.open(save_log_path.c_str(), std::ios_base::app);
    outfile << method << " " << person << " " << frame << " " << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << " " << rot_error << " " << dis_error << " " <<  fps << endl;
    outfile.close();
    ROS_INFO("SAVE DONE");
}

void printMatix4f(const Eigen::Matrix4f & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
    printf ("Translation vector :\n");
    printf ("T = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
}

void wait(int ms)
{

    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void getFeatures(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
{
    //COMPUTE NORMALS
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr treen (new pcl::search::KdTree<pcl::PointXYZRGB>);
    treen->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (treen);
    n.setKSearch (40);
    n.compute (*normals);
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);
    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    fpfh.setSearchMethod (tree);
    // Output datasets
    //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (0.05);
    // Compute the features
    fpfh.compute (*fpfhs);
}

//Draw the current particles
bool drawParticles (pcl::visualization::PCLVisualizer& viz)
{
    if ( main_tracker )
    {

        //Draw the current particles
        ParticleFilter::PointCloudStatePtr particles = main_tracker->getParticles ();
        if (particles)
        {
            //Set pointCloud with particle's points
            pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

            for (size_t i = 0; i < particles->points.size (); i++)
            {
                pcl::PointXYZ point;
                point.x = particles->points[i].x;
                point.y = particles->points[i].y;
                point.z = particles->points[i].z;
                particle_cloud->points.push_back (point);
            }

            if ( is_show_particle )
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 255, 0, 0);
                viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "particle_cloud");

                if (!viz.updatePointCloud (particle_cloud, red_color, "particle_cloud"))
                    viz.addPointCloud (particle_cloud, red_color, "particle_cloud");
            }

            //OK
            return true;
        }
        else
        {
            ROS_ERROR("No Particles");
        }


    }
    else
    {
        ROS_ERROR("No Tracker");
    }

    //No Particle
    return false;
}

bool first_estimation = false;

void do3DEstimation ()
{
	fps_counter++;

    if ( first_estimation == false )
    {
        first_estimation = true;
        kfilter1.setVars(2,0.01);
        kfilter2.setVars(2,0.01);
        kfilter3.setVars(2,0.01);
        kfilter4.setVars(2,0.01);
        kfilter5.setVars(2,0.01);
        kfilter6.setVars(2,0.01);
    }

    if ( tracking_mode == PF )
    {
        //PF
        ParticleT result = main_tracker->getResult ();
        transformation = main_tracker->toEigenMatrix (result);

        result_Matrix_PF = transformation.matrix();
        result_Matrix_PF(3,3) = 1;
        result_Matrix_PF(3,0) = 0;
        result_Matrix_PF(3,1) = 0;
        result_Matrix_PF(3,2) = 0;

        result_Matrix = result_Matrix_PF;

        float xx = result_Matrix(0,3);
        float yy = result_Matrix(1,3);
        float zz = result_Matrix(2,3);

        geometry_msgs::PointStamped msg;
        msg.point.x = xx;
        msg.point.y = yy;
        msg.point.z = zz;

        pub_pf_point.publish(msg);
    }

    if ( tracking_mode == PFICP )
    {
        //PFICP

        //PF
        ParticleT result = main_tracker->getResult ();
        transformation = main_tracker->toEigenMatrix (result);

        result_Matrix_PF = transformation.matrix();
        result_Matrix_PF(3,3) = 1;
        result_Matrix_PF(3,0) = 0;
        result_Matrix_PF(3,1) = 0;
        result_Matrix_PF(3,2) = 0;

        //ICP
        pcl::IterativeClosestPoint<PointTC, PointTC> icp;
        icp.setInputSource(result_cloud);
        icp.setInputTarget(stream_pre2_cloud);
        icp.setMaximumIterations(20);
        icp.align(*result_cloud);
        result_Matrix_ICP = icp.getFinalTransformation().cast<float>();

        result_Matrix = result_Matrix_PF * result_Matrix_ICP;
    }

    //Compute Benchmark Error "wow"
    //========================================================
    float roll , pitch , yaw;
    float rollg , pitchg , yawg;

    float x,y,z;
    float xg,yg,zg;

    transformation = result_Matrix;
    pcl::getEulerAngles(transformation,roll,pitch,yaw);
    roll  = roll  * 57.2957795;   //degree
    pitch = pitch * 57.2957795;    //degree
    yaw   = yaw   * 57.2957795;    //degree

    transformationg = ground_truth_tf;
    pcl::getEulerAngles(transformationg,rollg,pitchg,yawg);
    rollg  = rollg  * 57.2957795; //degree
    pitchg = pitchg * 57.2957795;  //degree
    yawg   = yawg   * 57.2957795;  //degree

    x = result_Matrix(0,3);
    y = result_Matrix(1,3);
    z = result_Matrix(2,3);

    xg = ground_truth_tf(0,3);
    yg = ground_truth_tf(1,3);
    zg = ground_truth_tf(2,3);

    
    ROS_INFO("==========");
    //Ground Truth
    ROS_INFO_STREAM("GTD Tracking TPY : "    << rollg << " , " << pitchg << " , " << yawg );
    //Our Method
    //ROS_INFO_STREAM("TMS Tracking TPY : "    << roll  << " , " << pitch  << " , " << yaw  );
    //Our Method
    float forest_yaw2 = 0;
    if ( forest_yaw > 0 ) forest_yaw2 = 180 - forest_yaw;
    else if ( forest_yaw < 0 ) forest_yaw2 = -1 * (180 + forest_yaw);

    ROS_INFO_STREAM("Forest Tracking TPY : " << forest_pitch   << " , " <<  forest_yaw2  << " , " << forest_roll );
    //Our Method
    ROS_INFO_STREAM("CLM Tracking TPY : "    << clm_roll  << " , " << clm_yaw   << " , " <<  clm_pitch );

    Eigen::Affine3f t1 = pcl::getTransformation(xg,yg,zg,rollg * 0.0174533,pitchg * 0.0174533,yawg * 0.0174533);
    result_Matrix_GT = t1.matrix();

    Eigen::Affine3f t2 = pcl::getTransformation(forest_x,forest_y,forest_z,forest_pitch * 0.0174533,forest_yaw2 * 0.0174533,forest_roll * 0.0174533);
    result_Matrix_Forest = t2.matrix();

    Eigen::Affine3f t3 = pcl::getTransformation(forest_x,forest_y,forest_z,clm_roll * 0.0174533,clm_yaw * 0.0174533,clm_pitch * 0.0174533);
    result_Matrix_CLM = t3.matrix();

    double kalman_a = kfilter1.tick(forest_pitch,clm_roll);
    double kalman_b = kfilter2.tick(forest_yaw2,clm_yaw);
    double kalman_c = kfilter3.tick(forest_roll,clm_pitch);

    double kalman_d = kfilter4.tick(forest_x,x);
    double kalman_e = kfilter5.tick(forest_y,y);
    double kalman_f = kfilter6.tick(forest_z,z);

    Eigen::Affine3f t4 = pcl::getTransformation(forest_x,forest_y,forest_z,kalman_a* 0.0174533,kalman_b* 0.0174533,kalman_c* 0.0174533);
    result_Matrix_kalman = t4.matrix();

    ROS_INFO_STREAM("Kalman : "    << kalman_a  << " , " << kalman_b   << " , " <<  kalman_c );
    
    //PF
    float rot_error1 = sqrt((roll-rollg)*(roll-rollg) + (pitch-pitchg)*(pitch-pitchg) + (yaw-yawg)*(yaw-yawg));
    float dist_error1 = sqrt((x-xg)*(x-xg) + (y-yg)*(y-yg) + (z-zg)*(z-zg));

    //Forest
    float rot_error2 = sqrt((forest_pitch-rollg)*(forest_pitch-rollg) + (forest_yaw2-pitchg)*(forest_yaw2-pitchg) + (forest_roll-yawg)*(forest_roll-yawg));
    float dist_error2 = sqrt((forest_x-xg)*(forest_x-xg) + (forest_y-yg)*(forest_y-yg) + (forest_z-zg)*(forest_z-zg));

    float rot_error3 = sqrt((clm_roll-rollg)*(clm_roll-rollg) + (clm_yaw-pitchg)*(clm_yaw-pitchg) + (clm_pitch-yawg)*(clm_pitch-yawg));
    float dist_error3 = sqrt((clm_x-xg)*(clm_x-xg) + (clm_y-yg)*(clm_y-yg) + (clm_z-zg)*(clm_z-zg));

    float rot_error4 = sqrt((kalman_a-rollg)*(kalman_a-rollg) + (kalman_b-pitchg)*(kalman_b-pitchg) + (kalman_c-yawg)*(kalman_c-yawg));
    float dist_error4 = sqrt((kalman_d-xg)*(kalman_d-xg) + (kalman_e-yg)*(kalman_e-yg) + (kalman_f-zg)*(kalman_f-zg));

    //Errors
    ROS_INFO_STREAM("Rotation Error PF     (deg) : " << rot_error1 <<  " , Distace Error (cm) : " << dist_error1 * 100);
    ROS_INFO_STREAM("Rotation Error Forest (deg) : " << rot_error2 <<  " , Distace Error (cm) : " << dist_error2 * 100);
    ROS_INFO_STREAM("Rotation Error CLM    (deg) : " << rot_error3 <<  " , Distace Error (cm) : " << dist_error3 * 100);
    ROS_INFO_STREAM("Rotation Error Kalman (deg) : " << rot_error4 <<  " , Distace Error (cm) : " << dist_error4 * 100);

    //void save_log(string method,float rot_error,float dis_error,float x,float y,float z,float roll,float pitch,float yaw,float frame,float person,float fps)

    if ( clm_ready && forest_ready )
    {
    clm_ready = false;
    forest_ready = false;
    //Save Log
    save_log("GT",0,0,xg,yg,zg,rollg,pitchg,yawg,frame_counter,benchmark_person_index,fps);
    save_log("PF",rot_error1,dist_error1 * 100,x,y,z,roll,pitch,yaw,frame_counter,benchmark_person_index,fps);
    save_log("Forest",rot_error2,dist_error2 * 100,forest_x,forest_y,forest_z,forest_pitch,forest_yaw2,forest_roll,frame_counter,benchmark_person_index,fps);
    save_log("CLM",rot_error3,dist_error3 * 100,clm_x,clm_y,clm_z,clm_roll,clm_yaw,clm_pitch,frame_counter,benchmark_person_index,fps);
    save_log("Kalman",rot_error4,dist_error4 * 100,kalman_d,kalman_e,kalman_f,kalman_a,kalman_b,kalman_c,frame_counter,benchmark_person_index,fps);
    }
}

void show (pcl::visualization::PCLVisualizer& viz)
{
    boost::mutex::scoped_lock lock (mutex_1);

    if ( is_vis_setting_done == false)
    {
        is_vis_setting_done = true;
        viz.setBackgroundColor(1,1,1);
        viz.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    }

    if (!is_point_cloud_shown)
    {
        is_point_cloud_shown = true;

        //TODO , REMOVE COLOR FROM DATAS
        for (size_t i = 0; i <  stream_pre2_cloud->points.size(); i++)
        {
            stream_pre2_cloud->points[i].r = 10;
            stream_pre2_cloud->points[i].g = 10;
            stream_pre2_cloud->points[i].b = 10;
        }

        if ( is_capture_mode )
        {
            is_capture_mode = false;
            std::string name = "Capture";

            if ( is_live == false)
                name += boost::lexical_cast<std::string>(benchmark_person_index) + ".ply";
            else
                name += ".ply";

            pcl::io::savePLYFile(name, *stream_pre2_cloud,false);

            ROS_INFO("Capture Saved");
        }

        //Transfor
        pcl::transformPointCloud<PointTC> (*ground_truth_cloud_temp, *ground_truth_cloud, ground_truth_tf);
        pcl::transformPointCloud<PointTC> (*mri_cloud, *result_cloud_GT, result_Matrix_GT);
        pcl::transformPointCloud<PointTC> (*mri_cloud, *result_cloud_Forest, result_Matrix_Forest);
        pcl::transformPointCloud<PointTC> (*mri_cloud, *result_cloud_CLM, result_Matrix_CLM);
        pcl::transformPointCloud<PointTC> (*mri_cloud, *result_cloud_Fused, result_Matrix_kalman);

        //Draw Stream Data
        pcl::visualization::PointCloudColorHandlerCustom<PointTC> stream_pre2_cloud_color (stream_pre2_cloud, 0, 0, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "stream_pre2_cloud");
        if (!viz.updatePointCloud (stream_pre2_cloud, "stream_pre2_cloud"))
            viz.addPointCloud (stream_pre2_cloud, stream_pre2_cloud_color,"stream_pre2_cloud");


        //Draw Result
        pcl::visualization::PointCloudColorHandlerCustom<PointTC> blue_color (result_cloud_GT, 0, 0, 255);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultcloudgt");
        if (!viz.updatePointCloud ( result_cloud_GT, blue_color, "resultcloudgt"))
            viz.addPointCloud ( result_cloud_GT, blue_color, "resultcloudgt");

        //Draw Result
        pcl::visualization::PointCloudColorHandlerCustom<PointTC> blue_red (result_cloud_Forest, 255, 0, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultcloudforest");
        if (!viz.updatePointCloud ( result_cloud_Forest, blue_red, "resultcloudforest"))
            viz.addPointCloud ( result_cloud_Forest, blue_red, "resultcloudforest");

        //Draw Result
        pcl::visualization::PointCloudColorHandlerCustom<PointTC> blue_green (result_cloud_CLM, 0, 255, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultcloudclm");
        if (!viz.updatePointCloud ( result_cloud_CLM, blue_green, "resultcloudclm"))
            viz.addPointCloud ( result_cloud_CLM, blue_green, "resultcloudclm");

        //Draw Result
        pcl::visualization::PointCloudColorHandlerCustom<PointTC> blue_pink (result_cloud_Fused, 255, 0, 255);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "resultcloudk");
        if (!viz.updatePointCloud ( result_cloud_Fused, blue_pink, "resultcloudk"))
            viz.addPointCloud ( result_cloud_Fused, blue_pink, "resultcloudk");

        //drawParticles (viz);

    }
    else
    {
        wait(1);
    }
}

int16_t* loadDepthImageCompressed( const char* fname )
{
    FILE* pFile = fopen(fname, "rb");
    if(!pFile){
        ROS_ERROR_STREAM("Could not load Depth for benchmark" << fname );
        return NULL;
    }

    int im_width = 0;
    int im_height = 0;
    bool success = true;

    success &= ( fread(&im_width,sizeof(int),1,pFile) == 1 ); // read width of depthmap
    success &= ( fread(&im_height,sizeof(int),1,pFile) == 1 ); // read height of depthmap

    int16_t* depth_img = new int16_t[im_width*im_height];

    int numempty;
    int numfull;
    int p = 0;

    while(p < im_width*im_height )
    {
        success &= ( fread( &numempty,sizeof(int),1,pFile) == 1 );

        for(int i = 0; i < numempty; i++)
            depth_img[ p + i ] = 0;

        success &= ( fread( &numfull,sizeof(int), 1, pFile) == 1 );
        success &= ( fread( &depth_img[ p + numempty ], sizeof(int16_t), numfull, pFile) == (unsigned int) numfull );
        p += numempty+numfull;
    }

    fclose(pFile);

    if(success)
        return depth_img;
    else{
        delete [] depth_img;
        return NULL;
    }
}

bool convertDepthToPointCloud(std::string path)
{
    //ROS_INFO_STREAM("Load depth from " << path);

    int16_t* depth_data = loadDepthImageCompressed(path.c_str());

    if ( !depth_data )
    {
        return false;
    }

    //ROS_INFO("Depth done");

    stream_cloud->width = 640;
    stream_cloud->height = 480;
    stream_cloud->points.resize (640*480);

    //ROS_INFO("Depth processing 10");

    register float constant = 1.0f / 525;
    register int centerX = (stream_cloud->width >> 1);
    int centerY = (stream_cloud->height >> 1);
    register int depth_idx = 0;

    //ROS_INFO("Depth processing 20");

    for (int v = -centerY; v < centerY; ++v)
    {
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
        {
            pcl::PointXYZRGB& pt = stream_cloud->points.at(depth_idx);
            pt.z = depth_data[depth_idx] * 0.001f;
            pt.x = static_cast<float> (u) * pt.z * constant;
            pt.y = static_cast<float> (v) * pt.z * constant;
        }
    }

    //ROS_INFO("Stream is ready");

    return true;
}

void filterPassThrough (const PointCloudTCConstPtr &cloud, PointCloudTC &result)
{
    //FilterPassthrough
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, z_limit);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
}

void gridSampleApprox (const PointCloudTCConstPtr &cloud, PointCloudTC &result, double leaf_size)
{
    //Voxelgrid
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_grid.filter (result);
}

void preProcessStreamPoints (const PointCloudTCConstPtr &cloud)
{
    if ( is_stream_callback_busy ) return;
    is_stream_callback_busy = true;

    boost::mutex::scoped_lock lock (mutex_2);

    stream_pre1_cloud.reset (new PointCloudTC);
    stream_pre2_cloud.reset (new PointCloudTC);

    filterPassThrough (stream_cloud, *stream_pre1_cloud);
    gridSampleApprox  (stream_pre1_cloud, *stream_pre2_cloud, downsampling_grid_size );

    main_tracker->setInputCloud (stream_pre2_cloud);
    main_tracker->compute ();

    is_point_cloud_shown = false;

    is_stream_callback_busy = false;
}

Eigen::Matrix4f readGroundTruth(std::string path)
{
    Eigen::Matrix4f transformation_matrix;

    std::string line1,line2,line3,line4;
    std::string line5;

    std::ifstream text;
    text.open(path.c_str(), std::ios_base::in);

    std::vector<std::string> strs1;
    std::vector<std::string> strs2;
    std::vector<std::string> strs3;
    std::vector<std::string> strs4;
    std::vector<std::string> strs5;

    if (text.is_open())
    {
        getline(text,line1);
        getline(text,line2);
        getline(text,line3);
        getline(text,line4);
        getline(text,line5);

        boost::split(strs1,line1,boost::is_any_of(" "));
        boost::split(strs2,line2,boost::is_any_of(" "));
        boost::split(strs3,line3,boost::is_any_of(" "));
        boost::split(strs5,line5,boost::is_any_of(" "));

        transformation_matrix(0,0) = atof(strs1.at(0).c_str());
        transformation_matrix(0,1) = atof(strs1.at(1).c_str());
        transformation_matrix(0,2) = atof(strs1.at(2).c_str());
        transformation_matrix(0,3) = atof(strs5.at(0).c_str()) / 1000;

        transformation_matrix(1,0) = atof(strs2.at(0).c_str());
        transformation_matrix(1,1) = atof(strs2.at(1).c_str());
        transformation_matrix(1,2) = atof(strs2.at(2).c_str());
        transformation_matrix(1,3) = atof(strs5.at(1).c_str()) / 1000;

        transformation_matrix(2,0) = atof(strs3.at(0).c_str());
        transformation_matrix(2,1) = atof(strs3.at(1).c_str());
        transformation_matrix(2,2) = atof(strs3.at(2).c_str());
        transformation_matrix(2,3) = atof(strs5.at(2).c_str()) / 1000;

        transformation_matrix(3,0) = 0;
        transformation_matrix(3,1) = 0;
        transformation_matrix(3,2) = 0;
        transformation_matrix(3,3) = 1;

        text.close();
    }
    else
    {
        ROS_ERROR_STREAM("Unable to open file : " << path);
        //return NULL;
    }

    return transformation_matrix;
}

void resetEngine(std::string template_model_path,std::string model_path)
{
    ROS_INFO("Reseting Engine");

    //Load Temaplate
    mri_template_cloud.reset(new PointCloudTC());
    pcl::io::loadPLYFile (template_model_path, *mri_template_cloud);

    //Load Model
    mri_cloud.reset(new PointCloudTC());
    pcl::io::loadPolygonFilePLY(model_path, *mri_mesh);
    pcl::fromPCLPointCloud2(mri_mesh->cloud, *mri_cloud);

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointTC, ParticleT> > tracker
            (new pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointTC, ParticleT> (8));

    ParticleT bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;

    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    main_tracker = tracker;
    main_tracker->setTrans (Eigen::Affine3f::Identity ());
    main_tracker->setStepNoiseCovariance (default_step_covariance);
    main_tracker->setInitialNoiseCovariance (initial_noise_covariance);
    main_tracker->setInitialNoiseMean (default_initial_mean);
    main_tracker->setIterationNum (1);
    main_tracker->setParticleNum (particles);
    main_tracker->setResampleLikelihoodThr(0.00);
    main_tracker->setUseNormal (false);

    //Setup coherence object for tracking
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTC>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTC>::Ptr
            (new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointTC> ());

    boost::shared_ptr<pcl::tracking::DistanceCoherence<PointTC> > distance_coherence
            = boost::shared_ptr<pcl::tracking::DistanceCoherence<PointTC> > (new pcl::tracking::DistanceCoherence<PointTC> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<PointTC> > search (new pcl::search::Octree<PointTC> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    main_tracker->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();

    PointCloudTCPtr transed_ref (new PointCloudTC);
    PointCloudTCPtr transed_ref_downsampled (new PointCloudTC);

    pcl::compute3DCentroid<PointTC> (*mri_template_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<PointTC> (*mri_template_cloud, *transed_ref, trans.inverse());
    //gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size);

    //set reference model and trans
    main_tracker->setReferenceCloud (transed_ref);
    main_tracker->setTrans (trans);

    ROS_INFO("Engine is ready");
}

void benchmarkLogic()
{
    if ( is_live ) return;

    ROS_INFO("Benchmark logic started");
    wait(3000);

    benchmark_person_index = 0;

    while(benchmark_person_index < benchmark_max_people_count + 1)
    {

        ROS_INFO_STREAM("Person ID : " << benchmark_person_index + 1 );

        //Load ground truth cloudforest_x
        std::string name_ply = benchmark_base_path + "models/";
        if ( benchmark_person_index + 1 < 10 )
            name_ply += "0" + boost::lexical_cast<std::string>(benchmark_person_index + 1) + ".ply";
        else
            name_ply +=  boost::lexical_cast<std::string>(benchmark_person_index + 1) + ".ply";

        std::string name_template = benchmark_base_path + "templates/";
        if ( benchmark_person_index + 1 < 10 )
            name_template += "0" + boost::lexical_cast<std::string>(benchmark_person_index + 1) + ".ply";
        else
            name_template +=  boost::lexical_cast<std::string>(benchmark_person_index + 1) + ".ply";

        ground_truth_cloud_temp.reset(new PointCloudTC);

        //Load the cloud as temp (we gone rotate it with gt transform later)
        //ROS_INFO_STREAM("Load ground truth ply " << name_ply);
        pcl::io::loadPolygonFilePLY(name_ply, *ground_truth_mesh);
        pcl::fromPCLPointCloud2(ground_truth_mesh->cloud, *ground_truth_cloud_temp);

        resetEngine(name_template,name_ply);
        //ROS_INFO("PLY Done");

        for ( int frame = 3 ; frame < 500 ; frame++)
        {
        	frame_counter = frame;

            std::string name = "";
            if ( benchmark_person_index + 1 < 10 )
                name = benchmark_base_path  + "0" + boost::lexical_cast<std::string>(benchmark_person_index + 1) + "/";
            else
                name = benchmark_base_path  +       boost::lexical_cast<std::string>(benchmark_person_index + 1) + "/";

            if ( frame < 10                 ) name  = name + "frame_0000" + boost::lexical_cast<std::string>( frame );
            if ( frame < 100 && frame >= 10 ) name  = name + "frame_000"  + boost::lexical_cast<std::string>( frame );
            if ( frame >= 100               ) name  = name + "frame_00"   + boost::lexical_cast<std::string>( frame );

            std::string name_color = name + "_rgb.png";
            std::string name_depth = name + "_depth.bin";
            std::string name_gt    = name + "_pose.txt";

            //Load RGB
            cv::Mat image;
            image = cv::imread(name_color.c_str(),CV_LOAD_IMAGE_COLOR);

            if(!image.data )
            {
                ROS_ERROR_STREAM("Could not load RGB for benchmark : " << name_color);
                continue;
            }

            //Load Depth Points
            bool result = convertDepthToPointCloud(name_depth);

            if ( result == false )
            {
                continue;
            }

            //ROS_INFO_STREAM("Load ground truth TF " << name_gt);
            ground_truth_tf = readGroundTruth(name_gt);

            sensor_msgs::PointCloud2 msgc;
            pcl::toROSMsg(*stream_cloud,msgc);

            msgc.header.frame_id = "camera_link";
            msgc.header.stamp = ros::Time();

            pub_cloud.publish(msgc);

            //Process
            preProcessStreamPoints(stream_cloud);

            do3DEstimation ();

            sensor_msgs::CameraInfo info;
            info.header.stamp = ros::Time::now();
            info.header.frame_id = "camera_rgb_optical_frame";
            info.height = 480;
            info.width = 640;
            info.K[0] = 525;
            info.K[1] = 0;
            info.K[2] = 319.5;
            info.K[3] = 0;
            info.K[4] = 525;
            info.K[5] = 239.5;
            info.K[6] = 0;
            info.K[7] = 0;
            info.K[8] = 1;

            info.P[0] = 525;
            info.P[1] = 0;
            info.P[2] = 319.5;
            info.P[3] = 0;
            info.P[4] = 0;
            info.P[5] = 525;
            info.P[6] = 239.5;
            info.P[7] = 0;
            info.P[8] = 0;
            info.P[9] = 0;
            info.P[10] = 1;
            info.P[11] = 0;

            info.R[0] = 1;
            info.R[1] = 0;
            info.R[2] = 0;
            info.R[3] = 0;
            info.R[4] = 1;
            info.R[5] = 0;
            info.R[6] = 0;
            info.R[7] = 0;
            info.R[8] = 1;

            info.D.push_back(0);
            info.D.push_back(0);
            info.D.push_back(0);
            info.D.push_back(0);
            info.D.push_back(0);

            info.distortion_model = "plumb_bob";

            pub_info.publish(info);

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(info.header, "bgr8", image).toImageMsg();
            pub_image.publish(msg);

            //cv::namedWindow( "RGB", cv::WINDOW_AUTOSIZE );   // Create a window for display.
            //cv::imshow("RGB,image);                      // Show our image inside it.
            //cv::waitKey(1);                                  // Wait for a keystroke in the window
            
            wait(20); //25 Target FPS
        }

        ROS_INFO_STREAM("Done");
        benchmark_person_index++;
    }

    ROS_INFO_STREAM("Benchmark logic done");
}

void callbackDepthPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
{
    if ( !is_live ) return; //Ignore data if we are in benchmark mode

    if (cloud_m->width != 0 )
    {
        pcl::fromROSMsg(*cloud_m, *stream_cloud);

        preProcessStreamPoints(stream_cloud);

        is_stream_points_ready = true;
    }
}


void callbackForest(const geometry_msgs::PoseStampedConstPtr &pose)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose->pose.orientation, quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    forest_x = pose->pose.position.x;
    forest_y = pose->pose.position.y;
    forest_z = pose->pose.position.z;

    forest_roll = roll * 57.2957795;
    forest_pitch = pitch * 57.2957795;
    forest_yaw = yaw * 57.2957795;

    forest_ready = true;
}


void callbackCLM(const geometry_msgs::PoseStampedConstPtr &pose)
{
    //tf::Quaternion quat;
    //tf::quaternionMsgToTF(pose->pose.orientation, quat);

    //double roll, pitch, yaw;
    //tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    clm_x = pose->pose.position.x;
    clm_y = pose->pose.position.y;
    clm_z = pose->pose.position.z;

    clm_roll = pose->pose.orientation.x;
    clm_pitch = pose->pose.orientation.y;
    clm_yaw = pose->pose.orientation.z;

    clm_ready = true;
}

void mainLogic()
{
    ROS_INFO("Main logic started");
    while(true)
    {
       wait(1000);
       fps = fps_counter;
       fps_counter = 0;
    }
   
}

void kukaLogic()
{
    ROS_INFO("KUKA logic started");
    wait(1000);
}

void doFeatureRigidRegistration()
{
    PointCloudNT::Ptr  object (new PointCloudNT);
    PointCloudNT::Ptr  object_aligned (new PointCloudNT);
    PointCloudNT::Ptr  scene (new PointCloudNT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    pcl::console::print_highlight ("Loading point clouds...\n");
    if ( pcl::io::loadPCDFile<PointNT> ("/home/edwin/obj.pcd", *object) < 0 || pcl::io::loadPCDFile<PointNT> ("/home/edwin/scene.pcd", *scene) < 0 )
    {
        pcl::console::print_error ("Error loading object/scene file!\n");
        return;
    }

    // Downsample
    pcl::console::print_highlight ("Downsampling...\n");
    pcl::VoxelGrid<PointNT> grid;
    const float leaf = 0.005f;
    grid.setLeafSize (leaf, leaf, leaf);
    grid.setInputCloud (object);
    grid.filter (*object);
    grid.setInputCloud (scene);
    grid.filter (*scene);

    // Estimate normals for scene
    pcl::console::print_highlight ("Estimating scene normals...\n");
    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.01);
    nest.setInputCloud (scene);
    nest.compute (*scene);

    // Estimate features
    pcl::console::print_highlight ("Estimating features...\n");
    FeatureEstimationT fest;
    fest.setRadiusSearch (0.025);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    // Perform alignment
    pcl::console::print_highlight ("Starting alignment...\n");
    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
    {
        pcl::ScopeTime t("Alignment");
        align.align (*object_aligned);
    }

    if (align.hasConverged ())
    {
        // Print results
        printf ("\n");
        Eigen::Matrix4f transformation = align.getFinalTransformation ();
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
        pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
        pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
        pcl::console::print_info ("\n");
        pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

        // Show alignment
        pcl::visualization::PCLVisualizer visu("Alignment");
        visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
        visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
        visu.spin ();
    }
    else
    {
        pcl::console::print_error ("Alignment failed!\n");
        return;
    }
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "tms_node");
    ros::Time::init();

    ROS_INFO("TMS Started");
    ROS_INFO_STREAM("Is live : " << is_live);

    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("TMS Tracking Viewer");
    viewer_->runOnVisualizationThread (boost::bind(&show, _1), "viz_cb");

    ros::Rate loop_rate(20); //50 Hz

    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe(camera_point_topic, 1, callbackDepthPoints);

    image_transport::ImageTransport it(nh);
    pub_image = it.advertise("/tms/color_image", 1);
    pub_info = nh.advertise<sensor_msgs::CameraInfo>("/tms/camera_info", 1);
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/tms/depth_registered/points", 1);
    pub_pf_point = nh.advertise<geometry_msgs::PointStamped>("/tms/pf_point",1);

    ros::Subscriber sub_forest_pose = nh.subscribe<geometry_msgs::PoseStamped>("head_pose",1,callbackForest);
    ros::Subscriber sub_clm_pose = nh.subscribe<geometry_msgs::PoseStamped>("clm_head_pose",1,callbackCLM);

    boost::thread thread_logic1(&mainLogic);
    boost::thread thread_logic2(&benchmarkLogic);
    boost::thread thread_logic3(&kukaLogic);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("TMS Terminated");
}

