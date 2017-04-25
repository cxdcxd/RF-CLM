// PF-CPD TRACKER

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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
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
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

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
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

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
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>
#include <dirent.h>
#include <limits>
#include <vector>

//PCL/IO
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/io.h>

//VTK
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>

//BOOST
#include <boost/lexical_cast.hpp>

//OPENCV
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

//EIGEN
#include <Eigen/Core>

using namespace cv;

//Global
bool is_stream_points_ready = false;
bool is_stream_callback_busy = false;
float z_limit = 1.5; //meter
int particles = 600;
image_transport::Publisher pub_image;
pcl::PolygonMesh::Ptr mri_mesh(new pcl::PolygonMesh());
pcl::PolygonMesh::Ptr ground_truth_mesh(new pcl::PolygonMesh());

//TYPEDEF

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
PointCloudTCPtr ground_truth_cloud(new PointCloudTC);
PointCloudTCPtr stream_cloud(new PointCloudTC);

//TF
Eigen::Matrix4f ground_truth_tf = Eigen::Matrix4f::Identity();

//CONFIG
bool is_live = false;
std::string main_template_path = "/home/edwin/outhand2.ply";
std::string main_mri_path = "/home/edwin/129.ply";
std::string camera_point_topic = "/camera/depth_registered/points";
std::string save_log_path = "/home/edwin/report/log.txt";

//CONFIG/BENCHMARK
int benchmark_max_people_count = 24;
int benchmark_person_index = 0;
std::string benchmark_base_path = "/home/edwin/hpdb/";

void save_log(float rot_error,float dis_error,float roll,float pitch,float yaw,float frame,float person,float fps)
{
    std::ofstream outfile;
    outfile.open(save_log_path.c_str(), std::ios_base::app);
    outfile << person << " " << frame << " " << roll << " " << pitch << " " << yaw << " " << rot_error << " " << dis_error << " " <<  fps << endl;
    outfile.close();
}

void wait(int ms)
{

    boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void show (pcl::visualization::PCLVisualizer& viz)
{
    
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

void convertDepthToPointCloud(std::string path)
{
    int16_t* depth_data = loadDepthImageCompressed(path.c_str());

    stream_cloud->width = 640;
    stream_cloud->height = 480;
    stream_cloud->points.resize (640*480);

    register float constant = 1.0f / 525;
    register int centerX = (stream_cloud->width >> 1);
    int centerY = (stream_cloud->height >> 1);
    register int depth_idx = 0;

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
}

void preProcessStreamPoints (const PointCloudTCConstPtr &cloud)
{
    if ( is_stream_callback_busy ) return;
    is_stream_callback_busy = true;

    //Preprocessing

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

void benchmarkLogic()
{
    if ( is_live ) return;

    ROS_INFO("Benchmark logic started");
    wait(3000);

    benchmark_person_index = 0;
    PointCloudTCPtr ground_truth_cloud_temp(new PointCloudTC);

    while(benchmark_person_index < benchmark_max_people_count + 1)
    {
        //Load ground truth cloud
        std::string name_ply = benchmark_base_path + boost::lexical_cast<std::string>(benchmark_person_index) + ".ply";
        
        ground_truth_cloud_temp.reset(new PointCloudTC);
        ground_truth_cloud_temp->sensor_origin_.setZero ();
        ground_truth_cloud_temp->sensor_orientation_.w () = 0.0f;
        ground_truth_cloud_temp->sensor_orientation_.x () = 1.0f;
        ground_truth_cloud_temp->sensor_orientation_.y () = 0.0f;
        ground_truth_cloud_temp->sensor_orientation_.z () = 0.0f;

        //Load the cloud as temp (we gone rotate it with gt transform later)
        pcl::io::loadPolygonFilePLY(name_ply, *ground_truth_mesh);
        pcl::fromPCLPointCloud2(ground_truth_mesh->cloud, *ground_truth_cloud_temp);

        for ( int frame = 3 ; frame < 500 ; frame++)
        {
            std::string name = "";

            if ( frame < 10                ) name  = benchmark_base_path + "frame_0000" + boost::lexical_cast<std::string>( frame );
            if ( frame < 100 && frame >= 10) name  = benchmark_base_path + "frame_000"  + boost::lexical_cast<std::string>( frame );
            if ( frame >= 100              ) name  = benchmark_base_path + "frame_00"   + boost::lexical_cast<std::string>( frame );

            std::string name_color = name + "_rgb.png";
            std::string name_depth = name + "_depth.bin";
            std::string name_gt    = name + "_pose.txt";

            //Load RGB
            cv::Mat image;
            image = cv::imread(name_color, CV_LOAD_IMAGE_COLOR);   

            if(!image.data )    
            {
                ROS_ERROR_STREAM("Could not load RGB for benchmark : " << name_color);
                return;
            }                         

            //Load Depth Points
            convertDepthToPointCloud(name_depth);

            //Load ground truth transfoms
            ground_truth_tf = readGroundTruth(name_gt);

            //Transform 
            pcl::transformPointCloud<PointTC> (*ground_truth_cloud_temp, *ground_truth_cloud, ground_truth_tf);
            
            //Reset Transforms
            stream_cloud->sensor_origin_.setZero();
            stream_cloud->sensor_orientation_.w () = 0.0f;
            stream_cloud->sensor_orientation_.x () = 1.0f;
            stream_cloud->sensor_orientation_.y () = 0.0f;
            stream_cloud->sensor_orientation_.z () = 0.0f;

            //Process
            preProcessStreamPoints(stream_cloud);

            //Publish Image
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            pub_image.publish(msg);
            
            wait(40); //25 Target FPS
        }

        benchmark_person_index++;
    }
}

void callbackDepthPoints(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
{
    if ( !is_live ) return; //Ignore data if we are in benchmark mode

    if (cloud_m->width != 0 )
    {
        //pcl::fromPCLPointCloud2(*cloud_m, *stream_cloud);
        //pcl::fromROSMsg(*cloud_m, *stream_cloud);
        
        //Reset Transforms
        stream_cloud->sensor_origin_.setZero();
        stream_cloud->sensor_orientation_.w () = 0.0f;
        stream_cloud->sensor_orientation_.x () = 1.0f;
        stream_cloud->sensor_orientation_.y () = 0.0f;
        stream_cloud->sensor_orientation_.z () = 0.0f;

        preProcessStreamPoints(stream_cloud);

        is_stream_points_ready = true;
    }
}


void mainLogic()
{
   ROS_INFO("Main logic started");
   wait(1000);
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

    //ROS Interface
    ros::NodeHandle nh;
    ros::Subscriber sub1 = nh.subscribe(camera_point_topic, 1, callbackDepthPoints);

    image_transport::ImageTransport it(nh);
    pub_image = it.advertise("tms/color_image", 1);

    boost::thread thread_logic1(&mainLogic);
    boost::thread thread_logic2(&benchmarkLogic);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("TMS Terminated");
}

