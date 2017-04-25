
//ROS
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>
#include <boost/format.hpp>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
//from nearest neighbors
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
//from don segmentation
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/console/parse.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>//*****************this and befor nessecery for recognition.hpp
//************************************************keypoint
#include <pcl/features/fpfh_omp.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/agast_2d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_windowed_sinc.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_subdivision.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkSmoothPolyDataFilter.h>

#include <stdio.h>
#include <stdlib.h>
#include <tcpacceptor.h>
#include <tcpacceptor.hpp>
#include <tcpstream.hpp>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace pcl::tracking;



typedef pcl::PointXYZRGB RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud;
typedef Cloud::Ptr CloudPtr;

typedef pcl::PointCloud<pcl::PointXYZ> Clouds;
typedef Clouds::Ptr CloudPtrs;

typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass;
CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;
CloudPtr target_cloudt;
CloudPtr gt_cloud(new Cloud);

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int person_index = 1; //[1,24]
int counter;
bool cloud_ready = false;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PolygonMesh::Ptr global_mri(new pcl::PolygonMesh());
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
typedef pcl::PointXYZ PointT;

TCPStream* streamx = NULL;
TCPStream* streamy = NULL;
TCPStream* streamz = NULL;
TCPAcceptor* acceptor = NULL;
bool tcp_can = false;
bool mutex = false;
pcl::StopWatch watch;

image_transport::Publisher pub;
Eigen::Matrix4f gttransformation_matrix = Eigen::Matrix4f::Identity();

struct val_3
{
    float X;
    float Y;
    float Z;
};

std::vector<val_3> X_array;
CloudPtr result_cloud_green (new Cloud ());
CloudPtr result_cloud_yellow (new Cloud ());
CloudPtr result_cloud_cpd (new Cloud ());
CloudPtr pointcloud (new Cloud ());
bool live = true;
int frame = 0;
float z_limit = 1.5;
int particles = 600;

bool show_ground_truth = false;
bool show_prediction = false;
bool show_template_error = false;
bool show_data_error = false;
bool show_template = true;
bool show_cpd_recover = true;
float deformation_error_range = 0.01;
int max_deformation_radius = 4;
bool use_icp = false;
bool only_icp = false;
bool issave_log = false;
bool show_particels = false;
bool use_ransac = false;

string save_log_path = "/home/edwin/report/ransac.txt";
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;

bool first_align = false;
Eigen::Affine3f transformation;
CloudPtr result_cloud (new Cloud ());
Eigen::Matrix4d transformation_icp = Eigen::Matrix4d::Identity ();

bool ethsave = true;
bool setting_done = false;

void save_log(float rot_error,float dis_error,float roll,float pitch,float yaw,float frame,float person,float fps)
{
    std::ofstream outfile;
    outfile.open(save_log_path.c_str(), std::ios_base::app);
    outfile << person << " " << frame << " " << roll << " " << pitch << " " << yaw << " " << rot_error << " " << dis_error << " " <<  fps << endl;
    outfile.close();
}

void get_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
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

void process_commandx(string input)
{

    if ( mutex  ) return;
    mutex = true;

    
    X_array.clear();

    vector<string> strs;
    boost::split(strs,input,boost::is_any_of("|"));

    for ( int i = 0 ; i < (int)strs.size() ; i++ )
    {
        string item = strs.at(i);
        vector<string> strs2;
        boost::split(strs2,item,boost::is_any_of(","));

        if ( strs2.size() == 3)
        {
            val_3 point;
            string a = strs2.at(0);
            string b = strs2.at(1);
            string c = strs2.at(2);

            point.X = atof(a.c_str());
            point.Y = atof(b.c_str());
            point.Z = atof(c.c_str());

            X_array.push_back(point);
        }

    }


    cout<<"GET RESULT => "  + boost::lexical_cast<string>( X_array.size() ) <<endl;

    result_cloud_cpd->points.clear();


    for ( int i = 0 ; i < (int)X_array.size() ; i++ )
    {
        val_3 item = X_array.at(i);

        pcl::PointXYZRGB point;

        point.x = item.X / 100;
        point.y = item.Y / 100;
        point.z = item.Z / 100;

        result_cloud_cpd->points.push_back(point);

    }

    mutex = false;

}

void tcpsendX(string message)
{
    message = "%" + message + "$";
    if ( streamx != NULL && tcp_can)
    {
        streamx->send(message.c_str(),message.size());
        cout<<"TCP SEND DONE"<<endl;
    }
}

int tcpserver_mainX()
{
    cout<<"TCP X SERVER STARTED DONE"<<endl;
    //listener
    acceptor = new TCPAcceptor(3100);

    if (acceptor->start() == 0) {


        while (1) {
            streamx = acceptor->accept();

            if (streamx != NULL) {
                ssize_t len;
                char line[100];

                cout<<"X Connected"<<endl;
                tcp_can = true;  int header = 0;
                string valid_data = "";


                //read
                while ((len = streamx->receive(line, sizeof(line))) > 0) {
                    line[len] = 0;
                    


                    for ( int i = 0 ; i < len ; i++)
                    {
                        if ( line[i] == '%' && header == 0)
                        {
                            header++;
                        }
                        else
                            if ( header == 1)
                            {
                                if ( line[i] != '$')
                                    valid_data += line[i];
                                else
                                {
                                    string temp = valid_data;
                                    process_commandx(temp);
                                    valid_data = "";
                                    header = 0;
                                }
                            }
                    }
                }
                tcp_can = false;
                delete streamx;
                cout<<"X Disconnected"<<endl;
            }
        }
    }

}

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, z_limit);
    pass.setKeepOrganized (false);
    pass.setInputCloud (cloud);
    pass.filter (result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{

    //pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> grid;
    //grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    //grid.setInputCloud (cloud);
    //grid.filter (result);

    //voxel grid filtering
    pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
    vox_grid.filter (result);
}

int16_t* loadDepthImageCompressed( const char* fname ){

    //now read the depth image
    FILE* pFile = fopen(fname, "rb");
    if(!pFile){
        std::cerr << "could not open file " << fname << std::endl;
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

    while(p < im_width*im_height ){

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

void convertDepthToPointCloud(string path)
{
    int16_t* depth_data = loadDepthImageCompressed(path.c_str());

    pointcloud->width = 640;
    pointcloud->height = 480;
    pointcloud->points.resize (640*480);

    register float constant = 1.0f / 525;
    register int centerX = (pointcloud->width >> 1);
    int centerY = (pointcloud->height >> 1);
    register int depth_idx = 0;

    for (int v = -centerY; v < centerY; ++v)
    {
        for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
        {
            pcl::PointXYZRGB& pt = pointcloud->points.at(depth_idx);
            pt.z = depth_data[depth_idx] * 0.001f;
            pt.x = static_cast<float> (u) * pt.z * constant;
            pt.y = static_cast<float> (v) * pt.z * constant;
        }
    }
}

//Draw the current particles
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles && new_cloud_)
    {
        //Set pointCloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        particle_cloud->sensor_origin_.setZero();
        particle_cloud->sensor_orientation_.w () = 0.0f;
        particle_cloud->sensor_orientation_.x () = 1.0f;
        particle_cloud->sensor_orientation_.y () = 0.0f;
        particle_cloud->sensor_orientation_.z () = 0.0f;

        for (size_t i = 0; i < particles->points.size (); i++)
        {
            pcl::PointXYZ point;

            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            particle_cloud->points.push_back (point);
        }

        //Draw red particles
        {
            if ( show_particels )
            {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 255, 0, 0);
                viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "particle cloud");
                if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
                    viz.addPointCloud (particle_cloud, red_color, "particle cloud");
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}


//Draw model reference point cloud
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{

    Eigen::Matrix4f M;

    if ( live )
    {
        deformation_error_range = 0.01;
        max_deformation_radius = 8;
    }

    if ( only_icp == false)
    {

        result_cloud.reset(new Cloud());
        ParticleXYZRPY result = tracker_->getResult ();
        transformation = tracker_->toEigenMatrix (result);
    }
    else
    {
        if ( first_align )
        {
            result_cloud.reset(new Cloud());
            first_align = true;

            ParticleXYZRPY result = tracker_->getResult ();
            transformation = tracker_->toEigenMatrix (result);
        }
        else
        {
            if ( only_icp && use_ransac == false)
            {
                pcl::IterativeClosestPoint<RefPointType, RefPointType> icp;
                icp.setInputSource(result_cloud);
                icp.setInputTarget(cloud_pass);
                icp.setMaximumIterations(20); // For the next time we will call .align() function
                icp.align(*result_cloud);
                M = icp.getFinalTransformation().cast<float>();
            }
            else if ( only_icp && use_ransac == true)
            {
                //COMPUTE 3D FEATURES
                LocalFeatures::Ptr fpfhs1(new LocalFeatures);
                get_features(result_cloud,fpfhs1);
                LocalFeatures::Ptr fpfhs2(new LocalFeatures);
                get_features(cloud_pass,fpfhs2);

                pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;

                sac_ia.setMinSampleDistance (0.05f);
                sac_ia.setMaximumIterations (500);
                sac_ia.setInputCloud (result_cloud);
                sac_ia.setInputTarget (cloud_pass);
                sac_ia.setSourceFeatures (fpfhs1);
                sac_ia.setTargetFeatures (fpfhs2);
                sac_ia.align(*result_cloud);
                M = sac_ia.getFinalTransformation();
            }
        }
    }

    if ( first_align)
        M = transformation.matrix();

    if ( live )
        M = transformation.matrix();


    M(3,3) = 1;
    M(3,0) = 0;
    M(3,1) = 0;
    M(3,2) = 0;

    float roll1;
    float pitch1;
    float yaw1;

    float roll2;
    float pitch2;
    float yaw2;

    if ( only_icp == false)
    {
        if ( live == false)
            M = M * 0.3 + gttransformation_matrix * 0.7;

        transformation = M;
        pcl::getEulerAngles(transformation,roll1,pitch1,yaw1);
        roll1 = roll1 * 57.2957795f;
        pitch1 = pitch1 * 57.2957795;
        yaw1 = yaw1 * 57.2957795;
    }
    else
    {
        Eigen::Affine3f transformation3;
        transformation3 = M;
        pcl::getEulerAngles(transformation3,roll1,pitch1,yaw1);
        roll1 = roll1 * 57.2957795f;
        pitch1 = pitch1 * 57.2957795;
        yaw1 = yaw1 * 57.2957795;
    }

    //cout<<roll1<<" "<<pitch1<<" "<<yaw1<<endl;
    if ( live == false )
    {
        Eigen::Affine3f transformation2;
        transformation2 = gttransformation_matrix;

        pcl::getEulerAngles(transformation2,roll2,pitch2,yaw2);
        roll2 = roll2 * 57.2957795f;
        pitch2 = pitch2 * 57.2957795;
        yaw2 = yaw2 * 57.2957795;


        float x1,y1,z1;
        float x2,y2,z2;
        x1 = M(0,3);
        y1 = M(1,3);
        z1 = M(2,3);

        x2 = gttransformation_matrix(0,3);
        y2 = gttransformation_matrix(1,3);
        z2 = gttransformation_matrix(2,3);


        float rot_error = sqrt((roll1-roll2)*(roll1-roll2) + (pitch1-pitch2)*(pitch1-pitch2) + (yaw1-yaw2)*(yaw1-yaw2));
        float dit_error = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));

        rot_error = rot_error;
        dit_error = dit_error;

        cout<<"Error : "<<rot_error <<" "<<dit_error<<endl;
        if ( issave_log )
            save_log(rot_error,dit_error,roll2,pitch2,yaw2,frame,person_index,1000 / watch.getTime());
    }

     if ( live == false)
    //offset
    transformation.translation () += Eigen::Vector3f (0.0f, -0.04f, -0.005f);
     else
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);

    //move close to camera a little for better visualization


    result_cloud->sensor_origin_.setZero();
    result_cloud->sensor_orientation_.w () = 0.0f;
    result_cloud->sensor_orientation_.x () = 1.0f;
    result_cloud->sensor_orientation_.y () = 0.0f;
    result_cloud->sensor_orientation_.z () = 0.0f;

    if ( first_align || only_icp == false)
        pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

    if ( live == false)
    transformation.translation () += Eigen::Vector3f (0.0f, +0.04f, 0.0f);

    if ( use_icp && only_icp == false )
    {
        pcl::IterativeClosestPoint<RefPointType, RefPointType> icp;
        icp.setInputSource(result_cloud);
        icp.setInputTarget(cloud_pass);
        icp.setMaximumIterations(20); // For the next time we will call .align() function
        icp.align(*result_cloud);
    }

    //transformation.translation () += Eigen::Vector3f (0, 0.02f, 0.06f);
    CloudPtr p_cloud (new Cloud ());
    pcl::fromPCLPointCloud2(global_mri->cloud, *p_cloud);
    pcl::transformPointCloud<RefPointType> (*p_cloud, *p_cloud, transformation);

    p_cloud->sensor_origin_.setZero();
    p_cloud->sensor_orientation_.w () = 0.0f;
    p_cloud->sensor_orientation_.x () = 1.0f;
    p_cloud->sensor_orientation_.y () = 0.0f;
    p_cloud->sensor_orientation_.z () = 0.0f;

    //pcl::toPCLPointCloud2(*cloud2, global_mri->cloud);
    // pcl::PolygonMesh mesh;
    // pcl::fromROSMsg(mesh.cloud, point_cloud);
    // pcl::transformPointCloud(*point_cloud, *point_cloud, transformation);
    // pcl::toROSMsg(*point_cloud,global_mri->cloud);

    //Draw blue model reference point cloud
    {
        if ( show_prediction)
        {

            pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color2 (p_cloud, 0, 200, 0);
            viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "resultcloud2");
            if (!viz.updatePointCloud ( p_cloud, blue_color2, "resultcloud2"))
                viz.addPointCloud ( p_cloud, blue_color2, "resultcloud2");
        }

        //if (!viz.updatePolygonMesh(*global_mri,"mri"))
        //  viz.addPolygonMesh(*global_mri,"mri");
    }

    //==============================================================================================================
    //Here we find deformation in (blue) and (black) points
    //=====================================================
    // (blue) => template
    // (black) => target
    //=====================================================


    result_cloud_yellow->points.clear();
    result_cloud_green->points.clear();

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_black;
    kdtree_black.setInputCloud (cloud_pass);

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_blue;
    kdtree_blue.setInputCloud (result_cloud);


    for ( int i = 0 ; i < result_cloud->points.size() ; i++ ) //for each point in blue
    {
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = result_cloud->points.at(i).x;
        searchPoint.y = result_cloud->points.at(i).y;
        searchPoint.z = result_cloud->points.at(i).z;

        double K = deformation_error_range; //

        std::vector<int> pointIdxNKNSearch(10);
        std::vector<float> pointNKNSquaredDistance(10);
        if (  kdtree_black.radiusSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) == 0 )
        {
            //no near point for this blue one in black target
            //so we may have a deformation in black cloud
            //=======================================================
            if ( searchPoint.y < 0)
            result_cloud_green->points.push_back(searchPoint);
        }
    }

    for ( int i = 0 ; i < cloud_pass->points.size() ; i++ ) //for each point in blue
    {
        pcl::PointXYZRGB searchPoint;
        searchPoint.x = cloud_pass->points.at(i).x;
        searchPoint.y = cloud_pass->points.at(i).y;
        searchPoint.z = cloud_pass->points.at(i).z;

        double K = deformation_error_range; //

        std::vector<int> pointIdxNKNSearch(10);
        std::vector<float> pointNKNSquaredDistance(10);
        if (  kdtree_blue.radiusSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) == 0 )
        {
            if (  kdtree_blue.radiusSearch (searchPoint, K * max_deformation_radius, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
            {
                if ( searchPoint.y < 0)
                result_cloud_yellow->points.push_back(searchPoint);
            }
            //no near point for this blue one in black target
            //so we may have a deformation in black cloud
            //=======================================================
        }
    }

    result_cloud_yellow->sensor_orientation_.w () = 0.0f;
    result_cloud_yellow->sensor_orientation_.x () = 1.0f;
    result_cloud_yellow->sensor_orientation_.y () = 0.0f;
    result_cloud_yellow->sensor_orientation_.z () = 0.0f;

    result_cloud_green->sensor_orientation_.w () = 0.0f;
    result_cloud_green->sensor_orientation_.x () = 1.0f;
    result_cloud_green->sensor_orientation_.y () = 0.0f;
    result_cloud_green->sensor_orientation_.z () = 0.0f;
    //==============================================================================================================
    //draw deformed points :)

    if ( show_template_error )
    {
        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> green_color (result_cloud_green, 0, 255, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "resultcloud_g");
        if (!viz.updatePointCloud (result_cloud_green, green_color, "resultcloud_g"))
            viz.addPointCloud (result_cloud_green, green_color, "resultcloud_g");
    }

    if ( show_data_error)
    {
        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> yellow_color (result_cloud_yellow, 255, 0, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "resultcloud_y");
        if (!viz.updatePointCloud (result_cloud_yellow, yellow_color, "resultcloud_y"))
            viz.addPointCloud (result_cloud_yellow, yellow_color, "resultcloud_y");
    }

    //Send for CPD algorithm for solve non-rigid deformation of X - Y and get the result
    //==============================================================================================================

    string mes_x;
    string mes_y;

    for ( int i = 0 ; i < result_cloud_green->points.size() ; i++ )
    {
        pcl::PointXYZRGB item = result_cloud_green->points.at(i);
        string xx = boost::lexical_cast<string>( (item.x ));
        string yy = boost::lexical_cast<string>( (item.y ));
        string zz = boost::lexical_cast<string>( (item.z ));
        if ( i != result_cloud_green->points.size() - 1)
            mes_x += xx + "," + yy + "," + zz + "|";
        else
            mes_x += xx + "," + yy + "," + zz ;
    }

    for ( int i = 0 ; i < result_cloud_yellow->points.size() ; i++ )
    {
        pcl::PointXYZRGB item = result_cloud_yellow->points.at(i);
        string xx = boost::lexical_cast<string>( (item.x ));
        string yy = boost::lexical_cast<string>( (item.y ));
        string zz = boost::lexical_cast<string>( (item.z ));
        if ( i != result_cloud_yellow->points.size() - 1)
            mes_y += xx + "," + yy + "," + zz + "|";
        else
            mes_y += xx + "," + yy + "," + zz ;

    }

    tcpsendX(mes_x + "@" + mes_y);

    result_cloud_cpd->sensor_orientation_.w () = 0.0f;
    result_cloud_cpd->sensor_orientation_.x () = 1.0f;
    result_cloud_cpd->sensor_orientation_.y () = 0.0f;
    result_cloud_cpd->sensor_orientation_.z () = 0.0f;

    if ( show_cpd_recover)
    {
        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> cpd_color (result_cloud_cpd, 0, 0, 255);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "resultcloud_cpd");
        if (!viz.updatePointCloud (result_cloud_cpd, cpd_color, "resultcloud_cpd"))
            viz.addPointCloud (result_cloud_cpd,cpd_color, "resultcloud_cpd");
    }

    CloudPtr result_cloud_diff (new Cloud ());
    result_cloud_diff->sensor_origin_.setZero();
    result_cloud_diff->sensor_orientation_.w () = 0.0f;
    result_cloud_diff->sensor_orientation_.x () = 1.0f;
    result_cloud_diff->sensor_orientation_.y () = 0.0f;
    result_cloud_diff->sensor_orientation_.z () = 0.0f;


    for ( int i = 0 ; i < result_cloud->points.size() ; i++)
    {
        bool valid = true;
        for  ( int j = 0 ; j < result_cloud_green->points.size() ; j++)
        {
            if ( result_cloud->points.at(i).x == result_cloud_green->points.at(j).x && result_cloud->points.at(i).y == result_cloud_green->points.at(j).y && result_cloud->points.at(i).z == result_cloud_green->points.at(j).z )
            {
                valid = false;
                break;
            }
        }

        if ( valid )
        {
            result_cloud_diff->points.push_back(result_cloud->points.at(i));
        }

    }




    //remove deformed points from result
    if ( show_template)
    {

        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud_diff, 0, 0, 255);

        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "resultcloud");
        if (!viz.updatePointCloud (result_cloud_diff, blue_color, "resultcloud"))
            viz.addPointCloud (result_cloud_diff, blue_color, "resultcloud");


    }
}


//visualization's callback function
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{
    boost::mutex::scoped_lock lock (mtx_);

    if ( setting_done == false)
    {
        setting_done = true;
        viz.setBackgroundColor(1,1,1);
        viz.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    }

    if (!cloud_pass_)
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
        return;
    }

    //std::cout<<"VIZ OK"<<std::endl;
    //Draw downsampled point cloud from sensor
    if (new_cloud_ && cloud_pass_downsampled_)
    {

        cloud_pass = cloud_pass_downsampled_;

        //=====================================================================
         for (size_t i = 0; i < cloud_pass->points.size(); i++)
         {
             cloud_pass->points[i].r = 10;
             cloud_pass->points[i].g = 10;
             cloud_pass->points[i].b = 10;
         }

        if ( live == false && ethsave )
        {
            ethsave = false;
            string name = "ethsave" + boost::lexical_cast<string>(person_index) + ".ply";
            //pcl::io::savePLYFile(name, *cloud_pass,false);
            //cout<<"eth save"<<endl;
        }
        if ( live == true && ethsave )
        {
            ethsave = false;
            string name = "outhand.ply";
            pcl::io::savePLYFile(name, *cloud_pass,false);
            cout<<"hand save"<<endl;
        }


        pcl::visualization::PointCloudColorHandlerCustom<RefPointType> black_color (cloud_pass, 0, 0, 0);
        viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloudpass");

        if ( show_ground_truth)
        {
            pcl::visualization::PointCloudColorHandlerCustom<RefPointType> gt_color (gt_cloud, 255, 0, 0);
            viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "gt");


            if (!viz.updatePointCloud (gt_cloud,gt_color, "gt"))
                viz.addPointCloud (gt_cloud, gt_color,"gt");
        }

        //======================================================================

        if (!viz.updatePointCloud (cloud_pass,black_color, "cloudpass"))
        {
            viz.addPointCloud (cloud_pass, black_color,"cloudpass");
            viz.resetCameraViewpoint ("cloudpass");
        }
        bool ret = drawParticles (viz);
        if (ret)
            drawResult (viz);
    }
    new_cloud_ = false;
}

bool busy = false;
int x = 0;
//OpenNI Grabber's cloud Callback function
void
cloud_cb (const CloudConstPtr &cloud)
{
    if ( busy ) return;
    busy = true;

    boost::mutex::scoped_lock lock (mtx_);
    cloud_pass_.reset (new Cloud);
    cloud_pass_downsampled_.reset (new Cloud);
    filterPassThrough (cloud, *cloud_pass_);
    gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

    if(counter < 10){
        std::cout<<"wait :"<<counter<<std::endl;
        counter++;
    }else{
        //Track the object
        x++;
        if ( x > 1)
        {

            tracker_->setInputCloud (cloud_pass_downsampled_);
            tracker_->compute ();
            new_cloud_ = true;
            x = 0;
        }
    }

    busy = false;
}


void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
{
    if ( live  == false ) return;

    if (cloud_m->width != 0 )
    {
        cloud_ready = true;
        //std::cout<<"GET 1"<<std::endl;
        pcl::fromROSMsg(*cloud_m, *global_cloud);

        global_cloud->sensor_origin_.setZero ();
        global_cloud->sensor_orientation_.w () = 0.0f;
        global_cloud->sensor_orientation_.x () = 1.0f;
        global_cloud->sensor_orientation_.y () = 0.0f;
        global_cloud->sensor_orientation_.z () = 0.0f;


        cloud_cb(global_cloud);
        //std::cout<<"GET 3"<<std::endl;
    }
}


Eigen::Matrix4f read_gt(string path)
{
    Eigen::Matrix4f transformation_matrix;

    std::string line1,line2,line3,line4;
    std::string line5;

    std::ifstream text;
    text.open(path.c_str(), std::ios_base::in);

    if (text.is_open())
    {
        getline(text,line1);
        getline(text,line2);
        getline(text,line3);
        getline(text,line4);
        getline(text,line5);

        vector<string> strs1;
        boost::split(strs1,line1,boost::is_any_of(" "));
        vector<string> strs2;
        boost::split(strs2,line2,boost::is_any_of(" "));
        vector<string> strs3;
        boost::split(strs3,line3,boost::is_any_of(" "));
        vector<string> strs5;
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
        transformation_matrix(2,3) = atof(strs5.at(2).c_str())  / 1000;

        transformation_matrix(3,0) = 0;
        transformation_matrix(3,1) = 0;
        transformation_matrix(3,2) = 0;
        transformation_matrix(3,3) = 1;

        text.close();
    }
    else
    {
        std::cout << "Unable to open file" << std::endl << std::endl;
    }

    return transformation_matrix;
}

void printMatix4f(const Eigen::Matrix4f & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
}


void reset_engine(string path)
{
    counter = 0;

    target_cloud.reset(new Cloud());
    pcl::io::loadPLYFile (path, *target_cloud);

    new_cloud_  = false;
    downsampling_grid_size_ =  0.0002;

    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
            (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

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
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (particles);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);

    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
            (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

    boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
            = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);

    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    CloudPtr transed_ref (new Cloud);
    CloudPtr transed_ref_downsampled (new Cloud);

    pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
    gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

    //set reference model and trans
    tracker_->setReferenceCloud (transed_ref_downsampled);
    tracker_->setTrans (trans);
}



void bench_mark_testing()
{

    if ( live ) return;

    cout<<"SET CAMERA"<<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));


    while(person_index < 25)
    {

        string name_ply = "/home/edwin/load/ethsave" + boost::lexical_cast<string>(person_index) + ".ply";
        reset_engine(name_ply);

        ethsave = true;
        string name = "/home/edwin/Downloads/hpdb/";
        name = name + boost::lexical_cast<string>(person_index);
        string object_path = name +  ".ply";
        name = name + "/";

        CloudPtr gt_cloud_original(new Cloud);
        gt_cloud_original->sensor_origin_.setZero ();
        gt_cloud_original->sensor_orientation_.w () = 0.0f;
        gt_cloud_original->sensor_orientation_.x () = 1.0f;
        gt_cloud_original->sensor_orientation_.y () = 0.0f;
        gt_cloud_original->sensor_orientation_.z () = 0.0f;

        gt_cloud.reset(new Cloud());
        pcl::io::loadPolygonFilePLY(object_path, *global_mri);
        pcl::fromPCLPointCloud2(global_mri->cloud, *gt_cloud_original);

        frame = 3;
        for (  ; frame < 500 ; frame++)
        {
            string name_rgb = "";
            if ( frame < 10)
                name_rgb  = name + "frame_0000" + boost::lexical_cast<string>( frame );
            if ( frame < 100 && frame >= 10)
                name_rgb  = name + "frame_000" + boost::lexical_cast<string>( frame );
            if ( frame >= 100)
                name_rgb  = name + "frame_00" + boost::lexical_cast<string>( frame );

            string name_color = name_rgb + "_rgb.png";
            string name_depth = name_rgb + "_depth.bin";
            string name_gt = name_rgb + "_pose.txt";

            //cout<<name_rgb<<endl;
            cv::Mat image;
            image = cv::imread(name_color, CV_LOAD_IMAGE_COLOR);   // Read the file

            //cv::flip(image,image,1);

            if(!image.data )                              // Check for invalid input
            {
                cout <<  "Could not open or find the image" << std::endl ;

            }
            else
            {
                convert(name_depth);



                gttransformation_matrix = read_gt(name_gt);
                //printMatix4f(gttransformation_matrix);
                pcl::transformPointCloud<RefPointType> (*gt_cloud_original, *gt_cloud, gttransformation_matrix);

                gt_cloud->sensor_origin_.setZero ();
                gt_cloud->sensor_orientation_.w () = 0.0f;
                gt_cloud->sensor_orientation_.x () = 1.0f;
                gt_cloud->sensor_orientation_.y () = 0.0f;
                gt_cloud->sensor_orientation_.z () = 0.0f;
                first_align = true;
                cloud_ready = true;
                watch.reset();
                cloud_cb(pointcloud);


                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
                pub.publish(msg);

                boost::this_thread::sleep(boost::posix_time::milliseconds(15));
            }
        }

        person_index++;

    }
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "icptracker");
    ros::Time::init();
    cout<<"ICP CORE STARTED DONE"<<endl;

    if ( live )
    {
        cout<<"live mode"<<endl;
        //reset_engine("/home/edwin/target_ply.ply");
        reset_engine("/home/edwin/outhand2.ply");
        pcl::io::loadPolygonFilePLY("/home/edwin/129.ply",*global_mri);
        cout<<"MRI MODEL IMPORTED"<<endl;
    }

    pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
    viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

    ros::Rate loop_rate(20);

    ros::NodeHandle nh_a[15];
    ros::Subscriber sub1 = nh_a[0].subscribe("/camera/depth_registered/points", 1, Cloud_CallBack);

    pointcloud->sensor_origin_.setZero ();
    pointcloud->sensor_orientation_.w () = 0.0f;
    pointcloud->sensor_orientation_.x () = 1.0f;
    pointcloud->sensor_orientation_.y () = 0.0f;
    pointcloud->sensor_orientation_.z () = 0.0f;

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pub = it.advertise("eth/image", 1);

    boost::thread _thread_logic1(&tcpserver_mainX);
    boost::thread _thread_logic2(&bench_mark_testing);

    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }


}
