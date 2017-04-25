//Amirkabir University of Technology

//ICP - Non Rigid ICP Algorithm implimentation with ROS - PCL

//ICP =>
//NICP1 =>
//NICP2 =>
//NICP3 =>



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

//BOOST
// #include <boost/thread.hpp>
// #include <boost/algorithm/string.hpp>
// #include <boost/lexical_cast.hpp>
// #include <boost/algorithm/string.hpp>
// #include <boost/lexical_cast.hpp>

// //PCL
// #include <pcl/filters/bilateral.h>
// #include <pcl/visualization/histogram_visualizer.h>
// #include <pcl/point_types.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>
// #include <pcl/registration/icp.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>
// #include <pcl/io/vtk_io.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/gp3.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/PCLPointCloud2.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_io.h>
// #include <pcl/surface/marching_cubes_hoppe.h>
// #include <pcl/surface/marching_cubes_rbf.h>
// #include <pcl/console/print.h>
// #include <pcl/console/parse.h>
// #include <pcl/console/time.h>
// #include <pcl/io/vtk_io.h>
// #include <pcl/surface/mls.h>
// #include <pcl/surface/poisson.h>
// #include <pcl/features/normal_3d_omp.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/surface/vtk_smoothing/vtk_utils.h>
// #include <pcl/registration/correspondence_estimation.h>
// #include <pcl/registration/correspondence_rejection_distance.h>
// #include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
// #include <pcl/registration/transformation_estimation_svd.h>
// #include <pcl/registration/ia_ransac.h>
// #include <pcl/filters/statistical_outlier_removal.h>

// //#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
// //#include <pcl/surface/on_nurbs/triangulation.h>

// //************************************************openCV
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include "opencv2/highgui/highgui_c.h"
// #include <opencv2/core/core.hpp>
// #include "opencv/cv.h"
// #include "opencv2/calib3d/calib3d.hpp"

// //********************************************** cv_bridge
// #include <sensor_msgs/Image.h>
// #include <stereo_msgs/DisparityImage.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/image_encodings.h>

// #include <cv_bridge/cv_bridge.h>


// //SYSTEM
// #include <math.h>
// #include <sstream>
// #include <iostream>
// #include <cstdio>
// #include <unistd.h>
// #include <cmath>
// #include <stdio.h>
// #include <stdlib.h>
// #include <tbb/atomic.h>
// #include <iostream>
// #include <string>
// #include <fstream>
// #include <iostream>
// #include <stdlib.h>
// #include <sstream>
// #include <vector>
// #include <fstream>
// #include <string>

// //VTK
// #include <vtkSmartPointer.h>
// #include <vtkExtractEdges.h>
// #include <vtkLine.h>
// #include <vtkCellArray.h>
// #include <vtkRenderWindow.h>
// #include <vtkOctreePointLocator.h>

// //EIGEN
// #include <eigen3/Eigen/Dense>
// //#include <eigen3/Eigen/Sparse>
// //#include <eigen3/unsupported/Eigen/SparseExtra>

// #include <iostream>
// #include <stdio.h>
// #include "omp.h"
// #include <boost/thread/thread.hpp>
// #include <math.h>
// #include <boost/filesystem.hpp>
// // timer
// #include <cstdio>
// #include <ctime>
// //*********************************************** ROS includes
// #include <stdexcept>
// // ROS core
// #include <ros/ros.h>
// #include <boost/thread/mutex.hpp>
// #include <image_transport/image_transport.h>



// void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m);

// std::string coutcolor0 = "\033[0;0m";
// std::string coutcolor_red = "\033[0;31m";
// std::string coutcolor_green = "\033[0;32m";
// std::string coutcolor_blue = "\033[0;34m";
// std::string coutcolor_magenta = "\033[0;35m";
// std::string coutcolor_brown = "\033[0;33m";

// using std::string;
// using std::exception;
// using std::cout;
// using std::cerr;
// using std::endl;

// using namespace std;
// using namespace boost;
// using namespace pcl;

// pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
// typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
// typedef pcl::PointXYZ PointT;

// bool next_iteration = false;
// bool app_exit = false;
// bool cloud_ready = false;

// typedef pcl::PointXYZ Point;

// void printMatix4f(const Eigen::Matrix4f & matrix)
// {
//     printf ("Rotation matrix :\n");
//     printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
//     printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
//     printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
//     printf ("Translation vector :\n");
//     printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
// }

// void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
// {
//     if (event.getKeySym() == "space" && event.keyDown())
//         next_iteration = true;

//     if (event.getKeySym() == "q" && event.keyDown())
//     {
//         app_exit = true;
//     }
// }

// struct val_3
// {
//     float X;
//     float Y;
//     float Z;
// };

// vector <val_3> points;

// void read_file()
// {
//     std::string line;
//     std::ifstream text;
//     text.open("/home/edwin/Target.txt", ios_base::in);

//     if (text.is_open())
//     {
//         getline(text,line);
//         while (text.good())
//         {
//             line = line.substr (0,line.size() - 2);
//             vector <string> fields;
//             boost::split( fields, line, boost::is_any_of( "," ) );
//             val_3 mpoint;
//             mpoint.X = atof(fields[0].c_str());
//             mpoint.Y = atof(fields[1].c_str());
//             mpoint.Z = atof(fields[2].c_str());

//             points.push_back(mpoint);
//             getline(text,line);

//         }
//         text.close();
//         //std::cout<<points.size()<<endl;
//     }
//     else
//     {
//         std::cout << "Unable to open file" << std::endl << std::endl;
//     }
// }

// pcl::PolygonMesh ICP_Generate_singleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// {
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud (cloud);
//     n.setInputCloud (cloud);
//     n.setSearchMethod (tree);
//     n.setKSearch (40);
//     n.compute (*normals);

//     cout<<coutcolor_magenta<<"Normal Done"<<coutcolor0<<endl;

//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr;
//     // Concatenate the XYZ and normal fields*
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

//     // Create search tree*
//     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//     tree2->setInputCloud (cloud_with_normals);

//     // Initialize objects
//     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//     pcl::PolygonMesh triangles;

//     // Set the maximum distance between connected points (maximum edge length)
//     gp3.setSearchRadius (0.05);

//     // Set typical values for the parameters
//     gp3.setMu (2.5);
//     gp3.setMaximumNearestNeighbors (100);
//     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//     gp3.setMinimumAngle(M_PI/18); // 10 degrees
//     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//     gp3.setNormalConsistency(false);

//     // Get result
//     gp3.setInputCloud (cloud_with_normals);
//     gp3.setSearchMethod (tree2);
//     gp3.reconstruct (triangles);


//     std::vector<int> parts = gp3.getPartIDs();
//     std::vector<int> states = gp3.getPointStates();

//     for ( int i = 0 ; i < parts.size() ; i++ )
//     {
//         cout<<i<<endl;
//     }

//     return triangles;
// }

// void get_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
// {

//     //COMPUTE NORMALS
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr treen (new pcl::search::KdTree<pcl::PointXYZ>);
//     treen->setInputCloud (cloud);
//     n.setInputCloud (cloud);
//     n.setSearchMethod (treen);
//     n.setKSearch (40);
//     n.compute (*normals);

//     // Create the FPFH estimation class, and pass the input dataset+normals to it
//     pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//     fpfh.setInputCloud (cloud);
//     fpfh.setInputNormals (normals);
//     // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

//     // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//     // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//     pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
//     fpfh.setSearchMethod (tree);

//     // Output datasets
//     //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

//     // Use all neighbors in a sphere of radius 5cm
//     // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//     fpfh.setRadiusSearch (0.05);

//     // Compute the features
//     fpfh.compute (*fpfhs);


// }

// void ICP_Align()
// {
//     //READ
//     PointCloudT::Ptr cloud_in1= PointCloudT::Ptr(new PointCloudT );
//     PointCloudT::Ptr cloud_in2=  PointCloudT::Ptr(new PointCloudT );
//     PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );

//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in1);

//     //CREATE A FAKE TRANSFORM
//     Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
//     float theta = M_PI/4; // The angle of rotation in radians
//     transformation_matrix (0,0) = cos(theta);
//     transformation_matrix (0,1) = -sin(theta);
//     transformation_matrix (1,0) = sin(theta);
//     transformation_matrix (1,1) = cos(theta);
//     transformation_matrix (2,3) = 1;

//     pcl::transformPointCloud (*cloud_in1, *cloud_in2, transformation_matrix);

//     //COMPUTE 3D FEATURES
//     LocalFeatures::Ptr fpfhs1(new LocalFeatures);
//     get_features(cloud_in1,fpfhs1);
//     LocalFeatures::Ptr fpfhs2(new LocalFeatures);
//     get_features(cloud_in2,fpfhs2);

//     //SAC ALIGN
//     pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;

//     sac_ia.setMinSampleDistance (0.05f);
//     sac_ia.setMaximumIterations (500);
//     sac_ia.setInputCloud (cloud_in1);
//     sac_ia.setInputTarget (cloud_in2);
//     sac_ia.setSourceFeatures (fpfhs1);
//     sac_ia.setTargetFeatures (fpfhs2);
//     sac_ia.align(*registration_output);

//     Eigen::Matrix4f trans = sac_ia.getFinalTransformation();


//     cout<<"ALIGN OUTPUT TRANS"<<endl;
//     printMatix4f(trans);

//     //pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
//     //sac.setInputCloud(source);
//     //sac.setTargetCloud(target);
//     //sac.setInlierThreshold(epsilon);
//     //sac.setMaxIterations(N);
//     //sac.setInputCorrespondences(correspondences);
//     //sac.getCorrespondences(inliers);
//     //Eigen::Matrix4f transformation = sac.getBestTransformation();

//     pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");
//     // Create two verticaly separated viewports

//     int vv(0);
//     viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//     viewer.setBackgroundColor(1,1,1);

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in1, 255, 20, 20);
//     viewer.addPointCloud (cloud_in1, cloud_in_color_h, "v1", vv);
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_in2, 20, 20, 255);
//     viewer.addPointCloud (cloud_in2, cloud_icp_color_h, "v2", vv);
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp2_color_h (registration_output, 0, 0, 0);
//     viewer.addPointCloud (registration_output, cloud_icp2_color_h, "v3", vv);

//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v1");
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v2");
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v3");

//     viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//     viewer.setSize(1280, 1024); // Visualiser window size
//     viewer.spin();
// }

// void ICP_SHOW_FPFH_Features()
// {
//     PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//     //fpfhs = LocalFeatures::Ptr (new LocalFeatures);
//     LocalFeatures::Ptr fpfhs(new LocalFeatures);
//     get_features(cloud_in,fpfhs);

//     pcl::visualization::PCLHistogramVisualizer hist;
//     hist.addFeatureHistogram(*fpfhs,"fpfh",33,"cloud",640,200);
//     hist.spin();

//     //Sampling
//     //PointCloud<int> indices;
//     //UniformSampling<PointT> uniform_sampling;
//     //uniform_sampling.setInputCloud (cloud);
//     //uniform_sampling.setRadiusSearch (0.05f);
//     //uniform_sampling.compute (indices);

//     //Correspondence
//     //CorrespondencesPtr corresps(new Correspondences);
//     //CorrespondenceEstimation<PointT, PointT> est;
//     //est.setInputSource (source_cloud);
//     //est.setInputTarget (target_cloud);
//     //est.determineCorrespondences (*corresps, max_dist);

//     //estimation

//     //pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZ, PointXYZ, double> te;
//     //te.setWeights (correspondence_weights);
//     //te.estimateRigidTransformation (*cloud_src, *cloud_tgt,*corresps_filtered, transform);
// }

// void ICP_Generate_Surface()
// {
//     //Generate Mesh
//     //ICP_Generate_Surface();
//     cout<<coutcolor_magenta<<"Surface Core Started"<<coutcolor0<<endl;

//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

//     // Normal estimation*
//     cout<<coutcolor_magenta<<"Normal Estimation"<<coutcolor0<<endl;

//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud (cloud);
//     n.setInputCloud (cloud);
//     n.setSearchMethod (tree);
//     n.setKSearch (40);
//     n.compute (*normals);

//     cout<<coutcolor_magenta<<"Normal Done"<<coutcolor0<<endl;

//     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr;
//     // Concatenate the XYZ and normal fields*
//     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

//     // Create search tree*
//     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//     tree2->setInputCloud (cloud_with_normals);

//     // Initialize objects
//     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//     pcl::PolygonMesh triangles;

//     // Set the maximum distance between connected points (maximum edge length)
//     gp3.setSearchRadius (0.05);

//     // Set typical values for the parameters
//     gp3.setMu (2.5);
//     gp3.setMaximumNearestNeighbors (100);
//     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//     gp3.setMinimumAngle(M_PI/18); // 10 degrees
//     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//     gp3.setNormalConsistency(false);

//     // Get result
//     gp3.setInputCloud (cloud_with_normals);
//     gp3.setSearchMethod (tree2);
//     gp3.reconstruct (triangles);

//     // Additional vertex information
//     std::vector<int> parts = gp3.getPartIDs();
//     std::vector<int> states = gp3.getPointStates();

//     // Visualization
//     pcl::visualization::PCLVisualizer viewer ("SURFACE");
//     // Create two verticaly separated viewports

//     int vv(0);
//     viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//     viewer.setBackgroundColor(1,1,1);

//     // Original point cloud is white
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud, 200, 60, 130);
//     viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v2", vv);
//     viewer.addPolygonMesh(triangles,"mesh",vv);
//     pcl::io::saveVTKFile ("mesh.vtk", triangles);
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_in_v2");
//     viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.02, "normals",vv);
//     viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//     viewer.setSize(1280, 1024); // Visualiser window size
//     viewer.spin();
// }

// void ICP_Thread()
// {
//     PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//     PointCloudT::Ptr cloud_tr	(new PointCloudT);      //Transformed point cloud
//     PointCloudT::Ptr cloud_icp	(new PointCloudT);      //ICP output point cloud
//     PointCloudT::Ptr cloud_voxel (new PointCloudT);     //Voxel Filtered

//     int iterations = 1;

//     //read_file();

//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//     //    printf ("\nLoaded file %s with %d points successfully\n\n", "TARGET", points.size());

//     //    cloud_in->width    = points.size() / 10 ;
//     //    cloud_in->height   = 1;
//     //    cloud_in->is_dense = false;
//     //    cloud_in->points.resize (cloud_in->width * cloud_in->height);

//     //    for (size_t i = 0; i < cloud_in->points.size() ; ++i)
//     //    {
//     //        cloud_in->points[i].x = points[i].X;
//     //        cloud_in->points[i].y = points[i].Y;
//     //        cloud_in->points[i].z = points[i].Z;
//     //    }

//     cout<<"IN SIZE "<< cloud_in->points.size ()<<endl;
//     //cout<<"IN FILTERED SIZE"<< cloud_voxel->points.size()<<endl;
//     //==================================================================== Cloude IN

//     //====================================================================
//     Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

//     // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//     float theta = M_PI/4; // The angle of rotation in radians
//     transformation_matrix (0,0) = cos(theta);
//     transformation_matrix (0,1) = -sin(theta);
//     transformation_matrix (1,0) = sin(theta);
//     transformation_matrix (1,1) = cos(theta);

//     // A translation on Z axis (0.4 meters)
//     transformation_matrix (2,3) = 1;

//     // Display in terminal the transformation matrix
//     std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//     printMatix4f(transformation_matrix);

//     // Executing the transformation
//     pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
//     *cloud_tr = *cloud_icp; // We backup cloud_icp into cloud_tr for later use

//     // The Iterative Closest Point algorithm
//     std::cout << "Initial iterations number is set to : " << iterations;
//     pcl::IterativeClosestPoint<PointT, PointT> icp;
//     icp.setMaximumIterations(iterations);
//     icp.setInputSource(cloud_icp);
//     icp.setInputTarget(cloud_in);
//     icp.align(*cloud_icp);
//     icp.setMaximumIterations(1); // For the next time we will call .align() function

//     if (icp.hasConverged()) {
//         printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//         std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//         transformation_matrix = icp.getFinalTransformation();
//         printMatix4f(transformation_matrix);
//     } else {
//         PCL_ERROR ("\nICP has not converged.\n");
//         return ;
//     }

//     // Visualization
//     pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");
//     // Create two verticaly separated viewports

//     int vv(0);
//     viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//     viewer.setBackgroundColor(1,1,1);

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255, 20, 20);
//     viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", vv);
//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 20, 20, 255);
//     viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", vv);

//     std::stringstream ss; ss << iterations;
//     std::string iterations_cnt = "Iterations = " + ss.str();
//     viewer.addText(iterations_cnt, 10, 60, 16, 0, 0, 0, "iterations_cnt", vv);

//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v2");
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v2");

//     // Set camera position and orientation
//     viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//     viewer.setSize(1280, 1024); // Visualiser window size

//     // Register keyboard callback :
//     viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);

//     pcl::PolygonMesh mesh1 = ICP_Generate_singleMesh(cloud_in);
//     viewer.addPolygonMesh(mesh1,"mesh1",vv);

//     pcl::PolygonMesh mesh2 = ICP_Generate_singleMesh(cloud_icp);
//     viewer.addPolygonMesh(mesh2,"mesh2",vv);

//     // Display the visualiser
//     while (!viewer.wasStopped ()) {
//         viewer.spinOnce ();

//         // The user pressed "space" :
//         if (next_iteration) {
//             icp.align(*cloud_icp);

//             if (icp.hasConverged()) {
//                 printf("\033[11A"); // Go up 11 lines in terminal output.
//                 printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//                 std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//                 transformation_matrix *= icp.getFinalTransformation();	// This is not very accurate !
//                 printMatix4f(transformation_matrix);					// Print the transformation between original pose and current pose

//                 ss.str (""); ss << iterations;
//                 std::string iterations_cnt = "ICP iterations = " + ss.str();
//                 viewer.updateText (iterations_cnt, 10, 60, 16, 0, 0, 0, "iterations_cnt");
//                 viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");

//                 mesh2 = ICP_Generate_singleMesh(cloud_icp);
//                 viewer.updatePolygonMesh(mesh2,"mesh2");

//             } else {
//                 PCL_ERROR ("\nICP has not converged.\n");
//                 return;
//             }
//         }

//         if ( app_exit ) break;
//         next_iteration = false;
//     }

//     cout<<"Viewer Closed"<<endl;
//     viewer.close();
//     cout<<"THREAD END"<<endl;
//     return;
// }

// void CONVERT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZ>::Ptr output)
// {
//     output->points.resize(input->size());

//     for (size_t i = 0; i < input->points.size(); i++)
//     {
//         output->points[i].x = input->points[i].x;
//         output->points[i].y = input->points[i].y;
//         output->points[i].z = input->points[i].z;
//     }

//     //Z Filter
//     const float depth_limit = 1.2;
//     pcl::PassThrough<pcl::PointXYZ> pass;
//     pass.setInputCloud (output);
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (0, depth_limit);
//     pass.filter (*output);

//     //voxel grid filtering
//     pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//     vox_grid.setInputCloud (output);
//     vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
//     vox_grid.filter (*output);

//     //noise removal
//     //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//     //sor.setInputCloud (output);
//     //sor.setMeanK (50);
//     //sor.setStddevMulThresh (1.0);
//     //sor.filter (*output);
// }

// void get_show_cloud()
// {
//     PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//     //Read Template
//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPLYFile("body_template.ply", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);


//     while (ros::ok())
//     {
//         if ( cloud_ready == false || global_cloud->width == 0 )
//         {
//             boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//             cout<<"wait for cloud"<<endl;
//         }
//         else
//         {
//             break;
//         }
//     }

//     cout<<"GET OUT"<<endl;

//     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

//     // Visualization
//     pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");

//     int vv(0);
//     viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//     viewer.setBackgroundColor(1,1,1);

//     PointCloudT::Ptr output(new PointCloudT);
//     CONVERT(global_cloud,output);

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (output, 20, 20, 20);
//     viewer.addPointCloud (output, cloud_in_color_h, "input", vv);
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (cloud_in, 220, 20, 20);
//     viewer.addPointCloud (cloud_in, cloud_in_color_h2, "template", vv);
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "template");

//     // Set camera position and orientation
//     viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//     viewer.setSize(1280, 1024); // Visualiser window size

//     pcl::io::savePLYFile("body_out.ply", *output,false);
//     PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );


//     int do_c = 0;
//     // Display the visualiser
//     LocalFeatures::Ptr fpfhs1(new LocalFeatures);
//     get_features(cloud_in,fpfhs1);

//     while (!viewer.wasStopped ())
//     {
//         viewer.spinOnce ();
//         CONVERT(global_cloud,output);
//         viewer.updatePointCloud (output, cloud_in_color_h, "input");

//         if ( do_c >= 20 )
//         {
//         do_c = 0;

//         LocalFeatures::Ptr fpfhs2(new LocalFeatures);
//         get_features(output,fpfhs2);

//         //SAC ALIGN
//         pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;

//         sac_ia.setMinSampleDistance (0.05f);
//         sac_ia.setMaximumIterations (500);
//         sac_ia.setInputCloud (cloud_in);
//         sac_ia.setInputTarget (output);
//         sac_ia.setSourceFeatures (fpfhs1);
//         sac_ia.setTargetFeatures (fpfhs2);
//         sac_ia.align(*registration_output);

//         Eigen::Matrix4f trans = sac_ia.getFinalTransformation();


//         cout<<"ALIGN OUTPUT TRANS"<<endl;
//         printMatix4f(trans);


//         viewer.updatePointCloud (registration_output, cloud_in_color_h2, "template");
//         }
//         //=======================================================================

//         do_c++;
//         boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//     }


//     //viewer.close();
//     return;
// }

// void get_show_cloud_ICP()
// {
//     PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//     //Read Template
//     pcl::PCLPointCloud2 cloud_blob;
//     pcl::io::loadPLYFile("face_template.ply", cloud_blob);
//     pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//     while (ros::ok())
//     {
//         if ( cloud_ready == false || global_cloud->width == 0 )
//         {
//             boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//             cout<<"wait for cloud"<<endl;
//         }
//         else
//         {
//             break;
//         }
//     }

//     cout<<"GET OUT"<<endl;

//     boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//     PointCloudT::Ptr output(new PointCloudT);
//     CONVERT(global_cloud,output);

//     cout<<"ALIGN START"<<endl;
//     PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );
//     //INITIAL ALIGNMENT
//     LocalFeatures::Ptr fpfhs1(new LocalFeatures);
//     get_features(cloud_in,fpfhs1);
//     LocalFeatures::Ptr fpfhs2(new LocalFeatures);
//     get_features(output,fpfhs2);

//     //SAC ALIGN
//     pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
//     sac_ia.setMinSampleDistance (0.05f);
//     sac_ia.setMaximumIterations (500);
//     sac_ia.setInputCloud (cloud_in);
//     sac_ia.setInputTarget (output);
//     sac_ia.setSourceFeatures (fpfhs1);
//     sac_ia.setTargetFeatures (fpfhs2);
//     sac_ia.align(*registration_output);

//     Eigen::Matrix4f trans = sac_ia.getFinalTransformation();

//     cout<<"ALIGN OUTPUT TRANS"<<endl;
//     printMatix4f(trans);

//     cout<<"ALIGN DONE"<<endl;

//     // Visualization
//     pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");

//     int vv(0);
//     viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//     viewer.setBackgroundColor(1,1,1);

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (output, 20, 20, 20);
//     viewer.addPointCloud (output, cloud_in_color_h, "input", vv);
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");

//     pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (registration_output, 220, 20, 20);
//     viewer.addPointCloud (registration_output, cloud_in_color_h2, "template", vv);
//     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "template");

//     // Set camera position and orientation
//     viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//     viewer.setSize(1280, 1024); // Visualiser window size

//    // pcl::io::savePLYFile("body_out.ply", *output,false);

//     int doc_count = 0;

//     while (!viewer.wasStopped ())
//     {

//         viewer.spinOnce ();
//         CONVERT(global_cloud,output);
//         viewer.updatePointCloud (output, cloud_in_color_h, "input");
//         viewer.updatePointCloud (registration_output, cloud_in_color_h2, "template");



//             int iterations = 1;
//             pcl::IterativeClosestPoint<PointT, PointT> icp;
//             icp.setMaximumIterations(iterations);
//             icp.setInputSource(registration_output);
//             icp.setInputTarget(output);
//             icp.align(*registration_output);
//             icp.setMaximumIterations(15); // For the next time we will call .align() function



//         boost::this_thread::sleep(boost::posix_time::milliseconds(20));
//     }

//     //viewer.close();
//     return;
// }


// void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
// {
//     if (cloud_m->width != 0 )
//     {
//         cloud_ready = true;
//         pcl::fromROSMsg(*cloud_m, *global_cloud);
//     }
// }



int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp");
    ros::Time::init();
    cout<<"ICP CORE STARTED DONE"<<endl;

    //boost::thread _thread_logic(&ICP_Thread);
    //boost::thread _thread_logic(&ICP_Generate_Surface);
    //boost::thread _thread_logic(&ICP_Align);
    //boost::thread _thread_logic(&get_show_cloud_ICP);
   // boost::thread _thread_logic(&CPD_main);

    ros::Rate loop_rate(20);

    //ros::NodeHandle nh_a[15];
    //ros::Subscriber sub1 = nh_a[0].subscribe("/camera/depth_registered/points", 1, Cloud_CallBack);

    while (ros::ok() && app_exit == false)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    //_thread_logic.interrupt();
    //_thread_logic.join();

    return 0;
}
