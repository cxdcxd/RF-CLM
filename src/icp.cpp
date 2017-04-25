////Amirkabir University of Technology

////ICP - Non Rigid ICP Algorithm implimentation with ROS - PCL

////ICP =>
////NICP1 =>
////NICP2 =>
////NICP3 =>



////ROS
//#include <ros/package.h>
//#include <image_transport/image_transport.h>
//#include "ros/ros.h"
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
//#include "std_msgs/Int32.h"
//#include "std_msgs/String.h"
//#include "std_msgs/Bool.h"
//#include "geometry_msgs/Twist.h"
//#include <sensor_msgs/LaserScan.h>

////BOOST
//#include <boost/thread.hpp>
//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>
//#include <boost/algorithm/string.hpp>
//#include <boost/lexical_cast.hpp>


////SYSTEM
//#include <math.h>
//#include <sstream>
//#include <iostream>
//#include <cstdio>
//#include <unistd.h>
//#include <cmath>
//#include <stdio.h>
//#include <stdlib.h>
//#include <tbb/atomic.h>
//#include <iostream>
//#include <string>
//#include <fstream>
//#include <iostream>
//#include <stdlib.h>
//#include <sstream>
//#include <vector>
//#include <fstream>
//#include <string>

////VTK
//#include <vtkSmartPointer.h>
//#include <vtkExtractEdges.h>
//#include <vtkLine.h>
//#include <vtkCellArray.h>
//#include <vtkRenderWindow.h>
//#include <vtkOctreePointLocator.h>

////EIGEN
//#include <eigen3/Eigen/Dense>
////#include <eigen3/Eigen/Sparse>
////#include <eigen3/unsupported/Eigen/SparseExtra>

//#include <iostream>
//#include <stdio.h>
//#include "omp.h"
//#include <boost/thread/thread.hpp>
//#include <math.h>
//#include <boost/filesystem.hpp>
//// timer
//#include <cstdio>
//#include <ctime>
////*********************************************** ROS includes
//#include <stdexcept>
//// ROS core
//#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>
//#include <image_transport/image_transport.h>

//#include <iostream>
//#include <stdio.h>
//#include "omp.h"
//#include <boost/thread/thread.hpp>
//#include <math.h>
//#include <boost/filesystem.hpp>
//// timer
//#include <cstdio>
//#include <ctime>
////*********************************************** ROS includes
//#include <stdexcept>
//// ROS core
//#include <ros/ros.h>
//#include <boost/thread/mutex.hpp>
//#include <image_transport/image_transport.h>
////************************************************ PCL includes

//#include <pcl_ros/point_cloud.h>
//#include <pcl/io/io.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/conversions.h>
//#include <pcl/point_types.h>
//#include <pcl/range_image/range_image_planar.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/common/common_headers.h>
//#include <pcl/visualization/cloud_viewer.h>
////from nearest neighbors
//#include <pcl/common/common.h>
//#include <pcl/common/transforms.h>
////from don segmentation
//#include <pcl/search/organized.h>
//#include <pcl/search/kdtree.h>
//#include <pcl/features/normal_3d_omp.h>
//#include <pcl/filters/conditional_removal.h>
//#include <pcl/features/don.h>
//#include <pcl/point_cloud.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/filters/project_inliers.h>
//#include <pcl/surface/convex_hull.h>
//#include <pcl/segmentation/extract_polygonal_prism_data.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/filters/filter.h>
//#include <pcl/console/parse.h>
//#include <pcl/sample_consensus/ransac.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
//#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/features/shot_omp.h>
//#include <pcl/keypoints/uniform_sampling.h>
//#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/recognition/cg/geometric_consistency.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/kdtree/impl/kdtree_flann.hpp>//*****************this and befor nessecery for recognition.hpp
////************************************************keypoint
//#include <pcl/features/fpfh_omp.h>
//#include <pcl/range_image/range_image.h>
//#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/keypoints/narf_keypoint.h>
//#include <pcl/features/narf_descriptor.h>
//#include <pcl/keypoints/sift_keypoint.h>
//#include <pcl/keypoints/harris_6d.h>
//#include <pcl/keypoints/harris_3d.h>
//#include <pcl/keypoints/agast_2d.h>
//#include <pcl/keypoints/susan.h>
//#include <pcl/keypoints/iss_3d.h>
////************************************************openCV
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/highgui/highgui_c.h"
//#include <opencv2/core/core.hpp>
//#include "opencv/cv.h"
//#include "opencv2/calib3d/calib3d.hpp"
////********************************************** cv_bridge
//#include <cv_bridge/cv_bridge.h>

//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>

//#include <limits>
//#include <fstream>
//#include <vector>
//#include <Eigen/Core>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/registration/ia_ransac.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include <pcl/registration/icp.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_lib_io.h>
//#include <pcl/io/ply_io.h>
//#include <pcl/io/vtk_io.h>


//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/openni_grabber.h>
//#include <pcl/console/parse.h>
//#include <pcl/common/time.h>
//#include <pcl/common/centroid.h>

//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/io/pcd_io.h>

//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/approximate_voxel_grid.h>

//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>

//#include <pcl/search/pcl_search.h>
//#include <pcl/common/transforms.h>

//#include <boost/format.hpp>

//#include <pcl/tracking/tracking.h>
//#include <pcl/tracking/particle_filter.h>
//#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
//#include <pcl/tracking/particle_filter_omp.h>
//#include <pcl/tracking/coherence.h>
//#include <pcl/tracking/distance_coherence.h>
//#include <pcl/tracking/hsv_color_coherence.h>
//#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
//#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

//using namespace pcl::tracking;

//typedef pcl::PointXYZRGBA RefPointType;
//typedef ParticleXYZRPY ParticleT;
//typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
//typedef Cloud::Ptr CloudPtr;
//typedef Cloud::ConstPtr CloudConstPtr;
//typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

//CloudPtr cloud_pass_;
//CloudPtr cloud_pass_downsampled_;
//CloudPtr target_cloud;

//boost::mutex mtx_;
//boost::shared_ptr<ParticleFilter> tracker_;
//bool new_cloud_;
//double downsampling_grid_size_;
//int counter;

//void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m);

//std::string coutcolor0 = "\033[0;0m";
//std::string coutcolor_red = "\033[0;31m";
//std::string coutcolor_green = "\033[0;32m";
//std::string coutcolor_blue = "\033[0;34m";
//std::string coutcolor_magenta = "\033[0;35m";
//std::string coutcolor_brown = "\033[0;33m";

//using std::string;
//using std::exception;
//using std::cout;
//using std::cerr;
//using std::endl;

//using namespace std;
//using namespace boost;
//using namespace pcl;

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

//typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloudT;
//typedef pcl::PointXYZ PointT;

//bool next_iteration = false;
//bool app_exit = false;
//bool cloud_ready = false;

//typedef pcl::PointXYZ Point;
//PointCloudT::Ptr output(new PointCloudT);

//void printMatix4f(const Eigen::Matrix4f & matrix)
//{
//    printf ("Rotation matrix :\n");
//    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0,0), matrix (0,1), matrix (0,2));
//    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1,0), matrix (1,1), matrix (1,2));
//    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2,0), matrix (2,1), matrix (2,2));
//    printf ("Translation vector :\n");
//    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0,3), matrix (1,3), matrix (2,3));
//}

//void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
//{
//    if (event.getKeySym() == "space" && event.keyDown())
//        next_iteration = true;

//    if (event.getKeySym() == "q" && event.keyDown())
//    {
//        app_exit = true;
//    }
//}

//struct val_3
//{
//    float X;
//    float Y;
//    float Z;
//};

//void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
//{
//  pcl::PassThrough<pcl::PointXYZRGBA> pass;
//  pass.setFilterFieldName ("z");
//  pass.setFilterLimits (0.0, 10.0);
//  pass.setKeepOrganized (false);
//  pass.setInputCloud (cloud);
//  pass.filter (result);
//}


//void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
//{
//  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
//  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
//  grid.setInputCloud (cloud);
//  grid.filter (result);
//}

////Draw the current particles
//bool
//drawParticles (pcl::visualization::PCLVisualizer& viz)
//{
//  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
//  if (particles && new_cloud_)
//    {
//      //Set pointCloud with particle's points
//      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
//      for (size_t i = 0; i < particles->points.size (); i++)
//    {
//      pcl::PointXYZ point;

//      point.x = particles->points[i].x;
//      point.y = particles->points[i].y;
//      point.z = particles->points[i].z;
//      particle_cloud->points.push_back (point);
//    }

//      //Draw red particles
//      {
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

//    if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
//      viz.addPointCloud (particle_cloud, red_color, "particle cloud");
//      }
//      return true;
//    }
//  else
//    {
//      return false;
//    }
//}

////Draw model reference point cloud
//void drawResult (pcl::visualization::PCLVisualizer& viz)
//{
//  ParticleXYZRPY result = tracker_->getResult ();
//  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

//  //move close to camera a little for better visualization
//  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
//  CloudPtr result_cloud (new Cloud ());
//  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

//  //Draw blue model reference point cloud
//  {
//    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

//    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
//      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
//  }
//}

////visualization's callback function
//void
//viz_cb (pcl::visualization::PCLVisualizer& viz)
//{
//  boost::mutex::scoped_lock lock (mtx_);

//  if (!cloud_pass_)
//    {
//      boost::this_thread::sleep (boost::posix_time::seconds (1));
//      return;
//   }

//  //Draw downsampled point cloud from sensor
//  if (new_cloud_ && cloud_pass_downsampled_)
//    {
//      CloudPtr cloud_pass;
//      cloud_pass = cloud_pass_downsampled_;

//      if (!viz.updatePointCloud (cloud_pass, "cloudpass"))
//    {
//      viz.addPointCloud (cloud_pass, "cloudpass");
//      viz.resetCameraViewpoint ("cloudpass");
//    }
//      bool ret = drawParticles (viz);
//      if (ret)
//        drawResult (viz);
//    }
//  new_cloud_ = false;
//}

////OpenNI Grabber's cloud Callback function
//void
//cloud_cb (const CloudConstPtr &cloud)
//{
//  boost::mutex::scoped_lock lock (mtx_);
//  cloud_pass_.reset (new Cloud);
//  cloud_pass_downsampled_.reset (new Cloud);
//  filterPassThrough (cloud, *cloud_pass_);
//  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

//  if(counter < 10){
//    counter++;
//  }else{
//    //Track the object
//    tracker_->setInputCloud (cloud_pass_downsampled_);
//    tracker_->compute ();
//    new_cloud_ = true;
//  }
//}

//void tracker_main()
//{
//    target_cloud.reset(new Cloud());

//    //load target

//      counter = 0;

//      //Set parameters
//      new_cloud_  = false;
//      downsampling_grid_size_ =  0.002;

//      std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
//      default_step_covariance[3] *= 40.0;
//      default_step_covariance[4] *= 40.0;
//      default_step_covariance[5] *= 40.0;

//      std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
//      std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

//      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
//        (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

//      ParticleT bin_size;
//      bin_size.x = 0.1f;
//      bin_size.y = 0.1f;
//      bin_size.z = 0.1f;
//      bin_size.roll = 0.1f;
//      bin_size.pitch = 0.1f;
//      bin_size.yaw = 0.1f;


//      //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
//      tracker->setMaximumParticleNum (1000);
//      tracker->setDelta (0.99);
//      tracker->setEpsilon (0.2);
//      tracker->setBinSize (bin_size);

//      //Set all parameters for  ParticleFilter
//      tracker_ = tracker;
//      tracker_->setTrans (Eigen::Affine3f::Identity ());
//      tracker_->setStepNoiseCovariance (default_step_covariance);
//      tracker_->setInitialNoiseCovariance (initial_noise_covariance);
//      tracker_->setInitialNoiseMean (default_initial_mean);
//      tracker_->setIterationNum (1);
//      tracker_->setParticleNum (600);
//      tracker_->setResampleLikelihoodThr(0.00);
//      tracker_->setUseNormal (false);


//      //Setup coherence object for tracking
//      ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
//        (new ApproxNearestPairPointCloudCoherence<RefPointType> ());

//      boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
//        = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
//      coherence->addPointCoherence (distance_coherence);

//      boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
//      coherence->setSearchMethod (search);
//      coherence->setMaximumDistance (0.01);

//      tracker_->setCloudCoherence (coherence);

//      //prepare the model of tracker's target
//      Eigen::Vector4f c;
//      Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
//      CloudPtr transed_ref (new Cloud);
//      CloudPtr transed_ref_downsampled (new Cloud);

//      pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
//      trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
//      pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
//      gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

//      //set reference model and trans
//      tracker_->setReferenceCloud (transed_ref_downsampled);
//      tracker_->setTrans (trans);

//      //Setup OpenNIGrabber and viewer
//      pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI Tracking Viewer");
//      //pcl::Grabber* interface = new pcl::OpenNIGrabber (device_id);
//     // boost::function<void (const CloudConstPtr&)> f =
//      //  boost::bind (&cloud_cb, _1);
//      //interface->registerCallback (f);

//      viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

//      //Start viewer and object tracking
//     // interface->start();
//     // while (!viewer_->wasStopped ())
//      //  boost::this_thread::sleep(boost::posix_time::seconds(1));
//     // interface->stop();
//}

//vector <val_3> points;

//void read_file()
//{
//    std::string line;
//    std::ifstream text;
//    text.open("/home/edwin/Target.txt", ios_base::in);

//    if (text.is_open())
//    {
//        getline(text,line);
//        while (text.good())
//        {
//            line = line.substr (0,line.size() - 2);
//            vector <string> fields;
//            boost::split( fields, line, boost::is_any_of( "," ) );
//            val_3 mpoint;
//            mpoint.X = atof(fields[0].c_str());
//            mpoint.Y = atof(fields[1].c_str());
//            mpoint.Z = atof(fields[2].c_str());

//            points.push_back(mpoint);
//            getline(text,line);

//        }
//        text.close();
//        //std::cout<<points.size()<<endl;
//    }
//    else
//    {
//        std::cout << "Unable to open file" << std::endl << std::endl;
//    }


//}

////pcl::PolygonMesh ICP_Generate_singleMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
////{
////    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
////    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
////    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
////    tree->setInputCloud (cloud);
////    n.setInputCloud (cloud);
////    n.setSearchMethod (tree);
////    n.setKSearch (40);
////    n.compute (*normals);

////    cout<<coutcolor_magenta<<"Normal Done"<<coutcolor0<<endl;

////    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr;
////    // Concatenate the XYZ and normal fields*
////    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
////    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

////    // Create search tree*
////    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
////    tree2->setInputCloud (cloud_with_normals);

////    // Initialize objects
////    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
////    pcl::PolygonMesh triangles;

////    // Set the maximum distance between connected points (maximum edge length)
////    gp3.setSearchRadius (0.05);

////    // Set typical values for the parameters
////    gp3.setMu (2.5);
////    gp3.setMaximumNearestNeighbors (100);
////    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
////    gp3.setMinimumAngle(M_PI/18); // 10 degrees
////    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
////    gp3.setNormalConsistency(false);

////    // Get result
////    gp3.setInputCloud (cloud_with_normals);
////    gp3.setSearchMethod (tree2);
////    gp3.reconstruct (triangles);


////    std::vector<int> parts = gp3.getPartIDs();
////    std::vector<int> states = gp3.getPointStates();

////    for ( int i = 0 ; i < parts.size() ; i++ )
////    {
////        cout<<i<<endl;
////    }

////    return triangles;
////}

//void get_features(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs)
//{

//    //COMPUTE NORMALS
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr treen (new pcl::search::KdTree<pcl::PointXYZ>);
//    treen->setInputCloud (cloud);
//    n.setInputCloud (cloud);
//    n.setSearchMethod (treen);
//    n.setKSearch (40);
//    n.compute (*normals);

//    // Create the FPFH estimation class, and pass the input dataset+normals to it
//    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
//    fpfh.setInputCloud (cloud);
//    fpfh.setInputNormals (normals);
//    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

//    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
//    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
//    pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
//    fpfh.setSearchMethod (tree);

//    // Output datasets
//    //pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

//    // Use all neighbors in a sphere of radius 5cm
//    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
//    fpfh.setRadiusSearch (0.05);

//    // Compute the features
//    fpfh.compute (*fpfhs);
//}

//void ICP_Align()
//{
//    //READ
//    PointCloudT::Ptr cloud_in1= PointCloudT::Ptr(new PointCloudT );
//    PointCloudT::Ptr cloud_in2=  PointCloudT::Ptr(new PointCloudT );
//    PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );

//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in1);

//    //CREATE A FAKE TRANSFORM
//    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
//    float theta = M_PI/4; // The angle of rotation in radians
//    transformation_matrix (0,0) = cos(theta);
//    transformation_matrix (0,1) = -sin(theta);
//    transformation_matrix (1,0) = sin(theta);
//    transformation_matrix (1,1) = cos(theta);
//    transformation_matrix (2,3) = 1;

//    pcl::transformPointCloud (*cloud_in1, *cloud_in2, transformation_matrix);

//    //COMPUTE 3D FEATURES
//    LocalFeatures::Ptr fpfhs1(new LocalFeatures);
//    get_features(cloud_in1,fpfhs1);
//    LocalFeatures::Ptr fpfhs2(new LocalFeatures);
//    get_features(cloud_in2,fpfhs2);

//    //SAC ALIGN
//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;

//    sac_ia.setMinSampleDistance (0.05f);
//    sac_ia.setMaximumIterations (500);
//    sac_ia.setInputCloud (cloud_in1);
//    sac_ia.setInputTarget (cloud_in2);
//    sac_ia.setSourceFeatures (fpfhs1);
//    sac_ia.setTargetFeatures (fpfhs2);
//    sac_ia.align(*registration_output);

//    Eigen::Matrix4f trans = sac_ia.getFinalTransformation();


//    cout<<"ALIGN OUTPUT TRANS"<<endl;
//    printMatix4f(trans);

//    //pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> sac;
//    //sac.setInputCloud(source);
//    //sac.setTargetCloud(target);
//    //sac.setInlierThreshold(epsilon);
//    //sac.setMaxIterations(N);
//    //sac.setInputCorrespondences(correspondences);
//    //sac.getCorrespondences(inliers);
//    //Eigen::Matrix4f transformation = sac.getBestTransformation();

//    pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");
//    // Create two verticaly separated viewports

//    int vv(0);
//    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//    viewer.setBackgroundColor(1,1,1);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in1, 255, 20, 20);
//    viewer.addPointCloud (cloud_in1, cloud_in_color_h, "v1", vv);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_in2, 20, 20, 255);
//    viewer.addPointCloud (cloud_in2, cloud_icp_color_h, "v2", vv);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp2_color_h (registration_output, 0, 0, 0);
//    viewer.addPointCloud (registration_output, cloud_icp2_color_h, "v3", vv);

//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v1");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v2");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "v3");

//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024); // Visualiser window size
//    viewer.spin();
//}

//void ICP_SHOW_FPFH_Features()
//{
//    PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//    //fpfhs = LocalFeatures::Ptr (new LocalFeatures);
//    LocalFeatures::Ptr fpfhs(new LocalFeatures);
//    get_features(cloud_in,fpfhs);

//    pcl::visualization::PCLHistogramVisualizer hist;
//    hist.addFeatureHistogram(*fpfhs,"fpfh",33,"cloud",640,200);
//    hist.spin();

//    //Sampling
//    //PointCloud<int> indices;
//    //UniformSampling<PointT> uniform_sampling;
//    //uniform_sampling.setInputCloud (cloud);
//    //uniform_sampling.setRadiusSearch (0.05f);
//    //uniform_sampling.compute (indices);

//    //Correspondence
//    //CorrespondencesPtr corresps(new Correspondences);
//    //CorrespondenceEstimation<PointT, PointT> est;
//    //est.setInputSource (source_cloud);
//    //est.setInputTarget (target_cloud);
//    //est.determineCorrespondences (*corresps, max_dist);

//    //estimation

//    //pcl::registration::TransformationEstimationPointToPlaneWeighted<PointXYZ, PointXYZ, double> te;
//    //te.setWeights (correspondence_weights);
//    //te.estimateRigidTransformation (*cloud_src, *cloud_tgt,*corresps_filtered, transform);


//}



//void ICP_Generate_Surface()
//{
//    //Generate Mesh
//    //ICP_Generate_Surface();
//    cout<<coutcolor_magenta<<"Surface Core Started"<<coutcolor0<<endl;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

//    // Normal estimation*
//    cout<<coutcolor_magenta<<"Normal Estimation"<<coutcolor0<<endl;

//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
//    tree->setInputCloud (cloud);
//    n.setInputCloud (cloud);
//    n.setSearchMethod (tree);
//    n.setKSearch (40);
//    n.compute (*normals);

//    cout<<coutcolor_magenta<<"Normal Done"<<coutcolor0<<endl;

//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerr;
//    // Concatenate the XYZ and normal fields*
//    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);

//    // Create search tree*
//    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
//    tree2->setInputCloud (cloud_with_normals);

//    // Initialize objects
//    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//    pcl::PolygonMesh triangles;

//    // Set the maximum distance between connected points (maximum edge length)
//    gp3.setSearchRadius (0.05);

//    // Set typical values for the parameters
//    gp3.setMu (2.5);
//    gp3.setMaximumNearestNeighbors (100);
//    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//    gp3.setMinimumAngle(M_PI/18); // 10 degrees
//    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//    gp3.setNormalConsistency(false);

//    // Get result
//    gp3.setInputCloud (cloud_with_normals);
//    gp3.setSearchMethod (tree2);
//    gp3.reconstruct (triangles);

//    // Additional vertex information
//    std::vector<int> parts = gp3.getPartIDs();
//    std::vector<int> states = gp3.getPointStates();

//    // Visualization
//    pcl::visualization::PCLVisualizer viewer ("SURFACE");
//    // Create two verticaly separated viewports

//    int vv(0);
//    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//    viewer.setBackgroundColor(1,1,1);

//    // Original point cloud is white
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud, 200, 60, 130);
//    viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v2", vv);
//    viewer.addPolygonMesh(triangles,"mesh",vv);
//    pcl::io::saveVTKFile ("mesh.vtk", triangles);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud_in_v2");
//    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.02, "normals",vv);
//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024); // Visualiser window size
//    viewer.spin();

//}



//void ICP_Thread()
//{
//    PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//    PointCloudT::Ptr cloud_tr	(new PointCloudT);      //Transformed point cloud
//    PointCloudT::Ptr cloud_icp	(new PointCloudT);      //ICP output point cloud
//    PointCloudT::Ptr cloud_voxel (new PointCloudT);     //Voxel Filtered

//    int iterations = 1;

//    //read_file();

//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile ("bun0.pcd", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//    //    printf ("\nLoaded file %s with %d points successfully\n\n", "TARGET", points.size());

//    //    cloud_in->width    = points.size() / 10 ;
//    //    cloud_in->height   = 1;
//    //    cloud_in->is_dense = false;
//    //    cloud_in->points.resize (cloud_in->width * cloud_in->height);

//    //    for (size_t i = 0; i < cloud_in->points.size() ; ++i)
//    //    {
//    //        cloud_in->points[i].x = points[i].X;
//    //        cloud_in->points[i].y = points[i].Y;
//    //        cloud_in->points[i].z = points[i].Z;
//    //    }

//    cout<<"IN SIZE "<< cloud_in->points.size ()<<endl;
//    //cout<<"IN FILTERED SIZE"<< cloud_voxel->points.size()<<endl;
//    //==================================================================== Cloude IN

//    //====================================================================
//    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

//    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//    float theta = M_PI/4; // The angle of rotation in radians
//    transformation_matrix (0,0) = cos(theta);
//    transformation_matrix (0,1) = -sin(theta);
//    transformation_matrix (1,0) = sin(theta);
//    transformation_matrix (1,1) = cos(theta);

//    // A translation on Z axis (0.4 meters)
//    transformation_matrix (2,3) = 1;

//    // Display in terminal the transformation matrix
//    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
//    printMatix4f(transformation_matrix);

//    // Executing the transformation
//    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
//    *cloud_tr = *cloud_icp; // We backup cloud_icp into cloud_tr for later use

//    // The Iterative Closest Point algorithm
//    std::cout << "Initial iterations number is set to : " << iterations;
//    pcl::IterativeClosestPoint<PointT, PointT> icp;
//    icp.setMaximumIterations(iterations);
//    icp.setInputSource(cloud_icp);
//    icp.setInputTarget(cloud_in);
//    icp.align(*cloud_icp);
//    icp.setMaximumIterations(1); // For the next time we will call .align() function

//    if (icp.hasConverged()) {
//        printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
//        transformation_matrix = icp.getFinalTransformation();
//        printMatix4f(transformation_matrix);
//    } else {
//        PCL_ERROR ("\nICP has not converged.\n");
//        return ;
//    }

//    // Visualization
//    pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");
//    // Create two verticaly separated viewports

//    int vv(0);
//    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//    viewer.setBackgroundColor(1,1,1);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255, 20, 20);
//    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", vv);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 20, 20, 255);
//    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", vv);

//    std::stringstream ss; ss << iterations;
//    std::string iterations_cnt = "Iterations = " + ss.str();
//    viewer.addText(iterations_cnt, 10, 60, 16, 0, 0, 0, "iterations_cnt", vv);

//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_in_v2");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_icp_v2");

//    // Set camera position and orientation
//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024); // Visualiser window size

//    // Register keyboard callback :
//    viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*) NULL);

//    pcl::PolygonMesh mesh1 = ICP_Generate_singleMesh(cloud_in);
//    viewer.addPolygonMesh(mesh1,"mesh1",vv);

//    pcl::PolygonMesh mesh2 = ICP_Generate_singleMesh(cloud_icp);
//    viewer.addPolygonMesh(mesh2,"mesh2",vv);

//    // Display the visualiser
//    while (!viewer.wasStopped ()) {
//        viewer.spinOnce ();

//        // The user pressed "space" :
//        if (next_iteration) {
//            icp.align(*cloud_icp);

//            if (icp.hasConverged()) {
//                printf("\033[11A"); // Go up 11 lines in terminal output.
//                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
//                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
//                transformation_matrix *= icp.getFinalTransformation();	// This is not very accurate !
//                printMatix4f(transformation_matrix);					// Print the transformation between original pose and current pose

//                ss.str (""); ss << iterations;
//                std::string iterations_cnt = "ICP iterations = " + ss.str();
//                viewer.updateText (iterations_cnt, 10, 60, 16, 0, 0, 0, "iterations_cnt");
//                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");

//                mesh2 = ICP_Generate_singleMesh(cloud_icp);
//                viewer.updatePolygonMesh(mesh2,"mesh2");

//            } else {
//                PCL_ERROR ("\nICP has not converged.\n");
//                return;
//            }
//        }

//        if ( app_exit ) break;
//        next_iteration = false;
//    }

//    cout<<"Viewer Closed"<<endl;
//    viewer.close();
//    cout<<"THREAD END"<<endl;
//    return;
//}



//int mainAlgorithmLoop (double stiffness, double minStiffness, double stiffnessStep, double maxDistance,
//                       vtkSmartPointer<vtkPolyData> pointsTargetpoly, vtkSmartPointer<vtkPolyData> transformedPointsTemplatepoly )
//{

//    //    vtkSmartPointer<vtkPoints> pointsTarget = vtkSmartPointer<vtkPoints>::New();
//    //    vtkSmartPointer<vtkPoints> transformedPointsTemplate = vtkSmartPointer<vtkPoints>::New();

//    //    const std::size_t V1 = pointsTargetpoly->GetNumberOfPoints();
//    //    const std::size_t V2 = transformedPointsTemplatepoly->GetNumberOfPoints();

//    //    for(int i = 0; i < V1; i++)
//    //    {
//    //        double point[3];
//    //        pointsTargetpoly->GetPoint(i, point);
//    //        pointsTarget->InsertNextPoint(point[0],point[1],point[2]);
//    //    }

//    //    int pointCountTemplate = V2;

//    //    for(int i = 0; i < V2; i++)
//    //    {
//    //        double point[3];
//    //        transformedPointsTemplatepoly->GetPoint(i, point);
//    //        transformedPointsTemplate->InsertNextPoint(point[0],point[1],point[2]);
//    //    }

//    //    cout<<"Pionts Target Vertex Count : "<<pointsTarget->GetNumberOfPoints()<<endl;
//    //    cout<<"Points Template Vertex Count : "<<transformedPointsTemplate->GetNumberOfPoints()<<endl;

//    //    //========================================================================================================
//    //    //Build Octtree
//    //    vtkSmartPointer<vtkOctreePointLocator> octreeResizedTarget = vtkSmartPointer<vtkOctreePointLocator>::New();
//    //    octreeResizedTarget->SetDataSet(pointsTargetpoly);
//    //    octreeResizedTarget->BuildLocator();

//    //    //extract template edges
//    //    vtkSmartPointer<vtkExtractEdges> extractEdges = vtkSmartPointer<vtkExtractEdges>::New();
//    //    extractEdges->SetInput(transformedPointsTemplatepoly);
//    //    extractEdges->Update();
//    //    vtkCellArray* linesTemplate = extractEdges->GetOutput()->GetLines();

//    //    cout<<"RUN LOOP"<<endl;
//    //    int correspondences[pointCountTemplate];
//    //    int currentCorrespondence;

//    //    double distance = 0;
//    //    int discarded = 0;
//    //    int errorInt = 0;
//    //    double epsilon = 1;

//    //    MatrixXd Xmatrix(4*pointCountTemplate, 3);
//    //    Xmatrix.setZero();

//    //    // The loop should start here
//    //    std::cout << "Overall points in the mesh: " << pointCountTemplate << std::endl;
//    //    std::cout << "Processing..." << std::endl;

//    //        while (stiffness > minStiffness) {

//    //            int loop = 1;
//    //            while (loop) {

//    //                for(int i = 0; i < pointCountTemplate; ++i)
//    //                {
//    //                    double point[3];
//    //                    transformedPointsTemplate->GetPoint(i, point);
//    //                    currentCorrespondence = octreeResizedTarget->FindClosestPoint( point[0], point[1], point[2], distance );
//    //                    if (distance < maxDistance) {
//    //                        correspondences[i] = currentCorrespondence;
//    //                    }
//    //                    else {
//    //                        correspondences[i] = -1;
//    //                        discarded++;
//    //                    }
//    //                }

//    //                std::cout << "Discarded points in this round of finding correspondences: " << discarded << std::endl;
//    //                discarded = 0;

//    //                // Equation

//    //                // Use the extract edges
//    //                int linesCountTemplate = linesTemplate->GetNumberOfCells();
//    //                cout<<"lines : "<<linesCountTemplate<<endl;

//    //                linesTemplate->InitTraversal();
//    //                vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();

//    //                // Matrix A

//    //                typedef Eigen::Triplet<double> T;
//    //                std::vector<T> aTripletList;
//    //                aTripletList.reserve(linesCountTemplate*8 + 4*pointCountTemplate);

//    //                linesTemplate->GetNextCell(idList);
//    //                for (int i = 0; i < linesCountTemplate; i++) {
//    //                    if (idList->GetId(0) < idList->GetId(1)) {
//    //                        for (int j = 0; j < 4; j++) {
//    //                            aTripletList.push_back(T(4*i + j, 4*idList->GetId(0) + j,-1*stiffness));
//    //                            aTripletList.push_back(T(4*i + j, 4*idList->GetId(1) + j,1*stiffness));
//    //                        }
//    //                    }
//    //                    else if (idList->GetId(1) < idList->GetId(0)) {
//    //                        for (int j = 0; j < 4; j++) {
//    //                            aTripletList.push_back(T(4*i + j, 4*idList->GetId(1) + j,-1*stiffness));
//    //                            aTripletList.push_back(T(4*i + j, 4*idList->GetId(0) + j,1*stiffness));
//    //                        }
//    //                    }
//    //                    linesTemplate->GetNextCell(idList);
//    //                }

//    //                for (int i = 0; i < pointCountTemplate; i++) {
//    //                    double point[3];
//    //                    int w = correspondences[i]!=-1 ? 1 : 0;
//    //                    transformedPointsTemplate->GetPoint(i, point);
//    //                    for (int j = 0; j < 3; j++) {
//    //                        aTripletList.push_back(T(4*linesCountTemplate + i, i*4 + j, point[j]*w));
//    //                    }
//    //                    aTripletList.push_back(T(4*linesCountTemplate + i, i*4 + 3, 1*w));
//    //                }

//    //                SparseMatrix<double> Amatrix(4*linesCountTemplate + pointCountTemplate, 4*pointCountTemplate);
//    //                Amatrix.setFromTriplets(aTripletList.begin(), aTripletList.end());

//    //                SparseMatrix<double> AmatrixT(4*pointCountTemplate, 4*linesCountTemplate + pointCountTemplate);
//    //                AmatrixT = Amatrix.transpose();
//    //                SparseMatrix<double> AmatrixFinal(4*pointCountTemplate, 4*pointCountTemplate);
//    //                AmatrixFinal = AmatrixT * Amatrix;

//    //                // Matrix B

//    //                MatrixXd Bmatrix(4*linesCountTemplate + pointCountTemplate, 3);
//    //                Bmatrix.setZero();

//    //                for (int i = 0; i < pointCountTemplate; i++) {
//    //                    double point[3];
//    //                    int w = correspondences[i]!=-1 ? 1 : 0;
//    //                    pointsTarget->GetPoint(correspondences[i], point);
//    //                    for (int j = 0; j < 3; j++) {
//    //                        Bmatrix(4*linesCountTemplate + i, j) = point[j]*w;
//    //                    }
//    //                }

//    //                MatrixXd BmatrixFinal(4*pointCountTemplate, 3);
//    //                BmatrixFinal.setZero();
//    //                BmatrixFinal = AmatrixT * Bmatrix;

//    //                // Temporal Matrix X

//    //                MatrixXd TempXmatrix(4*pointCountTemplate, 3);
//    //                TempXmatrix = Xmatrix;

//    //                // Solver

//    //                clock_t start;
//    //                double duration;

//    //                start = clock();

//    //                //BiCGSTAB<SparseMatrix<double> > solver;
//    //                //AmatrixFinal.makeCompressed();
//    //                //solver.compute(AmatrixFinal);
//    //                //Xmatrix = solver.solve(BmatrixFinal);

//    //                SparseLU<SparseMatrix<double, ColMajor>, COLAMDOrdering<int> > solver;
//    //                solver.analyzePattern(AmatrixFinal);
//    //                solver.factorize(AmatrixFinal);
//    //                Xmatrix = solver.solve(BmatrixFinal);

//    //                duration = ( clock() - start ) / (double) CLOCKS_PER_SEC;

//    //                std::cout<< "Duration of this solver round: " << duration << " seconds" << std::endl;

//    //                // Transformation

//    //                for (int i = 0; i < pointCountTemplate; i++) {
//    //                    double point[3];
//    //                    double result[3];
//    //                    transformedPointsTemplate->GetPoint(i, point);

//    //                    for (int j = 0; j < 3; j++) {
//    //                        result[j] = point[0] * Xmatrix(i*4 + 0,j) + point[1] * Xmatrix(i*4 + 1,j) + point[2] * Xmatrix(i*4 + 2,j) + 1 * Xmatrix(i*4 + 3,j);
//    //                    }
//    //                    transformedPointsTemplate->InsertPoint(i, result);

//    //                }

//    //                //          The visualization window is updated.
//    //                //          The inner loop ends when the norm value is smaller than the epsilon value.
//    //                //          The outer loop ends after some iterations with different stiffness weights.

//    //                double norm = (Xmatrix - TempXmatrix).norm();

//    //                std::cout << "The norm value in this round (inner loop): "<< norm << std::endl << std::endl;

//    //                //loop = 0;
//    //                if (norm < epsilon || norm != norm) {
//    //                    loop = 0;
//    //                    if (norm != norm) {
//    //                        cout << "Bad Matrix Error!" << endl;
//    //                        errorInt = 1;
//    //                    }
//    //                }

//    //                //window->Finalize();
//    //                window->Render();
//    //                //window->Start();
//    //            }

//    //            std::cout << "The stiffness value in this round (outer loop): "<< stiffness << std::endl;
//    //            std::cout << "======================================================== "<< std::endl << std::endl;
//    //            stiffness -= stiffnessStep;
//    //            if (errorInt)
//    //            {
//    //                break;
//    //            }

//    //        }

//    //        return 0;
//}

//void CONVERT(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZ>::Ptr output)
//{
//    output->points.resize(input->size());

//    for (size_t i = 0; i < input->points.size(); i++)
//    {
//        output->points[i].x = input->points[i].x;
//        output->points[i].y = input->points[i].y;
//        output->points[i].z = input->points[i].z;
//    }

//    //Z Filter
//    const float depth_limit = 0.8;
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (output);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (0, depth_limit);
//    pass.filter (*output);

//    //voxel grid filtering
//    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
//    vox_grid.setInputCloud (output);
//    vox_grid.setLeafSize (0.01f, 0.01f, 0.01f);
//    vox_grid.filter (*output);

//    //noise removal
//    //pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    //sor.setInputCloud (output);
//    //sor.setMeanK (50);
//    //sor.setStddevMulThresh (1.0);
//    //sor.filter (*output);
//}

////void get_show_cloud()
////{
////    PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
////    //Read Template
////    pcl::PCLPointCloud2 cloud_blob;
////    pcl::io::loadPLYFile("body_template.ply", cloud_blob);
////    pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);


////    while (ros::ok())
////    {
////        if ( cloud_ready == false || global_cloud->width == 0 )
////        {
////            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
////            cout<<"wait for cloud"<<endl;
////        }
////        else
////        {
////            break;
////        }
////    }

////    cout<<"GET OUT"<<endl;

////    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

////    // Visualization
////    pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");

////    int vv(0);
////    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
////    viewer.setBackgroundColor(1,1,1);

////    PointCloudT::Ptr output(new PointCloudT);
////    CONVERT(global_cloud,output);

////    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (output, 20, 20, 20);
////    viewer.addPointCloud (output, cloud_in_color_h, "input", vv);
////    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");

////    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (cloud_in, 220, 20, 20);
////    viewer.addPointCloud (cloud_in, cloud_in_color_h2, "template", vv);
////    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "template");

////    // Set camera position and orientation
////    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
////    viewer.setSize(1280, 1024); // Visualiser window size



////    PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );

////    int do_c = 0;
////    // Display the visualiser
////    LocalFeatures::Ptr fpfhs1(new LocalFeatures);
////    get_features(cloud_in,fpfhs1);

////    while (!viewer.wasStopped ())
////    {
////        viewer.spinOnce ();
////        CONVERT(global_cloud,output);
////        viewer.updatePointCloud (output, cloud_in_color_h, "input");

////        if ( do_c >= 20 )
////        {
////        do_c = 0;

////        LocalFeatures::Ptr fpfhs2(new LocalFeatures);
////        get_features(output,fpfhs2);

////        //SAC ALIGN
////        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;

////        sac_ia.setMinSampleDistance (0.05f);
////        sac_ia.setMaximumIterations (500);
////        sac_ia.setInputCloud (cloud_in);
////        sac_ia.setInputTarget (output);
////        sac_ia.setSourceFeatures (fpfhs1);
////        sac_ia.setTargetFeatures (fpfhs2);
////        sac_ia.align(*registration_output);

////        Eigen::Matrix4f trans = sac_ia.getFinalTransformation();


////        cout<<"ALIGN OUTPUT TRANS"<<endl;
////        printMatix4f(trans);


////        viewer.updatePointCloud (registration_output, cloud_in_color_h2, "template");
////        }
////        //=======================================================================

////        do_c++;
////        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
////    }


////    //viewer.close();
////    return;
////}

//void get_show_cloud_ICP()
//{
//    PointCloudT::Ptr cloud_in 	(new PointCloudT);      //Original point cloud
//    //Read Template
//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPLYFile("face_template.ply", cloud_blob);
//    pcl::fromPCLPointCloud2 (cloud_blob, *cloud_in);

//    while (ros::ok())
//    {
//        if ( cloud_ready == false || global_cloud->width == 0 )
//        {
//            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//            cout<<"wait for cloud"<<endl;
//        }
//        else
//        {
//            break;
//        }
//    }

//    cout<<"GET OUT"<<endl;

//    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//    PointCloudT::Ptr output(new PointCloudT);
//    CONVERT(global_cloud,output);

//    cout<<"ALIGN START"<<endl;
//    PointCloudT::Ptr registration_output=  PointCloudT::Ptr(new PointCloudT );
//    //INITIAL ALIGNMENT
//    LocalFeatures::Ptr fpfhs1(new LocalFeatures);
//    get_features(cloud_in,fpfhs1);
//    LocalFeatures::Ptr fpfhs2(new LocalFeatures);
//    get_features(output,fpfhs2);

//    //SAC ALIGN
//    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
//    sac_ia.setMinSampleDistance (0.05f);
//    sac_ia.setMaximumIterations (500);
//    sac_ia.setInputCloud (cloud_in);
//    sac_ia.setInputTarget (output);
//    sac_ia.setSourceFeatures (fpfhs1);
//    sac_ia.setTargetFeatures (fpfhs2);
//    sac_ia.align(*registration_output);

//    Eigen::Matrix4f trans = sac_ia.getFinalTransformation();

//    cout<<"ALIGN OUTPUT TRANS"<<endl;
//    printMatix4f(trans);

//    cout<<"ALIGN DONE"<<endl;

//    // Visualization
//    pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");

//    int vv(0);
//    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//    viewer.setBackgroundColor(1,1,1);

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (output, 20, 20, 20);
//    viewer.addPointCloud (output, cloud_in_color_h, "input", vv);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");

//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h2 (registration_output, 220, 20, 20);
//    viewer.addPointCloud (registration_output, cloud_in_color_h2, "template", vv);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "template");

//    // Set camera position and orientation
//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024); // Visualiser window size

//   // pcl::io::savePLYFile("body_out.ply", *output,false);

//    int doc_count = 0;

//    while (!viewer.wasStopped ())
//    {

//        viewer.spinOnce ();
//        CONVERT(global_cloud,output);
//        viewer.updatePointCloud (output, cloud_in_color_h, "input");
//        viewer.updatePointCloud (registration_output, cloud_in_color_h2, "template");



//            int iterations = 1;
//            pcl::IterativeClosestPoint<PointT, PointT> icp;
//            icp.setMaximumIterations(iterations);
//            icp.setInputSource(registration_output);
//            icp.setInputTarget(output);
//            icp.align(*registration_output);
//            icp.setMaximumIterations(15); // For the next time we will call .align() function



//        boost::this_thread::sleep(boost::posix_time::milliseconds(20));
//    }

//    //viewer.close();
//    return;
//}



//void save_file()
//{
//    std::string path = "";

//    path = "/home/edwin/cpd/build/bin/X.txt";

//    ofstream myfile;
//    myfile.open (path.c_str());

//    int V1 = 0;
//    V1 = output->points.size();

//    for ( int i = 0 ; i < V1 ; i++ )
//    {
//         pcl::PointXYZ item = output->points.at(i);
//         myfile << (item.x * 100) << "," << (item.y * 100) << "," << (item.z * 100) << endl;
//    }

//    myfile.close();

//    cout<<"X Updated..."<<endl;
//}

//void Create_Cloud_for_Registration()
//{
//    while (ros::ok())
//    {
//        if ( cloud_ready == false || global_cloud->width == 0 )
//        {
//            boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
//            cout<<"wait for cloud"<<endl;
//        }
//        else
//        {
//            break;
//        }
//    }

//    cout<<"GET OUT READY - LOOK AT TO KINECT PLEASE TO INITIAL ALIGNMENT"<<endl;
//    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

//    CONVERT(global_cloud,output);


//    pcl::visualization::PCLVisualizer viewer ("Edwin Babaians - Non rigid ICP - Amirkabir University of Technology");

//    int vv(0);
//    viewer.createViewPort (0.0, 0.0, 1.0, 1.0, vv);
//    viewer.setBackgroundColor(1,1,1);
//    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (output, 20, 20, 20);
//    viewer.addPointCloud (output, cloud_in_color_h, "input", vv);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "input");
//    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
//    viewer.setSize(1280, 1024); // Visualiser window size

//    pcl::io::savePLYFile("/home/edwin/target_ply.ply", *output,false);

//    while (1)
//    {
//        viewer.spinOnce ();
//        CONVERT(global_cloud,output);
//        //save_file();
//        viewer.updatePointCloud (output, cloud_in_color_h, "input");
//        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
//    }

//    return;
//}


//void Cloud_CallBack(const sensor_msgs::PointCloud2ConstPtr &cloud_m)
//{
//    if (cloud_m->width != 0 )
//    {
//        cloud_ready = true;
//        pcl::fromROSMsg(*cloud_m, *global_cloud);
//    }
//}

//int main(int argc, char **argv)
//{
//    ros::init(argc, argv, "icp");
//    ros::Time::init();
//    cout<<"ICP CORE STARTED DONE"<<endl;

//    //convert_CT_PLY_to_XYZ_Y();
//    //boost::thread _thread_logic(&ICP_Thread);
//    //boost::thread _thread_logic(&ICP_Generate_Surface);
//    //boost::thread _thread_logic(&ICP_Align);
//    boost::thread _thread_logic(&Create_Cloud_for_Registration);
//    // boost::thread _thread_logic(&CPD_main);

//    ros::Rate loop_rate(20);

//    ros::NodeHandle nh_a[15];
//    ros::Subscriber sub1 = nh_a[0].subscribe("/camera/depth_registered/points", 1, Cloud_CallBack);

//    while (ros::ok() && app_exit == false)
//    {
//        ros::spinOnce();
//        loop_rate.sleep();
//    }

//    _thread_logic.interrupt();
//    _thread_logic.join();

//    return 0;
//}
