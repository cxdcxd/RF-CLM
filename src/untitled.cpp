


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


