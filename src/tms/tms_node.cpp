
#include <tms_node.hh>


int main (int argc, char** argv)
{
    ros::init(argc, argv, "tms_node");
    ros::Time::init();

    ROS_INFO("TMS STARTED");

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
