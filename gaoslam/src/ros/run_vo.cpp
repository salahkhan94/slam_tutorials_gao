#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include "gaoslam/config.h"
#include "gaoslam/visual_odometry.h"
#include <cv_bridge/cv_bridge.h>

using namespace std;

class GrabandRun
{
public:
    GrabandRun()
    {
        myslam::Config::setParameterFile ( argv[1] );
        myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

        string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
        cout<<"dataset: "<<dataset_dir<<endl;
        ifstream fin ( dataset_dir+"/associate.txt" );
        if ( !fin )
        {
            cout<<"please generate the associate file called associate.txt!"<<endl;
            return 1;
        }
        myslam::Camera::Ptr camera ( new myslam::Camera );

        // visualization
        cv::viz::Viz3d vis ( "Visual Odometry" );
        cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
        cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
        cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
        vis.setViewerPose ( cam_pose );

        world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
        camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
        vis.showWidget ( "World", world_coor );
        vis.showWidget ( "Camera", camera_coor );
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void runvo(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,const cv::viz::Viz3d &vis)
    
};


int main ( int argc, char** argv )
{
    ros::init(argc, argv, "gaoslam");
    if ( argc != 2 )
    {
        cout<<"usage: run_vo parameter_file"<<endl;
        return 1;
    }
    GrabandRun igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth_registered/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&GrabandRun::GrabRGBD,&igb,_1,_2));

    ros::spin();
    return 0;
}
void GrabandRun::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    runvo(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
}

void GrabandRun::runvo(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,const cv::viz::Viz3d &vis)
{
    cout<<"****** loop "<<i<<" ******"<<endl;
    Mat color = im;
    Mat depth = depthmap;
    if ( color.data==nullptr || depth.data==nullptr )
        break;
    myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
    pFrame->camera_ = camera;
    pFrame->color_ = color;
    pFrame->depth_ = depth;
    pFrame->time_stamp_ = rgb_times[i];

    boost::timer timer;
    vo->addFrame ( pFrame );
    cout<<"VO costs time: "<<timer.elapsed() <<endl;

    if ( vo->state_ == myslam::VisualOdometry::LOST )
        break;
    SE3 Twc = pFrame->T_c_w_.inverse();

    // show the map and the camera pose
    cv::Affine3d M (
        cv::Affine3d::Mat3 (
            Twc.rotation_matrix() ( 0,0 ), Twc.rotation_matrix() ( 0,1 ), Twc.rotation_matrix() ( 0,2 ),
            Twc.rotation_matrix() ( 1,0 ), Twc.rotation_matrix() ( 1,1 ), Twc.rotation_matrix() ( 1,2 ),
            Twc.rotation_matrix() ( 2,0 ), Twc.rotation_matrix() ( 2,1 ), Twc.rotation_matrix() ( 2,2 )
        ),
        cv::Affine3d::Vec3 (
            Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
        )
    );

    Mat img_show = color.clone();
    for ( auto& pt:vo->map_->map_points_ )
    {
        myslam::MapPoint::Ptr p = pt.second;
        Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
        cv::circle ( img_show, cv::Point2f ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
    }

    cv::imshow ( "image", img_show );
    cv::waitKey ( 1 );
    vis.setWidgetPose ( "Camera", M );
    vis.spinOnce ( 1, false );
    cout<<endl;
}

"""
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    sub1 = n_.subscribe("rgb/camera_info", 5, &SubscribeAndPublish::Callback1, this);
    sub2 = n_.subscribe("depth/camera_info", 5, &SubscribeAndPublish::Callback2, this);
    sub3 = n_.subscribe("rgb/image_raw", 5, &SubscribeAndPublish::Callback3, this);
    sub4 = n_.subscribe("depth/image_raw", 5, &SubscribeAndPublish::Callback4, this);
  }

  void Callback1(const sensor_msgs::CameraInfo::ConstPtr& data)
  {
    sensor_msgs::CameraInfo im;
    im.header = data->header;
    im.header.frame_id = "zed";
    im.height = data->height;
    im.width = data->width;
    im.distortion_model = data->distortion_model;
    im.D = data->D;
    im.K = data->K;
    im.P = data->P;
    im.binning_x = data->binning_x;
    im.binning_y = data->binning_y;
    im.roi = data->roi;
  }

  void Callback2(const sensor_msgs::CameraInfo::ConstPtr& data)
  {
    sensor_msgs::CameraInfo im;
    im.header = data->header;
    im.header.frame_id = "zed";
    im.height = data->height;
    im.width = data->width;
    im.distortion_model = data->distortion_model;
    im.D = data->D;
    im.K = data->K;
    im.P = data->P;
    im.binning_x = data->binning_x;
    im.binning_y = data->binning_y;
    im.roi = data->roi;
    pub2.publish(im);
  }

  void Callback3(const sensor_msgs::Image::ConstPtr& data)
  {
    sensor_msgs::Image im;
    im.header = data->header;
    im.header.frame_id = "zed";
    im.height = data->height;
    im.width = data->width;
    im.encoding = data->encoding;
    im.is_bigendian = data->is_bigendian;
    im.step = data->step;
    im.data = data->data;
    pub3.publish(im);
  }

  void Callback4(const sensor_msgs::Image::ConstPtr& data)
  {
    sensor_msgs::Image im;
    im.header = data->header;
    im.header.frame_id = "zed";
    im.height = data->height;
    im.width = data->width;
    im.encoding = data->encoding;
    im.is_bigendian = data->is_bigendian;
    im.step = data->step;
    im.data = data->data;
    pub4.publish(im);
  }
  
private:
  ros::NodeHandle n_; 
  //ros::Publisher pub1,pub2,pub3,pub4;
  ros::Subscriber sub1,sub2,sub3,sub4;
  tf::Quaternion q;
  //std::string indica_link; 
};
"""

