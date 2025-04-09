#include "stereo-slam-node.hpp"

#include<opencv2/core/core.hpp>

#include <ros_utils.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

StereoSlamNode::StereoSlamNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify)
:   Node("ORB_SLAM3_ROS2"),
    m_SLAM(pSLAM)
{
    stringstream ss(strDoRectify);
    ss >> boolalpha >> doRectify;

    if (doRectify){

        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if(!fsSettings.isOpened()){
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0){
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
    }

    left_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/infra1/image_rect_raw");
    right_sub = std::make_shared<message_filters::Subscriber<ImageMsg> >(shared_ptr<rclcpp::Node>(this), "camera/infra2/image_rect_raw");

    pubPose_ = this->create_publisher<PoseMsg>("camera_pose", 1);
    pubTrackImage_ = this->create_publisher<ImageMsg>("tracking_image", 1);
    pubPcd_ = this->create_publisher<PcdMsg>("point_cloud", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy> >(approximate_sync_policy(10), *left_sub, *right_sub);
    syncApproximate->registerCallback(&StereoSlamNode::GrabStereo, this);

      // declare rosparameters
    this->declare_parameter("world_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("camera_frame", "odom");
    this->declare_parameter("camera_optical_frame", "odom");
}

StereoSlamNode::~StereoSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoSlamNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Copy the ros rgb image message to cv::Mat.
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Copy the ros depth image message to cv::Mat.
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw;
    if (doRectify){
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = m_SLAM->TrackStereo(imLeft, imRight, Utility::StampToSec(msgLeft->header.stamp));
    }
    else
    {
        Tcw = m_SLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, Utility::StampToSec(msgLeft->header.stamp));
    }
    Sophus::SE3f Twc = Tcw.inverse(); // camera optical frame pose in opencv coordinate

    // publish topics
    std::string world_frame = this->get_parameter("world_frame").as_string();
    std::string odom_frame = this->get_parameter("odom_frame").as_string();
    std::string camera_frame = this->get_parameter("camera_frame").as_string();
    std::string camera_optical_frame = this->get_parameter("camera_optical_frame").as_string();

    // define coordinate transforms ///
    // OpenCV to ROS FLU coordinate transforms
    Eigen::Matrix<float, 3, 3> cv_to_ros_rot; 
    Eigen::Matrix<float, 3, 1> cv_to_ros_trans; 
    cv_to_ros_rot << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    cv_to_ros_trans << 0, 0, 0;
    Sophus::SE3f cv_to_ros(cv_to_ros_rot, cv_to_ros_trans);
    std::cout << cv_to_ros.matrix() << std::endl; 

    // Coordinate Transform: OpenCV coordinate to ROS FLU coordinate
    Twc = cv_to_ros * Twc; // camera optical frame pose in ROS FLU map coorinate
    Twc = Twc * cv_to_ros.inverse(); // camera frame pose in ROS FLU map coorinate

    // Option1: publish map to odom tf from SLAM and odom to camera from VIO 
    // TF processing ////
    try {
        geometry_msgs::msg::TransformStamped camera_to_odom = tf_buffer_->lookupTransform(camera_frame, odom_frame, tf2::TimePointZero);
        Sophus::SE3f Tco= transform_to_SE3(camera_to_odom);
        Sophus::SE3f Two = Twc * Tco.inverse();
        publish_world_to_odom_tf(tf_broadcaster_, this->get_clock()->now(), Two, world_frame, odom_frame);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
        this->get_logger(), "Could not get transform %s to %s: %s",
        camera_frame.c_str(), odom_frame.c_str(), ex.what());
        return;
    }

    // Option2: publish map to camera tf from SLAM
    // publish_camera_tf(tf_broadcaster_, this->get_clock()->now(), Twc, world_frame, camera_frame);
    publish_camera_pose(pubPose_, this->get_clock()->now(), Twc, world_frame);
    publish_tracking_img(pubTrackImage_, this->get_clock()->now(), m_SLAM->GetCurrentFrame(), world_frame);
}
