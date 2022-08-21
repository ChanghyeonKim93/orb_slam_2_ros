#include "MonoNode.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport (node_handle);

    MonoNode node (ORB_SLAM2::System::MONOCULAR, node_handle, image_transport);
    node.Init();

    ros::spin();

    ros::shutdown();

    return 0;
}


MonoNode::MonoNode (ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : Node (sensor, node_handle, image_transport) {
  image_subscriber = image_transport.subscribe ("/camera/image_raw", 1, &MonoNode::ImageCallback, this);
  camera_info_topic_ = "/camera/camera_info";

  pub_image_ = image_transport.advertise("/scale_mono_vo/external_vo/image",1);
  pub_pose_ = node_handle.advertise<geometry_msgs::PoseStamped> ("/scale_mono_vo/external_vo/pose", 1);
}


MonoNode::~MonoNode () {
}


void MonoNode::ImageCallback (const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr cv_in_ptr;
  try {
      cv_in_ptr = cv_bridge::toCvShare(msg);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  current_frame_time_ = msg->header.stamp;
  ROS_INFO_STREAM("ORB SLAM gets a new image!");
  orb_slam_->TrackMonocular(cv_in_ptr->image,cv_in_ptr->header.stamp.toSec());

  Update ();

  // CHK, get current position.
  cv::Mat position = orb_slam_->GetCurrentPosition();
  if (!position.empty()) {
    tf2::Transform tf_position = TransformFromMat(position);

    // Make transform from camera frame to target frame
    tf2::Transform tf_position_target = TransformToTarget(tf_position, "camera_link", "camera_link");
    
    // CHK, Make message
    tf2::Stamped<tf2::Transform> tf_position_target_stamped;
    tf_position_target_stamped = tf2::Stamped<tf2::Transform>(tf_position_target, ros::Time::now(), "map");
    geometry_msgs::PoseStamped pose_msg;
    tf2::toMsg(tf_position_target_stamped, pose_msg);
    pose_msg.pose.position.x *= 5.0f;
    pose_msg.pose.position.y *= 5.0f;
    pose_msg.pose.position.z *= 5.0f;
    pub_pose_.publish(pose_msg);

    // publish image.
    pub_image_.publish(*msg);
  }
}
