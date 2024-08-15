/**
*
*/

#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <ros/ros.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// AirVO
#include "read_configs.h"
#include "dataset.h"
#include "map_builder.h"

// 添加 sensor_msgs/Imu.h 头文件
#include <sensor_msgs/Imu.h>

MapBuilder* p_map_builder;

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

double last_imu_t = 0;

std::mutex m_buf;

std::queue<sensor_msgs::ImuConstPtr> imu_buf;

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        last_imu_t = imu_msg->header.stamp.toSec();
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
}

void GrabStereo(const sensor_msgs::ImageConstPtr& imgLeft, const sensor_msgs::ImageConstPtr& imgRight){
    
    ImuDataList batch_imu_data;
    if (p_map_builder->UseIMU()) {
      m_buf.lock();
      auto timeStampL = imgLeft->header.stamp.toSec();
      if (imu_buf.empty() || imu_buf.back()->header.stamp.toSec() < timeStampL) {
        ROS_WARN("wait for imu, only should happen at the beginning");
        m_buf.unlock();
        return; 
      }
      // std::vector<sensor_msgs::ImuConstPtr> imu_msg;
      while (!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < timeStampL) {
        // imu_msg.emplace_back(imu_buf.front());
        ImuData imu;
        sensor_msgs::ImuConstPtr imu_msg = imu_buf.front();
        imu.timestamp = imu_msg->header.stamp.toSec();
        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        double rx = imu_msg->angular_velocity.x;
        double ry = imu_msg->angular_velocity.y;
        double rz = imu_msg->angular_velocity.z;
        Eigen::Vector3d angular_velocity{rx, ry, rz};
        imu.gyr = angular_velocity;
        imu.acc = linear_acceleration;
        batch_imu_data.emplace_back(imu);

        imu_buf.pop();
      }
      m_buf.unlock();
    }

    // Copy the ros image messages to cvMat
    cv_bridge::CvImageConstPtr cv_ptrLeft, cv_ptrRight;
    
    try{
        cv_ptrLeft = cv_bridge::toCvShare(imgLeft);
        cv_ptrRight = cv_bridge::toCvShare(imgRight);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    static int frame_id = 0;
    auto before_infer = std::chrono::steady_clock::now();

    InputDataPtr input_data = std::shared_ptr<InputData>(new InputData());
    input_data->index = frame_id;
    input_data->image_left = cv_ptrLeft->image.clone();
    input_data->image_right = cv_ptrRight->image.clone();
    input_data->time = imgLeft->header.stamp.toSec();
    input_data->batch_imu_data = batch_imu_data;

    if(input_data == nullptr) return;
    p_map_builder->AddInput(input_data);

    auto after_infer = std::chrono::steady_clock::now();
    auto cost_time = std::chrono::duration_cast<std::chrono::milliseconds>
    (after_infer - before_infer).count();

    std::cout << "i ===== " << frame_id++ << " Processing Time: " << cost_time << " ms. imu: " << batch_imu_data.size() << std::endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "air_slam_ros");

    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1){
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    // ROS
    std::string left_topic, right_topic, imu_topic;
    ros::param::get("~left_topic", left_topic);
    ros::param::get("~right_topic", right_topic);
    ros::param::get("~imu_topic", imu_topic);
    ros::NodeHandle node_handler;
    message_filters::Subscriber<sensor_msgs::Image> sub_img_left(node_handler, left_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_img_right(node_handler, right_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_img_left, sub_img_right);
    sync.registerCallback(boost::bind(&GrabStereo, _1, _2));

    // AirVO
    std::string config_path, model_dir;
    ros::param::get("~config_path", config_path);
    ros::param::get("~model_dir", model_dir);
    
    VisualOdometryConfigs configs(config_path, model_dir);
    std::cout << "config done" << std::endl;

    ros::param::get("~dataroot", configs.dataroot);
    ros::param::get("~camera_config_path", configs.camera_config_path);
    ros::param::get("~saving_dir", configs.saving_dir);
    std::string traj_path;
    ros::param::get("~traj_path", traj_path);

    p_map_builder = new MapBuilder(configs, node_handler);

    ros::Subscriber sub_imu;
    if (p_map_builder->UseIMU()) {
      std::cout << "use imu!" << std::endl;
      sub_imu = node_handler.subscribe(imu_topic, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }

    // Starts the operation
    ros::spin();

    // Shutting down
    std::cout << "Saving trajectory to " << traj_path << std::endl;
    p_map_builder->SaveTrajectory(traj_path);
    p_map_builder->SaveMap(configs.saving_dir);
    ros::shutdown();

    return 0;
}