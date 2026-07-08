/**
 * @file mono-slam-node.cpp
 * @brief Implementation of the MonoSlamNode Wrapper class.
 * @author Suchetan R S (rssuchetan@gmail.com)
 */
#include <thread>
#include <future>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include "mono-slam-node.hpp"

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3_Wrapper
{
    MonoSlamNode::MonoSlamNode(const std::string &strVocFile,
                               const std::string &strSettingsFile,
                               ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_MONO_ROS2")
    {
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>("camera/image_raw",10,std::bind(&MonoSlamNode::MONOCallback,
        									    	    this,std::placeholders::_1));

        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 1000, std::bind(&MonoSlamNode::ImuCallback, this, std::placeholders::_1));
        
        cameraPosePub_ = this->create_publisher<geometry_msgs::msg::Pose>("camera_pose", 10);
        trackingStatePub_ = this->create_publisher<std_msgs::msg::Int8>("orb_slam3/tracking_state", 10);

        bool bUseViewer =false;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("ros_visualization", rclcpp::ParameterValue(false));
        this->get_parameter("ros_visualization", rosViz_);

        this->declare_parameter("robot_base_frame", "base_link");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("no_odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("no_odometry_mode", no_odometry_mode_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("landmark_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("landmark_publish_frequency", landmark_publish_frequency_);
        
	    mapPointsCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapCurrentPointsTimer_ = this->create_wall_timer(std::chrono::milliseconds(5 * landmark_publish_frequency_), std::bind(&MonoSlamNode::saveCurrentMapPointCloud, this));
        RCLCPP_INFO(this->get_logger(), "Time is %d",landmark_publish_frequency_);

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(strVocFile, strSettingsFile,
                                                                            sensor, bUseViewer, rosViz_, robot_x_,
                                                                            robot_y_, global_frame_, odom_frame_id_, robot_base_frame_id_);
        map_control_srv_ = this->create_service<orb_slam3_ros2_wrapper::srv::MapControl>(
            "orb_slam3/map_control",
            std::bind(&MonoSlamNode::handleMapControl, this, std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "✅ Map control service registered: /orb_slam3/map_control");
        
        std::error_code ec;
        std::filesystem::create_directories(save_frame_dir_, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(), "Cannot create save dir: %s", ec.message().c_str());
        }
        
        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        worker_thread_ = std::thread(&MonoSlamNode::workerLoop, this);
        RCLCPP_INFO(this->get_logger(), "🔧 Worker thread started");

        RCLCPP_INFO(this->get_logger(), "CONSTRUCTOR END!");
    }

    MonoSlamNode::~MonoSlamNode()
    {
        rgbSub_.reset();
        imuSub_.reset();
        odomSub_.reset();
        interface_.reset();
        saveCurrentMapPointCloud();
        worker_running_ = false;
        cmd_cv_.notify_one();
        if (worker_thread_.joinable()) worker_thread_.join();
        RCLCPP_INFO(this->get_logger(), "🔧 Worker thread stopped");
        RCLCPP_INFO(this->get_logger(), "DESTRUCTOR!");
    }

    void MonoSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        interface_->handleIMU(msgIMU);
    }

    void MonoSlamNode::saveFrame(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                                 const Sophus::SE3f& Tcw)
    {
        try {
            uint64_t id = frame_save_counter_.fetch_add(1);
            std::string prefix = save_frame_dir_ + "/frame_" + std::to_string(id);
            std::string imgfile = prefix + ".jpg";
            std::string txtfile = prefix + ".txt";

            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msgRGB);
            cv::Mat img = cv_ptr->image;
            if (img.channels() == 1) {
                cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
            }
            if (!cv::imwrite(imgfile, img)) {
                RCLCPP_ERROR(this->get_logger(), "cv::imwrite failed: %s", imgfile.c_str());
            }

            auto pose_msg = typeConversion_.se3ToPoseMsg(Tcw);
            std::ofstream f(txtfile);
            if (f.is_open()) {
                f << std::fixed << std::setprecision(6)
                  << pose_msg.position.x << " "
                  << pose_msg.position.y << " "
                  << pose_msg.position.z << "\n";
                f.close();
            }

            RCLCPP_INFO(this->get_logger(), "💾 Saved frame %lu: %s + .txt", id, imgfile.c_str());

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "saveFrame error: %s", e.what());
        }
    }

    void MonoSlamNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msgOdom)
    {
        if(!no_odometry_mode_)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "OdomCallback");
            interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
        }
        else RCLCPP_WARN(this->get_logger(), "Odometry msg recorded but no odometry mode is true, set to false to use this odometry");
    }

    void MonoSlamNode::MONOCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        Sophus::SE3f Tcw;
        int trackingState = interface_->trackMONO(msgRGB, Tcw);
        
        std_msgs::msg::Int8 stateMsg;
        stateMsg.data = trackingState;
        trackingStatePub_->publish(stateMsg);
        
        auto camPose = typeConversion_.se3ToPoseMsg(Tcw);
        
        if (save_next_frame_.exchange(false)) {
            saveFrame(msgRGB, Tcw);
        }
        
        if (trackingState == 2)
        {
            isTracked_ = true;
            cameraPosePub_->publish(camPose);
        }
        else if (trackingState == 1)
        {
            geometry_msgs::msg::Pose notInitializedPose;
            notInitializedPose.position.x = -1.0;
            notInitializedPose.position.y = -1.0;
            notInitializedPose.position.z = -1.0;
            notInitializedPose.orientation.x = -1.0;
            notInitializedPose.orientation.y = -1.0;
            notInitializedPose.orientation.z = -1.0;
            notInitializedPose.orientation.w = -1.0;
            cameraPosePub_->publish(notInitializedPose);
        }
        else if (trackingState == 3)
        {
            geometry_msgs::msg::Pose trackingLostPose;
            trackingLostPose.position.x = -3.0;
            trackingLostPose.position.y = -3.0;
            trackingLostPose.position.z = -3.0;
            trackingLostPose.orientation.x = -3.0;
            trackingLostPose.orientation.y = -3.0;
            trackingLostPose.orientation.z = -3.0;
            trackingLostPose.orientation.w = -3.0;
            cameraPosePub_->publish(trackingLostPose);
        }
        else
        {
            geometry_msgs::msg::Pose noImagesPose;
            noImagesPose.position.x = 0.0;
            noImagesPose.position.y = 0.0;
            noImagesPose.position.z = 0.0;
            noImagesPose.orientation.x = 0.0;
            noImagesPose.orientation.y = 0.0;
            noImagesPose.orientation.z = 0.0;
            noImagesPose.orientation.w = 0.0;
            cameraPosePub_->publish(noImagesPose);
        }
    }

    void MonoSlamNode::publishCurrentMapPointCloud()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            sensor_msgs::msg::PointCloud2 mapPCL;
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getCurrentMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            currentMapPointsPub_->publish(mapPCL);

            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish current map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");
        }
    }

    void MonoSlamNode::saveCurrentMapPointCloud()
    {
        if (false)
        {
            if (isTracked_)
            {
                std::vector<Eigen::Vector3f> trackedMapPoints;
                interface_->getCurrentMapPointsToSave(trackedMapPoints);

                if(trackedMapPoints.size() == 0)
                    return;

                auto now = std::chrono::system_clock::now();
                auto in_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&in_time_t), "map_%Y%m%d_%H%M%S.ply");
                savePointsToPLY(trackedMapPoints, ss.str());

                auto t3 = std::chrono::high_resolution_clock::now();
                auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - now).count();
                RCLCPP_INFO_STREAM(this->get_logger(), "Time to save " << ss.str() <<" map points: " << time_publish_map_points << " seconds");
                RCLCPP_INFO_STREAM(this->get_logger(), "=======================");
                isMapPointsSaved = true;
            }
        }
        else
        {
        	RCLCPP_INFO_STREAM(this->get_logger(), "saveCurrentMapPointCloud() state  " << interface_->checkSLAMShutdown() <<" and next bool " << !isMapPointsSaved );
        }
    }

    void MonoSlamNode::savePointsToPLY(const std::vector<Eigen::Vector3f> &points, const std::string &filename) 
    {
        std::ofstream outFile(filename);
        if (!outFile.is_open()) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return;
        }

        outFile << "ply\n";
        outFile << "format ascii 1.0\n";
        outFile << "element vertex " << points.size() << "\n";
        outFile << "property float x\n";
        outFile << "property float y\n";
        outFile << "property float z\n";
        outFile << "end_header\n";

        for (const Eigen::Vector3f &point : points) {
            outFile << point[0] << " " << point[1] << " " << point[2] << "\n";
        }

        outFile.close();
        RCLCPP_INFO_STREAM(this->get_logger(), "Saved " << points.size() << " points to '" << filename << "'");
    }

    void MonoSlamNode::publishReferenceMapPointCloud()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            sensor_msgs::msg::PointCloud2 mapPCL;
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_create_mapPCL = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to create mapPCL object: " << time_create_mapPCL << " seconds");

            interface_->getReferenceMapPoints(mapPCL);

            if(mapPCL.data.size() == 0)
                return;

            auto t2 = std::chrono::high_resolution_clock::now();
            auto time_get_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to get current map points: " << time_get_map_points << " seconds");

            referenceMapPointsPub_->publish(mapPCL);
            auto t3 = std::chrono::high_resolution_clock::now();
            auto time_publish_map_points = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t2).count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time to publish map points: " << time_publish_map_points << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "=======================");
        }
    }
    
    void ORB_SLAM3_Wrapper::MonoSlamNode::changeMode(bool activate)
    {
        (void)activate;
    }

    void MonoSlamNode::combinedPublishCallback() 
    {
        this->publishCurrentMapPointCloud();
        this->publishReferenceMapPointCloud();
    }

void MonoSlamNode::handleMapControl(
    const std::shared_ptr<orb_slam3_ros2_wrapper::srv::MapControl::Request> req,
    std::shared_ptr<orb_slam3_ros2_wrapper::srv::MapControl::Response> res)
{
    RCLCPP_INFO(this->get_logger(), ">>> handleMapControl ENTER: cmd=%d", req->command);

    if (req->command == 5) {
        save_next_frame_ = true;
        res->success = true;
        res->message = "Next frame will be saved to " + save_frame_dir_;
        RCLCPP_INFO(this->get_logger(), ">>> handleMapControl EXIT (cmd=5): success=1");
        return;
    }

    // Команды 0-4 — в worker thread
    MapControlCmd cmd{req->command, req->filepath};
    {
        std::lock_guard<std::mutex> lock(cmd_queue_mutex_);
        cmd_queue_.push(cmd);
    }
    cmd_cv_.notify_one();

    res->success = true;
    res->message = "Command " + std::to_string(req->command) + " queued";
    RCLCPP_INFO(this->get_logger(), ">>> handleMapControl EXIT (cmd=%d): queued", req->command);
}

    void MonoSlamNode::workerLoop()
    {
        RCLCPP_INFO(this->get_logger(), "[WORKER] Thread started");
        
        while (worker_running_ || !cmd_queue_.empty()) {
            MapControlCmd cmd;
            
            {
                std::unique_lock<std::mutex> lock(cmd_queue_mutex_);
                cmd_cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
                    return !cmd_queue_.empty() || !worker_running_;
                });
                
                if (cmd_queue_.empty()) continue;
                
                cmd = std::move(cmd_queue_.front());
                cmd_queue_.pop();
                RCLCPP_INFO(this->get_logger(), "[WORKER] Processing command: %d", cmd.command);
            }
            
            std::pair<bool, std::string> result;
            auto cmd_start = std::chrono::high_resolution_clock::now();
            
            try {
                switch (cmd.command) {
                    case 0: {
                        bool ok = interface_->resetMap();
                        result = {ok, ok ? "Map reset" : "Reset failed"};
                        break;
                    }
                    case 1:
                        result = {interface_->saveMap(cmd.filepath), 
                                  "Saved to " + cmd.filepath};
                        break;
                    case 2:
                        result = {interface_->loadMap(cmd.filepath), 
                                  "Loaded from " + cmd.filepath};
                        break;
                    case 3:
                        interface_->enableLocalizationMode();
                        result = {true, "Localization mode enabled"};
                        break;
                    case 4:
                        interface_->disableLocalizationMode();
                        result = {true, "Localization mode disabled"};
                        break;
                    default:
                        result = {false, "Unknown command"};
                        break;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[WORKER] Exception: %s", e.what());
                result = {false, std::string("Exception: ") + e.what()};
            }
            
            auto cmd_end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(cmd_end - cmd_start).count();
            RCLCPP_INFO(this->get_logger(), "[WORKER] Command %d finished in %ld ms", cmd.command, duration);
        }
        
        RCLCPP_INFO(this->get_logger(), "[WORKER] Thread exiting");
    }

} // namespace ORB_SLAM3_Wrapper
