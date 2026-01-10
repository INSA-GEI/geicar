#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "vision_msgs/msg/detection3_d.hpp"
#include "vision_msgs/msg/detection2_d.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class TrashLocalizationNode : public rclcpp::Node 
{   
    public:
        TrashLocalizationNode() : Node("trash_localization_node"){
            // Declare parameters with default values
            this->declare_parameter<std::double_t>("update_period_in_s", 0.2);
            this->declare_parameter<std::string>("camera_left_target_topic", "/usb_cam_left/object_target");
            this->declare_parameter<std::string>("camera_right_target_topic", "/usb_cam_right/object_target");
            this->declare_parameter<std::string>("camera_left_info_topic", "/usb_cam_left/camera_info");
            this->declare_parameter<std::string>("camera_right_info_topic", "/usb_cam_right/camera_info");
            this->declare_parameter<std::string>("lidar_scan_topic", "/ld_lidar/scan");
            this->declare_parameter<std::string>("left_camera_frame", "camera_left_link");
            this->declare_parameter<std::string>("right_camera_frame", "camera_right_link");
            this->declare_parameter<std::string>("lidar_frame", "ld_lidar_link");
            this->declare_parameter<std::double_t>("tf_timeout", 3.0);

            // Get parameters
            auto update_period_ = std::chrono::duration<double>(this->get_parameter("update_period_in_s").as_double()); 
            tf_timeout_ = this->get_parameter("tf_timeout").as_double();
            left_camera_frame_ = this->get_parameter("left_camera_frame").as_string();
            right_camera_frame_ = this->get_parameter("right_camera_frame").as_string();
            lidar_frame_ = this->get_parameter("lidar_frame").as_string();

            // Subscribers from both cameras
            rclcpp::SensorDataQoS qos;
            qos.keep_last(1);

            camera_left_target_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2D>(
                this->get_parameter("camera_left_target_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_target_callback, this, std::placeholders::_1)
            );

            camera_left_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                this->get_parameter("camera_left_info_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_info_callback, this, std::placeholders::_1)
            );

            camera_right_target_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2D>(
                this->get_parameter("camera_right_target_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_target_callback, this, std::placeholders::_1)
            );
            camera_right_info_subscriber_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                this->get_parameter("camera_right_info_topic").as_string(), 
                qos,
                std::bind(&TrashLocalizationNode::camera_info_callback, this, std::placeholders::_1)
            );

            // Subscriber for LIDAR scan
            lidar_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                this->get_parameter("lidar_scan_topic").as_string(),
                rclcpp::SensorDataQoS(),
                std::bind(&TrashLocalizationNode::lidar_scan_callback, this, std::placeholders::_1)
            );

            // TF2 listeners for both cameras
            tf_cam_left_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_left_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_cam_left_buffer_);
            tf_cam_right_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_right_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_cam_right_buffer_);

            // TF2 broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            timer_ = this->create_wall_timer(
                update_period_, 
                std::bind(&TrashLocalizationNode::process_data, this)
            );
        }

    private:
        void camera_target_callback(const vision_msgs::msg::Detection2D::SharedPtr msg) {
            if (msg->header.frame_id == left_camera_frame_) {
                left_camera_target_ = *msg;
                // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Received left target stamped at %f seconds.", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
            } else if (msg->header.frame_id == right_camera_frame_) {
                right_camera_target_ = *msg;
                // RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Received right target stamped at %f seconds.", msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
            } else {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Received target from unknown camera frame: %s", msg->header.frame_id.c_str());
            }
        }

        void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            if (msg->header.frame_id == left_camera_frame_) {
                left_camera_info_ = *msg;
            } else if (msg->header.frame_id == right_camera_frame_) {
                right_camera_info_ = *msg;
            } else {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Received camera info from unknown camera frame: %s", msg->header.frame_id.c_str());
            }
        }

        void lidar_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            latest_lidar_scan_ = *msg;
        }

        /**
         * @brief Perform lookup in scan data to find the target detected by the camera, and broadcast its TF.
         * @note Currently only work with left camera data, can be extended to right camera similarly.
         * @note Broadcast the closest valid point in LIDAR scan as target position.
         * 
         */
        void process_data() {
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Processing data to localize target...");
            if (left_camera_target_.header.stamp.sec == 0) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for camera to detect targets...");
                return;
            }
            if (latest_lidar_scan_.header.stamp.sec == 0) {
                RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for LIDAR scan data...");
                return;
            }
            // Ignore old data
            auto now = this->now();
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Current time: %f seconds.", now.nanoseconds()/1000000000.0);
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Left camera target time: %f seconds.", rclcpp::Time(left_camera_target_.header.stamp).seconds());
            if ((now - rclcpp::Time(left_camera_target_.header.stamp)).seconds() > tf_timeout_) {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Left camera target data is too old.");
                return;
            }
            if ((now - rclcpp::Time(latest_lidar_scan_.header.stamp)).seconds() > tf_timeout_) {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] LIDAR scan data is too old.");
                return;
            }
            // Transform camera target to LIDAR frame
            double target_angle = compute_angle_from_camera(right_camera_target_, right_camera_info_);
            target_angle = transform_angle_to_lidar_frame(target_angle, right_camera_frame_, lidar_frame_);
            RCLCPP_INFO(this->get_logger(), "[TRASH_LOCALIZATION] Transformed target angle to LIDAR frame: %.3f radians", target_angle);
            if (std::isnan(target_angle)) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Failed to transform target angle to LIDAR frame.");
                return;
            }
            int target_index = find_target_in_lidar_scan(target_angle, latest_lidar_scan_);
            if (target_index >= 0) {
                broadcast_target_tf(latest_lidar_scan_.angle_min + target_index * latest_lidar_scan_.angle_increment, latest_lidar_scan_.ranges[target_index], lidar_frame_);
            } else {
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Target not found in LIDAR scan.");
            }
        }

        double compute_angle_from_camera(vision_msgs::msg::Detection2D & target, sensor_msgs::msg::CameraInfo & cam_info){
            // Compute angle of the target relative to the camera frame
            double cx = cam_info.k[2]; // Principal point x
            double fx = cam_info.k[0]; // Focal length x
            double pixel_x = target.bbox.center.position.x;
            double angle = atan2((pixel_x - cx), fx);
            return angle;            
        }

        double transform_angle_to_lidar_frame(double angle_in_camera, const std::string & camera_frame, const std::string & lidar_frame) {
            // Transform angle from camera frame to LIDAR frame using TF2
            geometry_msgs::msg::Vector3Stamped vec_in_camera, vec_in_lidar;
            vec_in_camera.header.frame_id = camera_frame;
            vec_in_camera.header.stamp = this->now();
            vec_in_camera.vector.x = sin(angle_in_camera);
            vec_in_camera.vector.y = 0.0;
            vec_in_camera.vector.z = cos(angle_in_camera);

            try {
                if (camera_frame == left_camera_frame_) {
                    vec_in_lidar = tf_cam_left_buffer_->transform(
                        vec_in_camera,
                        lidar_frame
                    );
                } else if (camera_frame == right_camera_frame_) {
                    vec_in_lidar = tf_cam_right_buffer_->transform(
                        vec_in_camera,
                        lidar_frame
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Unknown camera frame for angle transformation: %s", camera_frame.c_str());
                    return NAN;
                }
                double angle_in_lidar = atan2(vec_in_lidar.vector.x, vec_in_lidar.vector.y) + M_PI/2;
                return angle_in_lidar;          
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] TF2 Transform Error in angle transformation: %s", ex.what());
                return NAN;
            }
        }

        /**
         * @brief [NOT IN USE]Transforms a Detection3D target from a camera frame to the LIDAR frame. 
         * Caller must check the frame_id of the returned target to ensure successful transformation.
         * @note just in case we have 3D pose estimation from the camera, so not in use now.
         * 
         * @param cam_target The target detection in the camera frame.
         * @param lidar_frame The target frame to transform the detection into (LIDAR frame).
         * @return vision_msgs::msg::Detection3D The transformed target detection in the LIDAR frame.
         */
        vision_msgs::msg::Detection3D transform_target_to_lidar_frame(vision_msgs::msg::Detection3D & cam_target, const std::string & lidar_frame) {
            // Transform target position to LIDAR frame
            vision_msgs::msg::Detection3D lidar_target = cam_target;

            // Stamp the pose of input target
            geometry_msgs::msg::PoseStamped pose_in_camera, pose_in_lidar;
            pose_in_camera.header = cam_target.header;
            pose_in_camera.pose = cam_target.bbox.center;
            
            try {
                if (cam_target.header.frame_id == left_camera_frame_) {
                    pose_in_lidar = tf_cam_left_buffer_->transform(
                        pose_in_camera,
                        lidar_frame
                    );
                } else if (cam_target.header.frame_id == right_camera_frame_) {
                    pose_in_lidar = tf_cam_right_buffer_->transform(
                        pose_in_camera,
                        lidar_frame
                    );
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] Unknown camera frame: %s", cam_target.header.frame_id.c_str());
                    // Upper-level function should check frame id of the returned target
                    return lidar_target;
                }
                // Update the target with transformed pose
                lidar_target.header.frame_id = lidar_frame;
                lidar_target.header.stamp = this->now();
                lidar_target.bbox.center = pose_in_lidar.pose;  
                return lidar_target;          
            } catch (tf2::TransformException & ex) {
                RCLCPP_ERROR(this->get_logger(), "[TRASH_LOCALIZATION] TF2 Transform Error: %s", ex.what());
                // Upper-level function should check frame id of the returned target
                return lidar_target;
            }
        }

        /**
         * @brief Find target index in LIDAR scan data based on angle. Return the index of the closest valid point.
         * 
         * @param angle The angle (in radians) to search for the target in LIDAR frame.
         * @param scan The LIDAR scan data.
         * @return int The index of the target in the LIDAR scan ranges, or -1 if not found.
         */
        int find_target_in_lidar_scan(double angle, sensor_msgs::msg::LaserScan & scan) {
            const double search_angle_tolerance_ = 10*3.141592654/180.0; // 5 degrees in radians
            const int min_valid_lidar_points_ = 3;      // Minimum pixel needed to confirm target detection
            const int max_valid_lidar_points_ = 10;   // Maximum pixel to avoid false positives
            const double max_lidar_distance_m_ = 5.0;    // Maximum distance to consider LIDAR points valid
            const double min_lidar_distance_m_ = 0.1;    // Minimum distance to consider LIDAR points valid

            int index_min = std::round((angle - search_angle_tolerance_ - scan.angle_min) / scan.angle_increment);
            int index_max = std::round((angle+search_angle_tolerance_ - scan.angle_min) / scan.angle_increment);

            if (index_min < 0){
                index_min = 0;
            }
            if (index_max >= static_cast<int>(scan.ranges.size())){
                index_max = scan.ranges.size() - 1;
            }

            int pixel_detected = 0;
            
            std::array<int, max_valid_lidar_points_> index_detected = {0};
            int target_index = 0;

            // Perform sweep in LIDAR scan data within the angle range
            for (int i = index_min; i <= index_max; ++i){
                double distance = scan.ranges[i];
                if (distance >= min_lidar_distance_m_ && distance <= max_lidar_distance_m_){
                    if (pixel_detected < max_valid_lidar_points_){
                        index_detected[pixel_detected] = i;
                        pixel_detected++;
                    }
                }
            }
            RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Detected %d valid LIDAR points for target search.", pixel_detected);
            if (pixel_detected >= min_valid_lidar_points_){
                // Find minimum distance index among detected points
                double min_distance = scan.range_max + 1.0;
                for (int j = 0; j < std::min(pixel_detected, max_valid_lidar_points_); j++){
                    double distance = scan.ranges[index_detected[j]];
                    if (distance < min_distance){
                        min_distance = distance;
                        target_index = index_detected[j];;
                    }
                }
                return target_index;
            } else {    
                RCLCPP_WARN(this->get_logger(), "[TRASH_LOCALIZATION] Not enough valid LIDAR points detected for target.");
                return -1; // Indicate no valid target found
            }
        }

        void broadcast_target_tf(double angle, float distance, const std::string & lidar_frame) {
            geometry_msgs::msg::TransformStamped target_tf;
            target_tf.header.stamp = this->now();
            target_tf.header.frame_id = lidar_frame;
            target_tf.child_frame_id = "target_trash";
            target_tf.transform.translation.x = distance * cos(angle);
            target_tf.transform.translation.y = distance * sin(angle);
            target_tf.transform.translation.z = 0.05;
            target_tf.transform.rotation.x = 0.0;
            target_tf.transform.rotation.y = 0.0;
            target_tf.transform.rotation.z = 0.0;
            target_tf.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(target_tf);
        }

        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_left_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_left_info_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::Detection2D>::SharedPtr camera_right_target_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_right_info_subscriber_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_scan_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        vision_msgs::msg::Detection2D left_camera_target_;
        vision_msgs::msg::Detection2D right_camera_target_;
        sensor_msgs::msg::LaserScan latest_lidar_scan_;

        std::string left_camera_frame_;
        std::string right_camera_frame_;
        std::string lidar_frame_;

        sensor_msgs::msg::CameraInfo left_camera_info_;
        sensor_msgs::msg::CameraInfo right_camera_info_;

        std::unique_ptr<tf2_ros::Buffer> tf_cam_left_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_left_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_cam_right_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_right_listener_{nullptr};
        double tf_timeout_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrashLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}