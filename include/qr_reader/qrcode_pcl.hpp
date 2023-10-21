#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "image_transport/image_transport.hpp"
#include "qrcode_msgs/msg/qr_code.hpp" 
#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <visualization_msgs/msg/marker.hpp>

#include <pcl_ros/transforms.hpp>



namespace qrcode_pcl_ns
{
    class QrReader: public rclcpp::Node
    {
        public:
            QrReader();
            ~QrReader(){};
            
            void pcl_callback(
                const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcl_msg);

            void mark_qrcodes_on_image_fnc(cv::Mat &img, std::vector<cv::Mat> points);

            void publish_markers_fnc(std::vector<geometry_msgs::msg::Point> corners, std::string &sensor_frame_id);


        private:
            rclcpp::Publisher<qrcode_msgs::msg::QrCode>::SharedPtr detection_pub_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

            message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
            std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2> > msg_sync_;

            qrcode_msgs::msg::QrCode qrcode_msg_;

            cv::wechat_qrcode::WeChatQRCode  detect_qrcodes_;

            bool close_to_object_;
            bool debug_image_;
            bool debug_marker_;
            bool debug_computation_time_;
            // fot computatuion time
            rclcpp::Time start_time_;
            rclcpp::Time end_time_;


            void detect_qr_codes_(cv::Mat &img,
                                int &length, std::vector<std::string> &codes,
                                std::vector<geometry_msgs::msg::Point> &centers,
                                std::vector<geometry_msgs::msg::Point> &corners);

            void convert_positions_(std::vector<geometry_msgs::msg::Point> &pixel_positions, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                long int cols);
                                
            
    };
}
