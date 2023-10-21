#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "image_transport/image_transport.hpp"
#include "qrcode_msgs/msg/qr_code.hpp" 
#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>

namespace qrcode_image_ns
{
    class QrReader: public rclcpp::Node
    {
        public:
            QrReader();
            ~QrReader(){};
            
            void image_callback(
                const sensor_msgs::msg::Image::ConstSharedPtr & msg);
            
            void mark_qrcodes_on_image_fnc(cv::Mat &img, std::vector<cv::Mat> points);
                        
        private:
            rclcpp::Publisher<qrcode_msgs::msg::QrCode>::SharedPtr detection_pub_;
            
            image_transport::Subscriber image_sub_;

            qrcode_msgs::msg::QrCode qrcode_msg_;
            
            cv::wechat_qrcode::WeChatQRCode  detect_qrcodes_;

            bool qr_detected_{false};
            bool qr_detected_last_{true};
            bool debug_image_;
            bool debug_computation_time_;
            // for computatuion time
            rclcpp::Time start_time_;
            rclcpp::Time end_time_;

            void detect_qr_codes_(cv::Mat &img,
                                int &length, std::vector<std::string> &codes,
                                std::vector<geometry_msgs::msg::Point> &centers,
                                std::vector<geometry_msgs::msg::Point> &corners);

            
    };
}