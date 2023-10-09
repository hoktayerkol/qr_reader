#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "image_transport/image_transport.hpp"
#include "qrcode_msgs/msg/qr_code.hpp" 
#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>

namespace qr_reader_ns
{
    class QrReader: public rclcpp::Node
    {
        public:
            QrReader();
            //~QrReader();
            
            void image_callback(
                const sensor_msgs::msg::Image::ConstSharedPtr & msg);
                        
        private:
            rclcpp::Publisher<qrcode_msgs::msg::QrCode>::SharedPtr detection_pub_;
            image_transport::Subscriber image_sub_;
            qrcode_msgs::msg::QrCode qrcode_msg_;
            
            cv::wechat_qrcode::WeChatQRCode  detector_;

            std::vector<std::string> codes_last_;
            std::vector<geometry_msgs::msg::Point> corners_last_;

            bool debug_ {true};
            
    };
}
