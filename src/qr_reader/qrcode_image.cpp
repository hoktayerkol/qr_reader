#include <memory>
#include <vector>
#include <string>

#include "qr_reader/qrcode_image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>
#include "cv_bridge/cv_bridge.h"

#include "qrcode_msgs/msg/qr_code.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

// debug moda islem suresinide ekle
namespace qrcode_image_ns
{

using std::placeholders::_1;

    QrReader::QrReader() : Node("qrcode_image_node")
    {
        detection_pub_ = this->create_publisher<qrcode_msgs::msg::QrCode>("qrcode_detection", 10);
    
        image_sub_ = image_transport::create_subscription(this, "camera/image_raw", 
            std::bind(&QrReader::image_callback, this, _1), "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile() );
            // "raw", "compressed" and "theora"

        std::string model_folder = ament_index_cpp::get_package_share_directory("qr_reader");
        try{
            detect_qrcodes_ = cv::wechat_qrcode::WeChatQRCode(  
                model_folder + "/model/detect.prototxt", 
                model_folder + "/model/detect.caffemodel",
                model_folder + "/model/sr.prototxt", 
                model_folder + "/model/sr.caffemodel");
        } catch (cv::Exception &e){
            RCLCPP_ERROR(get_logger(), "cv::Eception: %s", e.what());
        }

        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        param_desc.description = "Sets debug_image mode!";
        declare_parameter("debug_image", false, param_desc);
        debug_image_ = get_parameter("debug_image").as_bool();

        param_desc.description = "Sets debug_computation_time mode!";
        declare_parameter("debug_computation_time", false, param_desc);
        debug_computation_time_ = get_parameter("debug_computation_time").as_bool();
    }

    void QrReader::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
        if (debug_computation_time_){
            start_time_ = get_clock()->now();
        }
       
        cv_bridge::CvImagePtr cv_ptr;
        try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
          
        int length;
        std::vector<std::string> codes;
        std::vector<geometry_msgs::msg::Point> centers;
        std::vector<geometry_msgs::msg::Point> corners;
        
        QrReader::detect_qr_codes_(cv_ptr->image, length, codes, centers, corners);

        if (!codes.empty()){
                               
            qrcode_msg_.count = length;
            qrcode_msg_.string = codes;
            qrcode_msg_.corners = corners;
            qrcode_msg_.center = centers;
            detection_pub_->publish(qrcode_msg_);

            RCLCPP_INFO_EXPRESSION(get_logger(), 
                qr_detected_!=qr_detected_last_, "QR code detected!");
            qr_detected_=true;
            
            if (debug_computation_time_){
                end_time_ = get_clock()->now();
                auto ct = (end_time_.nanoseconds() - start_time_.nanoseconds());
                RCLCPP_INFO(get_logger(), "Computaion time is: %lu usec", ct/1000 );
            }

        } else {qr_detected_=false;}

    } // QrReader::image_callback
    
    void QrReader::detect_qr_codes_(cv::Mat &img,
                                    int &length, std::vector<std::string> &codes,
                                    std::vector<geometry_msgs::msg::Point> &centers,
                                    std::vector<geometry_msgs::msg::Point> &corners){

        // baska yerde kullanmak icin points disarida tanimlandi
        std::vector<cv::Mat> points;
        auto data = detect_qrcodes_.detectAndDecode(img, points);
        
        geometry_msgs::msg::Point point;
                      
        if (!data.empty()) {

            length = data.size();

            for (int qr_ind = 0; qr_ind<length; qr_ind++) {
                geometry_msgs::msg::Point center_point;

                for (int i=0, j=0; i<4; i++, j++) {
                    point.x = points[qr_ind].at<float>(i,0);
                    point.y = points[qr_ind].at<float>(i,1);

                    corners.push_back(point);
                    
                    center_point.x += point.x;
                    center_point.y += point.y;
                }
                
                center_point.x /= 4;
                center_point.y /= 4;
                centers.push_back(center_point);
                codes.push_back(data[qr_ind]);
            }
            
        }

        if (debug_image_){
            QrReader::mark_qrcodes_on_image_fnc(img, points);     
        }


    } //QrReader::detect_qr_codes_

    void QrReader::mark_qrcodes_on_image_fnc(cv::Mat &img, 
        std::vector<cv::Mat> points) {

        int data_len = points.size();
        if (data_len>0){
            for (int qr_ind = 0; qr_ind<data_len; qr_ind++) {
                cv::Point p1(points[qr_ind].row(0));
                cv::Point p2(points[qr_ind].row(2));
                cv::rectangle(img, p1, p2, cv::Scalar(0, 0, 255), 3);
            }
        }

        cv::imshow("cv_ptr->image", img);
        cv::waitKey(1);

    } // QrReader::mark_qrcodes_on_image_fnc

} // end of namespace qrcode_image_ns