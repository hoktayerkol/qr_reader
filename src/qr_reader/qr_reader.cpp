#include <memory>
#include <vector>
#include <string>

#include "qr_reader/qr_reader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>
#include "cv_bridge/cv_bridge.h"

#include "qrcode_msgs/msg/qr_code.hpp"

// debug moda islem suresinide ekle
namespace qr_reader_ns
{

using std::placeholders::_1;

    QrReader::QrReader() : Node("qr_reader_node"), codes_last_({}), corners_last_({})
    {
        detection_pub_ = create_publisher<qrcode_msgs::msg::QrCode>("qrcode_detection", 10);
    
        image_sub_ = image_transport::create_subscription(this, "image_raw", 
            std::bind(&QrReader::image_callback, this, _1), "raw", rclcpp::SensorDataQoS().get_rmw_qos_profile() );

        detector_ = cv::wechat_qrcode::WeChatQRCode("../../model/detect.prototxt", 
                                                            "../../model/detect.caffemodel",
                                                            "../../model/sr.prototxt", 
                                                            "../../model/sr.caffemodel");
        
    }

    void QrReader::image_callback(
        const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
       
        cv_bridge::CvImagePtr cv_ptr;
        try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        }
          

        std::vector<cv::Mat> points;
        auto data = detector_.detectAndDecode(cv_ptr->image, points);
        
        geometry_msgs::msg::Point point;
        

        std::vector<std::string> codes;
        std::vector<geometry_msgs::msg::Point> centers;
        std::vector<geometry_msgs::msg::Point> corners;
        
        // burada bossa return falan denebilir
        // RCLCPP_INFO(get_logger(), "QR detrected: %s", data[k].c_str());
        if (data.empty()) {
            if (debug_)
                cv::destroyAllWindows();

            return;

        } else {

            int data_len = data.size();

            for (int qr_ind = 0; qr_ind<data_len; qr_ind++) {
                geometry_msgs::msg::Point center_point;

                for (int i=0, j=0; i<4; i++, j++) {
                    point.y = points[qr_ind].at<float>(i,0);
                    point.z = points[qr_ind].at<float>(i,1);
                    
                    corners.push_back(point);
                    
                    center_point.y += point.y;
                    center_point.z += point.z;
                }
                
                center_point.y /= 4;
                center_point.z /= 4;
                centers.push_back(center_point);
                codes.push_back(data[qr_ind]);
            }

            if (codes!=codes_last_ || corners!=corners_last_) {
                codes_last_=codes;
                corners_last_=corners;

                qrcode_msg_.string = codes;     
                qrcode_msg_.corners = corners;
                qrcode_msg_.center = centers;
                detection_pub_->publish(qrcode_msg_);
            }

            if (debug_){

                cv::Mat1f bbox;
                   
                for (int qr_ind = 0; qr_ind<data_len; qr_ind++) {
                    // for rectagle
                    cv::Point p1(points[qr_ind].row(0));
                    cv::Point p2(points[qr_ind].row(2));
                    cv::Point p3(centers[qr_ind].y, centers[qr_ind].z );

                    cv::rectangle(cv_ptr->image, p1, p2, cv::Scalar(0, 0, 255), 3);
                    //cv::line(cv_ptr->image, p1, p2, cv::Scalar(0,255,0), 3);
                    cv::circle(cv_ptr->image, p3, 10, cv::Scalar(255, 0, 0), 3);
                }
                cv::imshow("cv_ptr->image", cv_ptr->image);
                cv::waitKey(1);
            }

        }


    } // end of QrReader::image_callback
    





} // end of namespace qr_reader_ns