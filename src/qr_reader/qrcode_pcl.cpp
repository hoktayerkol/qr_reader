#include <memory>
#include <vector>
#include <string>
#include <algorithm>

#include "qr_reader/qrcode_pcl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"

#include "opencv2/opencv.hpp"
#include <opencv2/wechat_qrcode.hpp>
#include "cv_bridge/cv_bridge.h"

#include "qrcode_msgs/msg/qr_code.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <visualization_msgs/msg/marker.hpp>

//#include <filesystem>
//namespace fs = std::filesystem;
//std::string yol = "/home/oktay/ws_ros/install/qr_reader/share/";

#include <ament_index_cpp/get_package_share_directory.hpp>

// debug moda islem suresinide ekle
namespace qr_reader_ns
{

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

    QrReader::QrReader() : Node("qr_reader_node"), qrcode_msg_({}), start_time_{}
    {
        detection_pub_ = create_publisher<qrcode_msgs::msg::QrCode>("qrcode_detection", 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("qrcode_marker", 1);

        camera_info_sub_.subscribe(this, "camera/depth/camera_info");
        
        pcl_sub_.subscribe(this, "camera/points");
        msg_sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2> >(camera_info_sub_, pcl_sub_, 10);
        msg_sync_->registerCallback(std::bind(&QrReader::pcl_callback, this, _1, _2));
        
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

        param_desc.description = "Sets debug_marker mode!";
        declare_parameter("debug_marker", false, param_desc);
        debug_marker_ = get_parameter("debug_marker").as_bool();

        param_desc.description = "Sets debug_computation_time mode!";
        declare_parameter("debug_computation_time", false, param_desc);
        debug_computation_time_ = get_parameter("debug_computation_time").as_bool();
        
    } // end of constructor

    
    void QrReader::pcl_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & camera_info_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcl_msg){
        
        if (debug_computation_time_){
            start_time_ = get_clock()->now();
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB>);
        // PointXYZRGB PointXYZI etc
        pcl::fromROSMsg(*pcl_msg, *cloud);

        cv::Mat pcl2img(camera_info_msg->height, camera_info_msg->width, CV_8UC3, cvScalar(0,0,0));
        

        std::transform(cloud->points.begin(), cloud->points.end(), pcl2img.begin<cv::Vec3b>(),
            [this](auto in){return cv::Vec3b(in.b,in.g,in.r);});

        int length;
        std::vector<std::string> codes;
        std::vector<geometry_msgs::msg::Point> centers;
        std::vector<geometry_msgs::msg::Point> corners;
        
        QrReader::detect_qr_codes_(pcl2img, length, codes, centers, corners);

        if (!codes.empty()){
            
            // pixel position 2 world coordinates
            QrReader::convert_positions_(corners, cloud, pcl2img.cols);
            QrReader::convert_positions_(centers, cloud, pcl2img.cols);
                    
            qrcode_msg_.count = length;
            qrcode_msg_.string = codes;
            qrcode_msg_.corners = corners;
            qrcode_msg_.center = centers;
            detection_pub_->publish(qrcode_msg_);

            if (debug_computation_time_){
                end_time_ = get_clock()->now();
                auto ct = (end_time_.nanoseconds() - start_time_.nanoseconds());
                RCLCPP_INFO(get_logger(), "Computaion time is: %lu usec", ct/1000 );
            }

            if (debug_marker_){
                std::string sensor_frame_id = camera_info_msg->header.frame_id;
                QrReader::publish_markers_fnc(corners, sensor_frame_id);
            }
        }
    } // end of QrReader::pcl_callback


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


    } //end of QrReader::detect_qr_codes_

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

    } // end of QrReader::mark_qrcodes_on_image_fnc


    void QrReader::publish_markers_fnc(std::vector<geometry_msgs::msg::Point> corners, std::string &sensor_frame_id){

        visualization_msgs::msg::Marker qr_code_marker;
        qr_code_marker.header.frame_id = sensor_frame_id;
        qr_code_marker.header.stamp = get_clock()->now();
        qr_code_marker.type = visualization_msgs::msg::Marker::ARROW;
        qr_code_marker.action = visualization_msgs::msg::Marker::ADD;
        qr_code_marker.lifetime = rclcpp::Duration(1s);
        //marker.ns = "my_namespace";

        qr_code_marker.color.r = 1.0;
        qr_code_marker.color.g = 0.0;
        qr_code_marker.color.b = 0.0;
        qr_code_marker.color.a = 1.0;

        qr_code_marker.scale.x = 0.01;
        qr_code_marker.scale.y = 0.05;
        qr_code_marker.scale.z = 0.05;

        geometry_msgs::msg::Point p1, p2;

        p1.x = 0;
        p1.y = 0;
        p1.z = 0;

        for (long unsigned int indx=0; indx<corners.size(); indx+=4)
        {
            p2.x = (corners[indx].x + corners[indx+1].x + corners[indx+2].x + corners[indx+3].x)/4;
            p2.y = (corners[indx].y + corners[indx+1].y + corners[indx+2].y + corners[indx+3].y)/4;
            p2.z = (corners[indx].z + corners[indx+1].z + corners[indx+2].z + corners[indx+3].z)/4;

            qr_code_marker.id=indx;
            qr_code_marker.points= {p1,p2};
            marker_pub_->publish(qr_code_marker);
        }

    } // QrReader::publish_markers_fnc

    void QrReader::convert_positions_(std::vector<geometry_msgs::msg::Point> &pixel_positions, 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                long int cols)
    {
        for (auto pos_iter = pixel_positions.begin(); pos_iter<pixel_positions.end(); pos_iter++)
        {
            int x=std::round(pos_iter->x);
            int y=std::round(pos_iter->y);
            int index = y*cols +x;
            
            if(std::isinf(cloud->points[index].x) || 
                std::isinf(cloud->points[index].y) || 
                std::isinf(cloud->points[index].z))
            {
                pos_iter->x = 0;
                pos_iter->y = 0;
                pos_iter->z = 0;
                
                RCLCPP_INFO_EXPRESSION(get_logger(), close_to_object_,
                    "dept info is not available! too close to object(s)!");
                close_to_object_=false;
            } else {
                pos_iter->x = cloud->points[index].x;
                pos_iter->y = cloud->points[index].y;
                pos_iter->z = cloud->points[index].z;
                
                RCLCPP_INFO_EXPRESSION(get_logger(), !close_to_object_,
                    "dept info is available!");
                close_to_object_=true;
            }

        }

    } // QrReader::convert_positions_

} // namespace qr_reader_ns