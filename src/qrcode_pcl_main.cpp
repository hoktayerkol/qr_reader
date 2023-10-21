#include <memory>

#include "qr_reader/qrcode_pcl.hpp"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    auto node_qrcode_pcl = std::make_shared<qrcode_pcl_ns::QrReader>();

    rclcpp::spin(node_qrcode_pcl);

    rclcpp::shutdown();
    
    return 0;
}