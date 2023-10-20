#include <memory>

#include "qr_reader/qrcode_pcl.hpp"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    auto node_qr_reader = std::make_shared<qr_reader_ns::QrReader>();

    rclcpp::spin(node_qr_reader);

    rclcpp::shutdown();
    
    return 0;
}