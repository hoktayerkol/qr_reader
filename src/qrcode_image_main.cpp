#include <memory>

#include "qr_reader/qrcode_image.hpp"
#include "rclcpp/rclcpp.hpp"

int main( int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    auto node_qrcode_image = std::make_shared<qrcode_image_ns::QrReader>();

    rclcpp::spin(node_qrcode_image);

    rclcpp::shutdown();
    
    return 0;
}

//pcl olana bir bak bakalim camera info gereklimi
// mesajlara header ekle
// tum nodlarda topic nameleri ve node nameleri falanda duzenle
// name spaceleride duzenle
// fazlalik includeleri sil
// gereksiz kalsorleri de sil
// pcl li olanda depth info available degilse resim uzerindeki pos lari yayinla