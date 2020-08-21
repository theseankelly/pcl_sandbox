#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <functional>
#include <memory>


class CloudConverter : public rclcpp::Node
{
public:
  CloudConverter() : Node("cloud_converter")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ifm3d/camera/cloud",
      rclcpp::SensorDataQoS(),
      std::bind(&CloudConverter::cloud_callback, this, std::placeholders::_1));
  }
private:
  void cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr in);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};


void CloudConverter::cloud_callback(
  sensor_msgs::msg::PointCloud2::UniquePtr in)
{
  RCLCPP_INFO(this->get_logger(), "Got a cloud!");

  // convert to pcl
  pcl::PCLPointCloud2 pc2_in;
  pcl_conversions::toPCL(*in, pc2_in);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    point_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pc2_in, *point_cloud_in);
  RCLCPP_INFO(this->get_logger(), "CONVERSION TO PCL SUCCEEDED");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudConverter>());
  rclcpp::shutdown();
  return 0;
}
