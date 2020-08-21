#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <functional>
#include <memory>


class CloudTransformer : public rclcpp::Node
{
public:
  CloudTransformer() : Node("cloud_transformer")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_unique<tf2_ros::TransformListener>(*this->tf_buffer_);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "warped_cloud",
      rclcpp::SensorDataQoS());

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ifm3d/camera/cloud",
      rclcpp::SensorDataQoS(),
      std::bind(&CloudTransformer::cloud_callback, this, std::placeholders::_1));
  }
private:
  void cloud_callback(sensor_msgs::msg::PointCloud2::UniquePtr in);
  rclcpp::Subscription<pcl::PointCloud<pcl::PointXYZ>>::SharedPtr pcl_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  // tf
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};


void CloudTransformer::cloud_callback(
  sensor_msgs::msg::PointCloud2::UniquePtr in)
{
  RCLCPP_INFO(this->get_logger(), "Got a cloud!");

  geometry_msgs::msg::TransformStamped transform;
  try
    {
      transform = this->tf_buffer_->lookupTransform(
        "camera_link", "camera_optical_link", rclcpp::Time(0));
    }
  catch (const tf2::TransformException& ex)
    {
      RCLCPP_WARN(this->get_logger(), ex.what());
    }

  //// TODO -- transform the cloud. For now just create a copy.
  ////auto out = std::make_shared<sensor_msgs::msg::PointCloud2>(*in);

  pcl::PCLPointCloud2 pc2_in;
  pcl_conversions::toPCL(*in, pc2_in);
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    point_cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pc2_in, *point_cloud_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr
    point_cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl_ros::transformPointCloud("camera_optical_link", *point_cloud_in, *point_cloud_out, *tf_buffer_))
    {
      RCLCPP_INFO(this->get_logger(), "TRANSFORM SUCCEEDED");
    }
  else
    {
      RCLCPP_ERROR(this->get_logger(), "TRANSFORM FAILED");
    }

  // Convert back to ROS message
  pcl::PCLPointCloud2 pc2_out;
  pcl::toPCLPointCloud2(*point_cloud_out, pc2_out);
  sensor_msgs::msg::PointCloud2 message;
  pcl_conversions::fromPCL(pc2_out, message);
  this->publisher_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
