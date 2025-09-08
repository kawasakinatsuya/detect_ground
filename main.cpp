#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
# include <pcl/segmentation/sac_segmentation.h>
# include <pcl/filters/extract_indices.h>

class DistanceColorNode : public rclcpp::Node
{
public:
  DistanceColorNode()
  : Node("distance_color_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10,
      std::bind(&DistanceColorNode::cloud_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/detect_ground", 10);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true); 
    seg.setModelType(pcl::SACMODEL_PLANE); 
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients); 

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i=0; i < input->points.size(); ++i)
    {   
      pcl::PointXYZRGB pt;
      pt.x = input->points[i].x;
      pt.y = input->points[i].y;
      pt.z = input->points[i].z;

      if(pt.z > 0.1){
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
      }

      else{
        pt.r = 0;
        pt.g = 0;
        pt.b = 255;
      }
      
      output->points.push_back(pt);
    }

    for (size_t k=0; k < inliers->indices.size(); ++k)
    { 
      int num = inliers->indices[k];
      output->points[num].r = 0;
      output->points[num].g = 255;
      output->points[num].b = 0;
    }

    output->width = output->points.size();
    output->height = 1;

    output->header.frame_id = msg->header.frame_id;
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*output, out_msg);
    pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceColorNode>());
  rclcpp::shutdown();
  return 0;
}

