#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;


class pointCloudPublisher
{
public:
  pointCloudPublisher(ros::NodeHandle nh)
    : nh_(nh)
  {
    cam_info_sub_ = nh_.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 1, &pointCloudPublisher::cameraInfoCallback, this);
    dep_img_sub_ = nh_.subscribe<sensor_msgs::Image>("depth/image_raw", 1, &pointCloudPublisher::depthImageCallback, this);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("depth/points", 1);

    pcmsg.height = 50;
    pcmsg.width = 50;
    pcmsg.is_bigendian = false;
    pcmsg.point_step = 12;
    pcmsg.row_step = pcmsg.point_step * 50;
    pcmsg.is_dense = false;
    pcmsg.fields.resize(pcmsg.point_step / 4);
    pcmsg.fields[0].name = "x";
    pcmsg.fields[0].offset = 0;
    pcmsg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    pcmsg.fields[0].count = 1;
    pcmsg.fields[1].name = "y";
    pcmsg.fields[1].offset = 4;
    pcmsg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    pcmsg.fields[1].count = 1;
    pcmsg.fields[2].name = "z";
    pcmsg.fields[2].offset = 8;
    pcmsg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    pcmsg.fields[2].count = 1;
  }


  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    // For camera not rotate
    fx = msg->K[0];
    fy = msg->K[4];
    u0 = msg->K[2];
    v0 = msg->K[5];

    // For camera rotate 90deg (clockwise)
    /*
    fx = msg->K[4];
    fy = msg->K[0];
    u0 = 100 - msg->K[5];
    v0 = msg->K[2];
    */

    cam_info_flag = true;

    cam_info_sub_.shutdown();
  }


  void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    if (!cam_info_flag) {
      ROS_WARN("No Camera Info Received.");
      return;
    }

    pcmsg.header = msg->header;

    pcmsg.data.resize((pcmsg.height) * (pcmsg.width) * (pcmsg.point_step), 0x00);

    uint8_t *ptr = pcmsg.data.data();

    int minI, minJ;
    int data1, data2, data3, data4;
    int minDataIn2x2;

    for (int j = 0; j < 100; j+=2) {
      for (int i = 0; i < 100; i+=2) {
        data1 = (int)msg->data[j * 100 + i];
        data2 = (int)msg->data[j * 100 + i + 1];
        data3 = (int)msg->data[(j + 1) * 100 + i];
        data4 = (int)msg->data[(j + 1) * 100 + i + 1];

        minDataIn2x2 = std::min(std::min(data1, data2), std::min(data3, data4));

        if (minDataIn2x2 == data1) {
          minI = i;
          minJ = j;
        } 
        else if (minDataIn2x2 == data2) {
          minI = i + 1;
          minJ = j;
        } 
        else if (minDataIn2x2 == data3) {
          minI = i;
          minJ = j + 1;
        } 
        else if (minDataIn2x2 == data4) {
          minI = i + 1;
          minJ = j + 1;
        }

        float cx = (((float)minI) - u0) / fx;
        float cy = (((float)minJ) - v0) / fy;

        // AT+UNIT = 0, (p/5.1)^2
        float dst = std::pow(((float)minDataIn2x2 / 5.1), 2) / 1000;

        if (minDataIn2x2 == 255) {
          float x = std::numeric_limits<float>::infinity();
          float y = std::numeric_limits<float>::infinity();
          float z = std::numeric_limits<float>::infinity();
          *((float *)(ptr + 0)) = x;
          *((float *)(ptr + 4)) = y;
          *((float *)(ptr + 8)) = z;
        }
        else {
          float x = dst;
          float y = -dst * cx;
          float z = -dst * cy;
          *((float *)(ptr + 0)) = x;
          *((float *)(ptr + 4)) = y;
          *((float *)(ptr + 8)) = z;
        }

        ptr += pcmsg.point_step;
      }
    }

    pc_pub_.publish(pcmsg);
  }


private:
  ros::NodeHandle nh_;
  ros::Subscriber dep_img_sub_;
  ros::Subscriber cam_info_sub_;
  ros::Publisher pc_pub_;

  sensor_msgs::PointCloud2 pcmsg;

  double fx;
  double fy;
  double u0;
  double v0;

  bool cam_info_flag = false;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_to_point_cloud");
  ros::NodeHandle nh;

  pointCloudPublisher pc_publisher(nh);

  ROS_INFO("Point Cloud Publisher Start.");

  ros::spin();

  return 0;
}