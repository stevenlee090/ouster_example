/**
 * @file
 * @brief Example node to visualize range, noise and intensity images
 *
 * Publishes ~/range_image, ~/noise_image, and ~/intensity_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os1_cloud_node/points
 */

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ouster/os1.h>
#include <ouster/os1_packet.h>
#include <ouster/os1_util.h>
#include <ouster_ros/OS1ConfigSrv.h>
#include <ouster_ros/os1_ros.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace OS1 = ouster::OS1;

int main(int argc, char** argv) {
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh("~");

    // parse intensity cap parameter
    int intensity_cap = 255;
    nh.getParam("intensity_cap", intensity_cap);

    ouster_ros::OS1ConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OS1ConfigSrv>("os1_config");
    client.waitForExistence();
    if (!client.call(cfg)) {
        ROS_ERROR("Calling os1 config service failed");
        return EXIT_FAILURE;
    }

    auto H = OS1::pixels_per_column;
    auto W = OS1::n_cols_of_lidar_mode(
        OS1::lidar_mode_of_string(cfg.response.lidar_mode));

    const auto px_offset = ouster::OS1::get_px_offset(W);

    ros::Publisher range_image_pub =
        nh.advertise<sensor_msgs::Image>("range_image", 100);
    ros::Publisher noise_image_pub =
        nh.advertise<sensor_msgs::Image>("noise_image", 100);
    ros::Publisher intensity_image_pub =
        nh.advertise<sensor_msgs::Image>("intensity_image", 100);
    ros::Publisher reflectivity_image_pub =
        nh.advertise<sensor_msgs::Image>("reflectivity_image", 100);

    ouster_ros::OS1::CloudOS1 cloud{};

    auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
        pcl::fromROSMsg(*m, cloud);

        sensor_msgs::Image range_image;
        sensor_msgs::Image noise_image;
        sensor_msgs::Image intensity_image;
        sensor_msgs::Image reflectivity_image;

        range_image.width = W;
        range_image.height = H;
        range_image.step = W;
        range_image.encoding = "mono8";
        range_image.data.resize(W * H);
        range_image.header.stamp = m->header.stamp;

        noise_image.width = W;
        noise_image.height = H;
        noise_image.step = W;
        noise_image.encoding = "mono8";
        noise_image.data.resize(W * H);
        noise_image.header.stamp = m->header.stamp;

        intensity_image.width = W;
        intensity_image.height = H;
        intensity_image.step = W;
        intensity_image.encoding = "mono8";
        intensity_image.data.resize(W * H);
        intensity_image.header.stamp = m->header.stamp;
        
        cv::Mat img16(H, W, CV_16UC1);

        for (int u = 0; u < H; u++) {
            for (int v = 0; v < W; v++) {
                const size_t vv = (v + px_offset[u]) % W;
                const size_t index = vv * H + u;
                const auto& pt = cloud[index];

                if (pt.range == 0) {
                    range_image.data[u * W + v] = 0;
                } else {
                    range_image.data[u * W + v] =
                        255 - std::min(std::round(pt.range * 5e-3), 255.0);
                }
                noise_image.data[u * W + v] = std::min(pt.noise, (uint16_t)255);

                float pt_intensity = std::min(pt.intensity, (float)intensity_cap);
                pt_intensity = pt_intensity / (float)intensity_cap * 255.0;

                intensity_image.data[u * W + v] = pt_intensity;
                // intensity_image.data[u * W + v] = std::min(pt.intensity, 255.f);

                // build reflectivity image
                img16.at<uint16_t>(u, v) = pt.intensity;
                // img16.at<uint16_t>(u, v) = 60000;
            }
        }

        // ROS_INFO("intensity cap = %d", intensity_cap);

        range_image_pub.publish(range_image);
        noise_image_pub.publish(noise_image);
        intensity_image_pub.publish(intensity_image);

        // TODO: attempt to publish 16-bit reflectivity image
        cv::Mat img16_norm(H, W, CV_16UC1);
        cv::normalize(img16, img16_norm, 0, 65536, cv::NORM_MINMAX);

        cv_bridge::CvImage img_bridge;
        img_bridge = cv_bridge::CvImage(m->header, sensor_msgs::image_encodings::MONO16, img16_norm);
        img_bridge.toImageMsg(reflectivity_image);
        reflectivity_image_pub.publish(reflectivity_image);
    };

    auto pc_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("points", 500, cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}
