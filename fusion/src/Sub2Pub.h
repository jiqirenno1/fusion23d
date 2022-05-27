//
// Created by ubuntu on 2022/3/31.
//

#ifndef SRC_SUB2PUB_H
#define SRC_SUB2PUB_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>

#include "yolov5/YoloV5Detect.h"

#include "background/Bgmodel.h"
#include "ProcessPointClouds.h"
#include "tracking/Mot3D.h"

#include "fusion/img_det.h"
#include "fusion/img_det_array.h"



class Sub2Pub {
public:
    Sub2Pub(const std::string& topic_sub1, const std::string& topic_sub2, const std::string& topic_sub3, const std::string& yaml, const std::string& engine);
    void callback(const fusion::img_det_arrayConstPtr &input_img_det_msg,
                   const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
    void callback1(const sensor_msgs::ImageConstPtr &input_image_msg,
                  const fusion::img_det_arrayConstPtr &input_img_det_msg);
    cv::Rect get_rect1(cv::Mat& img, float bbox[4]);
    std::vector<cv::Point2d> cloud2pt2d(PtCdPtr cloud);
    bool point_rect(cv::Point2d & pp, cv::Rect& rect);

private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub_;
    message_filters::Subscriber<sensor_msgs::Image> img_sub_;
    message_filters::Subscriber<fusion::img_det_array>img_det_sub_;

    typedef message_filters::sync_policies::ApproximateTime<fusion::img_det_array, sensor_msgs::PointCloud2> syncpolicy;
    typedef message_filters::Synchronizer<syncpolicy> Sync;
    std::shared_ptr<Sync> sync_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, fusion::img_det_array> syncpolicy1;
    typedef message_filters::Synchronizer<syncpolicy1> Sync1;
    std::shared_ptr<Sync1> sync1_;

    ros::Publisher img_pub_;
    ros::Publisher lidar_pub_;
    ros::Publisher info_pub_;
    ros::Publisher marker_pub_;

    cv::Mat cam_intrinsic, cam_distcoeff, lidar2cam_R, lidar2cam_t;
    cv::Size img_size;

    //std::shared_ptr<YoloV5Detect> det_ptr;
    std::shared_ptr<Bgmodel> bgmodel_ptr;
    std::shared_ptr<Mot3D> mot_ptr{new Mot3D};
    //std::vector<fusion::img_det_array> segment;

    int count = 0;

};

#endif //SRC_SUB2PUB_H
