//
// Created by ubuntu on 2022/1/11.
//

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "src/yolov5/YoloV5Detect.h"

#include "det2d_node/img_det.h"
#include "det2d_node/img_det_array.h"


cv::Rect get_rect1(cv::Mat& img, float bbox[4]) {
    int l, r, t, b; //left right top bottom
    float r_w = Yolo::INPUT_W / (img.cols * 1.0);
    float r_h = Yolo::INPUT_H / (img.rows * 1.0);
    if (r_h > r_w) {
        l = bbox[0] - bbox[2] / 2.f;
        r = bbox[0] + bbox[2] / 2.f;
        t = bbox[1] - bbox[3] / 2.f - (Yolo::INPUT_H - r_w * img.rows) / 2;
        b = bbox[1] + bbox[3] / 2.f - (Yolo::INPUT_H - r_w * img.rows) / 2;
        l = l / r_w;
        r = r / r_w;
        t = t / r_w;
        b = b / r_w;
    } else {
        l = bbox[0] - bbox[2] / 2.f - (Yolo::INPUT_W - r_h * img.cols) / 2;
        r = bbox[0] + bbox[2] / 2.f - (Yolo::INPUT_W - r_h * img.cols) / 2;
        t = bbox[1] - bbox[3] / 2.f;
        b = bbox[1] + bbox[3] / 2.f;
        l = l / r_h;
        r = r / r_h;
        t = t / r_h;
        b = b / r_h;
    }
    return cv::Rect(l, t, r - l, b - t);
}

class Tranform
{
public:
    Tranform(ros::NodeHandle& nh, const std::string& topic_sub, const std::string& topic_pub, size_t buff_size);
    void callback(const sensor_msgs::ImageConstPtr& img_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::shared_ptr<YoloV5Detect> det_ptr = std::make_shared<YoloV5Detect>("/home/ubuntu/detect/myinference-new/config/yolov5s.engine");
};

Tranform::Tranform(ros::NodeHandle &nh, const std::string &topic_sub, const std::string &topic_pub, size_t buff_size) {
    nh_ = nh;
    sub_ = nh_.subscribe(topic_sub, buff_size, &Tranform::callback, this);
    pub_ = nh_.advertise<det2d_node::img_det_array>(topic_pub, 100);
}

void Tranform::callback(const sensor_msgs::ImageConstPtr &img_msg) {
    cv::Mat img;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR_STREAM("Cv_bridge Exception:"<<e.what());
        return;
    }
    img = cv_ptr->image;

    std::vector<std::vector<Yolo::Detection>> batch_res = det_ptr->SingleDetect(img);

    auto& res = batch_res[0];
    det2d_node::img_det_array img_dets;
    img_dets.header = img_msg->header;
    int nums = res.size();
    std::cout<<"det nums:  "<<nums<<std::endl;
    for (size_t j = 0; j < nums; j++) {
        det2d_node::img_det img_det;
        cv::Rect r = get_rect1(img, res[j].bbox);
        img_det.x = r.x;
        img_det.y = r.y;
        img_det.height = r.height;
        img_det.width = r.width;
        img_dets.array.push_back(img_det);
//        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
//        cv::putText(frame, std::to_string((int)res[j].class_id), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
    }

//    cv_bridge::CvImage output_image;
//    output_image.header.frame_id = "";
//    output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
//    output_image.image = img;
    pub_.publish(img_dets);


}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "det2d_node");
    ros::NodeHandle nh;
    Tranform transform(nh, "/neuvition_image", "/img_det", 10);
    ros::spin();

}
