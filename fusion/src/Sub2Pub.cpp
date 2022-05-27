//
// Created by ubuntu on 2022/3/31.
//

#include "Sub2Pub.h"
Sub2Pub::Sub2Pub(const std::string& topic_sub1, const std::string& topic_sub2, const std::string& topic_sub3, const std::string& yaml, const std::string& engine) {
    img_sub_.subscribe(nh_, topic_sub1, 1);
    lidar_sub_.subscribe(nh_, topic_sub2, 1);
    img_det_sub_.subscribe(nh_, topic_sub3, 1);

    sync_.reset(new Sync(syncpolicy(10), img_det_sub_, lidar_sub_));
    sync_->registerCallback(boost::bind(&Sub2Pub::callback, this, _1, _2));

    sync1_.reset(new Sync1(syncpolicy1(10), img_sub_, img_det_sub_));
    sync1_->registerCallback(boost::bind(&Sub2Pub::callback1, this, _1, _2));

    img_pub_=nh_.advertise<sensor_msgs::Image>("sync/img",1000);
    lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sync/lidar", 1000);
//    info_pub_ = nh_.advertise<fusion::det_array>("/wuti_info", 10);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/sync/markers", 2);

    //fusion param
    cv::FileStorage fs_reader(yaml, cv::FileStorage::READ);
    fs_reader["CameraMat"] >> cam_intrinsic;
    fs_reader["DistCoeff"] >> cam_distcoeff;
    fs_reader["ImageSize"] >> img_size;
    fs_reader["lidar2cam_R"]>>lidar2cam_R;
    fs_reader["lidar2cam_t"]>>lidar2cam_t;
    fs_reader.release();
    //

   // det_ptr = std::make_shared<YoloV5Detect>(engine);
//    bgmodel_ptr = std::make_shared<Bgmodel>(1, 20, 10, 100, -10, -5, 0);//change 0
    bgmodel_ptr = std::make_shared<Bgmodel>(0.5, 5, 5, 120, -1.5, -5.8, 0);
}

void Sub2Pub::callback1(const sensor_msgs::ImageConstPtr &img_msg,
                       const fusion::img_det_arrayConstPtr &img_det_msg) {
    cv::Mat img;
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("Cv_bridge Exception:" << e.what());
        return;
    }
    img = cv_ptr->image;

    std::cout<<"det 2d size:  "<<img_det_msg->array.size()<<std::endl;
    for(auto &e:img_det_msg->array)
    {
//        std::cout<<"x "<<e.x<<std::endl;
//        std::cout<<"y "<<e.y<<std::endl;
//        std::cout<<"h "<<e.width<<std::endl;
//        std::cout<<"w "<<e.height<<std::endl;
        cv::Rect r(e.x, e.y, e.width, e.height);
        cv::rectangle(img, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

    }


    cv_bridge::CvImage output_image;
    output_image.header.frame_id = "";
    output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    output_image.image = img;


    img_pub_.publish(output_image);
}
void Sub2Pub::callback(const fusion::img_det_arrayConstPtr &img_det_msg,
                       const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    count++;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr objects(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    //change1
    //theta atan2(dety, detz)
//    float theta = atan2(2.975, 14.11);
    float theta = atan2(2.139, 8.681);
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud, T);

    sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*cloud, cloud_out);
    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = "/neuvition";
    lidar_pub_.publish(cloud_out);

//    std::string img_name = std::to_string(img_msg->header.stamp.sec) + "_" + std::to_string(img_msg->header.stamp.nsec) + ".jpg";
//    std::string cloud_name = std::to_string(cloud_msg->header.stamp.sec) + "_" + std::to_string(cloud_msg->header.stamp.nsec) + ".pcd";
//
//    cv::imwrite("/home/ubuntu/calibrate_neuvition/m4/" + img_name, img);
//    pcl::io::savePCDFileASCII("/home/ubuntu/calibrate_neuvition/m4/" + cloud_name, *cloud);

    std::shared_ptr<ProcessPointClouds> ppc_ptr = std::make_shared<ProcessPointClouds>();

    if(count<60)
    {
        bgmodel_ptr->setBackground(cloud);
        std::cout<<"initialize"<<std::endl;
    }
    else
    {
        vector<Eigen::Vector3d> dets;

        std::pair<PtCdPtr, PtCdPtr> res_cloud = bgmodel_ptr->getDiff(cloud);
        objects = res_cloud.first;

        if (objects->size() > 5)
        {
            std::vector<PtCdPtr> clusters = ppc_ptr->Clustering(objects, 0.6, 5, 10000);

            visualization_msgs::MarkerArray cubes;
            visualization_msgs::Marker cube;
            cube.action = visualization_msgs::Marker::DELETEALL;
            cubes.markers.push_back(cube);
            marker_pub_.publish(cubes);

            if (!clusters.empty())
            {
                //track and get speed
                for (int i=0;i<clusters.size();i++)
                {
                    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
                    cluster = clusters[i];

                    PointT min_pt, max_pt;
                    pcl::getMinMax3D(*cluster, min_pt, max_pt);

                    Eigen::Vector3d deti;
                    deti << (min_pt.x+max_pt.x)/2, (min_pt.y+max_pt.y)/2, (min_pt.z+max_pt.z)/2;
                    dets.push_back(deti);

                    //publish markers
                    cube.action = visualization_msgs::Marker::ADD;
                    cube.header.frame_id = "neuvition";
                    cube.header.stamp = ros::Time::now();
                    cube.id = i;
                    cube.type = visualization_msgs::Marker::CUBE;
                    cube.scale.x = max_pt.x - min_pt.x;
                    cube.scale.y = max_pt.y - min_pt.y;
                    cube.scale.z = max_pt.z - min_pt.z;
                    cube.color.a = 0.5;
                    cube.color.r = 0;
                    cube.color.g = 255;
                    cube.color.b = 0;
                    cube.pose.position.x = (max_pt.x + min_pt.x)/2;
                    cube.pose.position.y = (max_pt.y + min_pt.y)/2;
                    cube.pose.position.z = (max_pt.z + min_pt.z)/2;
                    cube.pose.orientation.x = 0.0;
                    cube.pose.orientation.y = 0.0;
                    cube.pose.orientation.z = 0.0;
                    cube.pose.orientation.w = 1.0;
                    cubes.markers.push_back(cube);
                    marker_pub_.publish(cubes);
                }
                double time = 0;
                vector<pair<int, float>> idspeed = mot_ptr->update(dets, time);

                //match img and cloud base img
                for(auto &e:img_det_msg->array)
                {
                    cv::Rect rect(e.x, e.y, e.width, e.height);
                    int max_nums = 0;
                    int id = 0;
                    for(int obj=0;obj<clusters.size();obj++)
                    {
                        std::vector<cv::Point2d> cluster_2d;
                        cluster_2d = cloud2pt2d(clusters[obj]);

                        int nums = 0;
                        for(auto &p:cluster_2d)
                        {
                            if (point_rect(p, rect))
                                nums++;
                        }
                        if(nums>=max_nums)
                        {
                            max_nums = nums;
                            id = obj;
                        }
                    }

                    PointT min_pt0, max_pt0;
                    pcl::getMinMax3D(*clusters[id], min_pt0, max_pt0);
                    float pos = (max_pt0.z + min_pt0.z)/2;

                    if((max_nums>=10&&pos<50)||(max_nums>=1&&pos>=50))
                    {
                        float speed = 0;
                        if(idspeed[id].first != -1)
                        {

//                            std::cout<<"speed: "<<idspeed[id].second<<" " <<"id: "<<re<<std::endl;
                            speed = idspeed[id].second;
                        }

                        PointT min_pt, max_pt;
                        pcl::getMinMax3D(*clusters[id], min_pt, max_pt);

                        //publish markers
                        cube.action = visualization_msgs::Marker::ADD;
                        cube.header.frame_id = "neuvition";
                        cube.header.stamp = ros::Time::now();
                        cube.id = id;
                        cube.type = visualization_msgs::Marker::CUBE;
                        cube.scale.x = max_pt.x - min_pt.x;
                        cube.scale.y = max_pt.y - min_pt.y;
                        cube.scale.z = max_pt.z - min_pt.z;
                        cube.color.a = 0.5;
                        cube.color.r = 255;
                        cube.color.g = 0;
                        cube.color.b = 0;
                        cube.pose.position.x = (max_pt.x + min_pt.x)/2;
                        cube.pose.position.y = (max_pt.y + min_pt.y)/2;
                        cube.pose.position.z = (max_pt.z + min_pt.z)/2;
                        cube.pose.orientation.x = 0.0;
                        cube.pose.orientation.y = 0.0;
                        cube.pose.orientation.z = 0.0;
                        cube.pose.orientation.w = 1.0;
                        cubes.markers.push_back(cube);
                        marker_pub_.publish(cubes);


                    }

                }

                /*//match img and cloud base cloud
                for (int i = 0; i < clusters.size(); i++)
                {
                    pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
                    cluster = clusters[i];

                    std::vector<cv::Point2d> cluster_2d;
                    cluster_2d = cloud2pt2d(cluster);
                    for(auto &p:cluster_2d)
                    {
                        cv::circle(img, p, 1, cv::Scalar(0, 255, 255));
                    }

                    int max_nums = 0;
                    int id = 0;
                    for (int j = 0; j < res.size(); j++)
                    {
                        int nums = 0;
                        cv::Rect r = get_rect1(img, res[j].bbox);
                        for(auto &p:cluster_2d)
                        {
                            if (point_rect(p, r))
                            {
                                nums++;
                            }
                        }
                        if(nums>=max_nums)
                        {
                            max_nums = nums;
                            id = j;
                        }
                    }

                    if(max_nums>3)
                    {
                        for(auto &p:cluster_2d)
                        {
                            cv::circle(img, p, 1, cv::Scalar(0, 0, 255));
                        }

                    }

                }
                */

            }
        }
    }
//    lidar_pub_.publish(cloud_msg);


}

cv::Rect Sub2Pub::get_rect1(cv::Mat &img, float *bbox) {
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

std::vector<cv::Point2d> Sub2Pub::cloud2pt2d(PtCdPtr cloud) {
    pcl::PointCloud<PointT>::Ptr cloud_out(new pcl::PointCloud<PointT>);
    float theta = atan2(2.139, 8.681);
    Eigen::Affine3d T1 = Eigen::Affine3d::Identity();
    T1.rotate(Eigen::AngleAxisd(-theta, Eigen::Vector3d::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud_out, T1);

    std::vector<cv::Point3d> lidar_3d;
    std::vector<cv::Point2d> cam_2d;

    for(auto&e:cloud_out->points)
    {
        lidar_3d.emplace_back(e.x, e.y, e.z);
    }
    cv::projectPoints(lidar_3d, lidar2cam_R, lidar2cam_t, cam_intrinsic, cam_distcoeff, cam_2d);

    return cam_2d;
}

bool Sub2Pub::point_rect(cv::Point2d & pp, cv::Rect& rect) {
    double l = rect.x;
    double t = rect.y;
    double r = l + rect.width;
    double b = t + rect.height;
    if(pp.x>=l&&pp.x<=r&&pp.y>=t&&pp.y<=b)
        return true;
    return false;
}
