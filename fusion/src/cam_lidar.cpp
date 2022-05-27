//
// Created by ubuntu on 7/7/21.
//
#include "Sub2Pub.h"

// 设备序列号检测
bool detect_serial_number()
{
    FILE *pp = popen("cat /sys/firmware/devicetree/base/serial-number", "r"); // build pipe
    if (!pp)
        return 1;

    // collect cmd execute result
    char tmp[1024];
    while (fgets(tmp, sizeof(tmp), pp) != NULL)
    {
        // std::cout << tmp << std::endl; // can join each line as string
        std::cout << "\n";
    }
    pclose(pp);
    std::string serial_number = tmp;
    if( serial_number == "1424621088964" )  //nano
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char ** argv)
{
    if(true)
    {
        ros::init(argc, argv, "cam_lidar_node");
        std::string yaml_name = "/home/ubuntu/fujian/neuvition_fusion.yaml";//change to out
        std::string engine_name = "/home/ubuntu/detect/myinference-new/config/yolov5s.engine";//change to out
        Sub2Pub sub2pub("/neuvition_image", "/neuvition_cloud", "/img_det", yaml_name, engine_name);

        ros::spin();
    }
    return 0;

}

//[{'supercategory': 'Pedestrian', 'id': 1, 'name': 'Pedestrian'},
//{'supercategory': 'Cyclist', 'id': 2, 'name': 'Cyclist'},
//{'supercategory': 'Car', 'id': 3, 'name': 'Car'},
//{'supercategory': 'Truck', 'id': 4, 'name': 'Truck'},
//{'supercategory': 'Tram', 'id': 5, 'name': 'Tram'},
//{'supercategory': 'Tricycle', 'id': 6, 'name': 'Tricycle'}]
