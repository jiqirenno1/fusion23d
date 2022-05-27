//
// Created by ubuntu on 2021/9/24.
//

#include "Bgmodel.h"

void Bgmodel::setBackground(PtCdPtr cloud) {
    std::vector<int> bool_change {std::vector<int>(gridXYZ, 0)};
    for(auto &e:cloud->points)
    {
        float x = e.x;
        float y = e.y;
        float z = e.z;

        int indexX = std::min(std::max(int((x-oriX_)/res_), 0), gridX-1);
        int indexY = std::min(std::max(int((y-oriY_)/res_), 0), gridY-1);
        int indexZ = std::min(std::max(int((z-oriZ_)/res_), 0), gridZ-1);

        int index = indexY*gridXZ + indexX*gridZ + indexZ;
        if(bool_change[index]==0)
        {
            density[index] = density[index]+1;
            bool_change[index]=1;
        }

    }

}

std::pair<PtCdPtr, PtCdPtr> Bgmodel::getDiff(PtCdPtr input) {
    PtCdPtr backCloud(new pcl::PointCloud<PointT>);
    PtCdPtr frontCloud(new pcl::PointCloud<PointT>);

    for (auto &e:input->points) {
        float x = e.x;
        float y = e.y;
        float z = e.z;

//        if(((y>-5.7&&x>-2&&x<1.5)||(x>-2&&z>45&&x<1.5)))
        if(((y>-5.7&&x>-2)||(x>-2&&z>45)))
        {
            int indexX = std::min(std::max(int((x - oriX_) / res_), 0), gridX - 1);
            int indexY = std::min(std::max(int((y - oriY_) / res_), 0), gridY - 1);
            int indexZ = std::min(std::max(int((z - oriZ_) / res_), 0), gridZ - 1);

            int index = indexY * gridXZ + indexX * gridZ + indexZ;

            if (density[index] > thresh) {
                backCloud->points.push_back(e);
            } else {
                frontCloud->points.push_back(e);
            }
        }
    }

    std::pair<PtCdPtr, PtCdPtr>result(frontCloud, backCloud);
    return result;
}

PtCdPtr Bgmodel::getBackGroundCloud() {
    if(!is_expand)
    {
        std::vector<int> bg = density;
        is_expand = true;

        for(int i=0;i<gridXYZ;i++)
        {
            int value = bg[i];
            if(value>thresh)
            {
                int step = 1;
                for(int a=-step;a<step+1;a++)
                {
                    for(int b=-step;b<step+1;b++)
                    {
                        for(int c=-step;c<step+1;c++)
                        {
                            density[std::min(std::max(i+a+b*gridZ+c*gridXZ, 0), gridXYZ-1)] = thresh + 10;
                        }

                    }
                }
            }
        }

    }
    PtCdPtr outCloud(new pcl::PointCloud<PointT>);
    cv::Mat img = cv::Mat::zeros(gridZ, gridX, CV_8UC1);
    for(int i=0;i<gridX;i++)
    {
        for(int j=0;j<gridY;j++)
        {
            for(int k=0;k<gridZ;k++)
            {
                float x = oriX_ + res_*i;
                float y = oriY_ + res_*j;
                float z = oriZ_ + res_*k;
                PointT point;
                int indexX = std::min(std::max(int((x - oriX_) / res_), 0), gridX - 1);
                int indexY = std::min(std::max(int((y - oriY_) / res_), 0), gridY - 1);
                int indexZ = std::min(std::max(int((z - oriZ_) / res_), 0), gridZ - 1);

                int index = indexY * gridXZ + indexX * gridZ + indexZ;

                if (density[index] > thresh) {
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    outCloud->points.push_back(point);
                }
                img.at<uchar>(k, i) = img.at<uchar>(k, i) + density[index];

            }
        }
    }
    cv::imwrite("/home/ubuntu/bg.jpg", img);
    return outCloud;
}
