#include <iostream>
#include <ros/ros.h>
#include <tf2/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "GetRT.h"

cv::FileStorage fs("/home/tsg/cartographer_new111.yaml", cv::FileStorage::WRITE);
int frame_num=0;
cv::Mat Rcl=cv::Mat::zeros(3,3,CV_32F);
cv::Mat tcl=cv::Mat::zeros(3,1,CV_32F);
cv::Mat Rwl1=cv::Mat::eye(3,3,CV_32F);
cv::Mat twl1=cv::Mat::zeros(3,1,CV_32F);
GetRT GR;
int tt=1;

std::string convert (int num)
{
    std::stringstream ss;
    ss<<num;
    return ss.str();
}

void tf_handle(const tf2_msgs::TFMessage::ConstPtr &sub)
{
    Eigen::Vector3d t0(sub->transforms[0].transform.translation.x,sub->transforms[0].transform.translation.y,sub->transforms[0].transform.translation.z);
    Eigen::Quaterniond q0;
    q0.x()=sub->transforms[0].transform.rotation.x;
    q0.y()=sub->transforms[0].transform.rotation.y;
    q0.z()=sub->transforms[0].transform.rotation.z;
    q0.w()=sub->transforms[0].transform.rotation.w;
    Eigen::Matrix3d R0 = Eigen::Matrix3d(q0);

    cv::Mat Rwl=cv::Mat::eye(3,3,CV_32F);
    cv::Mat twl=cv::Mat::zeros(3,1,CV_32F);
    
    
    if(sub->transforms[0].header.stamp.toSec()>=1.5590270260168998e+09)
    {
        frame_num++;
        if(frame_num==1)
        {
            for(int m=0;m<3;m++)
            {
                twl1.at<float>(m,0)=t0(m,0);
                for(int n=0;n<3;n++)
                {
                Rwl1.at<float>(m,n)=R0(m,n);
                }
            }
        }
        else
        {
            for(int m=0;m<3;m++)
            {
                twl.at<float>(m,0)=t0(m,0);
                for(int n=0;n<3;n++)
                {
                Rwl.at<float>(m,n)=R0(m,n);
                }
            }
            cv::Mat Rcw=Rcl*Rwl.t()*Rwl1*Rcl.t();
            cv::Mat tcw=Rcl*Rwl.t()*(twl1-twl)+tcl-Rcw*tcl;
            fs<<"frame"+convert(frame_num)<<"{";
            fs<<"stamp"<<sub->transforms[0].header.stamp.toSec();
            fs<<"R"<<Rcw;
            fs<<"t"<<tcw<<"}";
        }
        

    }
    
    // cv::Mat Rcw=Rcl*Rwl.t();
    // cv::Mat tcw=-Rcw*twl+tcl;
    // frame_num++;
    // fs<<"frame"+convert(frame_num)<<"{";
    // fs<<"stamp"<<sub->transforms[0].header.stamp.toSec();
    // fs<<"R"<<Rcw;
    // fs<<"t"<<tcw<<"}";
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"storeyaml");
    ros::NodeHandle nh;
    GR.RT(Rcl, tcl);
    ros::Subscriber sub=nh.subscribe<tf2_msgs::TFMessage> ("/tf", 2, tf_handle);
    ros::spin();
    return 0;
}