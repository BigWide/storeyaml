#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "GetRT.h"

GetRT::GetRT()
{
}

GetRT::~GetRT()
{
}

void GetRT::RT(cv::Mat &Rcl, cv::Mat &tcl)
{
    Rcl.at<float>(0,0)=-0.996562587795097;
    Rcl.at<float>(0,1)=-0.06533690210995925;
    Rcl.at<float>(0,2)=-0.05093228671299116;

    Rcl.at<float>(1,0)=0.05025174187223036;
    Rcl.at<float>(1,1)=0.01201845898789988;
    Rcl.at<float>(1,2)=-0.9986642674504598;

    Rcl.at<float>(2,0)=0.06586175708214489;
    Rcl.at<float>(2,1)=-0.997790882833788;
    Rcl.at<float>(2,2)=-0.008693853450730371;


    tcl.at<float>(0,0)=-0.08544814942046484;
    tcl.at<float>(1,0)=-0.5684520875599799;
    tcl.at<float>(2,0)=-1.162070891954972;
}
