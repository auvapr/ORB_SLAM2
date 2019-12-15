/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/AuvState.h>
#include <sensor_msgs/Orientation.h>
#include <sensor_msgs/PoseEuler.h>
#include <auv_lib/transformation/pose_transform.h>


#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;


//posn vars
float x_init_auv, y_init_auv, z_init_auv = 0.0, ry_i_auv = 0.0, rp_i_auv = 0.0, rr_i_auv = 0.0;
float x_auv, y_auv, z_auv = 0.0, ry = 0.0, rp = 0.0, rr = 0.0;
cv::Mat vo_psn_init;
cv::Mat vo_or_init;
int gg = 0;
int cnt_garbage_frames = 0;
int cnt_thresh = 0;

//flicker removal vars
int num_frames = 1;
int frameCount = 0;
int kernel_size = 5;
int kabali;
std::vector<cv::Mat> currFrames;
std::vector<cv::Mat> loggedFrames;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

void callback(const sensor_msgs::AuvState::ConstPtr& posn)
{
    //ROS_INFO("boo");
    x_auv = posn->position.x - x_init_auv;
    y_auv = posn->position.y - y_init_auv;
    z_auv = posn->position.z - z_init_auv;

    ry = posn->orientation.yaw;
    rp = posn->orientation.pitch;
    rr = posn->orientation.roll;

    //sensor_msgs::PoseEuler psel;
    //psel.position.x = 1.0;
}



//flicker removal
cv::Mat flickerRemoval(cv::Mat img)
{
    cv::Mat temp, outt;
    std::vector<cv::Mat> out;
    cv::cvtColor(img, temp, 44);
    cv::split(temp, out);
    out[0].convertTo(outt, CV_64F);
    outt += 1;
    cv::log(outt, outt);
    if(frameCount < num_frames)
    {
        frameCount++;
        currFrames.push_back(temp.clone());
        loggedFrames.push_back(outt.clone());
        return img.clone();
    }
    else
    {
        for(int i = 0; i < num_frames-1; i++)
        {
            currFrames[i] = currFrames[i+1].clone();
            loggedFrames[i] = loggedFrames[i+1].clone();
        }
        currFrames[num_frames-1] = temp.clone();
        loggedFrames[num_frames-1] = outt.clone();
        cv::Mat medn = loggedFrames[num_frames-1].clone();
        for(int i = 0; i < medn.rows; i++)
        {
            double* midPtr = medn.ptr<double>(i);
            for(int j = 0; j < medn.cols; j++)
            {
                double* tmpPtr;
                std::vector<double> tmpcv;
                for(int k = 0; k < num_frames; k++)
                {
                    tmpPtr = loggedFrames[k].ptr<double>(i);
                    tmpcv.push_back(tmpPtr[j]);
                }
                std::sort(tmpcv.begin(), tmpcv.end());
                midPtr[j] = tmpcv[tmpcv.size()/2];
            }
        }
        cv::Mat diff, ldiff;
        diff = loggedFrames[(num_frames-1)/2] - medn;
        cv::GaussianBlur(diff, ldiff, cv::Size(kernel_size, kernel_size), 0, 0,
                                                         cv::BORDER_DEFAULT);
        cv::Mat finLogIm;
        finLogIm = loggedFrames[(num_frames-1)/2] - ldiff;
        std::vector<cv::Mat> planes;
        cv::split(currFrames[(num_frames-1)/2], planes);
        for(int i = 0; i < medn.rows; i++)
        {
            uchar* planeRptr = planes[0].ptr<uchar>(i);
            double* finLogPtr = finLogIm.ptr<double>(i);
            for(int j = 0; j < medn.cols; j++)
            {
                int var = (int)exp(finLogPtr[j])-1;
                if(var > 255)
                {
                    var = 255;
                }
                else if(var < 0)
                {
                    var = 0;
                }
                planeRptr[j] = (unsigned char) var;
            }
        }
        cv::Mat outLab, out;
        cv::merge(planes, outLab);
        cv::cvtColor(outLab, out, 56);
        return out;
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub2 = nodeHandler.subscribe("/localization/pose", 1, callback);

    /* ofstream f;
    f.open("/home/apurva/Desktop/data_lab3.txt");
    f.close(); */

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat preprocessedIm = flickerRemoval(cv_ptr->image); 

    cv::Mat Tcw = mpSLAM->TrackMonocular(preprocessedIm ,cv_ptr->header.stamp.toSec());

    if (Tcw.size() == cv::Mat::eye(4, 4, CV_32F).size())
    {
        cnt_garbage_frames++;
    }


    if (Tcw.size() == cv::Mat::eye(4, 4, CV_32F).size() && cnt_garbage_frames > cnt_thresh)
    {
        //ROS_INFO("loo");
        //std::cout << Tcw.rowRange(0,3).colRange(0,3).size();
        cv::Mat R3 = cv::Mat::zeros(Tcw.rowRange(0,3).colRange(0,3).size(), CV_32F);
        Tcw.rowRange(0,3).colRange(0,3).copyTo(R3);
        cv::Mat R3t = cv::Mat::zeros(R3.size(), CV_32F);
        cv::transpose(R3,R3t);
        cv::Mat trl = Tcw.rowRange(0,3).col(3);
        cv::Mat postn_vo = -1*R3t*trl; 
        
        if (gg == 0)
        {
            x_init_auv = x_auv;
            y_init_auv = y_auv;
            z_init_auv = z_auv;
            ry_i_auv = ry;
            rp_i_auv = rp;
            rr_i_auv = rr;
            gg += 1;

            postn_vo.copyTo(vo_psn_init);

            //std::cout << "Initial : " << ry_i_auv << ' ' << rp_i_auv << ' ' << rr_i_auv << endl;
        }

        postn_vo -= vo_psn_init;


        /* float rot3[3][3];
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                rot3[i][j] = R3t.at<float>(i,j);
                std::cout << rot3[i][j];
            }
        }
        float* M = &rot3[0][0]; */

        //std::cout << endl;
        sensor_msgs::PoseEuler psnvo;
        psnvo.position.x = postn_vo.at<float>(0,0);
        psnvo.position.y = postn_vo.at<float>(0,1);
        psnvo.position.z = postn_vo.at<float>(0,2);

        /* float q[4];
        auv_lib::orientationToQuaternion(M,q);
        auv_lib::quaternionToEuler(q, psnvo.orientation.roll, psnvo.orientation.pitch, psnvo.orientation.yaw);
         */

        float psninit[] = {0.0, 0.0, 0.0, rr_i_auv  , rp_i_auv, ry_i_auv};
        /* psninit.position.x = x_init_auv;
        psninit.position.y = y_init_auv;
        psninit.position.z = z_init_auv; 

        psninit.orientation.yaw = ry_i;
        psninit.orientation.roll = rr_i;
        psninit.orientation.pitch = rp_i;*/

        float psnauv[] = {x_auv, y_auv, z_auv, rr, rp, ry};
        /* psnauv.position.x = x_auv;
        psnauv.position.y = y_auv;
        psnauv.position.z = z_auv;

        psnauv.orientation.yaw = ry;
        psnauv.orientation.roll = rr;
        psnauv.orientation.pitch = rp; */
        

        float respsn[] = {0,0,0,0,0,0};
        auv_lib::transform(respsn, psnauv, psninit, true);

        std::cout << endl << postn_vo << endl;
        std::cout << respsn[1] << ' '  << respsn[2] << ' ' << respsn[0] << ' ' << endl;
        
        /* ofstream f;
        f.open("/home/apurva/Desktop/data_lab3.txt", ios::out | ios::app);
        f << endl << respsn[1] << ' '  << respsn[2] << ' ' << respsn[0] << ' ' << endl;
        f << psnvo.position << endl;
        f.close(); */

    }
    

}
