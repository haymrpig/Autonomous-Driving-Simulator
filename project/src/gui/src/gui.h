#ifndef GUI_H
#define GUI_H

#include "Parameters.h"
#include "src/driving.h"
#include "ros/ros.h"
#include "adasone_msgs/Steering.h"
#include "adasone_msgs/Point.h"
#include "sensor_msgs/NavSatFix.h"
#include "adasone_msgs/Odometry.h"
#include <QApplication>
#include <thread>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class Gui{
public:
    Gui();
    ~Gui();

    void RunGui(int, char**);

private:
    cv::Mat camimage;
    cv::Mat camimagelane;
    cv::Mat camimage4drawing;
    cv::Mat camimage4drawinglane;

  

    ros::NodeHandle nh_;
    ros::Subscriber sub_cam;
    //ros::Subscriber sub_steeringmap;
    ros::Subscriber sub_screenpoint;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_lanedetectresult;
    //ros::Subscriber sub_usr_steering;
    ros::Subscriber sub_userParam;
    ros::Subscriber sub_odometry;
    int image_width;
    int image_height;
    float unit;
    float absmaxmin;
    float num_diff;
    float pixeldiff;
    double timediff;
    ros::Time timestamp_temp;
    vector<double> timestamp;
    vector<vector<double>> plotpoint;
    vector<vector<double>> plotpoint1;
    vector<vector<double>> plotpoint2;
    vector<vector<double>> plotpoint3;
    vector<vector<double>> plotpoint4;
    vector<vector<double>> plotpoint5;


    vector<double> lanediffall;
    vector<double> mapdiffall;
    vector<double> weighteddiffall;

    double last_steering;
    bool first_input = false;
    void plotsteering(float error, float time, Scalar color);
    void plotweight(float error, float time, Scalar color);
    void plotlane(float error, float time, Scalar color);
    void plotCrossTrackError(float error, float time, Scalar color);
    void plotOdoCrossTrackError(float error, float time, Scalar color);
    void plotusrSteering(float error, float time, Scalar color);


    void Callbackimage(const sensor_msgs::ImageConstPtr& msg);
    void Callbackgps(const sensor_msgs::NavSatFix::ConstPtr& msg);

  	void CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg);
    void callback(const adasone_msgs::Steering::ConstPtr& usrNlane, const adasone_msgs::Steering::ConstPtr& map, const adasone_msgs::UserCustomParam::ConstPtr& lane);

    void callbackScreen(const adasone_msgs::Point::ConstPtr& HDpoint, const adasone_msgs::UserCustomParam::ConstPtr& lanepoint);
    void QtThread();
    
    Driving *qt_handle_{nullptr};
    QApplication *gui_app_{nullptr};

    double lookaheadx,lookaheady;
    Parameters param;
    Parameters* params;

    int argc_;
    char** argv_;


    std::thread qt_thread_;

    
};

#endif
