#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <istream>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <string>
#include <limits>
#include <vector>
#include <time.h>
#include <mutex>
#include <ros/ros.h>
#include <iostream>
#include <algorithm>


using namespace cv;
using namespace std;


class Parameters{
public:
	mutex mutexTest;
	Mat JIAT;
	Mat JIAT_expect;
	Mat JIAT_lane;
	Mat JIAT_screen;
	Mat JIAT_map;
	Mat JIAT_lane_recent;
	Mat JIAT_map_recent;
	Mat JIAT_combined;
  //VideoCapture screen;

	

	////////////////////////////////////////////////////////////// Hdmap
	vector<double> origin_llh;
	vector<vector<double>> waypoint_llh;
	vector<double> carPos_llh;

	Mat map;
	Mat logo;
	Mat wholeMap;
	Mat handle;
	Mat handle_gray;
	double theta_rad;
	double height_cam;
	double f;
	double down_dis;
	double alpha_rad;
	double car_length;
	int map_w, map_h;
	string textPath;
	string textPathmaprange;
	string mapPath;
	string screenPath;
	string wholeMapPath;
	ifstream fin;
	ifstream fin_map;
	
	int numwaypoint;
	Mat JIAT_copy;
	vector<double> origin_llh_rad;
	vector<double> origin_utm;
	vector<double> origin_utm_bias;
	vector<double> utm_bias_draw;
	vector<vector<double>> waypoint_distance;
	vector<vector<double>> waypointScreen;
		

	vector<double> utm_bias;
	vector<vector<double>> waypoint_llh_rad;
	vector<pair<double, double>> waypoint_utm;
	vector<vector<double>> waypoint_utm_bias;
	vector<vector<double>> waypoint_utm_bias_draw;
	vector<vector<double>> waypoint_utm_bias_origin;
	vector<vector<double>> waypoint_utm_rotate;

	vector<double> carPos_llh_rad;
	vector<double> carPos_utm;

	




	vector<double> present_car_pos;
	vector<vector<double>> present_car_pos_all;
	vector<double> utm_bias_sum;
	vector<double> steering_point_d;
	vector<double> lookahead_point;
	vector<double> lookaheadPointScreen;
	double past_steering;
	double present_car;
	double present_car_heading_rad;
	double weight;
	double lookahead_distance;

	int last_waypoint_index;
	int num;
	int screen_x;
	int screen_y;
	int lookahead_index;
	int indexInNum;
	int num_diff;
	float center_line;
	float cambias;
	float gpsbias;
	float headingbias;
	float smoothingweight;
	float Rweight;
	bool gpsonoff;
	bool plotclear;

	
	Mat histo_time;
	Mat histo_weight;
	Mat cross_error;
	vector<double> crosstrackerror;
	Mat image;
	////////////////////////////////////////////lane detect
	float rho = 1;
	float theta = CV_PI / 180;
	float trap_height = 0.5;
	float trap_bottom_width = 0.9;
	float trap_top_width = 0.07;
	bool first_lane_detect;
	bool first_lane_detect_d;
	bool first_lane_detect_b;

	int r, l, b;

	Mat img_bgr;
	Mat Rois, Rois2, Rois3;
	Mat canny;
	Mat img_copy;
	Mat histo;
	vector<float> centroid;
	Vec4i previous_1_u;
	Vec4i previous_1_m;
	Vec4i previous_1_b;
	Vec4i previous_u, previous_m, previous_b;
	vector<Vec4i> lines, lines2, lines3;
	Point points[4];
	Point points5[4];
	Point points8[4];
	Point midu, midm, midb;

	int center_0, center_1, center_2, center_3;
	float slope_center1, slope_center2;
	int centroidx, centroidy;

	float alpha;
	float beta;
	float dist;
	float x_diff;
	float y_diff;
	float h_diff;
	float w_diff;
	float lookaheadpointx;
	float lookaheadpointy;
	float Radius_lane;
	float usr_steering;
	float steering_angle_lane;
	ros::Time img_pre_time, img_cur_time;
	ros::Duration img_dur, img_delta;
	float img_time_diff;
	int lane_cnt;
	int lane_hist[50];
  //VideoCapture cap;
	string videoPath;
	float lane_margin;


	////////////////////////////////////////////gps
	vector<double> carPosGps;
	vector<double> carHeadingGps;

	/////////////////////////////////////////angleexpect
	double speed;
	double steeringAngle;

	double Radius, Length;
	int Mapx, Mapy;

	Mat expect;
	vector<vector<double>> expectedPoints;
	vector<vector<double>> expectedPoints_diff;
	vector<vector<double>> expectedPointsScreen;

	//Gps
	const std::string gps = std::string("/gps");

	//User custom parameter
	const std::string userParam = std::string("/userParam");

	//Hdmap steering 
	const std::string hdmap_steering = std::string("/hdmap_steering");
	const std::string mapsteeringerror = std::string("/mapsteeringerror");
	
	//lane detection steering
	const std::string lane_steering = std::string("/lane_steering");
	const std::string usrsteering = std::string("/usr_steering");
	const std::string flag = std::string("/flag");

	const std::string screenpoint = std::string("/screenpoint");

	//odometry
	const std::string odometry = std::string("/odometry");

};

#endif // PARAMETERS_H
