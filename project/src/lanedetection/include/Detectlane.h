#ifndef DETECTLANE_H
#define DETECTLANE_H
#include "Parameters.h"
#include <adasone_msgs/ObdParam.h>
#include <adasone_msgs/UserCustomParam.h>
#include <adasone_msgs/Steering.h>


class Detectlane {
public:
	Detectlane();
	~Detectlane();

	ros::NodeHandle nh_;
	ros::Publisher pub_lane_steering;
	ros::Publisher pub_steering;
	ros::Subscriber sub_video;
	ros::Subscriber sub_userParam;
	ros::Subscriber sub_ObdParam;
	ros::Subscriber sub_usr_steering;

	Parameters* params;
	Parameters lane;

	Vec2i upYpoint, downYpoint;
	float center_u, center_m;
	float y_point_u, y_point_m;
	float wheel_steering;


	void CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg);
  	void Callbacklane(const sensor_msgs::ImageConstPtr& msg);
	void CallbackObdParam(const adasone_msgs::ObdParam::ConstPtr& msg);
	void Callbackusrsteering(const adasone_msgs::Steering::ConstPtr& msg);

	Mat Region_Of_Interest(Mat img, Point* points);
	Mat Region_Of_Interest2(Mat img, Point* points, Point* points2);
	Vec4i draw_lane(Mat& img, vector<Vec4i> lines, Vec4i previous_1, Vec2i points, int detect_lane);
	void fit_line(const double* x, const size_t xstride, const double* y, const size_t ystride,	const size_t n,	double* c0, double* c1);
	void replaceprevious(Vec4i &up, Vec4i &down);
	void startdetect();
	void DrawResult();
	void init();

	float lanemargin(Vec4i down, int camcenterline);

	ros::Time steeringtimestamp;
	void convertImage(Mat src, Mat& dst);
	Point midPoint(Vec4i points);
	void roiImage(Mat src, Mat& dst1, Mat& dst2, Mat& dst3, Vec4i previousu, Vec4i previousm, Vec4i previousb);
	void storePrevious(Vec4i& previousu, Vec4i& previousm, Vec4i previousub, Vec4i previousmb);
	float CalculateSteering(float mid_u, float mid_m, float y_u, float y_m, int width, int height);
	int flag=0;

	void Run();
};

#endif // DETECTLANE_H
