#include <Detectlane.h>
using namespace std;
using namespace cv;


Detectlane::Detectlane() : nh_("~") {
	params = &lane;
	pub_lane_steering = nh_.advertise<adasone_msgs::UserCustomParam>(params->lane_steering.c_str(),1);
	pub_steering = nh_.advertise<adasone_msgs::Steering>(params->usrsteering.c_str(),1);
	//pub_previous_u = nh_.advertise<adasone_msgs::UserCustomParam>(params->roi_u.c_str(),1);
	//pub_previous_m = nh_.advertise<adasone_msgs::UserCustomParam>(params->roi_m.c_str(), 1);
	sub_video = nh_.subscribe("/image_raw_4", 1, &Detectlane::Callbacklane, this);
	sub_userParam = nh_.subscribe(params->userParam.c_str(), 1, &Detectlane::CallbackUserParam, this);
	sub_ObdParam = nh_.subscribe("/obd_param", 1, &Detectlane::CallbackObdParam, this);

	init();
}

Detectlane::~Detectlane(){
}

void Detectlane::init()
{
	params->first_lane_detect = false;
	params->first_lane_detect_d = false;
	params->first_lane_detect_b = false;

	int r = 0;
	int l = 0;
	int b = 0;

	float beformargin = 0;


	params->previous_1_u[0] = 408;
	params->previous_1_u[1] = 378;
	params->previous_1_u[2] = 291;
	params->previous_1_u[3] = 328;
	params->previous_1_m[0] = 504;
	params->previous_1_m[1] = 378;
	params->previous_1_m[2] = 176;
	params->previous_1_m[3] = 291;

	params->theta_rad = 87 * CV_PI/ 180;
	params->lookahead_distance = 20;
	params->height_cam = 1.5;
	params->car_length = 2.6;
	params->f = 270;
}

void Detectlane::Callbacklane(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    params->img_bgr = cv_bridge::toCvCopy(msg, "bgr8") -> image;
	flag = 1;
  }

  catch (cv_bridge::Exception& e)
  {
	  flag =0;
    ROS_ERROR("Could not convert to image!");
  }
}

void Detectlane::CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg)
{
  params->height_cam = msg->cam_height_float;
  params->f = msg->cam_focal;
  params->lookahead_distance = msg->lookahead_dist;
  params->theta_rad = msg->cam_tilt * CV_PI/180;
  params->center_line = msg->cam_centerline;
  params->numwaypoint = int(msg->num_waypoint);
  params->car_length = msg->carlength;
  params->first_lane_detect = msg->first_lane_detect;
  params->first_lane_detect_b = msg->first_lane_detect;
  params->first_lane_detect_d = msg->first_lane_detect;
}

void Detectlane::CallbackObdParam(const adasone_msgs::ObdParam::ConstPtr& msg)
{
	params->usr_steering = -1 * msg->steering_angle;
	steeringtimestamp = msg->header.stamp;
}


void Detectlane::DrawResult()
{
	line(params->img_bgr, Point(params->previous_u[0], 262), Point(params->previous_u[1], 234), Scalar(0,255, 0), 3);
	line(params->img_bgr, Point(params->previous_u[2], 262), Point(params->previous_u[3], 234), Scalar(0,255, 0), 3);
	line(params->img_bgr, Point(params->previous_m[0], 350), Point(params->previous_m[1], 262), Scalar(0,255, 0), 3);
	line(params->img_bgr, Point(params->previous_m[2], 350), Point(params->previous_m[3], 263), Scalar(0,255, 0), 3);
	line(params->img_bgr, Point(params->img_bgr.cols/2, 360), Point(params->img_bgr.cols/2, 0), Scalar(0, 0, 255), 1);
	circle(params->img_bgr, Point(params->lookaheadpointx, params->lookaheadpointy), 2, Scalar(0,0,255), 2);
}


Vec4i Detectlane::draw_lane(Mat& img, vector<Vec4i> lines, Vec4i previous, Vec2i points, int detect_lane)
{
	bool draw_right = true;
	bool draw_left = true;
	int width = img.cols;
	int height = img.rows;
	float slope_threshold = 0.5;
	vector<float> slopes;
	vector<Vec4i> new_lines;
	int pre_rx1, pre_rx2, pre_lx1, pre_lx2;
	pre_rx1 = previous[0];
	pre_rx2 = previous[1];
	pre_lx1 = previous[2];
	pre_lx2 = previous[3];
	Vec4i pr;
	pr[0] = pre_rx1;
	pr[1] = pre_rx2;
	pr[2] = pre_lx1;
	pr[3] = pre_lx2;

	for (int i = 0; i < lines.size(); i++)
	{
		Vec4i line = lines[i];
		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];
		float slope;

		if (x2 - x1 == 0)
			slope = 999.0;
		else
			slope = (y2 - y1) / (float)(x2 - x1);

		if (abs(slope) > slope_threshold)
		{
			slopes.push_back(slope);
			new_lines.push_back(line);
		}
	}
	vector<Vec4i> right_lines;
	vector<Vec4i> left_lines;

	for (int i = 0; i < new_lines.size(); i++)
	{
		Vec4i line = new_lines[i];
		float slope = slopes[i];

		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		float cx = width * 0.5;

		if (slope > 0)
			right_lines.push_back(line);
		else if (slope < 0)
			left_lines.push_back(line);
	}

	double right_lines_x[100];
	double right_lines_y[100];


	float right_m, right_b;

	int right_index = 0;
	for (int i = 0; i < right_lines.size(); i++)
	{
		Vec4i line = right_lines[i];
		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		right_lines_x[right_index] = x1;
		right_lines_y[right_index] = y1;
		right_index++;
		right_lines_x[right_index] = x2;
		right_lines_y[right_index] = y2;
		right_index++;

	}
	if (right_index > 0) {
		double c0, c1;
		fit_line(right_lines_x, 1, right_lines_y, 1, right_index, &c0, &c1);

		right_m = c1;
		right_b = c0;
		draw_right = true;
	}
	else {
		right_m = right_b = 1;
		draw_right = false;
	}


	double left_lines_x[100];
	double left_lines_y[100];
	float left_m, left_b;

	int left_index = 0;
	for (int i = 0; i < left_lines.size(); i++)
	{
		Vec4i line = left_lines[i];
		int x1 = line[0];
		int y1 = line[1];
		int x2 = line[2];
		int y2 = line[3];

		left_lines_x[left_index] = x1;
		left_lines_y[left_index] = y1;
		left_index++;
		left_lines_x[left_index] = x2;
		left_lines_y[left_index] = y2;
		left_index++;
	}

	if (left_index > 0) {
		double c0, c1;
		fit_line(left_lines_x, 1, left_lines_y, 1, left_index, &c0, &c1);

		left_m = c1;
		left_b = c0;
		draw_left = true;
	}
	else {
		left_m = left_b = 1;
		draw_left = false;

	}

	int y1 = points[0];
	int y2 = points[1];

	float right_x1 = (y1 - right_b) / right_m;
	float right_x2 = (y2 - right_b) / right_m;

	float left_x1 = (y1 - left_b) / left_m;
	float left_x2 = (y2 - left_b) / left_m;

	y1 = int(y1);
	y2 = int(y2);

	right_x1 = int(right_x1);
	right_x2 = int(right_x2);
	left_x1 = int(left_x1);
	left_x2 = int(left_x2);

	pr[0] = pre_rx1;
	pr[1] = pre_rx2;
	pr[2] = pre_lx1;
	pr[3] = pre_lx2;

	if (draw_right) {
		pr[0] = int(right_x1);
		pr[1] = int(right_x2);
	}
	if (draw_left) {
		pr[2] = int(left_x1);
		pr[3] = int(left_x2);
	}
	line(img, Point(pr[0], y1), Point(pr[1], y2), Scalar(0, 0, 255), 2);
	line(img, Point(pr[2], y1), Point(pr[3], y2), Scalar(0, 0, 255), 2);

	return pr;
}

void Detectlane::fit_line(const double* x, const size_t xstride,
	const double* y, const size_t ystride,
	const size_t n,
	double* c0, double* c1)
{
	double m_x = 0, m_y = 0, m_dx2 = 0, m_dxdy = 0;

	size_t i;

	for (i = 0; i < n; i++)
	{
		m_x += (x[i * xstride] - m_x) / (i + 1.0);
		m_y += (y[i * ystride] - m_y) / (i + 1.0);
	}

	for (i = 0; i < n; i++)
	{
		const double dx = x[i * xstride] - m_x;
		const double dy = y[i * ystride] - m_y;

		m_dx2 += (dx * dx - m_dx2) / (i + 1.0);
		m_dxdy += (dx * dy - m_dxdy) / (i + 1.0);
	}
	double s2 = 0, d2 = 0;
	double b = m_dxdy / m_dx2;
	double a = m_y - m_x * b;

	*c0 = a;
	*c1 = b;
}

Mat Detectlane::Region_Of_Interest(Mat img, Point * points)
{
	Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* ppt[1] = { points };
	int npt[] = { 4 };

	fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);

	Mat img_masked;
	bitwise_and(img, img_mask, img_masked);

	return img_masked;
}

Mat Detectlane::Region_Of_Interest2(Mat img, Point* points, Point* points2)
{
	Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC1);

	Scalar ignore_mask_color = Scalar(255, 255, 255);
	const Point* ppt[1] = { points };
	const Point* ppt2[1] = { points2 };
	int npt[] = { 4 };

	fillPoly(img_mask, ppt, npt, 1, Scalar(255, 255, 255), LINE_8);
	fillPoly(img_mask, ppt2, npt, 1, Scalar(255, 255, 255), LINE_8);

	Mat img_masked;
	bitwise_and(img, img_mask, img_masked);


	return img_masked;
}

void Detectlane::convertImage(Mat src, Mat& dst)
{
	cvtColor(src, dst, COLOR_BGR2GRAY);
	GaussianBlur(dst, dst, Size(3, 3), 0, 2);
	Canny(dst, dst, 100, 200, 3);
}

Point Detectlane::midPoint(Vec4i previous)
{
	Point ret;
	ret.x = (previous[0] + previous[1]) / 2;
	ret.y = (previous[2] + previous[3]) / 2;
	return ret;
}

void Detectlane::roiImage(Mat src, Mat& dst1, Mat& dst2, Mat& dst3, Vec4i previousu, Vec4i previousm, Vec4i previousb)
{
	if (!(params->first_lane_detect))
	{
		Point points1[4];
		points1[0] = Point(408, 262);
		points1[1] = Point(291, 262);
		points1[2] = Point(328, 234);
		points1[3] = Point(378, 234);
		dst1 = Region_Of_Interest(src, points1);
		params->l++;
	}
	if (params->first_lane_detect) {
		Point points1[4], points2[4];

		points1[0] = Point(previousu[2] + 15, 262);
		points1[1] = Point(previousu[2] - 15, 262);
		points1[2] = Point(previousu[3] - 15, 234);
		points1[3] = Point(previousu[3] + 15, 234);

		points2[0] = Point(previousu[0] + 15, 262);
		points2[1] = Point(previousu[0] - 15, 262);
		points2[2] = Point(previousu[1] - 15, 234);
		points2[3] = Point(previousu[1] + 15, 234);
		dst1 = Region_Of_Interest2(src, points1, points2);
	}
	// ROI for upper bound
	if (!params->first_lane_detect_d)
	{
		Point points3[4];
		points3[0] = Point(504, 350);
		points3[1] = Point(176, 350);
		points3[2] = Point(291, 262);
		points3[3] = Point(408, 262);
		dst2 = Region_Of_Interest(src, points3);
		params->r++;
	}
	if (params->first_lane_detect_d) {
		Point points3[4], points4[4];

		points3[0] = Point(previousm[2] + 20, 350);
		points3[1] = Point(previousm[2] - 20, 350);
		points3[2] = Point(previousm[3] - 20, 262);
		points3[3] = Point(previousm[3] + 20, 262);

		points4[0] = Point(previousm[0] + 20, 350);
		points4[1] = Point(previousm[0] - 20, 350);
		points4[2] = Point(previousm[1] - 20, 262);
		points4[3] = Point(previousm[1] + 20, 262);

		dst2 = Region_Of_Interest2(src, points3, points4);
	}
	// ROI for lower bound
}


void Detectlane::storePrevious(Vec4i& previousu, Vec4i& previousm, Vec4i previousub, Vec4i previousmb)
{
	previousu = previousub;
	previousm = previousmb;

}

void Detectlane::replaceprevious(Vec4i& up, Vec4i& down)
{
	int vanx = 358, vany = 215;
	int leftmem, rightmem;
	float up_left, up_right, down_left, down_right;

	up_left = ((vany - 262) * (up[2] - up[3])) / 28 + up[2];
	up_right = ((vany - 262) * (up[1] - up[0])) / 28 + up[0];

	down_left = ((vany - 350) * (down[2] - down[3])) / 28 + down[2];
	down_right = ((vany - 350) * (down[1] - down[0])) / 28 + down[0];


	bool joint_left = true, joint_right = true;

	if(up[2] != down[3])
	{
		joint_left = false;
	}
	if(up[0] != down[1])
	{
		joint_right = false;
	}

	if(abs(down_left - vanx) > abs(up_left - vanx)) leftmem = 0;
	else leftmem = 1;
	
	if(abs(down_right - vanx) > abs(up_right - vanx)) rightmem = 0;
	else rightmem = 1;

	if(!joint_left)
	{
		if(!leftmem) down[3] = up[2];
		else up[2] = down[3];
	}

	if(!joint_right)
	{
		if(!rightmem) down[1] = up[0];
		else up[0] = down[1];
	}
}

float Detectlane::CalculateSteering(float mid_u, float mid_m, float y_u, float y_m, int width, int height)
{
	float steering;
	float slope = (y_u - y_m) / (mid_u - mid_m);
	if(slope > 999) steering = 0;
	else
	{
		params->alpha = atan2(params->lookahead_distance , params->height_cam) - params->theta_rad;
		params->lookaheadpointy = ( height / 2 ) - params->f * tan(params->alpha);
		params->x_diff = (params->lookaheadpointy - y_m) / slope + mid_m - width;
		params->y_diff = params->f * tan(params->alpha);
		params->beta = atan2(params->x_diff , params->f);
		params->lookaheadpointx = params->x_diff + width;
		params->w_diff = tan(params->beta) * sqrt(params->height_cam * params->height_cam + params->lookahead_distance * params->lookahead_distance );
		params->h_diff = params->height_cam * tan(params->theta_rad + params->alpha);

		params->Radius_lane = (params->h_diff *params-> h_diff) / (2*params->w_diff) + params->w_diff / 2;
		if (params->Radius_lane < 0)
		{
			steering =  -1 * atan2(params->car_length, 2*abs(params->Radius_lane));
		}
		else
		{	
			steering = atan2(params->car_length, 2*params->Radius_lane);
		}
	}	
	return steering;
}

float Detectlane::lanemargin(Vec4i down, int camcenterline )
{
	float margin;
	float pixellength;
	int left_bot, right_bot;


	left_bot = down[2];
	right_bot = down[0];

	pixellength = 3.5 / (right_bot - left_bot);


	margin = pixellength * (camcenterline - ((left_bot + right_bot) / 2));


	return margin;
}

void Detectlane::startdetect()
{
	if(flag)
	{
		int width = params->img_bgr.cols;
		int height = params->img_bgr.rows;
		convertImage(params->img_bgr, params->canny);
		roiImage(params->canny, params->Rois, params->Rois2, params->Rois3, params->previous_1_u, params->previous_1_m, params->previous_1_b);

		if (params->l > 20) {
			params->first_lane_detect = true;
			params->l = 20;
		}
		
		if (params->r > 20)
		{
			params->first_lane_detect_d = true;
			params->r = 20;
		}

		if (params->b > 20) {
			params->first_lane_detect_b = true;
			params->b = 20;
		}
		
		upYpoint = {262, 234};
		downYpoint = {350, 262};

		HoughLinesP(params->Rois, params->lines, params->rho, params->theta, 20, 10, 10);
		HoughLinesP(params->Rois2, params->lines2, params->rho, params->theta, 30, 10, 10);
		// find staight lines in the ROI

		params->previous_u = draw_lane(params->img_copy, params->lines, params->previous_1_u, upYpoint, params->r);
		params->previous_m = draw_lane(params->img_copy, params->lines2, params->previous_1_m, downYpoint, params->l);
		// choose the fit line by regulating the candidate
		
		replaceprevious(params->previous_u, params->previous_m);

		storePrevious(params->previous_1_u, params->previous_1_m, params->previous_u, params->previous_m);


		params->midu = midPoint(params->previous_u);
		params->midm = midPoint(params->previous_m);

		center_u = (params->midu.x + params->midu.y)/2;
		center_m = (params->midm.x + params->midm.y)/2;

		y_point_u = 248;
		y_point_m = 306;
		
		wheel_steering = CalculateSteering(center_u, center_m, y_point_u, y_point_m, params->center_line, height);
		wheel_steering = wheel_steering * 15.4;
		adasone_msgs::Steering steering_msg;
		adasone_msgs::UserCustomParam msg;
		
		params->steering_angle_lane = wheel_steering;
		params->steering_angle_lane = params->steering_angle_lane * 180 / CV_PI;

		params->lane_margin = lanemargin(params->previous_m, params->center_line);

		msg.lanemargin = params->lane_margin;
		
		for(int i = 0; i < 4; i++)
		{
			msg.previous_u[i] = params->previous_u[i];
		}

		for(int i = 0; i < 4; i++)
		{
			msg.previous_m[i] = params->previous_m[i];
		}
		// publish the 4_x_coordinate 

		msg.circle_point[0] = params->lookaheadpointx;
		msg.circle_point[1] = params->lookaheadpointy;
		msg.header.stamp = steeringtimestamp;
		pub_lane_steering.publish(msg);
		
		steering_msg.header.stamp = steeringtimestamp;
		steering_msg.usr_steering = params->usr_steering;
		steering_msg.lane_steering = params->steering_angle_lane;
		pub_steering.publish(steering_msg);

		// publish calculated steering angle with lane detection
	}
}

void Detectlane::Run() {
	ros::Rate loop_rate(20);
	
	while (ros::ok()) {
		ros::spinOnce();
		startdetect();
		loop_rate.sleep();	
	}

}




