//#pragma once
#include <Hdmap.h>
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <algorithm>
#include <stack>
using namespace std;

/*
 * not to confuse topic name, topic names are in 
 * the Parameters.h file.
 * c_str() means the pointer of string in Parameter.h 
*/
Hdmap::Hdmap() : nh_("~") {
  params = &param;
  pub_map_steering = nh_.advertise<adasone_msgs::Steering>(params->hdmap_steering.c_str(),1);
  pub_flag = nh_.advertise<adasone_msgs::Flag>(params->flag.c_str(),1);
  pub_map_screenpoint = nh_.advertise<adasone_msgs::Point>(params->screenpoint.c_str(),1);
  sub_obd_data = nh_.subscribe("/obd_param", 1, &Hdmap::CallbackObdParam, this);
  sub_userParam = nh_.subscribe(params->userParam.c_str(), 1, &Hdmap::CallbackUserParam, this);
  sub_margin = nh_.subscribe("/lane_steering", 1, &Hdmap::CallbackMargin, this);
  init();		
}

Hdmap::~Hdmap(){
	delete[] lane;
	delete[] maprange_;
}


void Hdmap::readFromFile()
{
	readcomplete = 0;
	///////////////////////////////////////get lane struct from file////////////////////////
	//
	int cntwaypoint = 0, temp;
	double x,y;
	int index;
	
	params->fin >> lanestructnum;
	while (!(params->fin.eof())){
		params->fin >> index;
		params->fin >> lane[cntwaypoint].id;
		params->fin >> lane[cntwaypoint].fromnode;
		params->fin >> lane[cntwaypoint].tonode;
		params->fin >> lane[cntwaypoint].rightlaneindex;
		params->fin >> lane[cntwaypoint].leftlaneindex;
		params->fin >> lane[cntwaypoint].coordinateNum;
		params->fin >> lane[cntwaypoint].lanenum;
		params->fin >> lane[cntwaypoint].linkedNodeNum;

		for(int i=0;i<lane[cntwaypoint].linkedNodeNum;i++)
		{
			params->fin >> temp;
			lane[cntwaypoint].linkedNode.push_back(temp);
		}

		for(int i=0;i<lane[cntwaypoint].coordinateNum;i++)
		{
			params->fin >> x;
			params->fin >> y;
			lane[cntwaypoint].coordinate.push_back({ x,y });
		}
		cntwaypoint += 1; // total number of lane structure
	}


	//////////////////////////////////get splited map from txt file//////////////////////////////////// 
	int cntmaprange = 0;
	
	params->fin_map >> diff_x >> diff_y; //splited map x&y length
	while (!(params->fin_map.eof())){
		params->fin_map >> x >> y >> temp; // leftup corner coordinate(x,y), num_struct_index(temp)

		maprange_[cntmaprange].struct_num = temp;
		maprange_[cntmaprange].array.first = x;
		maprange_[cntmaprange].array.second = y;

		for(int i=0;i<temp;i++)
		{
			params->fin_map >> index;
			maprange_[cntmaprange].structIndex.push_back(index);
		}
		cntmaprange += 1; //total number of map structure
	}
	maprangestructnum = cntmaprange - 1;

	readcomplete = 1;// flag, if lane info & map info were read
}

void Hdmap::init()
{
	params->textPath = "/home/hong/simulator/project/data/final_new.txt";
	params->fin.open(params->textPath);
	if (!params->fin.is_open())
		cout << "cannot open utm file" << endl;
	
	params->textPathmaprange = "/home/hong/simulator/project/data/net.txt";
	params->fin_map.open(params->textPathmaprange);
	if (!params->fin_map.is_open())
		cout << "cannot open maprange file" << endl;
	
	
	lane = new lane1[900000];
	maprange_ = new maprange[10000];
	params->JIAT_screen = cv::imread("/home/hong/simulator/project/data/seoulbusan.JPG");
	cv::resize(params->JIAT_screen, params->JIAT, Size(585,720));

	readFromFile();
	f = 270;
	thetaRad = 87*PI/180;
	centerLine = 360;
	camBias = 0;
	screenX = 720;
	screenY = 480;
	isgpson = false;
	ispathfound = false;
	usergpson = false;
	distTotal = 30.0;
	extractDist = 100.0;
	obd_first_input = false;
	margin = 0;
	carLength=2.6;
	lookAheadDist = 20;
	params->past_steering = 0;
	params->car_length = 2.6;
	params->lookahead_distance = 20;
	params->theta_rad = 87 * PI / 180;
	params->height_cam = 1.5;
	params->down_dis = 0.5;
	params->alpha_rad = 0;
	params->last_waypoint_index = 0;
	params->map_w = 500;
	params->map_h = 500;
	params->screen_x = 720;
	params->screen_y = 480;
	params->center_line = 360;
	params->num = 2000;
	params->f = 270;
	params->gpsbias = 0.35;
	params->headingbias = 0.027;
}

void Hdmap::CallbackMargin(const adasone_msgs::UserCustomParam::ConstPtr& msg)
{
	margin = msg->lanemargin;
}

void Hdmap::CallbackObdParam(const adasone_msgs::ObdParam::ConstPtr& msg)
{
	if(!obd_first_input) {
		obd_pre_time = msg->header.stamp;
	}
	else {
		obd_pre_time = obd_cur_time;
	}
	obd_speed = msg->float_speed / 3.6;
	obd_cur_time = msg->header.stamp;
	obd_time_diff = obd_cur_time.toSec() - obd_pre_time.toSec();
	obd_first_input = true;
}

void Hdmap::callback(const sensor_msgs::NavSatFix::ConstPtr& gps_, const geometry_msgs::QuaternionStamped::ConstPtr& heading_)
{
	timestamp_temp = gps_->header.stamp;
	timestamp= timestamp_temp.toSec();
	if(userparamsetting)
	{
		tf::Quaternion q(
			heading_->quaternion.x,
			heading_->quaternion.y,
			heading_->quaternion.z,
			heading_->quaternion.w
		);

		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw, 0);
	
		if(gps_->status.status != 2) isgpson = false;
		else isgpson = true;

		gps_lon = gps_->latitude;
		gps_lat = gps_->longitude;
		gpstimestamp = gps_->header.stamp;
		carpos = {gps_lat, gps_lon};
		WGS2UTM(carpos, carpos, 1, band, zone);	
		
		heading = yaw + params->headingbias;

		carpos.first -= params->gpsbias*cos(heading);
		carpos.second += params->gpsbias*sin(heading);
	}
}

////////// get user parameters from gui ////////////////////
void Hdmap::CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg)
{
  camHeight = msg->cam_height_float;
  f = msg->cam_focal;
  lookAheadDist = msg->lookahead_dist;
  thetaRad = msg->cam_tilt * PI/180;
  centerLine = msg->cam_centerline;
  camBias = msg->cambias;
  params->gpsbias = msg->gpsbias;
  screenX = int(msg->imagewidth);
  screenY = int(msg->imageheight);
  carLength = msg->carlength;
  usergpson = msg->gpsonoff;
  params->headingbias = msg->headingbias;
  params->smoothingweight = msg->smoothingweight;
  params->Rweight = msg->Rweight;
  params->plotclear = msg->plotclear;
  userparamsetting = true;
}


double Hdmap::calculateDist(pair<double, double> pre, pair<double, double> next)
{
	double xDiff = pre.first - next.first;
	double yDiff = pre.second - next.second;
	return  sqrt(xDiff * xDiff + yDiff * yDiff);
}

bool Hdmap::firstLocalization(struct maprange* map_, struct lane1* lane_, int mapSize, int mapX, int mapY, pair<double, double> gpsUTM, int& nodeIndex, int& waypointIndex, double& minError)
{
	int index = -1;
	for (int i = 0; i < mapSize; i++) {
		if (map_[i].array.first <= gpsUTM.first && gpsUTM.first <= map_[i].array.first+mapX) {
			if (map_[i].array.second <= gpsUTM.second && gpsUTM.second <= map_[i].array.second + mapY) {
				index = i;
				break;
			}
		}
	}
	minError = 10000000000;
	double error = -1;
	int laneIndex = -1;
	for (int i = 0; i < map_[index].structIndex.size(); i++) {
		laneIndex = map_[index].structIndex[i];
		for (int j = 0; j < lane_[laneIndex].coordinateNum-1; j++) {
			pair<double, double> prewaypoint, nextwaypoint;
			prewaypoint = lane_[laneIndex].coordinate[j];
			nextwaypoint = lane_[laneIndex].coordinate[j+1];
			if (!crosstrackError(prewaypoint, nextwaypoint, gpsUTM, error)) return false;
			if (minError > abs(error)) {
				minError = abs(error);
				nodeIndex = laneIndex;
				waypointIndex = j;
			}
		}
	}
	return true;
}

void Hdmap::makeCourse(vector<int>& parentNode, vector<int>& course, int nodeIndex, int startNodeIndex)
{
	if (nodeIndex == startNodeIndex) {
		course.push_back(startNodeIndex);
		return;
	}
	makeCourse(parentNode, course, parentNode[nodeIndex], startNodeIndex);
	course.push_back(nodeIndex);
	return;
}

bool Hdmap::findCourse(lane1* lane_, vector<int>& course, int laneSize, int startNodeIndex, int endNodeIndex)
{
	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
	pq.push({ 0, startNodeIndex });
	vector<int> parentNode(laneSize, -1);
	vector<int> weight(laneSize, 1000000000);
	while (!pq.empty()) {
		int preWeight = pq.top().first;
		int preNode = pq.top().second;
		pq.pop();
		
		int preNumWaypoint = lane_[preNode].coordinateNum;
		int lLink = lane_[preNode].leftlaneindex;
		if( lLink != -1){
			int nextLeftNumWaypoint = lane_[lLink].coordinateNum;
			double leftLaneDist = calculateDist(lane_[preNode].coordinate[preNumWaypoint-1], lane_[lLink].coordinate[nextLeftNumWaypoint-1]);
			if (weight[lane_[preNode].leftlaneindex] > preWeight + 10 && leftLaneDist < 5) {
				weight[lane_[preNode].leftlaneindex] = preWeight + 10;
				parentNode[lane_[preNode].leftlaneindex] = preNode;
				pq.push({ preWeight + 10, lane_[preNode].leftlaneindex });
			}
		}
		
		int rLink = lane_[preNode].rightlaneindex;
		if( rLink != -1 ){
			int nextRightNumWaypoint = lane_[rLink].coordinateNum;
			double rightLaneDist = calculateDist(lane_[preNode].coordinate[preNumWaypoint - 1], lane_[rLink].coordinate[nextRightNumWaypoint - 1]);
			if (weight[lane_[preNode].rightlaneindex] > preWeight + 10 && rightLaneDist < 5) {
				weight[lane_[preNode].rightlaneindex] = preWeight + 10;
				parentNode[lane_[preNode].rightlaneindex] = preNode;
				pq.push({ preWeight + 10, lane_[preNode].rightlaneindex });
			}
		}

		for (int i = 0; i < lane_[preNode].linkedNodeNum; i++) {
			int nextWeight = 1;
			int nextNode = lane_[preNode].linkedNode[i];
			int weightSum = preWeight + nextWeight;
			if (weight[nextNode] > weightSum) {
				weight[nextNode] = weightSum;
				parentNode[nextNode] = preNode;
				pq.push({ weightSum, nextNode });
			}
		}
		
		if (parentNode[endNodeIndex] != -1) {
			makeCourse(parentNode, course, endNodeIndex, startNodeIndex);
			return true;
		}
	}
	return false;
}
void Hdmap::drawRoute(Mat& picture, vector<pair<double,double>>& waypoint,pair<double, double> bias){
	for (int i = 0; i < waypoint.size(); i++) {
		double x = waypoint[i].first - bias.first;
		double y = waypoint[i].second - bias.second;
		cv::circle(picture, Point(int(0.0027 * x), 720-int(0.0027 * y)), 2, Scalar(0, 255, 0), 1, 8, 0);
	}
}

bool Hdmap::extrackWaypoint(struct lane1* lane_, int preWaypoint, int preNodeIndex, vector<int>& course,vector<pair<double,double>>& extractedWaypoint, pair<double,double> carPos, double distRange)
{
	int preNodeIndexTemp = preNodeIndex;
	int preNode = course[preNodeIndexTemp];
	int indexTemp = preWaypoint;
	stack<pair<double, double>> st;
	while (preNodeIndexTemp >= 0 && preNodeIndexTemp < course.size()) {
		if (calculateDist(carPos, lane_[preNode].coordinate[indexTemp]) < distRange) {
			st.push(lane_[preNode].coordinate[indexTemp]);
			if (indexTemp > 0) {
				indexTemp--;
			}
			else if (preNodeIndexTemp > 0) {
				preNode = course[--preNodeIndexTemp];
				indexTemp = lane_[preNode].coordinateNum - 1;
			}
			else break;
		}
		else break;
	}
	while (!st.empty()) {
		extractedWaypoint.push_back(st.top());
		st.pop();
	}

	preNodeIndexTemp = preNodeIndex;
	preNode = course[preNodeIndexTemp];
	indexTemp = preWaypoint+1;
	
	if (indexTemp >= lane_[preNode].coordinateNum) {
		if (preNodeIndexTemp >= course.size() - 1) return false; 
		else {
			preNode = course[++preNodeIndexTemp];
		}
	}

	while (preNodeIndexTemp >= 0 && preNodeIndexTemp < course.size()) {
		if (calculateDist(carPos, lane_[preNode].coordinate[indexTemp]) < distRange) {
			extractedWaypoint.push_back(lane_[preNode].coordinate[indexTemp]);
			if (indexTemp +1 < lane_[preNode].coordinateNum) {
				indexTemp++;
			}
			else if (preNodeIndexTemp+1 < course.size() ) {
				preNodeIndexTemp++;
				preNode = course[preNodeIndexTemp];
				indexTemp = 0;
			}
			else break;
		}
		else break;
	}
	return true;
}

bool Hdmap::transformWaypoint(vector<pair<double, double>>& extractedWaypoint, double carHeading, vector<pair<double, double>>& movedWaypoint, vector<pair<double, double>>& rotatedWaypoint, pair<double, double> currentPos)
{
	double xBias, yBias;
	pair<double, double> rotate;
	pair<double, double> origin;
	xBias = currentPos.first;
	yBias = currentPos.second;
	for (int i = 0; i < extractedWaypoint.size(); i++) {
		origin.first = extractedWaypoint[i].first - xBias;
		origin.second = extractedWaypoint[i].second - yBias;
		movedWaypoint[i]={origin};

		rotateWaypoint(movedWaypoint[i], rotate, carHeading);
		rotatedWaypoint[i]={rotate};
	}
	return true;
}

bool Hdmap::calculateSteering(vector<pair<double, double>> rotatedWaypoint, double lookAheadDist, double& pastSteering, double carLength, pair<double,double>& lookAheadPoint)
{
	lookAheadPoint.second = lookAheadDist;
	for (int i = 1; i < rotatedWaypoint.size(); i++) {

		if (lookAheadDist < rotatedWaypoint[i].second) {
			double px = rotatedWaypoint[i-1].first;
			double py = rotatedWaypoint[i-1].second;
			double nx = rotatedWaypoint[i].first;
			double ny = rotatedWaypoint[i].second;
			if (px == nx) {
				lookAheadPoint.first = px;
			}
			else {
				double slope = (py - ny) / (px - nx);
				if (slope == 0) slope += 0.00001;
				double constant = ny- (slope)*nx;
				lookAheadPoint.first = (lookAheadPoint.second - constant) / slope;
			}
			break;
		}
	}
	if (lookAheadPoint.first == 0) {
		pastSteering = 0;
		return true;
	}
	else {
		double slope = (lookAheadPoint.second) / (lookAheadPoint.first);
		double constant = lookAheadPoint.second - (slope)*lookAheadPoint.first;
		double R = slope * lookAheadPoint.second / 2 + lookAheadPoint.first / 2;

		pastSteering = atan(carLength / (R * 2)) * 180 / PI;
	}
	return false;
}

double Hdmap::Calculate_alpha(double y_diff, double theta_rad, double height) {
	double alpha_rad;
	alpha_rad = atan(y_diff / height) - theta_rad;
	return alpha_rad;
}

double Hdmap::Calculate_beta(double theta_rad, double alpha_rad, double height, double x_diff) {
	double temp = height / (cos(theta_rad + alpha_rad));
	double beta;
	beta = atan(x_diff / temp);
	return beta;
}

double Hdmap::Calculate_screen_y(double f, double alpha_rad) {
	double screen_y = f * tan(alpha_rad);
	return screen_y;
}

double Hdmap::Calculate_screen_x(double f, double alpha_rad, double beta_rad) {
	double screen_x = f * tan(beta_rad) / cos(alpha_rad);

	return screen_x;
}
void Hdmap::waypoint2screenpoint(vector<pair<double, double>>& rotatedWaypoint, vector<pair<double,double>>& screenWaypoint,double thetaRad, double camHeight, double f, int screenX, int screenY, double centerLine, double camBias) {

	double alpha_rad, y, beta_rad, x;

	for (int i = 0; i < rotatedWaypoint.size(); i++) {
		alpha_rad = Calculate_alpha(rotatedWaypoint[i].second, thetaRad, camHeight);
		y = Calculate_screen_y(f, alpha_rad) + (screenY/2);

		beta_rad = Calculate_beta(thetaRad, alpha_rad, camHeight, rotatedWaypoint[i].first);
		x = Calculate_screen_x(f, alpha_rad, beta_rad) + (screenX/2 + camBias);
		
		if( y >= 0)
		{
			screenWaypoint.push_back({x, y});
		}
	}
}

void Hdmap::WGS2UTM(pair<double,double> llh, pair<double,double>& utm, int num, const char band, int zone) {
		double latitude = llh.second;
		double longitude = llh.first;
		double a = WGS84_A;
		double eccSquared = UTM_E2;
		double k0 = UTM_K0;
		double LongOrigin;
		double eccPrimeSquared;
		double N, T, C, A, M;
		double x, y;

		double LongTemp = (longitude + 180) - int((longitude + 180) / 360) * 360 - 180;
		double latitude_rad = latitude * PI / 180;
		double longitude_rad = LongTemp * PI / 180;
		double LongOriginRad;

		LongOrigin = (zone - 1) * 6 - 180 + 3;
		LongOriginRad = LongOrigin * PI / 180;

		eccPrimeSquared = (eccSquared) / (1 - eccSquared);

		N = a / sqrt(1 - eccSquared * sin(latitude_rad) * sin(latitude_rad));
		T = tan(latitude_rad) * tan(latitude_rad);
		C = eccPrimeSquared * cos(latitude_rad) * cos(latitude_rad);
		A = cos(latitude_rad) * (longitude_rad - LongOriginRad);

		M = a * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256) * latitude_rad - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(2 * latitude_rad) + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * latitude_rad) - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * latitude_rad));

		x = (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0);
		y = (double)(k0 * (M + N * tan(latitude_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));
		
		utm={ x, y };

}

bool Hdmap::crosstrackError(pair<double, double> prewaypoint, pair<double, double> nextwaypoint, pair<double, double> currentPos, double& error)
{
	double dist1 = -1, dist2 = -1;
	double p_x, p_y, n_x, n_y, o_x, o_y;
	p_x = prewaypoint.first;
	p_y = prewaypoint.second;
	n_x = nextwaypoint.first;
	n_y = nextwaypoint.second;
	o_x = currentPos.first;
	o_y = currentPos.second;
	dist1 = pow(p_x -o_x, 2) + pow(p_y - o_y, 2);
	dist2 = pow(n_x -o_x, 2) + pow(n_y - o_y, 2);
	pair<double, double> a, b;
	if (dist1 > dist2) {
		a = { p_x - n_x, p_y - n_y };
		b = {o_x - n_x, o_y - n_y };
	}
	else {
		a = { n_x - p_x, n_y - p_y };
		b = {o_x - p_x, o_y - p_y };
	}
	vector<pair<double, double>> waypointTemp;
	double headingTemp = atan2((n_x - p_x), (n_y - p_y));
	waypointTemp.push_back({o_x,o_y});
	waypointTemp.push_back({n_x,n_y});
	transformWaypoint(waypointTemp, headingTemp, waypointTemp , waypointTemp ,currentPos);
	int sign = 1;
	waypointTemp[0].first < waypointTemp[1].first ? sign = -1 : sign = 1;
	double slope, constant;
	if ((a.first * b.first + a.second * b.second) < 0) {
		error = sqrt(min(dist1, dist2))*sign;
		return true;
	}
	else {
		if (n_x - p_x == 0) error = abs(p_y - o_y);
		else {
			slope = (n_y - p_y) / (n_x - p_x);
			constant = n_y - slope * n_x;
			error = abs((-1 * slope * o_x + o_y - constant) / sqrt(slope * slope + 1) )*sign;
		}
		return true;
	}
	return false;
}

bool Hdmap::localizationDR(lane1* lane_, double dist, int& preWaypoint, int& preNodeIndex, vector<int>& course, pair<double, double>& carPosDR, double margin, double& error)
{
	int preNode = course[preNodeIndex];
	double distTemp = 0, preX = carPosDR.first, preY = carPosDR.second, nextX = 0, nextY=0;
	double xdiff=0, ydiff=0;
	int cnt=0;
	while (preNodeIndex >=0 && preNodeIndex < course.size() ) {
		preNode = course[preNodeIndex];
		nextX = lane_[preNode].coordinate[preWaypoint].first;
		nextY = lane_[preNode].coordinate[preWaypoint].second;
		double xdiff = nextX - preX;
		double ydiff = nextY -preY;
		distTemp = sqrt( xdiff*xdiff + ydiff*ydiff );
		distTemp = cnt==0?sqrt(distTemp*distTemp-error*error):distTemp;
		cnt++;
		dist -= distTemp;
		if (dist < 0){
			double ratio = -1 * dist / distTemp;
			carPosDR = {preX + xdiff * ratio,  preY + ydiff * ratio};
			double heading = atan2((nextX - preX), (nextY - preY));
			rotateWaypoint(carPosDR, carPosDR, heading);
			carPosDR.first += margin;
			rotateWaypoint(carPosDR, carPosDR, -1 * heading);
			crosstrackError({preX, preY}, {nextX, nextY}, carPosDR, error);
			break;	
		} 

		preX = lane_[preNode].coordinate[preWaypoint].first;
		preY = lane_[preNode].coordinate[preWaypoint].second;

		preWaypoint++;
		if (preWaypoint >= lane_[preNode].coordinateNum) {
			preNodeIndex++;
			preWaypoint = 0;
		}
	}
	
	return true;
}

bool Hdmap::localization(lane1* lane_, int& preWaypoint, int& preNodeIndex, vector<int>& course, pair<double, double>& carPos, double& error, double dist){
	double minError=10000000000;
	double errorTemp;
	double distTemp;
	pair<double, double> prePos = lane_[course[preNodeIndex]].coordinate[preWaypoint];
	int nodeIndexTemp = preNodeIndex;
	int waypointTemp = preWaypoint;
	int nodeIndexTemp1 = preNodeIndex;
	int waypointTemp1 = preWaypoint;
	while (nodeIndexTemp >= 0 && nodeIndexTemp < course.size()) {
		if( waypointTemp+1 >= lane_[course[nodeIndexTemp]].coordinateNum ){
			if( nodeIndexTemp+1 < course.size() ){
				nodeIndexTemp++;
				waypointTemp = 0;
			}
			else{
				cout << "no waypoint left" << endl;
				return false;
			} 
		}
		else waypointTemp++;

		crosstrackError(lane_[course[nodeIndexTemp1]].coordinate[waypointTemp1], lane_[course[nodeIndexTemp]].coordinate[waypointTemp], carPos, errorTemp);
		if( minError >= abs(errorTemp) ){
			error = errorTemp;
			minError = abs(errorTemp);
			preWaypoint = waypointTemp1;
			preNodeIndex = nodeIndexTemp1;
		}
		if( calculateDist(prePos, lane_[course[nodeIndexTemp]].coordinate[waypointTemp]) > dist ) return true;
		

		nodeIndexTemp1 = nodeIndexTemp;
		waypointTemp1 = waypointTemp;
	}
	return false;
}
bool Hdmap::rotateWaypoint(pair<double, double> target, pair<double, double>& output, double heading)
{
	output.first = target.first * cos(heading) - target.second * sin(heading);
	output.second = target.first * sin(heading) + target.second * cos(heading);
	return true;
}





void Hdmap::ControlCar()
{
	if( !userparamsetting || !readcomplete ) return;
	if( !ispathfound ){
		if( isgpson ){
			double min_dist=10000000;
			int closestwaypointindex = 0;
			int index; 
			//string destination = "A117CR700361";
			int targetLaneIndex = 8258;
			carpos_utm = carpos;
			if( firstLocalization(maprange_, lane, maprangestructnum, diff_x, diff_y, carpos_utm, laneIndex, waypointIndex, minError) ){
				if( findCourse(lane, course, lanestructnum, laneIndex, targetLaneIndex) ){
					for(int i=0;i<course.size();i++)
					{
						params->waypoint_utm.insert(params->waypoint_utm.end(), lane[course[i]].coordinate.begin(), lane[course[i]].coordinate.end());
					}
					pair<double, double> bias = {316000, 3892000};
				}
				else{
					cout << "path found error!!!!" << endl;
				}
			}
			laneIndex = 0;
			fixedWaypointIndex = waypointIndex;
			fixedLaneIndex = laneIndex;
			ispathfound = true;
			return;
		}
		else return;
	}

	if( !isgpson || !usergpson ){
		dist = obd_time_diff * obd_speed;
		localizationDR(lane, dist, waypointIndex, laneIndex, course, carpos_utm, margin, error);
		////
		distTotal += dist;
	}
	else{
		carpos_utm = carpos;
		waypointIndex = fixedWaypointIndex;
		laneIndex = fixedLaneIndex;
		localization(lane, waypointIndex, laneIndex, course, carpos_utm, error, distTotal);
		fixedWaypointIndex = waypointIndex;
		fixedLaneIndex = laneIndex;
		distTotal = 30.0;
	}
	clock_t prevClock;
	clock_t curClock;
	prevClock = clock();
	string strFps;
	string strText;
	string strSteering;
	adasone_msgs::Steering steering;
	adasone_msgs::Point point;
	adasone_msgs::Screenpoint screen_xy;
	screenWaypoint.clear();
	extrackWaypoint(lane, waypointIndex, laneIndex, course, extractedWaypoint, carpos_utm, extractDist);
	movedWaypoint = extractedWaypoint;
	rotatedWaypoint = extractedWaypoint;
	transformWaypoint(extractedWaypoint, heading, movedWaypoint, rotatedWaypoint, carpos_utm);
	calculateSteering(rotatedWaypoint, lookAheadDist, pastSteering, carLength, lookAheadPoint);
	waypoint2screenpoint(rotatedWaypoint, screenWaypoint, thetaRad, camHeight, f, screenX, screenY, centerLine, camBias);
	vector<pair<double,double>> lookAheadScreen;
	vector<pair<double,double>> lookAheadTemp;
	lookAheadTemp.push_back(lookAheadPoint);
	waypoint2screenpoint(lookAheadTemp, lookAheadScreen, thetaRad, camHeight, f, screenX, screenY, centerLine, camBias);
	
	point.lookaheadpoint_x = lookAheadScreen[0].first;
	point.lookaheadpoint_y = lookAheadScreen[0].second;
	steering.map_steering = pastSteering*15.4;
	steering.header.stamp = gpstimestamp;
	steering.disttotal = distTotal;
	steering.error = error;
	if( carpos != carpos_utm ){
		pair<double,double> temp = {carpos_utm.first-carpos.first, carpos_utm.second-carpos.second};
		rotateWaypoint(temp, temp, heading);
		steering.posError[0]=temp.first;
		steering.posError[1]=temp.second;
	}
	else{
		steering.posError[0]=0;
		steering.posError[1]=0;
	}
	steering.carPos[0]=carpos_utm.first;
	steering.carPos[1]=carpos_utm.second;
	
	for(auto x: screenWaypoint){
		screen_xy.screen_x = x.first;
		screen_xy.screen_y = x.second;

		point.screenpoint.push_back(screen_xy);
	}
	point.header.stamp = gpstimestamp;





	pub_map_steering.publish(steering);
	pub_map_screenpoint.publish(point);		
	paramClear();
}

void Hdmap::paramClear()
{
	extractedWaypoint.clear();
	isgpson = false;
}

void Hdmap::Run() {
	ros::Rate loop_rate(20);
	
	message_filters::Subscriber<sensor_msgs::NavSatFix> sub_gps(nh_, "/fix", 1);
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> sub_heading(nh_, "/heading", 1);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix,geometry_msgs::QuaternionStamped> MySyncPolicy_;
    message_filters::Synchronizer<MySyncPolicy_> sync_(MySyncPolicy_(10), sub_gps, sub_heading);
    sync_.registerCallback(boost::bind(&Hdmap::callback, this, _1, _2));

	while (ros::ok()) {
		ros::spinOnce();
		ControlCar();
		loop_rate.sleep();	
	}
}


