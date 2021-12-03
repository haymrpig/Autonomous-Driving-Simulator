#ifndef HDMAP_H
#define HDMAP_H

#include <stack>
#include <tf/tf.h>
#include "Parameters.h"
#include <adasone_msgs/ObdParam.h>
#include <adasone_msgs/UserCustomParam.h>
#include <adasone_msgs/GpsPoint.h>
#include <adasone_msgs/Steering.h>
#include <adasone_msgs/Flag.h>
#include <adasone_msgs/Point.h>
#include <adasone_msgs/Screenpoint.h>
#include <adasone_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <typeinfo>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define PI 3.14159265358979323846
#define WGS84_A		6378137.0		// major axis
#define WGS84_B		6356752.31424518	// minor axis
#define WGS84_F		0.0033528107		// ellipsoid flattening
#define WGS84_E		0.0818191908		// first eccentricity
#define WGS84_EP	0.0820944379		// second eccentricity
#define UTM_K0		0.9996			// scale factor
#define UTM_FE		500000.0		// false easting
#define UTM_FN_N	0.0           // false northing, northern hemisphere
#define UTM_FN_S	10000000.0    // false northing, southern hemisphere
#define UTM_E2		(WGS84_E*WGS84_E)	// e^2
#define UTM_E4		(UTM_E2*UTM_E2)		// e^4
#define UTM_E6		(UTM_E4*UTM_E2)		// e^6
#define UTM_EP2		(UTM_E2/(1-UTM_E2))	// e'^2


using namespace std;


struct lane1 {
	string id;
	string fromnode;
	string tonode;
	int num_waypoint;
	int lanenum;

	int coordinateNum;
	int leftlaneindex = -1;
	int rightlaneindex = -1;

	int linkedNodeNum = 0;
	vector<int> linkedNode;
	vector<pair<double, double>> coordinate;
};

struct maprange{
	pair<double, double> array;
	int struct_num;
	vector<int> structIndex;
};

class Hdmap{
public:
	Hdmap();
	~Hdmap();
	
	ros::NodeHandle nh_;
	ros::Publisher pub_map_steering;
	ros::Publisher pub_flag;
	ros::Publisher pub_map_screenpoint;
	ros::Subscriber sub_userParam;
	ros::Subscriber sub_obd_data;
	ros::Subscriber sub_margin;

	ros::Time gpstimestamp;
	ros::Time timestamp_temp;
	ros::Time obd_pre_time;
	ros::Time obd_cur_time;

	Parameters* params;
	Parameters param;

	lane1* lane;
	maprange* maprange_;

	const char band = 's';


	bool obd_first_input;
	bool ispathfound;
	bool isgpson;
	bool usergpson;
	bool readcomplete=false;
	bool userparamsetting=false;
	bool gpson=false;
	bool findpath = false;
	int lanestructnum;
	int maprangestructnum;
	int zone = 52;
	int fixedWaypointIndex;
	int fixedLaneIndex;
	int laneIndex;
	int waypointIndex;
	double diff_x, diff_y;
	double timestamp;
	double obd_speed;
	double obd_time_diff;
	double extractDist;
	double distTotal;
	double dist; //DR dist
	double error;
	double f;
	double thetaRad;
	double centerLine;
	double camBias;
	double camHeight;
	double screenX;
	double screenY;
	double lookAheadDist;
	double pastSteering;
	double carLength;
	double minError;
	double heading;
	double margin;
	double gps_lat;
	double gps_lon;
	double roll, pitch, yaw;
	pair<double, double> carpos_utm;
	pair<double, double> lastFixPos;
	pair<double, double> lookAheadPoint;
	pair<double, double> carpos_DR;
	pair<double, double> carpos;
	vector<int> course;
	vector<pair<double, double>> movedWaypoint;
	vector<pair<double, double>> rotatedWaypoint;
	vector<pair<double, double>> screenWaypoint;
	vector<pair<double, double>> extractedWaypoint;

	double Calculate_screen_x(double f, double alpha_rad, double beta_rad);
	double Calculate_screen_y(double f, double alpha_rad);
	double Calculate_beta(double theta_rad, double alpha_rad, double height, double x_diff);
	double Calculate_alpha(double y_diff, double theta_rad, double height);
	double calculateDist(pair<double, double> pre, pair<double, double> next);

	void waypoint2screenpoint(vector<pair<double, double>>& rotatedWaypoint, vector<pair<double,double>>& screenWaypoint,double thetaRad, double camHeight, double f, int screenX, int screenY, double centerLine, double camBias);
	void WGS2UTM(vector<vector<double>> llh, vector<vector<double>>* utm, int num, const char band, int zone);
	void drawRoute(Mat& picture, vector<pair<double,double>>& waypoint, pair<double, double> bias);
	void makeCourse(vector<int>& parentNode, vector<int>& course, int nodeIndex, int startNodeIndex);
	void WGS2UTM(pair<double,double> llh, pair<double,double>& utm, int num, const char band, int zone);
	
	bool findCourse(lane1* lane_, vector<int>& course, int laneSize, int startNodeIndex, int endNodeIndex);
	bool localization(lane1* lane_, int& preWaypoint, int& preNodeIndex, vector<int>& course, pair<double, double>& carPos, double& error, double dist);
	bool localizationDR(lane1* lane_, double dist, int& preWaypoint, int& preNodeIndex, vector<int>& course, pair<double, double>& carPosDR, double margin, double& error);
	bool calculateSteering(vector<pair<double, double>> rotatedWaypoint, double lookAheadDist, double& pastSteering, double carLength, pair<double, double>& lookAheadPoint);
	bool transformWaypoint(vector<pair<double, double>>& extractedWaypoint, double carHeading, vector<pair<double, double>>& movedWaypoint, vector<pair<double, double>>& rotatedWaypoint, pair<double, double> currentPos);
	bool extrackWaypoint(struct lane1* lane_, int preWaypoint, int preNodeIndex, vector<int>& course,vector<pair<double,double>>& extractedWaypoint, pair<double,double> carPos, double distRange);
	bool rotateWaypoint(pair<double, double> target, pair<double, double>& output, double heading);	
	bool firstLocalization(struct maprange* map_, struct lane1* lane_, int mapSize, int mapX, int mapY, pair<double, double> gpsUTM, int& nodeIndex, int& waypointIndex, double& minError);
	bool crosstrackError(pair<double, double> prewaypoint, pair<double, double> nextwaypoint, pair<double, double> currentPos, double& error);

	void CallbackMargin(const adasone_msgs::UserCustomParam::ConstPtr& msg);
	void CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg);
  	void CallbackObdParam(const adasone_msgs::ObdParam::ConstPtr& msg);
	void callback(const sensor_msgs::NavSatFix::ConstPtr& gps_, const geometry_msgs::QuaternionStamped::ConstPtr& heading_);
	void ControlCar();
	void readFromFile();
	void paramClear();
	void init();
	void Run();
};
#endif //HDMAP_H
