#include "gui.h"

Gui::Gui(){
    params = &param;

    sub_cam = nh_.subscribe("/image_raw_4", 1, &Gui::Callbackimage, this);
    sub_gps = nh_.subscribe("/fix", 1,&Gui::Callbackgps, this);
    sub_userParam = nh_.subscribe(params->userParam.c_str(), 1, &Gui::CallbackUserParam, this);

    image_height = 801;
	image_width = 2001;
	unit = 0.1; //y axis unit
	absmaxmin = 4; // y axis max/min 
	num_diff = absmaxmin / unit; 
	pixeldiff = (image_height/2) / num_diff;
  

    params->histo_time = Mat::zeros(image_height ,image_width, CV_8UC3);
	params->histo_time = Scalar(255,255,255);
    params->histo_weight = Mat::zeros(image_height, image_width, CV_8UC3);
    params->histo_weight = Scalar(255, 255, 255);
    params->cross_error = Mat::zeros(image_height, image_width, CV_8UC3);
    params->cross_error = Scalar(125, 125, 125);
	line(params->histo_time, Point(51, 0), Point(51, image_height), Scalar(0,0,0), 1); // y axis
	line(params->histo_time, Point(0, image_height/2+1), Point(image_width, image_height/2+1), Scalar(0,0,0), 1); // x axis
    line(params->histo_weight, Point(51, 0), Point(51, image_height), Scalar(0,0,0), 1); // y axis
	line(params->histo_weight, Point(0, image_height/2+1), Point(image_width, image_height/2+1), Scalar(0,0,0), 1); // x axis
    line(params->cross_error, Point(0, image_height/5), Point(image_width, image_height/5), Scalar(255, 255,255), 6); // y axis
	line(params->cross_error, Point(0, image_height * 4/5 ), Point(image_width, image_height * 4/5 ), Scalar(255,255,255), 6); // x axis
   	line(params->cross_error, Point(0, image_height /2), Point(image_width, image_height /2), Scalar(200,200,200), 3); // x axis
    

    
    for(int i=0;i<20;i++)//200 sec
	{ 
		line(params->histo_time, Point(51+100*i, image_height/2+2), Point(51+100*i, image_height/2-2), Scalar(0,0,0), 1);
		if( i%5 == 0 )
		{
			line(params->histo_time, Point(51+100*i, image_height/2+5), Point(51+100*i, image_height/2-5), Scalar(0,0,0), 1);
            line(params->histo_weight, Point(51+100*i, image_height/2+5), Point(51+100*i, image_height/2-5), Scalar(0,0,0), 1);
		}
	}
	/////// draw y axis
	for(int i = 1;i < num_diff+1;i++)
	{
		line(params->histo_time, Point(46,(image_height/2+1)+pixeldiff*i ), Point(56,(image_height/2+1)+pixeldiff*i ), Scalar(0,0,0), 1); //positive
		line(params->histo_time, Point(46,(image_height/2+1)-pixeldiff*i ), Point(56,(image_height/2+1)-pixeldiff*i ), Scalar(0,0,0), 1); //negative
        line(params->histo_weight, Point(46,(image_height/2+1)+pixeldiff*i ), Point(56,(image_height/2+1)+pixeldiff*i ), Scalar(0,0,0), 1); //positive
		line(params->histo_weight, Point(46,(image_height/2+1)-pixeldiff*i ), Point(56,(image_height/2+1)-pixeldiff*i ), Scalar(0,0,0), 1); //negative
	}
    timediff = 0;
}

void Gui::plotsteering(float error, float time, Scalar color)
{
	if( plotpoint.size() > 1 )
	{
		plotpoint.erase(plotpoint.begin());
	}
	
	int errorround = round(error*3);

    if( plotpoint.size() == 1 )
    {
	    line(params->histo_time, Point(plotpoint[0][0],plotpoint[0][1]), Point(51+100*time, (image_height/2+1) + error*10*pixeldiff), color, 2);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint.push_back({51+100*time,(image_height/2+1) + error*10*pixeldiff}); // push back previous plotpoint
}


void Gui::plotlane(float error, float time, Scalar color)
{
	if( plotpoint2.size() > 1 )
	{
		plotpoint2.erase(plotpoint2.begin());
	}
	
	int errorround = round(error*3);

    if( plotpoint2.size() == 1 )
    {
	    line(params->histo_time, Point(plotpoint2[0][0],plotpoint2[0][1]), Point(51+100*time, (image_height/2+1) + error*10*pixeldiff), color, 3);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint2.push_back({51+100*time,(image_height/2+1) + error*10*pixeldiff}); // push back previous plotpoint
}

void Gui::plotweight(float error, float time, Scalar color)
{
	if( plotpoint1.size() > 1 )
	{
		plotpoint1.erase(plotpoint1.begin());
	}
	
	int errorround = round(error);

    if( plotpoint1.size() == 1 )
    {
	    line(params->histo_weight, Point(plotpoint1[0][0],plotpoint1[0][1]), Point(51+100*time, (image_height/2+1) + error*4*pixeldiff), color, 2);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint1.push_back({51+100*time,(image_height/2+1) + error*4*pixeldiff}); // push back previous plotpoint
}
void Gui::plotusrSteering(float error, float time, Scalar color)
{
	if( plotpoint5.size() > 1 )
	{
		plotpoint5.erase(plotpoint5.begin());
	}
	
	int errorround = round(error);

    if( plotpoint5.size() == 1 )
    {
	    line(params->histo_weight, Point(plotpoint5[0][0],plotpoint5[0][1]), Point(51+100*time, (image_height/2+1) + error*4*pixeldiff), color, 2);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint5.push_back({51+100*time,(image_height/2+1) + error*4*pixeldiff}); // push back previous plotpoint
}


void Gui::plotCrossTrackError(float error, float time, Scalar color)
{
	if( plotpoint3.size() > 1 )
	{
		plotpoint3.erase(plotpoint3.begin());
	}
	
	int errorround = round(error*10);

    if( plotpoint3.size() == 1 )
    {
	    line(params->cross_error, Point(plotpoint3[0][0],plotpoint3[0][1]), Point(51+100*time, (image_height/2+1) + error*10*pixeldiff), color, 4);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint3.push_back({51+100*time,(image_height/2+1) + error*10*pixeldiff}); // push back previous plotpoint
}

void Gui::plotOdoCrossTrackError(float error, float time, Scalar color)
{
	if( plotpoint4.size() > 1 )
	{
		plotpoint4.erase(plotpoint4.begin());
	}
	
	int errorround = round(error*10);

    if( plotpoint4.size() == 1 )
    {
	    line(params->cross_error, Point(plotpoint4[0][0],plotpoint4[0][1]), Point(51+100*time, (image_height/2+1) + error*10*pixeldiff), color, 4);
	    //draw line ( previous plot point, current plot point )
    }
	plotpoint4.push_back({51+100*time,(image_height/2+1) + error*10*pixeldiff}); // push back previous plotpoint
}




Gui::~Gui(){
    int sum = 0;
    int min = 9999;
    int max = -9999;
    for(int i=0;i<lanediffall.size();i++)
    {
        sum+= lanediffall[i];
        if( lanediffall[i] > max)
        {
            max = lanediffall[i];
        }
        if( lanediffall[i] < min)
        {
            min = lanediffall[i];
        }
    }

    sum = 0;
    min = 9999;
    max = -9999;
    for(int i=0;i<mapdiffall.size();i++)
    {
        sum+= mapdiffall[i];
        if( mapdiffall[i] > max)
        {
            max = mapdiffall[i];
        }
        if( mapdiffall[i] < min)
        {
            min = mapdiffall[i];
        }
    }

    sum = 0;
    min = 9999;
    max = -9999;
    for(int i=0;i<weighteddiffall.size();i++)
    {
        sum+= weighteddiffall[i];
        if( weighteddiffall[i] > max)
        {
            max = weighteddiffall[i];
        }
        if( weighteddiffall[i] < min)
        {
            min = weighteddiffall[i];
        }
    }

}

void Gui::CallbackUserParam(const adasone_msgs::UserCustomParam::ConstPtr& msg)
{
  params->height_cam = msg->cam_height_float;
  params->f = msg->cam_focal;
  params->lookahead_distance = msg->lookahead_dist;
  params->theta_rad = msg->cam_tilt * PI/180;
  params->center_line = msg->cam_centerline;
  params->num = int(msg->num_waypoint);
  params->cambias = msg->cambias;
  params->gpsbias = msg->gpsbias;
  params->screen_x = int(msg->imagewidth);
  params->screen_y = int(msg->imageheight);
 
}

void Gui::Callbackimage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        camimage = cv_bridge::toCvCopy(msg, "bgr8") -> image;
        QImage image(camimage.data, camimage.cols,
                 camimage.rows, camimage.step[0], QImage::Format_RGB888);
        image = image.rgbSwapped();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Cannot convert image!");
    }
}

void Gui::Callbackgps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    qt_handle_->Updategpslat(msg->latitude);
    qt_handle_->Updategpslon(msg->longitude);
}

void Gui::callback(const adasone_msgs::Steering::ConstPtr& usrNlane, const adasone_msgs::Steering::ConstPtr& map, const adasone_msgs::UserCustomParam::ConstPtr& lane)//
{
    timestamp_temp = map->header.stamp;
    timestamp.push_back( timestamp_temp.toSec());
    qt_handle_->Updateusrsteering(usrNlane->usr_steering);
    qt_handle_->Updatesteering(map->map_steering);
    qt_handle_->Updatelanesteering(usrNlane->lane_steering);
    qt_handle_->Updatedisttotal(map->disttotal);
    qt_handle_->Updateodometryx(map->carPos[0]);
    qt_handle_->Updateodometryy(map->carPos[1]);
    qt_handle_->Updatexdiff(map->posError[0]);
    qt_handle_->Updateydiff(map->posError[1]);


    double steering_map, steering_lane, steering_weighted;
    double lanemargin, odomargin;
    steering_map = map->map_steering;
    steering_lane = usrNlane->lane_steering;    
    lanemargin = lane->lanemargin;
    odomargin = map->error;

    if(first_input)
    {  
   
         steering_weighted = (steering_map + steering_lane) * 0.5 * 0.7  + last_steering *  0.3;

    }
    else
    {   
        steering_weighted = (steering_map + steering_lane) * 0.5;
        first_input = true;
    }
    last_steering = steering_weighted;
 
    qt_handle_->Updateweightsteering(steering_weighted);
    
    Scalar mapcolor(255,0,0);
    Scalar lanecolor(78,154,6);
    Scalar weightedcolor(0,0,255);  
    Scalar laneCrossColor(0, 0, 255);
    Scalar OdoCrossColor(0, 255, 0);

    if( timestamp.size() > 2)
    {
        timediff += timestamp[timestamp.size()-1] - timestamp[timestamp.size()-2];
        if(  timediff > 0.1){
            plotsteering(steering_map - usrNlane->usr_steering, timestamp.back()-timestamp.front(),mapcolor);
            plotlane(steering_lane - usrNlane->usr_steering, timestamp.back()-timestamp.front(),lanecolor);
            plotweight(steering_weighted, timestamp.back()-timestamp.front(),weightedcolor);
            plotusrSteering(usrNlane->usr_steering, timestamp.back()-timestamp.front(),lanecolor);
            plotCrossTrackError(lanemargin, timestamp.back()-timestamp.front(),laneCrossColor);
            plotOdoCrossTrackError(odomargin,  timestamp.back()-timestamp.front(), OdoCrossColor);
           
           

            QImage image(params->histo_time.data, params->histo_time.cols,
                    params->histo_time.rows, params->histo_time.step[0], QImage::Format_RGB888);
            image = image.rgbSwapped();
            ////// 히스토그램 이미지 넣기 ///////
        
            QImage image1(params->histo_weight.data, params->histo_weight.cols, params->histo_weight.rows, params->histo_weight.step[0], QImage::Format_RGB888);
            image1 = image1.rgbSwapped();   


            QImage image2(params->cross_error.data, params->cross_error.cols, params->cross_error.rows, params->cross_error.step[0], QImage::Format_RGB888);
            image2 = image2.rgbSwapped();       
            
            qt_handle_->Updatecam(image);
            qt_handle_->Updateweight(image1);
            qt_handle_->UpdateCrossTrackError(image2);
            timediff = 0;
        }
    }
    if( timestamp.back()-timestamp.front() > 20 )
    {
        params->histo_time = Scalar(255,255,255);
        params->histo_weight = Scalar(255,255,255);

        line(params->histo_time, Point(51, 0), Point(51, image_height), Scalar(0,0,0), 1); // y axis
        line(params->histo_time, Point(0, image_height/2+1), Point(image_width, image_height/2+1), Scalar(0,0,0), 1); // x axis
        line(params->histo_weight, Point(51, 0), Point(51, image_height), Scalar(0,0,0), 1); // y axis
        line(params->histo_weight, Point(0, image_height/2+1), Point(image_width, image_height/2+1), Scalar(0,0,0), 1); // x axis
        
        params->cross_error = Scalar(125,125,125);

    for(int i=0;i<20;i++)//200 sec
	{ 
		line(params->histo_time, Point(51+100*i, image_height/2+2), Point(51+100*i, image_height/2-2), Scalar(0,0,0), 1);
		line(params->histo_weight, Point(51+100*i, image_height/2+2), Point(51+100*i, image_height/2-2), Scalar(0,0,0), 1);

        line(params->cross_error, Point(0, image_height/5), Point(image_width, image_height/5), Scalar(255, 255,255), 6); // y axis
        line(params->cross_error, Point(0, image_height * 4/5 ), Point(image_width, image_height * 4/5 ), Scalar(255,255,255), 6); // x axis
        line(params->cross_error, Point(0, image_height /2), Point(image_width, image_height/2), Scalar(200, 200, 200), 3);

		if( i%5 == 0 )
		{
			line(params->histo_time, Point(51+100*i, image_height/2+5), Point(51+100*i, image_height/2-5), Scalar(0,0,0), 1);
            line(params->histo_weight, Point(51+100*i, image_height/2+5), Point(51+100*i, image_height/2-5), Scalar(0,0,0), 1);
		}
	} 
	/////// draw y axis
	for(int i = 1;i < num_diff+1;i++)
	{
		line(params->histo_time, Point(46,(image_height/2+1)+pixeldiff*i ), Point(56,(image_height/2+1)+pixeldiff*i ), Scalar(0,0,0), 1); //positive
		line(params->histo_time, Point(46,(image_height/2+1)-pixeldiff*i ), Point(56,(image_height/2+1)-pixeldiff*i ), Scalar(0,0,0), 1); //negative
        line(params->histo_weight, Point(46,(image_height/2+1)+pixeldiff*i ), Point(56,(image_height/2+1)+pixeldiff*i ), Scalar(0,0,0), 1); //positive
		line(params->histo_weight, Point(46,(image_height/2+1)-pixeldiff*i ), Point(56,(image_height/2+1)-pixeldiff*i ), Scalar(0,0,0), 1); //negative
	}
        
        timestamp.clear();
        plotpoint.clear();
        plotpoint1.clear();
        plotpoint2.clear();
        plotpoint3.clear();
        plotpoint4.clear();
        plotpoint5.clear();

    }
}

void Gui::callbackScreen(const adasone_msgs::Point::ConstPtr& HDpoint, const adasone_msgs::UserCustomParam::ConstPtr& lanepoint)
{
    if(camimage.rows>0){
        camimage.copyTo(camimage4drawing);
        for(int i=0;i<HDpoint->screenpoint.size();i++)
        {
            if ( HDpoint->screenpoint[i].screen_y < 240 )
            {
                circle(camimage4drawing, Point(HDpoint->screenpoint[i].screen_x, params->screen_y - HDpoint->screenpoint[i].screen_y), 1, Scalar(0, 0, 255), 1);
            }
        }
        lookaheadx = HDpoint->lookaheadpoint_x;
        lookaheady = HDpoint->lookaheadpoint_y;

        line(camimage4drawing, Point(params->center_line,0), Point(params->center_line,params->screen_y), Scalar(255,255,255), 1);
        line(camimage4drawing, Point(0,params->screen_y - lookaheady), Point(params->screen_x,params->screen_y - lookaheady), Scalar(255,255,255), 1);
        circle(camimage4drawing, Point(lookaheadx, params->screen_y - lookaheady), 2, Scalar(255,0,0), 3);        
        
        line(camimage4drawing, Point(lanepoint->previous_u[0], 262), Point(lanepoint->previous_u[1], 234), Scalar(0,255, 0), 3);
        line(camimage4drawing, Point(lanepoint->previous_u[2], 262), Point(lanepoint->previous_u[3], 234), Scalar(0,255, 0), 3);
        line(camimage4drawing, Point(lanepoint->previous_m[0], 350), Point(lanepoint->previous_m[1], 262), Scalar(0,255, 0), 3);
        line(camimage4drawing, Point(lanepoint->previous_m[2], 350), Point(lanepoint->previous_m[3], 263), Scalar(0,255, 0), 3);      
        circle(camimage4drawing, Point(lanepoint->circle_point[0], lanepoint->circle_point[1]), 2, Scalar(0,255,0), 3);

        QImage image(camimage4drawing.data, camimage4drawing.cols, camimage4drawing.rows, camimage4drawing.step[0], QImage::Format_RGB888);
        image = image.rgbSwapped();
        qt_handle_->Updatecamdraw(image);

        
    }
}
void Gui::QtThread()
{
    QApplication gui_app(argc_, argv_);
    Driving w;

    qt_handle_ = &w;
    gui_app_ = &gui_app;

    qt_handle_->show();       
    gui_app_->exec();
}

void Gui::RunGui(int argc, char** argv)
{
    ros::Rate loop_rate(20);

    argc_ = argc;
    argv_ = argv;

    qt_thread_= std::thread(&Gui::QtThread, this);

    message_filters::Subscriber<adasone_msgs::Steering> sub_usrNlanesteering(nh_, params->usrsteering.c_str(), 1);
    message_filters::Subscriber<adasone_msgs::Steering> sub_mapsteering(nh_, params->hdmap_steering.c_str(), 1);
    message_filters::Subscriber<adasone_msgs::UserCustomParam> sub_lanemargin(nh_, params->lane_steering.c_str(), 1);//
    typedef message_filters::sync_policies::ApproximateTime<adasone_msgs::Steering,adasone_msgs::Steering, adasone_msgs::UserCustomParam> MySyncPolicy; // 
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_usrNlanesteering, sub_mapsteering, sub_lanemargin); //
    sync.registerCallback(boost::bind(&Gui::callback, this, _1, _2, _3)); //


    message_filters::Subscriber<adasone_msgs::Point> sub_screenpoint(nh_, params->screenpoint.c_str(), 1);
    message_filters::Subscriber<adasone_msgs::UserCustomParam> sub_lanedetectresult(nh_, params->lane_steering.c_str(), 1);
    typedef message_filters::sync_policies::ApproximateTime<adasone_msgs::Point,adasone_msgs::UserCustomParam> MySyncPolicy_;
    message_filters::Synchronizer<MySyncPolicy_> sync_(MySyncPolicy_(10), sub_screenpoint, sub_lanedetectresult);
    sync_.registerCallback(boost::bind(&Gui::callbackScreen, this, _1, _2));

    while(1)
    {
        if(qt_handle_)
        {
            break;
        }
    }
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
}
