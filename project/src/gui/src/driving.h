#ifndef DRIVING_H
#define DRIVING_H
#include <QMainWindow>
#include <QDebug>
#include "adasone_msgs/UserCustomParam.h"
#include "Parameters.h"
#include "adasone_msgs/Odometry.h"

#define PI 3.14159265358979323846
namespace Ui {
class Driving;
}

class Driving : public QMainWindow{
  Q_OBJECT

public:
  explicit Driving(QWidget* parent = nullptr);
  ~Driving();
  void Updatecam(QImage data);
  void UpdateCrossTrackError(QImage data);
  void Updatecamdraw(QImage data);
  void Updateweight(QImage data);
  void Updatesteering(float data);
  void Updatelanesteering(float data);
  void Updateusrsteering(float data);
  void Updateweightsteering(float data);
  void Updategpslon(float data);
  void Updategpslat(float data);
  void Updateodometryx(float data);
  void Updateodometryy(float data);
  void Updatedisttotal(float data);
  void Updatexdiff(float data);
  void Updateydiff(float data);

  //void addPoint(double x, double y);
  //void plot();
  void Initialization();
  void InitQtConnect();
  void InitQttext();

  ros::NodeHandle nh_;
  ros::Publisher pub_userparam;

  Parameters param;
  Parameters* params;

  adasone_msgs::UserCustomParam msgs;

signals:
  void Callupdatecam(QImage);
  void CallupdateCrossTrackError(QImage);
  void Callupdatecamdraw(QImage);
  void Callupdateweight(QImage);
  void Callupdatesteering(float);
  void Callupdatelanesteering(float);
  void Callupdateusrsteering(float);
  void Callupdateweightsteering(float);
  void Callupdategpslon(float);
  void Callupdategpslat(float);
  void Callupdateodometryx(float);
  void Callupdateodometryy(float);
  void Callupdatedisttotal(float);
  void Callupdatexdiff(float);
  void Callupdateydiff(float);


private slots:
  void Updatecamslot(QImage);
  void UpdateCrossTrackErrorslot(QImage);
  void Updatecamdrawslot(QImage image_camdraw);
  void Updateweightslot(QImage);
  void Updatesteeringmap(float);
  void Updatesteeringlane(float);
  void Updatesteeringusr(float);
  void Updatesteeringweight(float);
  void Updategpslonslot(float);
  void Updategpslatslot(float);
  void Updateodometryxslot(float);
  void Updateodometryyslot(float);
  void Updatedisttotalslot(float);
  void Updatexdiffslot(float);
  void Updateydiffslot(float);

  void paramsinput();

private:
  Ui::Driving* ui_;
  
  QImage image_cam_temp;
  QImage image_cte_temp;
  QImage image_camdraw_temp;
  QImage image_camdraw_temp1;
  QImage image_weight_draw;

  QString lookaheaddistance = "20.0";
  QString camheight = "1.5";
  QString camfocal = "270";
  QString camtilt = "87";
  QString camcenterline = "360";
  QString numwaypoint = "100";
  QString carlength = "2.6";
  QString cambias = "0";
  QString gpsbias = "0.35";
  QString imagewidth = "720";
  QString imageheight = "480";
  QString headingbias = "0.027";
  QString smoothingweight = "0.6";
  QString Rweight = "2";
  float lookaheaddistance_float;
  float camheight_float;
  float camfocal_float;
  float camtilt_float;
  float camcenterline_float;
  
  //QVector<double> q_x, q_y;
};

#endif // TEST_H
