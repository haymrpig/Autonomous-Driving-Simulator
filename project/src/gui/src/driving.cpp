#include "driving.h"
#include "ui_driving.h"

Driving::Driving(QWidget* parent)
    : QMainWindow(parent), ui_(new Ui::Driving) {
  Initialization();
  params = &param;
  pub_userparam = nh_.advertise<adasone_msgs::UserCustomParam>(params->userParam.c_str(), 1);
}

Driving::~Driving() {
  delete ui_;
}

void Driving::Initialization() {
  ui_->setupUi(this);
  InitQttext();
  InitQtConnect();
}

void Driving::InitQttext(){
  ui_->lookaheaddistance->setText(lookaheaddistance);
  ui_->camheight->setText(camheight);
  ui_->camfocal->setText(camfocal);
  ui_->camtilt->setText(camtilt);
  ui_->camcenterline->setText(camcenterline);
  ui_->carlength->setText(carlength);
  ui_->numwaypoint->setText(numwaypoint);
  ui_->cambias->setText(cambias);
  ui_->gpsbias->setText(gpsbias);
  ui_->imagewidth->setText(imagewidth);
  ui_->imageheight->setText(imageheight);
  ui_->headingbias->setText(headingbias);
  ui_->smoothingweight->setText(smoothingweight);
  ui_->Rweight->setText(Rweight);
  ui_->gpscheck->setChecked(true);
}

void Driving::InitQtConnect() {
  connect(this, SIGNAL(Callupdatecam(QImage)), this, SLOT(Updatecamslot(QImage)));
  connect(this, SIGNAL(CallupdateCrossTrackError(QImage)), this, SLOT(UpdateCrossTrackErrorslot(QImage)));
  connect(this, SIGNAL(Callupdatesteering(float)), this, SLOT(Updatesteeringmap(float)));
  connect(this, SIGNAL(Callupdatelanesteering(float)), this, SLOT(Updatesteeringlane(float)));
  connect(this, SIGNAL(Callupdateusrsteering(float)), this, SLOT(Updatesteeringusr(float)));
  connect(this, SIGNAL(Callupdateweightsteering(float)), this, SLOT(Updatesteeringweight(float)));
  connect(this, SIGNAL(Callupdatecamdraw(QImage)), this, SLOT(Updatecamdrawslot(QImage)));
  connect(this, SIGNAL(Callupdateweight(QImage)), this, SLOT(Updateweightslot(QImage)));
  connect(this, SIGNAL(Callupdategpslon(float)), this, SLOT(Updategpslonslot(float)));
  connect(this, SIGNAL(Callupdategpslat(float)), this, SLOT(Updategpslatslot(float)));
  connect(this, SIGNAL(Callupdateodometryx(float)), this, SLOT(Updateodometryxslot(float)));
  connect(this, SIGNAL(Callupdateodometryy(float)), this, SLOT(Updateodometryyslot(float)));
  connect(this, SIGNAL(Callupdatedisttotal(float)), this, SLOT(Updatedisttotalslot(float)));
  connect(this, SIGNAL(Callupdatexdiff(float)), this, SLOT(Updatexdiffslot(float)));
  connect(this, SIGNAL(Callupdateydiff(float)), this, SLOT(Updateydiffslot(float)));
  connect(ui_->enter, SIGNAL(clicked()), this, SLOT(paramsinput()));

}

void Driving::Updatecam(QImage data) {
  emit Callupdatecam(data);
}

void Driving::UpdateCrossTrackError(QImage data) {
  emit CallupdateCrossTrackError(data);
}

void Driving::Updatecamdraw(QImage data){
  emit Callupdatecamdraw(data);
}

void Driving::Updateweight(QImage data){
  emit Callupdateweight(data);
}

void Driving::Updatelanesteering(float data){
  emit Callupdatelanesteering(data);
}

void Driving::Updateweightsteering(float data){
  emit Callupdateweightsteering(data);
}

void Driving::Updateusrsteering(float data){
  emit Callupdateusrsteering(data);
}

void Driving::Updatesteering(float data){
  emit Callupdatesteering(data);
}

void Driving::Updategpslon(float data){
  emit Callupdategpslon(data);
}

void Driving::Updategpslat(float data){
  emit Callupdategpslat(data);
}

void Driving::Updateodometryx(float data){
  emit Callupdateodometryx(data);
}

void Driving::Updateodometryy(float data){
  emit Callupdateodometryy(data);
}

void Driving::Updatedisttotal(float data){
  emit Callupdatedisttotal(data);
}

void Driving::Updatexdiff(float data){
  emit Callupdatexdiff(data);
}

void Driving::Updateydiff(float data){
  emit Callupdateydiff(data);
}


void Driving::Updatecamslot(QImage image_cam){
  image_cam_temp = image_cam; 
  QImage image_cam_temp1;

  image_cam_temp1 = image_cam_temp.scaledToWidth(ui_->cameraview->width(), Qt::SmoothTransformation);

  ui_->cameraview->setPixmap(QPixmap::fromImage(image_cam_temp1));
}

void Driving::UpdateCrossTrackErrorslot(QImage image_cam1){
  image_cte_temp = image_cam1; 
  QImage image_cte_temp1;

  image_cte_temp1 = image_cte_temp.scaledToWidth(ui_->crosstrackerror->width(), Qt::SmoothTransformation);

  ui_->crosstrackerror->setPixmap(QPixmap::fromImage(image_cte_temp1));
}


void Driving::Updatecamdrawslot(QImage image_camdraw){
    image_camdraw_temp = image_camdraw;
    QImage image_camdraw_temp_;

    image_camdraw_temp_ = image_camdraw_temp.scaledToWidth(ui_->cameradrawview->width(), Qt::SmoothTransformation);

    ui_->cameradrawview->setPixmap(QPixmap::fromImage(image_camdraw_temp_));
}


void Driving::Updateweightslot(QImage image_camdraw){

    image_weight_draw = image_camdraw;
    QImage image_camdraw_temp_;

    image_camdraw_temp_ = image_weight_draw.scaledToWidth(ui_->steeringweight->width(), Qt::SmoothTransformation);

    ui_->steeringweight->setPixmap(QPixmap::fromImage(image_camdraw_temp_));

}

void Driving::Updatesteeringmap(float steering) {
  QString steering_temp1 = QString::number(steering);

  ui_->steering->setText(steering_temp1);
}

void Driving::Updatesteeringlane(float steering) {
  QString steering_temp1 = QString::number(steering);

  ui_->lanesteering->setText(steering_temp1);
}

void Driving::Updatesteeringusr(float steering) {
  QString steering_temp1 = QString::number(steering);

  ui_->usrsteering->setText(steering_temp1);
}

void Driving::Updatesteeringweight(float steering)
{
   QString steering_temp1 = QString::number(steering);

   ui_->weightsteering->setText(steering_temp1);
}

void Driving::Updategpslonslot(float gpslon)
{
  if(ui_->gpscheck->isChecked())
  {
    QString gps_temp = QString::number(gpslon);
    ui_->gpslon->setText(gps_temp);

  }
  else
  {
    QString nogps = "no signal";
    QString gps_temp = nogps;
    ui_->gpslon->setText(gps_temp);

  }
}

void Driving::Updategpslatslot(float gpslat)
{
  if(ui_->gpscheck->isChecked())
  {
    QString gps_temp = QString::number(gpslat);
      ui_->gpslat->setText(gps_temp);
  }
  else
  {
    QString nogps = "no signal";
    QString gps_temp = nogps;
      ui_->gpslat->setText(gps_temp);
  }
}


void Driving::Updateodometryxslot(float odox)
{

    QString odo_temp = QString::number(odox);
    ui_->odometryx->setText(odo_temp);

}
void Driving::Updateodometryyslot(float odoy)
{

    QString odo_temp = QString::number(odoy);
    ui_->odometryy->setText(odo_temp);
}

void Driving::Updatedisttotalslot(float distance)
{

    QString diff_temp = QString::number(distance);
    ui_->disttotal->setText(diff_temp);
}

void Driving::Updatexdiffslot(float diffx)
{
    QString diff_temp = QString::number(diffx);
    ui_->xdiff->setText(diff_temp);
}

void Driving::Updateydiffslot(float diffy)
{
    QString diff_temp = QString::number(diffy);
    ui_->ydiff->setText(diff_temp);
}

void Driving::paramsinput(){
  lookaheaddistance = ui_->lookaheaddistance->text();
  ui_->lookaheaddistance->setText(lookaheaddistance);
  msgs.lookahead_dist = lookaheaddistance.toFloat();
  
  camheight = ui_->camheight->text();
  ui_->camheight->setText(camheight);
  msgs.cam_height_float = camheight.toFloat();

  camfocal = ui_->camfocal->text();
  ui_->camfocal->setText(camfocal);
  msgs.cam_focal = camfocal.toFloat();

  camtilt = ui_->camtilt->text();
  ui_->camtilt->setText(camtilt);
  msgs.cam_tilt = camtilt.toFloat();
  
  camcenterline = ui_->camcenterline->text();
  ui_->camcenterline->setText(camcenterline);
  msgs.cam_centerline = camcenterline.toFloat();
  
  numwaypoint = ui_->numwaypoint->text();
  ui_->numwaypoint->setText(numwaypoint);
  msgs.num_waypoint = numwaypoint.toFloat();

  carlength = ui_->carlength->text();
  ui_->carlength->setText(carlength);
  msgs.carlength = carlength.toFloat();
  
  cambias = ui_->cambias->text();
  ui_->cambias->setText(cambias);
  msgs.cambias = cambias.toFloat(); 
  
  gpsbias = ui_->gpsbias->text();
  ui_->gpsbias->setText(gpsbias);
  msgs.gpsbias = gpsbias.toFloat();

  imagewidth = ui_->imagewidth->text();
  ui_->imagewidth->setText(imagewidth);
  msgs.imagewidth = imagewidth.toFloat();

  imageheight = ui_->imageheight->text();
  ui_->imageheight->setText(imageheight);
  msgs.imageheight = imageheight.toFloat();
  
  headingbias = ui_->headingbias->text();
  ui_->headingbias->setText(headingbias);
  msgs.headingbias = headingbias.toFloat();

  smoothingweight = ui_->smoothingweight->text();
  ui_->smoothingweight->setText(smoothingweight);
  msgs.smoothingweight = smoothingweight.toFloat();
  
  Rweight = ui_->Rweight->text();
  ui_->Rweight->setText(Rweight);
  msgs.Rweight = Rweight.toFloat();

  msgs.plotclear = true;
  msgs.first_lane_detect = false;
  if( ui_->gpscheck->isChecked() ){
    msgs.gpsonoff = true;
  }
  else{
    msgs.gpsonoff = false;
  }

  pub_userparam.publish(msgs);
}

// void Driving::EnterPressEvent(QKeyEvent *event)
// {

// }




