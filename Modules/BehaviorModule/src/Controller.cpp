#include <Controller.h>

void motionBody(PolyDriver* driver, vector<double> encoders){
    IPositionControl *position;
    if (!driver->view(position)) {
        return;
    }
    int n=0;
    position->getAxes(&n);

    for(int i = 0; i < encoders.size(); i++){
        position->setRefSpeed(i,Info::velocity);
        position->positionMove(i,encoders[i]);
    }
    return;
}

MotorsThread::MotorsThread(PolyDriver* driver, vector<double> encoders) {
	this->driver = driver;
	this->encoders = encoders;
}

MotorsThread::MotorsThread(PolyDriver* driver, double encoder) {
    this->driver = driver;
    this->encoders.push_back(encoder);
}

void MotorsThread::motion(){
    api_caller = new boost::thread(::motionBody, driver, encoders);
}

bool MotorsThread::join(){
    if (!api_caller->timed_join(boost::posix_time::seconds(TIMEOUT_MOTION))) {
        return false;
    }
    return true;
}

int Guardian::lastError = UNKNOWN;

Guardian::Guardian(string name, int batteryIndex, double batteryLimit, string batteryAlarm,
                   int accIndex, double accLimit){ // with battery status check
   this->name = name;
   this->batteryIndex = batteryIndex;
   this->accIndex = accIndex;
   this->batteryLimit = batteryLimit;
   this->accLimit = accLimit;
   this->batteryAlarm = batteryAlarm;
   this->fall = false;
}

/**
* Guardian main loop.
*/
void Guardian::run(){

  // ** Torque error avoidance **
  TorqueControl t;
  t.start();
  // **

  int i = 0;
  int noerr = 0;
  double offs = 0;
  int lowBattery = 0;
  Info::sensorPort->setStrict();
  while (yarp::os::Thread::isStopping() != true) {
      while (Info::sensorPort->getPendingReads() == 0) {
          Time::delay(0.2);
      }
      Bottle* b = Info::sensorPort->read(true);
      double v = b->get(batteryIndex).asDouble();
      int err = b->get(b->size() - 1).asDouble();

      // ** Robot fault detection **
      // Only after several UNKNOWN, i actually put the fault to UNKNOWN because
      // this code only means that no fault are deteced.
      if (err != UNKNOWN) {
           Guardian::lastError = err;
           noerr = 0;
      }
      else {
           if (noerr > SWITCHUNKNOWN && Guardian::lastError != UNKNOWN) {
                // if I saw no error for long time, but a fault is detected I consider a unknown origin
                Guardian::lastError = UNKNOWN;
           }
           noerr++;
      }

      // ** Battery status check **
      if (i % 500 == 0) cout << "Battery status: " << v/10 << " V" << endl;
      if(v < batteryLimit) {
          lowBattery++;
      }
      if (lowBattery > 5) { //sometimes the voltage goes down only temporarily
          cout << "LOW BATTERY: " << v/10 << " V" << endl;
          if (batteryAlarm == "") {
              system("speaker-test -t sine -f 300 -p 2000 -l 1 2>&1");
          }
          else {
              string tool = "mpg123";
              system((tool + " " + batteryAlarm).c_str());
          }
          lowBattery = 0;
      }
      // **

      // ** Fall detection **
      if (!fall) {
          vector<double> acc;
          acc.push_back(b->get(accIndex).asDouble());
          acc.push_back(b->get(accIndex + 1).asDouble());
          acc.push_back(b->get(accIndex + 2).asDouble());
          double tot = 0;
          for (int i = 0; i < 3; i++) {
              tot += acc[i]*acc[i];
          }
          if (i < 20) {
              offs += sqrt(tot);
          }
          else {
              if(i == 20){
                  cout << "Fall detection started" << endl;
                  offs /= 20;
              }
              if (abs(offs - sqrt(tot)) > accLimit) {
                  cout << "FALL" << endl;
                  fall = true;
              }
          }
      }
      // **
      i++;
  }
  t.stop();

}
