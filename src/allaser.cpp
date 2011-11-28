/**
 * @author pinkasfeld joseph
 * Copyright (c) Aldebaran Robotics 2010, 2011 All Rights Reserved.
 */

#include "allaser.h"

#include <iostream>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <sstream>

#include <pthread.h>
#include <signal.h>
#include <cmath>

#include <qi/log.hpp>

#if defined (__linux__)
#include <sys/prctl.h>
#endif

extern "C" {
# include "urg_ctrl.h"
# include "scip_handler.h"
}

#define MODE_ON 0
#define MODE_OFF 1
#define SEND_MODE_ON 2
#define SEND_MODE_OFF 3

#define MIDDLE_ANGLE 384
#define DEFAULT_MIN_ANGLE 44
#define DEFAULT_MAX_ANGLE 725
#define MIN_ANGLE_LASER 44
#define MAX_ANGLE_LASER 725
#define RESOLUTION_LASER 1024
#define MIN_LENGTH_LASER 20
#define MAX_LENGTH_LASER 5600


#include <sys/time.h>

using namespace AL;
using namespace std;
pthread_t urgThreadId;
boost::shared_ptr<ALMemoryProxy> gSTM;
static int mode = MODE_ON;
static urg_t urg;
static int angle_min = DEFAULT_MIN_ANGLE;
static int angle_max = DEFAULT_MAX_ANGLE;
static int length_min = MIN_LENGTH_LASER;
static int length_max = MAX_LENGTH_LASER;

static void urg_exit(urg_t *urg, const char *message) {
  qiLogInfo("hardware.allaser", "%s: %s\n", message, urg_error(urg));
  urg_disconnect(urg);
  qiLogInfo("hardware.allaser", "urg_exit\n");
  pthread_exit((void *)NULL);
}

#define URG_DEFAULT_SPEED (19200)
#define URG_FAST_SPEED (115200)

void * urgThread(void * arg) {

#if defined (__linux__)
  // thread name
  prctl(PR_SET_NAME, "ALLaser::urgThread", 0, 0, 0);
#endif

  ALValue urgdata;
  long *data = NULL;
  int data_max;
  int ret;
  int n;
  int i,imemory,refTime,end,sampleTime, beginThread;
  std::string valueName="Device/Laser/Value";

  /* Connection */

  connectToLaser();

  /* Reserve the Receive data buffer */
  data_max = urg_dataMax(&urg);
  if (data_max <= 0) {
    perror("data_max is less than 0");
    pthread_exit((void *)NULL);
  }
  data = (long*)malloc(sizeof(long) * data_max);
  memset(data, 0, sizeof(long) * data_max);

  if (data == NULL) {
    perror("data buffer");
    pthread_exit((void *)NULL);
  }
  /* prepare ALValue for ALMemory*/
  urgdata.arraySetSize(data_max);
  for (i=0; i< data_max;i++)
  {
    urgdata[i].arraySetSize(4);
    urgdata[i][0]= (int)0;
    urgdata[i][1]= (double)0.0;
    urgdata[i][2]= (int)0;
    urgdata[i][3]= (int)0;
  }
  /* insert ALvalue in ALMemory*/

  gSTM->insertData(valueName,urgdata);

  gSTM->insertData("Device/Laser/LaserEnable", (bool) 1);
  gSTM->insertData("Device/Laser/MinAngle",(float)((2.0 * M_PI)
          * (DEFAULT_MIN_ANGLE - MIDDLE_ANGLE) / RESOLUTION_LASER));
  gSTM->insertData("Device/Laser/MaxAngle",(float)((2.0 * M_PI)
          * (DEFAULT_MAX_ANGLE - MIDDLE_ANGLE) / RESOLUTION_LASER));
  gSTM->insertData("Device/Laser/MinLength",(float)(length_min));
  gSTM->insertData("Device/Laser/MaxLength",(float)(length_max));

  stringstream ss;
  ss << "ALLaser running";
  qiLogInfo("hardware.laser") << ss.str() << std::endl;

  while(1)
  {
    if(mode==SEND_MODE_ON){
      ret = urg_laserOn(&urg);
      if (ret < 0) {
        urg_exit(&urg, "urg_requestData()");
      }
      mode=MODE_ON;
      /* Connection */
      connectToLaser();
    }
    if(mode==SEND_MODE_OFF){
      scip_qt(&urg.serial_, NULL, ScipNoWaitReply);
      mode=MODE_OFF;
    }
    if(mode==MODE_ON){
      /* Request Data using GD-Command */
      ret = urg_requestData(&urg, URG_GD, URG_FIRST, URG_LAST);
      if (ret < 0) {
        urg_exit(&urg, "urg_requestData()");
      }

      refTime = getLocalTime();
      /* Obtain Data */
      n = urg_receiveData(&urg, data, data_max);
      qiLogDebug("hardware.laser") << " n " << n << " expected " <<
            angle_max - angle_min << std::endl;
      if (n < 0) {
        urg_exit(&urg, "urg_receiveData()");
      }
      end= getLocalTime();
      sampleTime=end-refTime;

      imemory=0;
      for (i = 0; i < n; ++i) {
        int x, y;
        double angle = urg_index2rad(&urg, i);

        qiLogDebug("hardware.laser") << i << " angle " << angle <<
              " urgAngle " << urg_index2rad(&urg, i) <<
              " dist " << data[i] << std::endl;

        int length = data[i];

        if( length >= length_min && length <= length_max
            && i >= angle_min && i <= angle_max ){
          x = (int)((double)length * cos(angle));
          y = (int)((double)length * sin(angle));
          urgdata[imemory][0]= length;
          urgdata[imemory][1]= angle;
          urgdata[imemory][2]= x;
          urgdata[imemory][3]= y;
          imemory++;
        }
      }
      for(;imemory<data_max;imemory++){
        urgdata[imemory][0]= 0;
        urgdata[imemory][1]= 0;
        urgdata[imemory][2]= 0;
        urgdata[imemory][3]= 0;
      }
      gSTM->insertData(valueName,urgdata);
      usleep(1000);
    }else{
      usleep(10000);
    }
  }
  urg_disconnect(&urg);

  free(data);
}



//______________________________________________
// constructor
//______________________________________________
ALLaser::ALLaser(boost::shared_ptr<ALBroker> pBroker, const std::string& pName ): ALModule(pBroker, pName )
{
  setModuleDescription( "Allow control over Hokuyo laser when available on Nao's head." );

  functionName("laserOFF", "ALLaser", "Disable laser light");
  BIND_METHOD( ALLaser::laserOFF );

  functionName("laserON", "ALLaser", "Enable laser light and sampling");
  BIND_METHOD( ALLaser::laserON );

  functionName("setOpeningAngle", "ALLaser", "Set openning angle of the laser");
  addParam("angle_min_f", "float containing the min value in rad, this value must be upper than -2.35619449 ");
  addParam("angle_max_f", "float containing the max value in rad, this value must be lower than 2.092349795 ");
  addMethodExample( "python",
        "# Set the opening angle at -90/90 degres\n"
        "laser = ALProxy(\"ALLaser\",\"127.0.0.1\",9559)\n"
        "laser.setOpeningAngle(-1.570796327,1.570796327)\n"
      );
  BIND_METHOD( ALLaser::setOpeningAngle );

  functionName("setDetectingLength", "ALLaser", "Set detection threshold of the laser");
  addParam("length_min_l", "int containing the min length that the laser will detect(mm), this value must be upper than 20 mm");
  addParam("length_max_l", "int containing the max length that the laser will detect(mm), this value must be lower than 5600 mm");
  addMethodExample( "python",
        "# Set detection threshold at 500/3000 mm\n"
        "laser = ALProxy(\"ALLaser\",\"127.0.0.1\",9559)\n"
        "laser.setDetectingLength(500,3000)\n"
      );
  BIND_METHOD( ALLaser::setDetectingLength );

  // get broker on DCM and ALMemory
  try {
    gSTM = getParentBroker()->getMemoryProxy();
  } catch(ALError& e) {
    qiLogError("hardware.allaser")
        << "Could not connect to Memory. Error: " << e.what() << std::endl;
  }

  pthread_create(&urgThreadId, NULL, urgThread, NULL);
}

//______________________________________________
// destructor
//______________________________________________
ALLaser::~ALLaser()
{
  pthread_cancel(urgThreadId);
}

void ALLaser::laserOFF(void){
  mode = SEND_MODE_OFF;
  gSTM->insertData("Device/Laser/LaserEnable",(bool)0);
}

void ALLaser::laserON(void){
  mode = SEND_MODE_ON;
  gSTM->insertData("Device/Laser/LaserEnable",(bool)1);
}

void ALLaser::setOpeningAngle(const float& angle_min_f,
                              const float& angle_max_f){
  angle_min = MIDDLE_ANGLE + (int)(RESOLUTION_LASER
                                   * (float)angle_min_f / (2.0 * M_PI));
  angle_max = MIDDLE_ANGLE + (int)(RESOLUTION_LASER*(float)angle_max_f
                                   / (2.0 * M_PI));
  if(angle_min<MIN_ANGLE_LASER) angle_min = MIN_ANGLE_LASER;
  if(angle_max>MAX_ANGLE_LASER) angle_max = MAX_ANGLE_LASER;
  if(angle_min>=angle_max){
    angle_min = MIN_ANGLE_LASER;
    angle_max = MAX_ANGLE_LASER;
  }
  gSTM->insertData("Device/Laser/MinAngle",(float)((2.0 * M_PI)
          * (angle_min - MIDDLE_ANGLE) / RESOLUTION_LASER));
  gSTM->insertData("Device/Laser/MaxAngle",(float)((2.0 * M_PI)
          * (angle_max - MIDDLE_ANGLE) / RESOLUTION_LASER));
}

void ALLaser::setDetectingLength(const int& length_min_l,
                                 const int& length_max_l){
  length_min=(int)length_min_l;
  length_max=(int)length_max_l;
  if(length_min<MIN_LENGTH_LASER) length_min = MIN_LENGTH_LASER;
  if(length_max>MAX_LENGTH_LASER) length_min = MAX_LENGTH_LASER;
  if(length_min>=length_max){
    length_min = MIN_LENGTH_LASER;
    length_max = MAX_LENGTH_LASER;
  }
  gSTM->insertData("Device/Laser/MinLength",(int)length_min);
  gSTM->insertData("Device/Laser/MaxLength",(int)length_max);
}


//_________________________________________________
// Service functions
//_________________________________________________


int connectToLaserViaUSB(void) {
  int success = -1;
  const char deviceUSB[] = "/dev/ttyUSB0"; /* Example when using Linux  */
  // Try to connect at low speed.
  success = urg_connect(&urg, deviceUSB, URG_DEFAULT_SPEED);
  if (success < 0) {
    std::stringstream ss;
    ss << "Connection failure to " << deviceUSB << " at "
        << URG_DEFAULT_SPEED << ". " << urg_error(&urg);
    qiLogDebug("hardware.allaser") << ss.str() << std::endl;
    return success;
  }
  else {
    // Try to switch to full speed.
    scip_ss(&urg.serial_, URG_FAST_SPEED);
    urg_disconnect(&urg);
    success = urg_connect(&urg, deviceUSB, URG_FAST_SPEED);
    if (success < 0) {
      std::stringstream ss;
      ss << "Connection failure to " << deviceUSB << " at "
          << URG_FAST_SPEED << ". " << urg_error(&urg);
      qiLogDebug("hardware.allaser") << ss.str() << std::endl;
      // Fall back to low speed.
      success = urg_connect(&urg, deviceUSB, URG_DEFAULT_SPEED);
      return success;
    }
    else {
      return success;
    }
  }
}

int connectToLaserViaACM(void) {
  int success = -1;
  const char deviceACM[] = "/dev/ttyACM0"; /* Example when using Linux  */
  success = urg_connect(&urg, deviceACM, URG_DEFAULT_SPEED);
  if (success < 0) {
    std::stringstream ss;
    ss << "Connection failure to " << deviceACM << " at "
        << URG_DEFAULT_SPEED << ". " << urg_error(&urg);
    qiLogDebug("hardware.allaser") << ss.str() << std::endl;
    return success;
  }
  else {
    // Try to switch to full speed.
    scip_ss(&urg.serial_, URG_FAST_SPEED);
    urg_disconnect(&urg);
    success = urg_connect(&urg, deviceACM, URG_FAST_SPEED);
    if (success < 0)
    {
      std::stringstream ss;
      ss << "Connection failure from " << deviceACM << " at "
          << URG_FAST_SPEED << ". " << urg_error(&urg);
      qiLogDebug("hardware.allaser") << ss.str() << std::endl;
      // Fall back to low speed.
      success = urg_connect(&urg, deviceACM, URG_DEFAULT_SPEED);
      return success;
    }
    else {
      return success;
    }
  }
}


void connectToLaser(void){
  int success = connectToLaserViaUSB();
  if (success < 0) {
    std::stringstream ss;
    ss << "Connection failure to USB, trying ACM... ";
    qiLogDebug("hardware.allaser") << ss.str() << std::endl;
    success = connectToLaserViaACM();
    if (success < 0) {
      std::stringstream ss;
      ss << "Connection to laser failed";
      qiLogDebug("hardware.allaser") << ss.str() << std::endl;
      pthread_exit((void *)NULL);
    }
    else {
      std::stringstream ss;
      ss << "Connection to laser via ACM";
      qiLogDebug("hardware.allaser") << ss.str() << std::endl;
    }
  }
  else {
    std::stringstream ss;
    ss << "Connection to laser via USB";
    qiLogDebug("hardware.allaser") << ss.str() << std::endl;
  }
}

unsigned int getLocalTime(void){
  struct timeval tv;
  unsigned int val;

  gettimeofday(&tv, NULL);

  val = (unsigned int)((unsigned int)(tv.tv_usec/1000)
                       + (unsigned int)(tv.tv_sec*1000));

  // Time in ms
  return val;
}

double index2rad(int index){

  double radian = (2.0 * M_PI) *
                  (index - MIDDLE_ANGLE) / RESOLUTION_LASER;

  return radian;
}
