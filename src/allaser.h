/**
 * @author pinkasfeld joseph
 * Copyright (c) Aldebaran Robotics 2010 All Rights Reserved.
 */

#ifndef ALLaser_H
#define ALLaser_H

#include <alcore/alptr.h>
#include <alcommon/almodule.h>

namespace AL
{
  class ALBroker;
}


class ALLaser : public AL::ALModule
{

  public:

    /**
     * Default Constructor.
     */
    ALLaser(boost::shared_ptr<AL::ALBroker> pBroker, const std::string& pName );

    /**
     * Destructor.
     */
    virtual ~ALLaser();


    /**
      * Put the laser in OFF mode
      **/
    void laserOFF(void);

    /**
      * Put the laser in ON mode
      **/
    void laserON(void);

    /**
      * Change openning angle
      **/
    void setOpeningAngle(const AL::ALValue& angle_min_f, const AL::ALValue& angle_max_f);

    /**
      * Change detecting length
      **/
    void setDetectingLength(const AL::ALValue& length_min_l,const AL::ALValue& length_max_l);
};

void connectToLaser(void);
uInt32 getLocalTime(void);
double index2rad(int index);

#endif // ALLaser_H
