/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#ifndef ALLaser_H
#define ALLaser_H

#include <boost/shared_ptr.hpp>
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
    void setOpeningAngle(const float& angle_min_f, const float& angle_max_f);

    /**
      * Change detecting length
      **/
    void setDetectingLength(const int& length_min_l, const int& length_max_l);
};

void connectToLaser(void);
unsigned int getLocalTime(void);
double index2rad(int index);

#endif // ALLaser_H
