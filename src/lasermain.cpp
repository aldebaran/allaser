/*
 * Copyright (c) 2012 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

#ifndef _WIN32
#include <signal.h>
#endif

#include <boost/shared_ptr.hpp>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>

using namespace std;
using namespace AL;

#include "allaser.h"

extern "C"
{

int _createModule( boost::shared_ptr<ALBroker> pBroker )
{
  // init broker with the main broker instance
  // from the parent executable
  ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
  ALBrokerManager::getInstance()->addBroker(pBroker);
  ALModule::createModule<ALLaser>(pBroker,"ALLaser" );
  return 0;
}

} // extern "C"
