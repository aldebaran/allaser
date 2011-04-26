/**
 *
 * \section Author
 * @author Pinkasfeld Joseph
 */

#ifndef _WIN32
#include <signal.h>
#endif

#include <alcore/alptr.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

using namespace std;
using namespace AL;

#include "allaser.h"

#ifndef LASER_IS_REMOTE

#ifdef _WIN32
#define ALCALL __declspec(dllexport)
#else
#define ALCALL
#endif
#else
#define ALCALL
#endif

extern "C"
{
ALCALL int _createModule( boost::shared_ptr<ALBroker> pBroker )
{
  // init broker with the main broker instance
  // from the parent executable
  ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
  ALBrokerManager::getInstance()->addBroker(pBroker);
  ALModule::createModule<ALLaser>(pBroker,"ALLaser" );
  return 0;
}

ALCALL int _closeModule(  )
{
  return 0;
}
}


#ifdef LASER_IS_REMOTE
int main(int argc, char *argv[] )
{
  // pointer on createModule
  TMainType sig;
  sig = &_createModule;
  // call main
  ALTools::mainFunction("laser",argc, argv,sig);
}
#endif
