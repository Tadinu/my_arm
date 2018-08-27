#include <functional> // std::bind
#include <QThread>
#include <QVector3D>

#include "RbRobotSensorAdapter.h"
#include "RbGlobal.h"
#include "VREPAdapter.h"

RbRobotSensorAdapter* RbRobotSensorAdapter::_instance = nullptr;
RbRobotSensorAdapter* RbRobotSensorAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RbRobotSensorAdapter();
    }

    return _instance;
}

bool RbRobotSensorAdapter::checkInstance()
{
    return _instance != nullptr;
}

RbRobotSensorAdapter::RbRobotSensorAdapter():
                  _mutex(new QMutex())
{
}

RbRobotSensorAdapter::~RbRobotSensorAdapter()
{
    _mutex->tryLock(500);
    _mutex->unlock(); // futile if tryLock() failed!
    delete _mutex;
}

void RbRobotSensorAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RbRobotSensorAdapter::initialize()
{
    // Do sth else...

    // Initialize sensor agent list
    this->initializeSensorAgents();
}

void RbRobotSensorAdapter::initializeSensorAgents()
{
    // static to be passed into const RbSensorProperties&
    static RbSensorAgent::RbSensorProperties sensorPropList[RB_SENSOR_TOTAL-RB_SENSOR_FIRST] =
    {
        {RbGlobal::RB_SENSOR_TYPE_VISION,     RB_SENSOR_FRONT_VISION,     "frontCamSensor"                  },
        {RbGlobal::RB_SENSOR_TYPE_VISION,     RB_SENSOR_GROUND_VISION,    "groundCamSensor"                 },
        {RbGlobal::RB_SENSOR_TYPE_FLOOR,      RB_SENSOR_FLOOR,            "floorSensor"                     },
        {RbGlobal::RB_SENSOR_TYPE_FORCE,      RB_SENSOR_FORCE,            "bumpSensor"                      },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_1,     "Pioneer_p3dx_ultrasonicSensor1"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_2,     "Pioneer_p3dx_ultrasonicSensor2"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_3,     "Pioneer_p3dx_ultrasonicSensor3"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_4,     "Pioneer_p3dx_ultrasonicSensor4"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_5,     "Pioneer_p3dx_ultrasonicSensor5"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_6,     "Pioneer_p3dx_ultrasonicSensor6"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_7,     "Pioneer_p3dx_ultrasonicSensor7"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_8,     "Pioneer_p3dx_ultrasonicSensor8"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_9,     "Pioneer_p3dx_ultrasonicSensor9"  },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_10,    "Pioneer_p3dx_ultrasonicSensor10" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_11,    "Pioneer_p3dx_ultrasonicSensor11" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_12,    "Pioneer_p3dx_ultrasonicSensor12" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_13,    "Pioneer_p3dx_ultrasonicSensor13" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_14,    "Pioneer_p3dx_ultrasonicSensor14" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_15,    "Pioneer_p3dx_ultrasonicSensor15" },
        {RbGlobal::RB_SENSOR_TYPE_ULTRASONIC, RB_SENSOR_ULTRASONIC_16,    "Pioneer_p3dx_ultrasonicSensor16" },
        {RbGlobal::RB_SENSOR_TYPE_TACTILE,    RB_SENSOR_MAIN_BODY_TACTILE, "mainBodyTactileSensor"          },
        {RbGlobal::RB_SENSOR_TYPE_TACTILE,    RB_SENSOR_HEAD_TACTILE,      "headTactileSensor"              }
    };

    for(int i = RB_SENSOR_FIRST; i < RB_SENSOR_TOTAL; i++) {
        _sensorAgentList << new RbSensorAgent(sensorPropList[i]);
    }
}

void RbRobotSensorAdapter::runSensorOperation()
{
    for(int i = 0; i < _sensorAgentList.size(); i++) {
        _sensorAgentList[i]->startThreading();
    }
}

RbSensorAgent* RbRobotSensorAdapter::sensorAgent(int sensorId)
{
    for(int i = 0; i < _sensorAgentList.size(); i++) {
        if(_sensorAgentList[i]->id() == sensorId) {
            return _sensorAgentList[i];
        }
    }
    return nullptr;
}

bool RbRobotSensorAdapter::isHalted()
{
    int haltedSensorId = -1;
    for(int i = 0; i < _sensorAgentList.size(); i++) {
        if(_sensorAgentList[i]->isHalted()) {
            return haltedSensorId;
        }
    }

    return haltedSensorId;
}

void RbRobotSensorAdapter::setVisionSensorImage(int sensorId, const QImage& image)
{
    if(sensorId == RbRobotSensorAdapter::RB_SENSOR_FRONT_VISION)
        _frontVisionSensorImage = image;
    else if(sensorId == RbRobotSensorAdapter::RB_SENSOR_GROUND_VISION)
        _groundVisionSensorImage = image;
}

QImage& RbRobotSensorAdapter::getVisionSensorImage(int sensorId)
{
    return (sensorId == RbRobotSensorAdapter::RB_SENSOR_FRONT_VISION)  ? _frontVisionSensorImage  :
           /*(sensorId == RbRobotSensorAdapter::RB_SENSOR_GROUND_VISION) ? */_groundVisionSensorImage;
}
