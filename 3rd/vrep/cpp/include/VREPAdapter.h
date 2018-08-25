#ifndef _ROBOT_VREP_ADAPTER_H_
#define _ROBOT_VREP_ADAPTER_H_

#include <QtCore>
#include <QMutex>
#include <QVector3D>

extern "C" {
    #include "extApi.h"
}

#define VREP_INSTANCE() VREPAdapter::getInstance()

class VREPAdapter : public QObject {
    Q_OBJECT

    // http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
    //enum {
    //    simx_return_ok (0) The function executed fine
    //    simx_return_novalue_flag (1 (i.e. bit 0)) There is no command reply in the input buffer. This should not always be considered as an error, depending on the selected operation mode
    //    simx_return_timeout_flag (2 (i.e. bit 1)) The function timed out (probably the network is down or too slow)
    //    simx_return_illegal_opmode_flag (4 (i.e. bit 2)) The specified operation mode is not supported for the given function
    //    simx_return_remote_error_flag (8 (i.e. bit 3)) The function caused an error on the server side (e.g. an invalid handle was specified)
    //    simx_return_split_progress_flag (16 (i.e. bit 4)) The communication thread is still processing previous split command of the same type
    //    simx_return_local_error_flag (32 (i.e. bit 5)) The function caused an error on the client side
    //    simx_return_initialize_error_flag (64 (i.e. bit 6)) simxStart was not yet called
    //};

public:
    // V-REP INFO -----------------------------------------
    //
    int initializeVREP();
    void finalizeVREP();
    void startSimulation(int clientID);
    void endSimulation(int clientID);

    void lock();
    void unlock();

    // V-REP COPPELIA ++
    // Source: ~/V-REP/programming/remoteApiBindings/lib
    void testConnectionToServer();
    void testCommands();
    // V-REP COPPELIA --

    bool isConnectedToServer();
    int vrepRobotHandle() const { return _vrepRobotHandle; }
    int vrepClientId()    const { return _vrepClientId;    }

    // Service member functions ------------------------------------
    QVector3D getObjectWorldPosition(const char* objectName);
    QVector3D getObjectOrientation(const char* objectName);
    float getObjectVelocity(const char* objectName);

public:
    static VREPAdapter* getInstance();
    ~VREPAdapter();
    static void deleteInstance();
    static bool checkInstance();

    int waitForCommandSentToServer();
signals:

private:
    VREPAdapter();
    static VREPAdapter* _instance;

    int _vrepRobotHandle;
    int _vrepClientId;

    // Mutex to allow only one agent threat to make a call to V-REP API Server at a time!
    QMutex* _vrepMutex;
};

#endif // _ROBOT_VREP_ADAPTER_H_

