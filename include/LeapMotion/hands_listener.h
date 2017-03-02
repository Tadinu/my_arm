#ifndef HANDS_LISTENER_H
#define HANDS_LISTENER_H

#include <iostream>
#include <string.h>
#include "Leap.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <vector>
#include <sstream>
#include <thread>
#include <mutex>

using namespace Leap;
using namespace std;

#define CLEAP_HANDS_TOPIC      ("hands")
#define CLEAP_HANDS_BONE_TOPIC ("hands_line")

#include <functional>
typedef std::function<void ()> VoidCallback;
typedef std::function<void (const std::vector<float>&)> FingerGestureCallback;

class HandsListener : public Listener
{
public:
    enum HAND_BONE_TYPE {
        HAND_BONE_FIRST        = 0,
        HAND_BONE_METACARPAL   = Bone::TYPE_METACARPAL,   /**< Bone connected to the wrist inside the palm */
        HAND_BONE_PROXIMAL     = Bone::TYPE_PROXIMAL,     /**< Bone connecting to the palm */
        HAND_BONE_INTERMEDIATE = Bone::TYPE_INTERMEDIATE, /**< Bone between the tip and the base*/
        HAND_BONE_DISTAL       = Bone::TYPE_DISTAL,       /**< Bone at the tip of the finger */
        HAND_BONE_TOTAL
    };

    enum FINGER_GESTURE {
        FINGER_UP,
        FINGER_DOWN,
        FINGER_DOWN_MOVE,

        FINGER_GESTURE_TOTAL
    };

    static void regEmitFingerGestureCallback(int gestureId, FingerGestureCallback callback) {
        if(gestureId < 0 || gestureId >= FINGER_GESTURE_TOTAL)
            return;
        _emitFingerGestureCallBack[gestureId] = callback;
    }
    static void unregEmitFingerPosUpdatedCallback(int gestureId) {
        if(gestureId < 0 || gestureId >= FINGER_GESTURE_TOTAL)
            return;
        _emitFingerGestureCallBack[gestureId] = nullptr;
    }
    void emitFingerGesture(int gestureId, const std::vector<float>& tipPoint) {
        if(gestureId < 0 || gestureId >= FINGER_GESTURE_TOTAL)
            return;
        if(_emitFingerGestureCallBack[gestureId])
            _emitFingerGestureCallBack[gestureId](tipPoint);
    }
    // -------------------------------------------------------------------
    static void regEmitFingerPosUpdatedCallback(VoidCallback callback) {
        _emitFingerPosUpdatedCallBack = callback;
    }
    static void unregEmitFingerPosUpdatedCallback() {
        _emitFingerPosUpdatedCallBack = nullptr;
    }

    void emitFingerPosUpdated() {
        if(_emitFingerPosUpdatedCallBack) {
            //std::this_thread::sleep_for(std::chrono::milliseconds(500));
            _emitFingerPosUpdatedCallBack();
        }
    }
    // -------------------------------------------------------------------
    HandsListener(ros::NodeHandle* nodeHandle);

    unsigned int seq;
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);

    HandList& getHands();
    FingerList getFingerList(int hand_id);
    std::vector<std::vector<double>> getFingerJointValues(int hand_id);

private:
    static VoidCallback _emitFingerPosUpdatedCallBack;
    static FingerGestureCallback _emitFingerGestureCallBack[FINGER_GESTURE_TOTAL];

    ros::NodeHandle* _node_handle;
    ros::Publisher _pub_marker_array;
    ros::Publisher _pub_bone_only;

    HandList _hands;
    std::mutex _mutex;
};

#endif // HANDS_LISTENER_H
