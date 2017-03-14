#include "LeapMotion/hands_listener.h"
#include "KsGlobal.h"

#define CDESCENDING_ZDELTA (0.5)
using namespace Leap;
using namespace std;

VoidCallback HandsListener::_emitFingerPosUpdatedCallBack = nullptr;
FingerGestureCallback HandsListener::_emitFingerGestureCallBack[HandsListener::FINGER_GESTURE_TOTAL];

HandsListener::HandsListener(ros::NodeHandle* nodeHandle):
               _node_handle(nodeHandle)
{

}

HandList& HandsListener::getHands()
{
    return _hands;
}

FingerList HandsListener::getFingerList(int hand_id)
{
    if(hand_id < 0 || hand_id >= _hands.count())
        return FingerList();

    return _hands[hand_id].fingers();
}

std::vector<Vector> HandsListener::getFingerTipsPoses(int hand_id)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if(hand_id < 0 || hand_id >= _hands.count())
        return std::vector<Vector>();

    std::vector<Vector> fingerTipsPoses;
    FingerList fingerList = getFingerList(hand_id);

    for(int i = 0; i < fingerList.count(); i++) {
        fingerTipsPoses.push_back(fingerList[i].tipPosition()); // tipVelocity
    }
    return fingerTipsPoses;
}

std::vector<std::vector<double>> HandsListener::getFingerJointValues(int hand_id)
{
    std::lock_guard<std::mutex> lock(_mutex);

    std::vector<std::vector<double>> fingerJointValues;
    if(hand_id < 0 || hand_id >= _hands.count()) {
        return fingerJointValues;
    }

    // -------------------------------------------------------------------------------------------
    Vector palmNormal =_hands[hand_id].palmNormal();
    FingerList fingerList = getFingerList(hand_id);
    static std::vector<Vector> abdJointNormalVectors;
    static std::vector<double> proximalJointValues;
    proximalJointValues.reserve(fingerList.count());
    abdJointNormalVectors.reserve(fingerList.count());

    for(int i = 0; i < fingerList.count(); i++) {
        std::vector<double> subFingerJointValues;
        //
        // ABD JOINT (Finger root)
        // !NOTE: THUMB DOES NOT HAVE DISTAL BONE!!! BUT LEAP MOTION DO THE INVERSE, IGNORING METACARPAL INSTEAD OF DISTAL FOR THUMB!
        Vector metacarpalLinkVector = fingerList[i].bone(i == 0 ? Bone::TYPE_PROXIMAL     : Bone::TYPE_METACARPAL).
                                                    direction();
        Vector proximalLinkVector   = fingerList[i].bone(i == 0 ? Bone::TYPE_INTERMEDIATE : Bone::TYPE_PROXIMAL).
                                                    direction();
        double proximalJointValue   = proximalLinkVector.angleTo(metacarpalLinkVector);


        proximalJointValues[i]      = proximalJointValue;
        // ---------------------------------------------------------------------------------
        if(i == 0) {
            subFingerJointValues.push_back(metacarpalLinkVector.
                                           angleTo(fingerList[1].bone(Bone::TYPE_METACARPAL).direction()));

            abdJointNormalVectors[i] = metacarpalLinkVector;
        }
        else {
            Vector tempVector1 = (i < 4) ? (fingerList[i+1].bone(Bone::TYPE_METACARPAL).nextJoint() - fingerList[i].bone(Bone::TYPE_METACARPAL).nextJoint()):
                                           (fingerList[i-1].bone(Bone::TYPE_METACARPAL).nextJoint() - fingerList[i].bone(Bone::TYPE_METACARPAL).nextJoint());

            Matrix rotationMatrix(tempVector1, proximalJointValue);
            Vector tempVector2 = rotationMatrix.transformDirection(metacarpalLinkVector);

            static double metacarpalJointValue = 0;
            double newMetacarpalJointValue = tempVector2.angleTo(proximalLinkVector);
            if(newMetacarpalJointValue < M_PI/10) {
                metacarpalJointValue = newMetacarpalJointValue;
                //
                //if(std::abs(proximalJointValue - proximalJointValues[i]) <= M_PI/30) {
                //
                bool test = (tempVector2.cross(proximalLinkVector).normalized().
                            dot(abdJointNormalVectors[i]) > 0);
                metacarpalJointValue *= (test ? 1 : -1);
                //
                abdJointNormalVectors[i] = tempVector2.cross(proximalLinkVector).normalized();
                //}
            }
            subFingerJointValues.push_back(metacarpalJointValue);
        }

        // FINGER JOINTS
        for(int j = HandsListener::HAND_BONE_FIRST; j < HandsListener::HAND_BONE_TOTAL-1; j++) {
            // Note -Angle(Bone[j], Bone[j+1]);
            if(i == 0 && j == HandsListener::HAND_BONE_METACARPAL) {
                continue; // LEAP MOTION IGNORES METACARPAL INSTEAD OF DISTAL FOR THUMB!
            }
            subFingerJointValues.push_back(-fingerList[i].bone((Bone::Type)(j)).direction().
                                           angleTo(fingerList[i].bone((Bone::Type)(j+1)).direction())
                                          );
        }
        //
        fingerJointValues.push_back(subFingerJointValues);
    }

    return fingerJointValues;
}

void HandsListener::onInit(const Controller& controller) {
    std::cout << "Leap Hands Initialized" << std::endl;
    _pub_marker_array = _node_handle->advertise<visualization_msgs::MarkerArray>(CLEAP_HANDS_TOPIC, 1);
    _pub_bone_only    = _node_handle->advertise<visualization_msgs::Marker>(CLEAP_HANDS_BONE_TOPIC, 1);
}

void HandsListener::onConnect(const Controller& controller) {
  std::cout << "Leap Hands Connected" << std::endl;
  controller.enableGesture(Gesture::TYPE_CIRCLE);
  controller.enableGesture(Gesture::TYPE_KEY_TAP);
  controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
  controller.enableGesture(Gesture::TYPE_SWIPE);
}

void HandsListener::onDisconnect(const Controller& controller) {
  // Note: not dispatched when running in a debugger.
  std::cout << "Leap Hands Disconnected" << std::endl;
}

void HandsListener::onExit(const Controller& controller) {
  std::cout << "Leap Hands Exited" << std::endl;
}

void HandsListener::onFrame(const Controller& controller) {
    std::lock_guard<std::mutex> lock(_mutex);
    //
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    //ROS_INFO("flags = %i", (int) controller.policyFlags());
    visualization_msgs::Marker marker_msg, joint_msg;
    visualization_msgs::MarkerArray marker_array_msg;

    marker_msg.header.frame_id=joint_msg.header.frame_id = CLEAP_BASE_FRAME;
    marker_msg.header.stamp=joint_msg.header.stamp=ros::Time::now();
    marker_msg.ns="leap_marker";
    joint_msg.ns="joint";
    marker_msg.id = 0;
    joint_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::LINE_LIST;
    joint_msg.type = visualization_msgs::Marker::SPHERE;
    marker_msg.scale.x = 0.1f;
    joint_msg.scale.x = joint_msg.scale.y = joint_msg.scale.z = 0.03f;
    joint_msg.color.r = .0f;
    joint_msg.color.g = 1.0f;
    joint_msg.color.b = 1.0f;
    joint_msg.color.a = 0.7f;
    marker_msg.action = joint_msg.action = visualization_msgs::Marker::ADD;
    marker_msg.lifetime = joint_msg.lifetime = ros::Duration(0.1);

    _hands = frame.hands();

    //!NOTE: SOMEHOW, WHEN WE EMIT A SIGNAL HERE  INSIDE THE CALLBACK, NO SLOT WOULD BE INVOKED!
    //this->emitFingerPosUpdated();

    for (HandList::const_iterator hl = _hands.begin(); hl != _hands.end(); ++hl) {
        // Get the first hand
        const Hand hand = *hl;

        // Get the Arm bone
        // Get fingers
        FingerList fingers = hand.fingers();
        for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
            const Finger finger = *fl;

            // Get finger bones
            for (int b = 0; b < 4; b++) {
                Bone::Type boneType = static_cast<Bone::Type>(b);
                Bone bone = finger.bone(boneType);

                geometry_msgs::Point point;
                if(b == 0) {
                    point.x = joint_msg.pose.position.x =  -bone.prevJoint().x/100;
                    point.y = joint_msg.pose.position.y = bone.prevJoint().z/100; // !NOTE: We interchange z & y here to make hand directed upward!
                    point.z = joint_msg.pose.position.z = bone.prevJoint().y/100 - CDESCENDING_ZDELTA;
                    marker_array_msg.markers.push_back(joint_msg);
                    marker_msg.points.push_back(point);
                }
                joint_msg.id++;
                point.x = joint_msg.pose.position.x =  -bone.nextJoint().x/100;
                point.y = joint_msg.pose.position.y = bone.nextJoint().z/100;     // !NOTE: We interchange z & y here to make hand directed upward!
                point.z = joint_msg.pose.position.z = bone.nextJoint().y/100 - CDESCENDING_ZDELTA;
                marker_msg.points.push_back(point);
                marker_array_msg.markers.push_back(joint_msg);

                std_msgs::ColorRGBA color;
                color.r = 1.0f; color.g=.0f; color.b=.0f, color.a=1.0f;
                marker_msg.colors.push_back(color);
                //marker_msg.colors.push_back(color);
            }
        }
    }

    // Send the markers to Rviz by publishing them over designated topics
    _pub_marker_array.publish(marker_array_msg);
    _pub_bone_only.publish(marker_msg);
    // -----------------------------------------------------------------------------------
    //
    static Vector lastPointFingerTipZ;
    if(_hands.count() > 0) {
        FingerList fingers = _hands[0].fingers();
        Vector tipPoint = fingers[1].bone(Bone::TYPE_DISTAL).nextJoint();
        float zTipPointDelta = tipPoint.z - lastPointFingerTipZ.z;

        int fingerGesture = (zTipPointDelta > 0.1)  ? FINGER_UP :
                            (zTipPointDelta <- 0.1) ? FINGER_DOWN : FINGER_DOWN_MOVE;
        std::vector<float> tipPointF;
        tipPointF.push_back(tipPoint.x); tipPointF.push_back(tipPoint.y); tipPointF.push_back(tipPoint.z - CDESCENDING_ZDELTA);
        this->emitFingerGesture(fingerGesture, tipPointF);
        lastPointFingerTipZ = tipPoint;
    }
}

void HandsListener::onFocusGained(const Controller& controller) {
  std::cout << "Leap Hands Focus Gained" << std::endl;
}

void HandsListener::onFocusLost(const Controller& controller) {
  std::cout << "Focus Lost" << std::endl;
}

void HandsListener::onDeviceChange(const Controller& controller) {
  std::cout << "Device Changed" << std::endl;
  const DeviceList devices = controller.devices();

  for (int i = 0; i < devices.count(); ++i) {
    std::cout << "id: " << devices[i].toString() << std::endl;
    std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
  }
}

void HandsListener::onServiceConnect(const Controller& controller) {
  std::cout << "Service Connected" << std::endl;
}

void HandsListener::onServiceDisconnect(const Controller& controller) {
  std::cout << "Service Disconnected" << std::endl;
}
