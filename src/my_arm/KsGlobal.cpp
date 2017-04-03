#include <QXmlStreamWriter>
#include <QDesktopWidget>

//#include <QMutexLocker>
#include <QWriteLocker>
#include <QFuture>
#include <QtConcurrent/QtConcurrentRun>

#include <QCryptographicHash> // For hashing the user info
#include <QDebug>

#include "KsGlobal.h"

const char* KsGlobal::CBRHAND_ARM_JOINTS[KsGlobal::VBRHAND_ARM_JOINT_TOTAL] = {
    ("base_joint"),                // Revolute   Z : base_link <-> body1z
    ("j2"),                        // Continuous Y : body1     <-> body10
    ("j20"),                       // Fixed        : body10    <-> body2
    ("j3"),                        // Continuous Y : body2     <-> body20
    ("j30"),                       // Fixed        : body20    <-> body3
    ("j4"),                        // Revolute   Z : body3     <-> brHand

    ("finger_1_prox_joint"),
    ("finger_1_med_joint"),
    ("finger_1_dist_joint"),

    ("finger_2_prox_joint"),
    ("finger_2_med_joint"),
    ("finger_2_dist_joint"),

    ("finger_3_med_joint"),
    ("finger_3_dist_joint")
};

const char* KsGlobal::CBRHAND_ARM_LINKS[KsGlobal::VBRHAND_ARM_JOINT_TOTAL+1] = {
    CBASE_LINK,
    ("body2"),
    ("body20"),
    ("body3"),
    ("body30"),
    ("body4"),

    ("brHand"),
    ("finger_1_prox_link"),
    ("finger_1_med_liink"),
    ("finger_1_dist_link"),

    ("finger_2_prox_link"),
    ("finger_2_med_link"),
    ("finger_2_dist_link"),

    ("finger_3_med_link"),
    ("finger_3_dist_link")
};

const char* KsGlobal::CPISA_SOFT_HAND_ARM_JOINTS[KsGlobal::VPISA_SOFT_HAND_ARM_JOINT_TOTAL] = {
#ifndef USING_PISA_SOFT_HAND_ONLY
    ("base_joint"),                // Revolute   Z : base_link <-> body1z
    ("j2"),                        // Continuous Y : body1     <-> body10
    ("j20"),                       // Fixed        : body10    <-> body2
    ("j3"),                        // Continuous Y : body2     <-> body20
    ("j30"),                       // Fixed        : body20    <-> body3
    ("j4"),                        // Revolute   Z : body3     <-> brHand
#endif
    // PACMAN VERION ALREADY INCLUDES THE COUPLER, CLAMP AND BASE
    //
    //("softHandWrist_softHand_kuka_coupler_joint"),
    //("softHandWrist_softHand_kuka_coupler_base_joint"),
    //
    //("softHand_kuka_coupler_softHand_clamp_joint"),
    //

    ("softHand_palm_joint"),

    ("softHand_synergy_joint"),

    ("softHand_thumb_abd_joint"),
    ("softHand_thumb_inner_joint"),
    ("softHand_thumb_inner_joint_mimic"),
    ("softHand_thumb_outer_joint"),
    ("softHand_thumb_outer_joint_mimic"),

    ("softHand_index_abd_joint"),
    ("softHand_index_inner_joint"),
    ("softHand_index_inner_joint_mimic"),
    ("softHand_index_middle_joint"),
    ("softHand_index_middle_joint_mimic"),
    ("softHand_index_outer_joint"),
    ("softHand_index_outer_joint_mimic"),

    ("softHand_middle_abd_joint"),
    ("softHand_middle_inner_joint"),
    ("softHand_middle_inner_joint_mimic"),
    ("softHand_middle_middle_joint"),
    ("softHand_middle_middle_joint_mimic"),
    ("softHand_middle_outer_joint"),
    ("softHand_middle_outer_joint_mimic"),

    ("softHand_ring_abd_joint"),
    ("softHand_ring_inner_joint"),
    ("softHand_ring_inner_joint_mimic"),
    ("softHand_ring_middle_joint"),
    ("softHand_ring_middle_joint_mimic"),
    ("softHand_ring_outer_joint"),
    ("softHand_ring_outer_joint_mimic"),

    ("softHand_little_abd_joint"),
    ("softHand_little_inner_joint"),
    ("softHand_little_inner_joint_mimic"),
    ("softHand_little_middle_joint"),
    ("softHand_little_middle_joint_mimic"),
    ("softHand_little_outer_joint"),
    ("softHand_little_outer_joint_mimic")
};

const char* KsGlobal::CJACO_ARM_JOINTS[KsGlobal::VJACO_ARM_JOINT_TOTAL] = {
    "jaco_arm_joint", // Fixed
    "jaco_base_internal", // Fixed

    "jaco_ring_1_joint", // Fixed
    "jaco_arm_0_joint",  // Continuous
    "jaco_ring_2_joint", // Fixed
    "jaco_arm_1_joint",  // Revolute
    "jaco_ring_3_joint", // Fixed
    "jaco_arm_2_joint",  // Revolute
    "jaco_ring_4_joint", // Fixed
    "jaco_arm_3_joint",  // Continuous
    "jaco_ring_5_joint", // Fixed
    "jaco_arm_4_joint",  // Continuous
    "jaco_ring_6_joint", // Fixed
    "jaco_arm_5_joint",  // Continuous

    "jaco_fingers_base_joint", // Fixed
    "jaco_finger_mount_index_fixed", // Fixed
    "jaco_finger_joint_0", // Revolute
    "jaco_finger_joint_1", // Fixed
    "jaco_finger_mount_thumb_fixed", // Fixed
    "jaco_finger_joint_2", // Revolute
    "jaco_finger_joint_3", // Fixed
    "jaco_finger_mount_pinkie_fixed" // Fixed
    "jaco_finger_joint_4", // Revolute
    "jaco_finger_joint_5", // Fixed
};

const char* KsGlobal::CSHADOWHAND_ARM_JOINTS[KsGlobal::VSHADOW_HAND_ARM_JOINT_TOTAL] = {
    "rh_world_joint",                         // Fixed
    // Forearm
    // Wrist
    "rh_WRJ2",                                // Revolute
    // Palm
    "rh_WRJ1",                                // Revolute

    // Fingers
    // Thumb
    "rh_THJ5",                                // Thumb base, revolute
    "rh_THJ4",                                // Thumb proximal, revolute
    "rh_THJ3",                                // Thumb hub, revolute
    "rh_THJ2",                                // Thumb middle, revolute
    "rh_THJ1",                                // Thumb distal, revolute
    "rh_THtip",                               // Thumb tip

    // Index
    "rh_FFJ4",                                // knuckle, Revolute
    "rh_FFJ3",                                // Proximal, Revolute
    "rh_FFJ2",                                // Standard Middle , Revolute
    "rh_FFJ1",                                // Distal , Revolute

    // Middle
    "rh_MFJ4",                                // knuckle, Revolute
    "rh_MFJ3",                                // Proximal, Revolute
    "rh_MFJ2",                                // Standard Middle , Revolute
    "rh_MFJ1",                                // Distal , Revolute

    // Ring
    "rh_RFJ4",                                // knuckle, Revolute
    "rh_RFJ3",                                // Proximal, Revolute
    "rh_RFJ2",                                // Standard Middle , Revolute
    "rh_RFJ1",                                // Distal , Revolute

    // Little Finger (Pinkie)
    "rh_LFJ5",                                // lfmetacarpal, Revolute
    "rh_LFJ4",                                // knuckle, Revolute
    "rh_LFJ3",                                // Proximal, Revolute
    "rh_LFJ2",                                // Standard Middle , Revolute
    "rh_LFJ1"                                 // Distal , Revolute
};

const char* KsGlobal::CSHADOWHAND_ARM_LINKS[] = {
    //"rh_forearm", // No sensor
    //"rh_wrist",   // No sensor
    "rh_palm",

    // FF
    "rh_ffknuckle",
    "rh_ffproximal",
    "rh_ffmiddle",
    "rh_ffdistal",

    // LF
    "rh_ffmetacarpal",
    "rh_ffknuckle",
    "rh_ffproximal",
    "rh_ffmiddle",
    "rh_ffdistal",

    // MF
    "rh_mfknuckle",
    "rh_mfproximal",
    "rh_mfmiddle",
    "rh_mfdistal",

    // RF
    "rh_rfknuckle",
    "rh_rfproximal",
    "rh_rfmiddle",
    "rh_rfdistal",

     // TH
    "rh_thbase",
    "rh_thproximal",
    "rh_thhub",
    "rh_thmiddle",
    "rh_thdistal"
};

KsGlobal* KsGlobal::_instance  = nullptr;
QApplication* KsGlobal::_ksApp = nullptr;
QObject* KsGlobal::_k3dQMLCom  = nullptr;

KsGlobal::KsGlobal(QObject* parent):
          QObject(parent)
{

}

KsGlobal* KsGlobal::getInstance()
{
    if (!_instance)
        _instance = new KsGlobal();

    return _instance;
}

KsGlobal::~KsGlobal(void)
{
    delete _instance;
}

QObject* KsGlobal::k3dQMLCom()
{
    return _k3dQMLCom;
}

void KsGlobal::regK3DQMLCom(QObject* k3dQMLCom)
{
    _k3dQMLCom = k3dQMLCom;
}

void KsGlobal::startWaitInfoTimer(int interval)
{
#if 0
    // Main thread only
    // 
    _waitInfoTimer.setInterval(interval);
    _waitInfoTimer.setSingleShot(true);

    connect(&_waitInfoTimer, SIGNAL(timeout()), _instance, SLOT(clearWaitingInfo()));
    _waitInfoTimer.start();
#endif
}

void KsGlobal::stopWaitInfoTimer()
{
 #if 0
    // Main thread only
    // 
    _waitInfoTimer.stop();
    disconnect(&_waitInfoTimer, SIGNAL(timeout()), _instance, SLOT(clearWaitingInfo()));
#endif
}

void KsGlobal::clearWaitingInfo()
{
    K3D_CLEAR_WAITING_INFO(); // This is thread-safe!
}

QApplication* KsGlobal::ksApp()                     { return _ksApp;}
