#include <functional> // std::bind
#include <QThread>
#include "RobotRealSenseAdapter.h"

RobotRealSenseAdapter* RobotRealSenseAdapter::_instance = nullptr;
RobotRealSenseAdapter* RobotRealSenseAdapter::getInstance()
{
    if(_instance == nullptr) {
        _instance = new RobotRealSenseAdapter();
    }

    return _instance;
}

RobotRealSenseAdapter::RobotRealSenseAdapter():
                  _pMutex(new QMutex(QMutex::Recursive))
{
}

RobotRealSenseAdapter::~RobotRealSenseAdapter()
{
    _pMutex->tryLock(500);


    _pMutex->unlock(); // futile if tryLock() failed!
    delete _pMutex;
}

void RobotRealSenseAdapter::deleteInstance()
{
    delete _instance;
    _instance = nullptr;
}

void RobotRealSenseAdapter::initRealSense(ros::NodeHandle* node_handle)
{
    assert(node_handle);

    return;
}

std::vector<std::vector<double>> RobotRealSenseAdapter::getFingerJointValues(int hand_id)
{
    QMutexLocker lock(_pMutex);
    return std::vector<std::vector<double>>();
}

void RobotRealSenseAdapter::emitFingerPosesChanged()
{
    ROS_INFO("RobotRealSenseAdapter::emitFingerPosesChanged()");
    emit fingerPosesChanged();
}


/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2014 Intel Corporation. All Rights Reserved.

*******************************************************************************/
/*
Description:
This is the hand tracking and gesture recognition procedural sample that shows how to enable tracking the joints and gesture recognition.

1- Enable the RGB and depth image.
2- Enable the hand analysis pipeline.
2- Initialize the pipeline camera-> handanalysis using the Init function.
3- In the loop, use the AcquireFrame function to wait for all streams and hand analysis data to be ready,
and then retrieve it through the QuerySample function.
4- Loop through querying all detected hands and querying the data for joint tracking, gestures and alerts fired, and
render the frame or stream using the HandRender class provided in the hand_analysis_procedural folder in the samples.
5- Release the frame to read the next sample through the ReleaseFrame function.
6- Finally, use the release function to clean up.
*/

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif
#include "RealSense/include/pxc/pxcsensemanager.h"
#include "RealSense/include/pxc/pxchandconfiguration.h"
#include "RealSense/include/pxc/pxchandmodule.h"
#if 0
#include "RealSense/include/handanalysis_render.h"  //SDK provided utility class used for rendering face data (packaged in libpxcutils.lib)
#endif

#define NUM_HANDS 2

int doHandAnalyzing()
{
#if 0
    // error checking Status
    pxcStatus sts;

#if 0
    // initialize the util render
    HandRender *renderer = new HandRender(L"PROCEDURAL HAND TRACKING");
#endif

    // create the PXCSenseManager
    PXCSenseManager *psm=0;
    psm = PXCSenseManager::CreateInstance();
    if (!psm) {
        ROS_INFO("Unable to create the PXCSenseManager\n");
        return 1;
    }

    // select the depth stream of size 640x480
    psm->EnableStream(PXCCapture::STREAM_TYPE_DEPTH, 640, 480);

    // enable hand analysis in the multimodal pipeline
    sts = psm->EnableHand();
    if (sts < PXC_STATUS_NO_ERROR) {
        ROS_INFO("Unable to enable Hand Tracking\n");
        return 2;
    }

    // retrieve hand module if ready - called in the setup stage before AcquireFrame
    PXCHandModule* handAnalyzer = psm->QueryHand();
    if (!psm) {
        ROS_INFO("Unable to retrieve hand results\n");
        return 3;
    }

    // initialize the PXCSenseManager
    if(psm->Init() < PXC_STATUS_NO_ERROR) return 4;

    // retrieves an instance of the PXCHandData interface
    PXCHandData* outputData = handAnalyzer->CreateOutput();

    // retrieves an instance of the PXCHandData PXCHandConfiguration
    PXCHandConfiguration* config = handAnalyzer->CreateActiveConfiguration();

    // enable or disable features in hand module
    config->EnableNormalizedJoints(false);
    config->EnableAllAlerts();
    config->EnableAllGestures();

    //make the config changes effective
    config->ApplyChanges();

    // stream data
    int fnum = 0; //frame counter
    PXCImage *depthIm = NULL; //init depth im
    while (psm->AcquireFrame(true)>=PXC_STATUS_NO_ERROR) {

        // increment frame counter since a frame is acquired
        fnum++;

        //update the output data to the latest availible
        outputData->Update();

        // create data structs for storing data
        PXCHandData::GestureData gestureData;
        PXCHandData::JointData nodes[NUM_HANDS][PXCHandData::NUMBER_OF_JOINTS]={};
        pxcCHAR gestures[NUM_HANDS][PXCHandData::MAX_NAME_SIZE] = {};
        PXCHandData::BodySideType handSide[NUM_HANDS] = {PXCHandData::BODY_SIDE_UNKNOWN};

        // iterate through hands
        pxcUID handID;
        pxcU16 numOfHands = outputData->QueryNumberOfHands();
        for(pxcU16 i = 0 ; i < numOfHands ; i++)
        {
            // get hand joints by time of appearence
            PXCHandData::IHand* handData;
            if(outputData->QueryHandData(PXCHandData::ACCESS_ORDER_BY_TIME,i,handData) == PXC_STATUS_NO_ERROR)
            {
                // iterate through Joints and get joint data
                for(int j = 0; j < PXCHandData::NUMBER_OF_JOINTS ; j++)
                {
                    handData->QueryTrackedJoint((PXCHandData::JointType)j,nodes[i][j]);
                }
            }
        }

        // iterate through fired gestures
        for(unsigned int i = 0; i < outputData->QueryFiredGesturesNumber(); i++)
        {
            // initialize data
            wmemset(gestures[i], 0, sizeof(gestures[i]));
            handSide[i] = PXCHandData::BODY_SIDE_UNKNOWN;

            // get fired gesture data
            if(outputData->QueryFiredGestureData(i,gestureData) == PXC_STATUS_NO_ERROR)
            {
                // get hand data related to fired gesture
                PXCHandData::IHand* handData;
                if(outputData->QueryHandDataById(gestureData.handId,handData) == PXC_STATUS_NO_ERROR)
                {
                    // save gesture only if you know that its right/left hand
                    if(!handData->QueryBodySide() == PXCHandData::BODY_SIDE_UNKNOWN)
                    {
                        wmemcpy_s (gestures[i],sizeof(gestureData.name), gestureData.name, sizeof(gestureData.name));
                        handSide[i] = handData->QueryBodySide();
                    }
                }
            }
        }

        // iterate through Alerts
        PXCHandData::AlertData alertData;
        for(int i = 0 ; i <outputData->QueryFiredAlertsNumber(); i++)
        {
            pxcStatus sts = outputData->QueryFiredAlertData(i,alertData);
            if(sts==PXC_STATUS_NO_ERROR)
            {
                // Display last alert - see AlertData::Label for all available alerts
                switch(alertData.label)
                {
                case PXCHandData::ALERT_HAND_DETECTED:
                    {
                        ROS_INFO("Last Alert: Hand Detected\n");
                        break;
                    }
                case PXCHandData::ALERT_HAND_NOT_DETECTED:
                    {
                        ROS_INFO("Last Alert: Hand Not Detected\n");
                        break;
                    }
                }
            }
        }

        // retrieve all available image samples
        PXCCapture::Sample *sample = psm->QuerySample();

        // retrieve the image or frame by type
        depthIm = sample->depth;

        // render the frame
        if (!renderer->RenderFrame(depthIm, handAnalyzer, nodes, gestures, handSide)) break;

        // release or unlock the current frame to go fetch the next frame
        psm->ReleaseFrame();
    }

    //delete the configuration
    config->Release();

    // delete the HandRender instance
    renderer->Release();

    // close the last opened stream and release any session and processing module instances
    psm->Release();
#endif
    return 0;
}
