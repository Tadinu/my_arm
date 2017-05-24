/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2014 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#pragma once
#include <list>
#include <map>
#include "RealSense/pxc/pxcsensemanager.h"
#include "RealSense/pxc/pxchandmodule.h"
#include "RealSense/pxc/pxchanddata.h"
#include <vector>

#include <QColor>

class HandRender {
public:
    HandRender(pxcCHAR *title=0) {}
    bool RenderFrame(PXCImage *rgbImage, PXCHandModule *detector,
        PXCHandData::JointData nodes[][PXCHandData::NUMBER_OF_JOINTS],
        pxcCHAR gestureName[][PXCHandData::MAX_NAME_SIZE], PXCHandData::BodySideType *handSide);

protected:

#if 0
    virtual void DrawMore(HDC hdc, double scale_x, double scale_y);
#endif

    struct Line {
        int x0, y0;
        int x1, y1;
    };

    struct Node {
        int x, y;
        int radius;
        QColor color;
    };

    std::list<Line>     m_lines;
    std::map<std::pair<int,PXCHandData::JointType>,Node> m_nodes;  //int is for hand id

    struct Gesture {
        PXCHandData::BodySideType handSide;
        pxcCHAR name[PXCHandData::MAX_NAME_SIZE*2];
    };

    std::list<Gesture>  m_gestures;
};

