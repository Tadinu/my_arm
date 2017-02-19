/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2013-2014 Intel Corporation. All Rights Reserved.

*******************************************************************************/
#include "hands_publisher.h"

#define MAX_NUM_OF_HANDS 2
#define TEXT_HEIGHT 16

#define GESTURE_BOX_SIZE 40
#define GESTURE_COUNT    30
#define COLOR_RED        QColor(255,0,0)
#define COLOR_GREEN      QColor(0,255,0)
#define COLOR_BLUE       QColor(0,0,255)
#define COLOR_YELLOW     QColor(255,255,0)

static wchar_t *HandLabels[] = {
    L"LEFT HAND",
    L"RIGHT HAND"
};

int prevNum = 0;
int curNum = 0;
int toClear = false;

bool HandRender::RenderFrame(PXCImage *image, PXCHandModule *detector,
                             PXCHandData::JointData nodes[][PXCHandData::NUMBER_OF_JOINTS],
                             pxcCHAR gestureName[][PXCHandData::MAX_NAME_SIZE],
                             PXCHandData::BodySideType *handSide)
{
    m_lines.clear();
    m_nodes.clear();

    if (detector) {
        PXCImage::ImageInfo imInfo;
        imInfo = image->QueryInfo();

        curNum = 0;

        for (int i = 0; i < MAX_NUM_OF_HANDS; ++i)
        {
            //Draw Joints
            for (int j = 0; j < PXCHandData::NUMBER_OF_JOINTS; ++j)  {
                if (nodes[i][j].confidence == 0) continue;

                int x=(int)nodes[i][j].positionImage.x ;
                int y=(int)nodes[i][j].positionImage.y ;

                int radius=2;
                Node node={(int)x,(int)y,radius,COLOR_GREEN};
                m_nodes[std::pair<int,PXCHandData::JointType>(i,(PXCHandData::JointType)j)]=node;
                curNum++;
            }


            int wristX=(int)nodes[i][0].positionImage.x;
            int wristY=(int)nodes[i][0].positionImage.y;
            for (int j = 2 ;j < (PXCHandData::NUMBER_OF_JOINTS-1); j++)
            {
                if(nodes[i][j].confidence == 0) continue;

                if(j == 5 || j == 9 || j == 13 || j == 17)
                {
                    continue;
                }

                if(j == 2 || j == 6 || j == 10 || j == 14 || j == 18)
                {
                    int x0=(int)nodes[i][0].positionImage.x;
                    int y0=(int)nodes[i][0].positionImage.y;
                    int x1=(int)nodes[i][j].positionImage.x;
                    int y1=(int)nodes[i][j].positionImage.y;

                    if (x0 > 0 && x0 < imInfo.width && y0 > 0 && y0 < imInfo.height &&
                        x1 > 0 && x1 < imInfo.width && y1 > 0 && y1 < imInfo.height) {
                            Line line={(int)x0,(int)y0,(int)x1,(int)y1};
                            m_lines.push_back(line);
                    }
                }

                int x0=(int)nodes[i][j].positionImage.x;
                int y0=(int)nodes[i][j].positionImage.y;
                int x1=(int)nodes[i][j+1].positionImage.x;
                int y1=(int)nodes[i][j+1].positionImage.y;

                if (x0 > 0 && x0 < imInfo.width && y0 > 0 && y0 < imInfo.height &&
                    x1 > 0 && x1 < imInfo.width && y1 > 0 && y1 < imInfo.height) {
                        Line line={(int)x0,(int)y0,(int)x1,(int)y1};
                        m_lines.push_back(line);
                }
            }

            //SET gesture only if we know if its right/left hand
            if(! handSide[i] == PXCHandData::BODY_SIDE_UNKNOWN)
            {
                Gesture gesture;
#if 0
                wmemcpy_s(gesture.name,sizeof(gestureName[i]), gestureName[i], sizeof(gestureName[i])/sizeof(gestureName[0][0]));
#endif
                gesture.handSide = handSide[i];
                if (m_gestures.size()>1) {
                    if(m_gestures.front().handSide == handSide[i])
                        m_gestures.pop_front();
                    else
                        m_gestures.pop_back();
                }

                m_gestures.push_back(gesture);
            }
        }

        if(prevNum!=curNum)
        {
            if(toClear == false && !m_gestures.empty())
                toClear = true;
        }
        prevNum = curNum;
    }

    return true;
}

#if 0
void HandRender::DrawMore(HDC hdc, double sx, double sy) {
    HPEN red=CreatePen(PS_SOLID, 4, COLOR_RED);
    SelectObject(hdc, red);
    for (std::list<Line>::iterator itrl=m_lines.begin(); itrl!=m_lines.end(); itrl++) {
        MoveToEx(hdc, (int)(itrl->x0*sx), (int)(itrl->y0*sy), NULL);
        LineTo(hdc, (int)(itrl->x1*sx), (int)(itrl->y1*sy));
    }
    DeleteObject(red);

    for (std::map<std::pair<int,PXCHandData::JointType>,Node>::iterator itrn=m_nodes.begin(); itrn!=m_nodes.end(); itrn++) {
        HPEN color=CreatePen(PS_SOLID, 2, itrn->second.color);
        SelectObject(hdc, color);

        int x=(int)(itrn->second.x*sx);
        int y=(int)(itrn->second.y*sy);
        int rx=(int)(itrn->second.radius*sx);
        int ry=(int)(itrn->second.radius*sy);

        MoveToEx(hdc, x, y, NULL);
        Arc(hdc,x-rx,y-ry,x+rx,y+ry,x+rx,y+ry,x+rx,y+ry);
        DeleteObject(color);
    }



        HFONT hfont;
        LOGFONT logFont;
        memset(&logFont, 0, sizeof(logFont));
        logFont.lfHeight = 15; // see PS
        logFont.lfWeight = FW_BOLD;
        strcpy_s(logFont.lfFaceName, "Arial");
        hfont = CreateFontIndirect(&logFont);
        SetTextColor(hdc, COLOR_GREEN);
        SelectObject(hdc, hfont);
        int locX=1, locY=1;
        int cnt=0;

        PXCHandData::BodySideType prevSide = PXCHandData::BODY_SIDE_UNKNOWN;

        for (std::list<Gesture>::iterator itrg=m_gestures.begin();itrg!=m_gestures.end();itrg++) {
            if(prevSide!=itrg->handSide)
            {
                int x=locX*sx;
                int y=((locY*sy)+TEXT_HEIGHT*cnt);
                WCHAR line[256];
                wcscpy_s(line,sizeof(line)/sizeof(pxcCHAR), itrg->name);
                TextOut(hdc, x, y, (LPCSTR)itrg->name, wcslen(itrg->name)*2);

                if(itrg->handSide == PXCHandData::BODY_SIDE_LEFT)
                {
                    TextOut(hdc, x+ wcslen(itrg->name)*15, y, (LPCSTR)HandLabels[0], wcslen(HandLabels[0])*2);
                }else if(itrg->handSide == PXCHandData::BODY_SIDE_RIGHT){
                    TextOut(hdc, x+ wcslen(itrg->name)*15, y, (LPCSTR)HandLabels[1], wcslen(HandLabels[1])*2);
                }

                cnt++;
                prevSide = itrg->handSide;
            }
        }
        DeleteObject(hfont);


    if(toClear)
    {
        if(!m_gestures.empty())
            m_gestures.clear();
        toClear = false;
    }
}
#endif
