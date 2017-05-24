/*******************************************************************************

INTEL CORPORATION PROPRIETARY INFORMATION
This software is supplied under the terms of a license agreement or nondisclosure
agreement with Intel Corporation and may not be copied or disclosed except in
accordance with the terms of that agreement
Copyright(c) 2012 2013 Intel Corporation. All Rights Reserved.

*******************************************************************************/

#include "face_render.h"
#include "pxcfacedata.h"
#include <tchar.h>
#include <assert.h>
#include "util_render.h"
#include "pxcdefs.h"

#define TEXT_HEIGHT 16

FaceRender::FaceRender(pxcCHAR* title) : UtilRender(title), faceData(NULL)
{

}

void FaceRender::SetFaceData(PXCFaceData *data)
{
	faceData = data;
}

bool FaceRender::DrawDetection(PXCFaceData::Face* trackedFace, double scaleX, double scaleY, HDC hdc, PXCRectI32& outLocation)
{
	if (trackedFace->QueryDetection() == NULL)
	{
		return false;
	}
	pxcBool hasRect = trackedFace->QueryDetection()->QueryBoundingRect(&outLocation);
	if (hasRect)
	{
		RECT rectangle = { (LONG)(outLocation.x * scaleX), (LONG)(outLocation.y * scaleY), (LONG)((outLocation.w + outLocation.x) * scaleX),
			(LONG)((outLocation.h + outLocation.y) * scaleY) };
		DrawEdge(hdc, &rectangle, EDGE_RAISED, BF_TOP | BF_RIGHT | BF_LEFT | BF_BOTTOM);
	}
	return hasRect > 0;
}

void FaceRender::DrawFaceID(PXCFaceData::Face* trackedFace, double scaleX, double scaleY, HDC hdc, PXCRectI32& inLocation)
{
	int x = (int)(inLocation.x * scaleX - 10);
	int y = (int)((inLocation.y + inLocation.h) * scaleY - TEXT_HEIGHT * 5 - 10);
	int id = trackedFace->QueryUserID();

	TCHAR faceId[32];
	_stprintf_s(faceId, 32, TEXT("%d"), id);

	HFONT hFont = CreateFont(20, 0, 0, 0, FW_BOLD, 0, 0, 0, 0, 0, 0, 2, 0, L"MONOSPACE");
	SetTextColor(hdc, RGB(0x10, 0xFF, 0x00));
	SelectObject(hdc, hFont);

	TextOut(hdc, x, y, faceId, (int)_tcslen(faceId));

	DeleteObject(hFont);
	hFont = CreateFont(8, 0, 0, 0, FW_LIGHT, 0, 0, 0, 0, 0, 0, 2, 0, L"MONOSPACE");
	SetTextColor(hdc, RGB(0xFF, 0xFF, 0xFF));
	SelectObject(hdc, hFont);
}

void FaceRender::DrawLandmarks(PXCFaceData::Face* trackedFace, double scaleX, double scaleY, HDC hdc)
{
	const PXCFaceData::LandmarksData* ldOutput = trackedFace->QueryLandmarks();
	if (ldOutput == NULL)
		return;

	PXCFaceData::LandmarkPoint* landmarkPoints = new PXCFaceData::LandmarkPoint[ldOutput->QueryNumPoints()];
	pxcBool hasPoints = ldOutput->QueryPoints(landmarkPoints);

	if (hasPoints)
	{
		for (int pointIndex = 0; pointIndex < ldOutput->QueryNumPoints(); ++pointIndex)
		{
			int x = (int)(landmarkPoints[pointIndex].image.x * scaleX);
			int y = (int)(landmarkPoints[pointIndex].image.y * scaleY);

			if (!landmarkPoints[pointIndex].confidenceImage) // skip landmarks with low confidence.
			{
				SetTextColor(hdc, RGB(255, 0, 0));
				TextOut(hdc, x, y, L"•", 1);
			}
			else
			{
				SetTextColor(hdc, RGB(0, 255, 0));
				TextOut(hdc, x, y, L"x", 1);
			}
		}
	}
	delete[] landmarkPoints;
}


void FaceRender::DrawMore(HDC hdc, double scaleX, double scaleY)
{
	int numberOfDetectedFaces;
	if (faceData == NULL)
		return;
	PXCFaceData::Face** trackedFaces = faceData->QueryFaces(&numberOfDetectedFaces);

	for (int faceIndex = 0; faceIndex < numberOfDetectedFaces; ++faceIndex)
	{
		PXCFaceData::Face* trackedFace = trackedFaces[faceIndex];
		PXCRectI32 location;
		bool detectionDrawn = DrawDetection(trackedFace, scaleX, scaleY, hdc, location);
		if (detectionDrawn)
		{
			DrawFaceID(trackedFace, scaleX, scaleY, hdc, location);
		}
		DrawLandmarks(trackedFace, scaleX, scaleY, hdc);
	}
}
