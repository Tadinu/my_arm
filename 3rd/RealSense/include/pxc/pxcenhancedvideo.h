/*
Copyright (c) 2014-2015, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/** @file pxcenhancedvideo.h
    Defines the PXCEnhancedVideo interface, which programs may use to process a video stream 
	using enhanced videography features.
 */
#pragma once
#include "pxcbase.h"
#include "pxcimage.h"

/**
	This class defines a standard interface for enhanced photography algorithms.
*/
class PXCEnhancedVideo:public PXCBase {
public:

	PXC_CUID_OVERWRITE(PXC_UID('E','V','I','N'));

	/* 
		Input param for Tracking Method: 
		LAYER: track the depth layer in each frame
		OBJECT: track the slected object in each frame
	*/
	enum TrackMethod {
		LAYER = 0, 
		OBJECT, 
	};


   	/**
	 *  EnableTracker: creates an object tracker with a specific tracking method and an 
     *  initial bounding mask as a hint for the object to detect. 
	 *  boundingMask: a hint on what object to detect setting the target pixels to 255 and background to 0.  
	 *  method: Tracking method for depth layer tracking or object tracking.
	*/
	virtual pxcStatus PXCAPI EnableTracker(const PXCImage *boundingMask, TrackMethod method) = 0;
	__inline pxcStatus EnableTracker(const PXCImage *boundingMask) { 
		return EnableTracker(boundingMask, TrackMethod::LAYER);
	}

	/* 
	 *  QueryTrackedObject: returns the tracked object selected in EnableTracker() after every processed frame.
	 *  Returns a mask in the form of PXCImage with detected pixels set to 255 and undetected pixels set to 0.
	 *   returned PXCImage is managed internally APP should not release: TO DO!!
	*/
	virtual PXCImage* PXCAPI QueryTrackedObject() = 0; 

};