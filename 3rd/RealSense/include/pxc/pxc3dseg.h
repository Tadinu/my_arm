/*
Copyright (c) 2014, Intel Corporation

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
/// @file pxc3dseg.h
/// User Segmentation video module interface

#ifndef PXC3DSEG_H
#define PXC3DSEG_H
#include "pxccapture.h"

class PXC3DSeg : public PXCBase
{
public:
    /// Return a reference to the most recent segmented image.
    /// The returned object's Release method can be used to release the reference.
    virtual PXCImage* PXCAPI AcquireSegmentedImage(void)=0;

    PXC_CUID_OVERWRITE(PXC_UID('S', 'G', 'I', '1'));

    enum AlertEvent 
    {
        ALERT_USER_IN_RANGE = 0,
        ALERT_USER_TOO_CLOSE,
        ALERT_USER_TOO_FAR
    };

    struct AlertData 
    {
        pxcI64     timeStamp;
        AlertEvent label;
        pxcI32     reserved[5];
    };

    class AlertHandler
    {
    public:
        virtual void PXCAPI OnAlert(const AlertData& data)=0;
    };

    /// Optionally register to receive event notifications.
    /// A subsequent call will replace the previously registered handler object.
    /// Subscribe(NULL) to unsubscribe.
    virtual void PXCAPI Subscribe(AlertHandler* handler)=0;

	virtual pxcStatus PXCAPI SetFrameSkipInterval(pxcI32 skipInterval)=0;
};
#endif


