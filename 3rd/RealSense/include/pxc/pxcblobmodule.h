/*
Copyright (c) 2011-2015, Intel Corporation

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
#pragma once
#include "pxcblobconfiguration.h"
#include "pxcblobdata.h"

/**
	@Class PXCBlobModule 
	@brief The main interface to the blob module's classes.
	
	The blob module allows you to extract "blobs" (silhouettes of objects identified by the sensor) and their contour lines.
	Use the PXCBlobModule interface to access to the module's configuration and blob and contour line data.
*/
class PXCBlobModule : public PXCBase 
{
public:

	PXC_CUID_OVERWRITE(PXC_UID('B','M','M','D'));

	/** 
	@brief Create a new instance of the blob module's active configuration.
	Use the PXCBlobConfiguration object to examine the current configuration or to set new configuration values.
	@return A pointer to the configuration instance.
	@see PXCBlobConfiguration
	*/
	virtual PXCBlobConfiguration* PXCAPI CreateActiveConfiguration() = 0;

	/** 
	@brief Create a new instance of the blob module's output data (extracted blobs and contour lines).
	@return A pointer to a PXCBlobData instance.
	@see PXCBlobData
	*/
	virtual PXCBlobData* PXCAPI CreateOutput() = 0;
};
