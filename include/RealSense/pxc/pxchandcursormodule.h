/*
Copyright (c) 2013-2016, Intel Corporation

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
/** @file PXCHandCursorModule.h
    Defines the PXCHandCursorModule interface, which gives access the hand cursor module's configuration and output data.
 */
#pragma once
#include "pxcbase.h"


class PXCCursorConfiguration;
class PXCCursorData;


/**
    @Class PXCHandCursorModule 
    The main interface to the hand cursor module's classes.\n
    Use this interface to access the hand cursor module's configuration and output data.
*/
class PXCHandCursorModule : public PXCBase 
{
public:

    PXC_CUID_OVERWRITE(PXC_UID('H','C','M','N'));

	/** 
    @brief Create a new instance of the hand cursor module's active configuration.
    Multiple configuration instances can be created in order to define different configurations for different stages of the application.
    You can switch between the configurations by calling the ApplyChanges method of the required configuration instance.
    @return A pointer to the configuration instance.
    @see PXCCursorConfiguration
    */
	virtual PXCCursorConfiguration* PXCAPI CreateActiveConfiguration() = 0;

	/** 
    @brief Create a new instance of the hand cursor module's current output data.
    Multiple instances of the output can be created in order to store previous tracking states. 
    @return A pointer to the output data instance.
    @see PXCCursorData
    */
	virtual PXCCursorData* PXCAPI CreateOutput() = 0;
	

};
