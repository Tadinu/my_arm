/*
Copyright (c) 2015-2016, Intel Corporation

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

/** 
@file pxcobjectrecognitionconfiguration.h
@Defines the PXCObjectRecognitionConfiguration interface, which programs may use to process snapshots of captured frames
@to recognize pre-trained objects.
 */

#pragma once
#include "pxcstatus.h"
class PXCObjectRecognitionConfiguration :public PXCBase
{
public:
	PXC_DEFINE_CONST(MAX_PATH_NAME,256);
	 enum RecognitionConfidence
    {
        HIGH,
        MEDIUM,
        LOW
    };

    enum RecognitionMode
	{
		SINGLE_RECOGNITION,
		LOCALIZATION,
		LOCALIZATION_AND_TRACKING,
		TRACKING_ONLY,
		PROPOSAL_ONLY
	};

	enum LocalizationMechanism
	{
		LM_EDGE_BOXES,
		LM_OBJECTS_ON_PLANE
	};

	struct RecognitionConfiguration
	{
		RecognitionMode mode;
		RecognitionConfidence confidence;
	};

	PXC_CUID_OVERWRITE(PXC_UID('O', 'B', 'J', 'C'));
	 
	 /**
	@brief Apply the configuration changes to the module.
	This method must be called in order to apply the current configuration changes.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	@return PXC_STATUS_DATA_NOT_INITIALIZED - configuration was not initialized.                        
    */
    virtual pxcStatus PXCAPI ApplyChanges() = 0;

    /**  
	@brief Restore configuration settings to the default values.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	@return PXC_STATUS_DATA_NOT_INITIALIZED - configuration was not initialized.                       
    */
    virtual pxcStatus PXCAPI RestoreDefaults() = 0;

	/**
	@brief Return the current active recognition configuration.
	@return PXCRecognitionConfig - the current configuration.
	@see PXCRecognitionConfiguration
    */
	virtual RecognitionConfiguration PXCAPI QueryRecognitionConfiguration() const =0;

	/**
	@brief Return the number of classes which supported by the recognition configuration.
	@return pxcI32 - the current configuration.
    */
	virtual pxcI32 PXCAPI QueryNumberOfClasses() const = 0;

	/**
	@brief Set the active recognition configuration.
	@param[in] rConfig - struct with the desired confidence and mode of recognition.
	@return PXC_STATUS_NO_ERROR - on operation succeeded.  
	@see PXCRecognitionConfiguration
    */
	virtual pxcStatus PXCAPI SetRecognitionConfiguration(const RecognitionConfiguration& rConfig) =0;

	/** 
	@brief Return the name of the active classifier model.
	@return the active classifier name
	*/    
	virtual const pxcCHAR* PXCAPI QueryActiveClassifier() const = 0;

	/** 
	@brief Query the ROI.
	@return the current ROI.
	*/    
	virtual PXCRectF32 PXCAPI QueryROI() const = 0;

	/** 
	@brief Query the absolute ROI.
	@return the current absolute ROI.
	*/    
	virtual PXCRectI32 PXCAPI QueryAbsoluteROI() const = 0;

	/** 
	@brief Query the current localization mechanism.
	@return the current localization mechanism.
	*/    
	virtual LocalizationMechanism PXCAPI QueryLocalizationMechanism() const = 0;

	/** 
	@brief Set the active localization mechanism.
	@brief The localization mechanism is activated only if LOCALIZATION or LOCALIZATION_AND_TRACKING or PROPOSAL_ONLY  mode have been selected.  
	@param[in] lm - the selected localization method.
	@see LocalizationMechanism enum.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	*/    
	virtual pxcStatus PXCAPI SetLocalizationMechanism(const LocalizationMechanism lm) = 0;

	/** 
	@brief Set the active classifier model.
	@param[in] configFilePath - relative path to the classifier model file from RSSDK data path.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	*/    
	virtual pxcStatus PXCAPI SetActiveClassifier(const pxcCHAR* configFilePath) = 0;
	
	/** 
	@brief Set or unset the segmented image feature.
	@param[in] enable - boolean value to enable/disable the segmentation.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	*/    
	virtual pxcStatus PXCAPI EnableSegmentation(const pxcBool enable) = 0;
	
	/** 
	@brief Get the current state of segmented image feature.
	@return  A boolean value which indicate if segmentation is enabled or not.
	*/   
	virtual pxcBool PXCAPI  IsSegmentationEnabled() = 0;

	/** 
	@brief get the current state of absolute ROI feature.
	*/   
	virtual pxcBool PXCAPI  IsAbsoluteRoiEnabled() const = 0;

	
	/** 
	@brief Add a ROI to the classification.
	@param[in] roi - a ROI rectangle to be added.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	*/    
	virtual pxcStatus PXCAPI AddROI(const PXCRectF32& roi) =0;

	/** 
	@brief Add an absolute ROI to the classification. This roi is absolute by which the image will be croped.
	@param[in] ROI - a ROI rectangle to be added.
	@return PXC_STATUS_NO_ERROR - operation succeeded.
	*/    
	virtual pxcStatus PXCAPI AddAbsoluteROI(const PXCRectF32& roi) =0;

	/** 
	@brief Remove all ROIs from the classification and set the ROI to the whole image.
	*/
	virtual void PXCAPI ClearAllROIs() =0;

	/** 
	@brief Set or unset the absolute ROI feature.
	@param[in] enable - boolean value to enable/disable the absolute ROI.
	*/    
	virtual void PXCAPI EnableAbsoluteROI(const pxcBool enable) = 0;

	/** 
	@brief Set the initial rois to track by tracking module .
	@brief Enables only when tracking bool is on and all other are off.
	@brief The number of rectangles will be between 1 and 5.
	@param[in] rois - pointer to the rectangles tracked by tracker module.
	@param[in] nRois - number the rectangles tracked by tracker module.
	@return the status of the operation.
    */    
	virtual pxcStatus PXCAPI SetTrackingROIs(const PXCRectF32* rois,const  pxcI32 nRois) = 0;


};