/*
Copyright (c) 2013-2014, Intel Corporation

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
/** @file pxcspeechrecognition.h
 */
#pragma once
#include "pxcaudiosource.h"

/**
    This class defines a standard interface for performing speech recognition.
*/
class PXCSpeechRecognition:public PXCBase {
public:
    PXC_CUID_OVERWRITE(0x8013C527);
    PXC_DEFINE_CONST(NBEST_SIZE,4);
    PXC_DEFINE_CONST(SENTENCE_BUFFER_SIZE,1024);
    PXC_DEFINE_CONST(TAG_BUFFER_SIZE,1024);

    /**
        @struct NBest
        The NBest data structure describes the NBest data returned from the recognition engine.
    */
    struct NBest {
        pxcI32  label;                  /** The label that refers to the recognized speech */
        pxcI32  confidence;             /** The confidence score of the recognitio: 0-100. */
        pxcCHAR sentence[SENTENCE_BUFFER_SIZE]; /** The recognized sentence */
        pxcCHAR tags[TAG_BUFFER_SIZE];  /** @reserved Additional tags */
   };

    /**
        @struct RecognitionData
        The data structure describes the recgonized speech data.
    */
    struct RecognitionData {
        pxcI64  timeStamp;              /** The time stamp of the recognition, in 100ns. */
        pxcUID  grammar;                /** The grammar identifier for command and control, or zero for dictation. */
        pxcI32  duration;               /** The duration of the speech, in ms. */
        NBest   scores[NBEST_SIZE];     /** The top-N recognition results for command and control. */
        pxcI32  reserved[16];           /** @reserved */
    };

    /**
        @enum AlertType
        Enumerates all supported alert events.
    */
    enum AlertType {
        ALERT_VOLUME_HIGH           = 0x00001,        /** The volume is too high. */
        ALERT_VOLUME_LOW            = 0x00002,        /** The volume is too low. */
        ALERT_SNR_LOW               = 0x00004,        /** Too much noise. */
        ALERT_SPEECH_UNRECOGNIZABLE = 0x00008,        /** There is some speech available but not recognizable. */
        ALERT_SPEECH_BEGIN          = 0x00010,        /** The begining of a speech. */
        ALERT_SPEECH_END            = 0x00020,        /** The end of a speech. */
        ALERT_RECOGNITION_ABORTED   = 0x00040,        /** The recognition is aborted due to device lost, engine error, etc. */
		ALERT_RECOGNITION_END       = 0x00080,        /** The recognition is completed. The audio source no longer provides data. */
    };

    /**
        @struct AlertData
        Describe the alert parameters.
    */
    struct AlertData {
        pxcI64      timeStamp;        /** The time stamp of when the alert occurs, in 100ns. */
        AlertType   label;            /** The alert event label. */
        pxcI32      reserved[6];      /** @reserved */
    };

    /**
        The handler class for the recognition events.
    */
    class Handler {
    public:

        /**
            @brief The function is invoked when there is some speech recognized.
            @param[in] data            The data structure to describe the recognized speech.
        */
        virtual void PXCAPI OnRecognition(const RecognitionData* /*data*/) {}

        /**
            @brief The function is triggered by any alert event.
            @param[in] data            The data structure to describe the alert.
        */
        virtual void PXCAPI OnAlert(const AlertData* /*data*/) {}
    };

    /** 
        @enum LanguageType
        Enumerate all supported languages.
    */
    enum LanguageType {
        LANGUAGE_US_ENGLISH     = PXC_UID('e','n','U','S'),        /** US English */
        LANGUAGE_GB_ENGLISH     = PXC_UID('e','n','G','B'),        /** British English */
        LANGUAGE_DE_GERMAN      = PXC_UID('d','e','D','E'),        /** German */
        LANGUAGE_US_SPANISH     = PXC_UID('e','s','U','S'),        /** US Spanish */
		LANGUAGE_LA_SPANISH     = PXC_UID('e','s','L','A'),        /** Latin American Spanish */
        LANGUAGE_FR_FRENCH      = PXC_UID('f','r','F','R'),        /** French */
        LANGUAGE_IT_ITALIAN     = PXC_UID('i','t','I','T'),        /** Italian */
        LANGUAGE_JP_JAPANESE    = PXC_UID('j','a','J','P'),        /** Japanese */
        LANGUAGE_CN_CHINESE     = PXC_UID('z','h','C','N'),        /** Simplified Chinese */
        LANGUAGE_BR_PORTUGUESE  = PXC_UID('p','t','B','R'),        /** Portuguese */
		LANGUAGE_RU_RUSSIAN     = PXC_UID('r','u','R','U'),        /** Russian */
    };

    /**
        Describe the algorithm configuration parameters.
    */
    struct ProfileInfo {
        pxcCHAR         speaker[128];   /** The optional speaker name for adaptation */
        LanguageType    language;       /** The supported language */
        pxcI32          endOfSentence;  /** The length of end of sentence silence in ms: 100-10000 */
        pxcI32          threshold;      /** The recognition confidence threshold: 0-100 */
        pxcI32          reserved[13];
    };

    /** 
        @enum GrammarFileType
        Enumerate all supported grammar file types.
    */
	enum GrammarFileType {
		GFT_NONE              = 0,  /**  unspecified type, use filename extension */
		GFT_LIST              = 1,  /**  text file, list of commands */
		GFT_JSGF              = 2,  /**  Java Speech Grammar Format */
		GFT_COMPILED_CONTEXT  = 5,  /**  Previously compiled format (vendor specific) */
	} ;

    /** 
        @enum VocabFileType
        Enumerate all supported vocabulary file types.
    */
	enum VocabFileType {
		VFT_NONE              = 0,  /**  unspecified type, use filename extension */
		VFT_LIST              = 1,  /**  text file*/
	} ;

    /**
        @brief The function returns the available algorithm configurations.
        @param[in]  idx         The zero-based index to retrieve all algorithm configurations.
        @param[out] pinfo       The algorithm configuration, to be returned.
        @return PXC_STATUS_NO_ERROR            Successful execution.
        @return PXC_STATUS_ITEM_UNAVAILABLE    There is no more configuration.
    */
    virtual pxcStatus PXCAPI QueryProfile(pxcI32 idx, ProfileInfo *pinfo)=0;

    /**
        @brief The function returns the working algorithm configurations.
        @param[out] pinfo       The algorithm configuration, to be returned.
        @return PXC_STATUS_NO_ERROR            Successful execution.
    */
    pxcStatus __inline QueryProfile(ProfileInfo *pinfo) {
        return QueryProfile(WORKING_PROFILE,pinfo); 
    }

    /**
        @brief The function sets the working algorithm configurations. 
        @param[in] config       The algorithm configuration.
        @return PXC_STATUS_NO_ERROR            Successful execution.
    */
    virtual pxcStatus PXCAPI SetProfile(ProfileInfo *config)=0;

    /** 
        @brief The function builds the recognition grammar from the list of strings. 
        @param[in] gid          The grammar identifier. Can be any non-zero number.
        @param[in] cmds         The string list.
        @param[in] labels       Optional list of labels. If not provided, the labels are 1...ncmds.
        @param[in] ncmds        The number of strings in the string list.
        @return PXC_STATUS_NO_ERROR            Successful execution.
    */
    virtual pxcStatus PXCAPI BuildGrammarFromStringList(pxcUID gid, pxcCHAR *cmds[], pxcI32 *labels, pxcI32 ncmds)=0;

    /** 
        @brief The function deletes the specified grammar and releases any resources allocated.
        @param[in] gid          The grammar identifier.
        @return PXC_STATUS_NO_ERROR                Successful execution.
        @return PXC_STATUS_ITEM_UNAVAILABLE        The grammar is not found.
    */
    virtual pxcStatus PXCAPI ReleaseGrammar(pxcUID gid)=0;

    /** 
        @brief The function sets the active grammar for recognition.
        @param[in] gid          The grammar identifier.
        @return PXC_STATUS_NO_ERROR                Successful execution.
        @return PXC_STATUS_ITEM_UNAVAILABLE        The grammar is not found.
    */
    virtual pxcStatus PXCAPI SetGrammar(pxcUID gid)=0;

    /** 
        @brief The function sets the dictation recognition mode. 
        The function may take some time to initialize.
        @return PXC_STATUS_NO_ERROR                Successful execution.
    */
    pxcStatus __inline SetDictation(void) { 
        return SetGrammar(0); 
    }

    /** 
        @brief The function starts voice recognition.
        @param[in] source       The optional audio source. If omitted, use system default.
        @param[in] handler      The callback handler instance.
        @return PXC_STATUS_NO_ERROR                Successful execution.
    */
    virtual pxcStatus PXCAPI StartRec(PXCAudioSource *source, Handler *handler)=0;

    /** 
        @brief The function stops voice recognition immediately.
    */
    virtual void PXCAPI StopRec(void)=0;

	/** 
        @brief The function create grammar from file
		@param[in] gid                  The grammar identifier. Can be any non-zero number.
		@param[in] fileType             The file type from GrammarFileType structure.
        @param[in] grammarFilename      The full path to file.
        @return PXC_STATUS_NO_ERROR                Successful execution.
		@return PXC_STATUS_EXEC_ABORTED            Incorrect file extension.
    */
	virtual pxcStatus PXCAPI BuildGrammarFromFile(pxcUID gid, GrammarFileType fileType, pxcCHAR *grammarFilename)=0;

	/** 
        @brief The function create grammar from memory
		@param[in] gid                  The grammar identifier. Can be any non-zero number.
		@param[in] fileType             The file type from GrammarFileType structure.
        @param[in] grammarMemory        The grammar specification.
		@param[in] memSize              The size of grammar specification.
        @return PXC_STATUS_NO_ERROR                Successful execution.
		@return PXC_STATUS_EXEC_ABORTED            Incorrect file type.
		@return PXC_STATUS_HANDLE_INVALID          Incorect memSize or grammarMemory equal NULL.
    */
	virtual pxcStatus PXCAPI BuildGrammarFromMemory(pxcUID gid, GrammarFileType fileType, const void  *grammarMemory, pxcI32 memSize)=0;

	/** 
        @brief The function get array with error
		@param[in] gid                  The grammar identifier. Can be any non-zero number.
        @return pxcCHAR *                NULL terminated array with error or NULL in case of internal error.
    */
    virtual const pxcCHAR * PXCAPI GetGrammarCompileErrors(pxcUID gid)=0; 

	/**
	@brief The function add file with vocabulary 
		@param[in] fileType             The vocabulary file type.
        @param[in] vocabFileName        The full path to file.
        @return PXC_STATUS_NO_ERROR     Successful execution.
	*/
	virtual pxcStatus PXCAPI AddVocabToDictation(VocabFileType fileType, pxcCHAR *vocabFileName)=0;
};
