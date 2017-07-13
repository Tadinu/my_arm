
#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#include <stdint.h>

#include <string>
#include <vector>

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS COMMON DEFINE =================================================================================
//
#define MY_ARM_RVIZ

// PACKAGE NAME --
#define CROS_MY_ARM_PACKAGE_NAME ("my_arm")

// FRAMES --
#define CWORLD_FRAME ("world")
#define CBASE_LINK ("base_link")
#define RAD_2_ANGLE(rad)   ((rad*180)/V_PI)
#define ANGLE_2_RAD(angle) ((angle*V_PI)/180)

// VMARKER --
#define VMARKER_INSTANCE() VMarker::getInstance()

// LEAP / REALSENSE HANDS --
//
#define ROBOT_LEAP_HANDS
//#define ROBOT_REAL_SENSE_HANDS
#define CLEAP_BASE_FRAME (CWORLD_FRAME)

// DART --
//#define ROBOT_DART // Not successfully building Dart with ROS and Boost yet.
#ifdef ROBOT_DART
#define VDART_ADAPTER() RobotDartAdapter::getInstance()
#endif

// VOXELYZE --
// ROBOT_VOXELYZE --
//
#define ROBOT_VOXELYZE
//#define USING_PISA_SOFT_HAND_ONLY

// MOVEIT --
//#define ROBOT_MOVEIT

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ROS COMMON INCLUDE =================================================================================
//
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/common.h>
#include <ros/cpp_common_decl.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////
// COMMON TYPES =================================================================================
//
#define V_PI (3.1415926535897931)

typedef char VInt8;
typedef unsigned char VUInt8;
typedef VInt8  VByte;
typedef VUInt8 VUByte;

typedef VUInt8 VERROR;

typedef unsigned int VUINT;

// KChar is always wchar_t
// KStdString is always std::wstring
typedef wchar_t VChar;
typedef std::wstring VStdString;
typedef std::vector<VStdString> VStdStringVector;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// COMMON OPERATIONS ============================================================================
//
#define VSTRCMP(str1, str2) _wcsicmp(str1, str2) // By default: wstring

#define VPRT_DOUBLE_2_LONG(value) (static_cast<ulong>(value + 0.5)) // [0.0->0.5] = 0, [0.6]->[0.9] = 1

#define VERROR_SET(error, errorCode) { \
    if (error) {                          \
        *error = errorCode;               \
    }                                     \
}

namespace CommonBase {
    /* ----------------------- COMMON DEFINES ---------------------------------------------------------- */
    static const unsigned VMIN_REQUIRED_MEMORY = 200 * 1024 * 1024; // Minimum required memory for 3D App(Unyk, Karpenter) to operate
    static const unsigned VSERVER_TIMEOUT     = 60000; //ms
    /* ----------------------- COMMON INFO ------------------------------------------------------------- */

    /* ----------------------- PROGRESS PHASE VALUE ----------------------------------------------------- */
    // Other > 0 Values: PROGRESS_RUNNING
    static const VInt8 PROGRESS_PREPARE     = 0;
    static const VInt8 PROGRESS_CLOSE       = PROGRESS_PREPARE - 1;
    static const VInt8 PROGRESS_TEXT_ONLY   = PROGRESS_CLOSE   - 1;
    static const VInt8 PROGRESS_IGNORE_VAL  = PROGRESS_TEXT_ONLY;
    static const VInt8 PROGRESS_REFRESHING  = PROGRESS_TEXT_ONLY - 1;

    static const VInt8 VPROGRESS_MAX_VAL    = 100;
    static const VInt8 VPROGRESS_LIMIT_VAL  = (99 * VPROGRESS_MAX_VAL) / 100;

    static const std::wstring NULL_WSTR     = std::wstring(L"\1").replace(0, 1, 1, L'\0');

    static const int NULL_INDEX = -1;

    static const int DAY_DIFF_ACTIVE = 30;

    /* ----------------------- ERROR CODE --------------------------------------------------------------- */
    enum VERROR {
        VERROR_SUCCESS = 0,
        VERROR_NETWORK_CONNECTION,
        VERROR_NETWORK_DATA_FAILED,

        VERROR_TOTAL
    };

    enum PARSE_SERVER_RESPONSE{
        PARSE_SERVER_RESPONSE_SHOW_NONE             = 0x01,
        PARSE_SERVER_RESPONSE_SHOW_SUCCESS          = 0x02,
        PARSE_SERVER_RESPONSE_SHOW_FAILED           = 0x04,
        PARSE_SERVER_RESPONSE_SHOW_DEBUG            = 0x08,
        PARSE_SERVER_RESPONSE_SHOW_CONNECTION_ERROR = 0x10,
        NOT_PARSE_SERVER_RESPONSE                   = 0x20,

        PARSE_SERVER_RESPONSE_TOTAL
    };

    // Texts - defined in ksWord::wText[CommonBase::TOTAL] (commonresource.h)
    enum PROGRESS_TEXT {
        /* ----------------------- PROGRESS DIALOG TEXT ------------------------------------------------- */
        /* Format of ksWord::wText[TextId]:{%1,%2,%3, LValue} in self-defined order.  */
        /* ----------------------- ADD BELOW: OTHER TEXTS ----------------------------- */
        TOTAL

    };

    /* ----------------------- COMMON LIB UTILITIES----------------------------------------------------- */
    //
}; // end namespace CommonBase

class VException : public std::runtime_error
{
public:
    typedef std::runtime_error _Mybase;
public:
    explicit VException(const std::string& message) : _Mybase(message)
    {

    }
    explicit VException(const char *_Message) : _Mybase(_Message)
    {   // construct from message string
    }
};

namespace CB = CommonBase;
#endif // COMMON_DEFINE_H
