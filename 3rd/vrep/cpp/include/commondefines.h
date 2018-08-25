#ifndef COMMON_DEFINE_H
#define COMMON_DEFINE_H

#include <iostream>
using namespace std;

//
#define ROBOT_SENSORS

////////////////////////////////////////////////////////////////////////////////////////////////////////
// COMMON TYPES =================================================================================
//
typedef char VInt8;
typedef unsigned char VUInt8;
typedef VInt8  VByte;
typedef VUInt8 VUByte;

typedef VUInt8 VERROR;

typedef unsigned int VUINT;

////////////////////////////////////////////////////////////////////////////////////////////////////////
// COMMON OPERATIONS ============================================================================
//

namespace CommonBase {
    /* ----------------------- V-REP SERVER INFO -------------------------------------------------------- */
    static const int CSERVER_PORT = 19999;
    static const char* CSERVER_REMOTE_API_OBJECT_NAME = "remoteApiCommandServer";
    //
} // end namespace CommonBase

namespace CB = CommonBase;
#endif // COMMON_DEFINE_H
