// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

// Make sure to have the server side running in V-REP!
// Start the server from a child script with following command:
// simExtRemoteApiStart(portNumber) -- starts a remote API server service on the specified port

#include <stdio.h>
#include <stdlib.h>

extern "C" {
    #include "extApi.h"
}

int main(int argc,char* argv[])
{
    int portNb=19999;

    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);
    if (clientID!=-1)
    {
        while (simxGetConnectionId(clientID)!=-1)
        {
            //number objectHandle=simCreatePureShape(number primitiveType,number options,table_3 sizes,number mass,table_2 precision=nil)
            extApi_sleepMs(5);
        }
        simxFinish(clientID);
    }
    return(0);
}

