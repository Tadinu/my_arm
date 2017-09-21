-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

getObjPos=function(inInts,inFloats,inStrings,inBuffer)
	-- inInts, inFloats and inStrings are tables
	-- inBuffer is a string
    
    -- Perform any type of operation here.
    local objectName   = inStrings[1]    
    local objectHandle = simGetObjectHandle(objectName)    
    local objectPos    = simGetObjectPosition(objectHandle,-1)
   
	return {},{objectPos},{},''
end

getObjVel=function(inInts,inFloats,inStrings,inBuffer)
	-- inInts, inFloats and inStrings are tables
	-- inBuffer is a string
    
    -- Perform any type of operation here.
    local objectName   = inStrings[1]    
    local objectHandle = simGetObjectHandle(objectName)    
    local objectLinearVel, objectAngularVel = simGetObjectVelocity(objectHandle)
   
	return {},{objectLinearVel},{},''
end

moveJoint=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle   = inInts[1]
    local jointName     = inStrings[1]
    local jointDeltaPos = inFloats[1]
    local fullRobotName = simGetObjectName(robotHandle)
    simAddStatusbarMessage("String Signal"..jointName..jointDeltaPos)    
    local stringSignal  = jointName..jointDeltaPos
    simSetStringSignal(fullRobotName..'moveJoint',stringSignal)
   
	return {},{},{},''
end

reloadFallingObjects=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle   = inInts[1]
    local fullRobotName = simGetObjectName(robotHandle)
    local objectNames   = inStrings[1]    
    print("Object Names",objectNames)    
    simSetStringSignal(fullRobotName..'reloadFallingObjects',objectNames)
   
	return {},{},{},''
end

detectCollisionWith=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle  = inInts[1]
    local objectName   = inStrings[1]    
    local objectHandle = simGetObjectHandle(objectName) 
    local ret = simCheckCollision(robotHandle, objectHandle)
	return {ret},{},{},''
end

if (sim_call_type==sim_childscriptcall_initialization) then

    -- Put some initialization code here
    simExtRemoteApiStart(19999)

    -- Make sure you read the section on "Accessing general-type objects programmatically"
    -- For instance, if you wish to retrieve the handle of a scene object, use following instruction:
    --
    -- handle=simGetObjectHandle('sceneObjectName')
    -- 
    -- Above instruction retrieves the handle of 'sceneObjectName' if this script's name has no '#' in it
    --
    -- If this script's name contains a '#' (e.g. 'someName#4'), then above instruction retrieves the handle of object 'sceneObjectName#4'
    -- This mechanism of handle retrieval is very convenient, since you don't need to adjust any code when a model is duplicated!
    -- So if the script's name (or rather the name of the object associated with this script) is:
    --
    -- 'someName', then the handle of 'sceneObjectName' is retrieved
    -- 'someName#0', then the handle of 'sceneObjectName#0' is retrieved
    -- 'someName#1', then the handle of 'sceneObjectName#1' is retrieved
    -- ...
    --
    -- If you always want to retrieve the same object's handle, no matter what, specify its full name, including a '#':
    --
    -- handle=simGetObjectHandle('sceneObjectName#') always retrieves the handle of object 'sceneObjectName' 
    -- handle=simGetObjectHandle('sceneObjectName#0') always retrieves the handle of object 'sceneObjectName#0' 
    -- handle=simGetObjectHandle('sceneObjectName#1') always retrieves the handle of object 'sceneObjectName#1'
    -- ...
    --
    -- Refer also to simGetCollisionhandle, simGetDistanceHandle, simGetIkGroupHandle, etc.
    --
    -- Following 2 instructions might also be useful: simGetNameSuffix and simSetNameSuffix

end


if (sim_call_type==sim_childscriptcall_actuation) then

    -- Put your main ACTUATION code here

    -- For example:
    --
    -- local position=simGetObjectPosition(handle,-1)
    -- position[1]=position[1]+0.001
    -- simSetObjectPosition(handle,-1,position)

end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

    -- Put some restoration code here

end