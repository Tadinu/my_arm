-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

getObjPos=function(objectName)
	-- inInts, inFloats and inStrings are tables
	-- inBuffer is a string
    
    -- Perform any type of operation here.   
    local objectHandle = simGetObjectHandle(objectName)    
    local objectPos    = simGetObjectPosition(objectHandle,-1)
   
	return objectPos
end

getObjVel=function(objectName)
	-- inInts, inFloats and inStrings are tables
	-- inBuffer is a string
    
    -- Perform any type of operation here.
    local objectHandle = simGetObjectHandle(objectName)    
    local objectLinearVel, objectAngularVel = simGetObjectVelocity(objectHandle)
   
	return objectLinearVel
end

loadFallingObjects=function()
    --simAddStatusbarMessage("Fall-load"..fallingObjectNames[1])
    local tbox1 = simCreatePureShape(1, 8, {0.2, 0.1, 0.07}, 0, nil)
    simSetObjectPosition(tbox1, model, {posX , posY , posZ})
    simSetObjectName(tbox1, fallingObjectNames[1])
                    
    local tbox2 = simCreatePureShape(1, 8, {0.2, 0.1, 0.07}, 0, nil)
    simSetObjectPosition(tbox2, model, {-posX , posY , posZ})
    simSetObjectName(tbox2, fallingObjectNames[2])
    
    local tbox3 = simCreatePureShape(1, 8, {0.2, 0.1, 0.07}, 0, nil)
    simSetObjectPosition(tbox3, model, {posX + 0.3 , posY , posZ})
    simSetObjectName(tbox3, fallingObjectNames[3])
    
    local tbox4 = simCreatePureShape(1, 8, {0.2, 0.1, 0.07}, 0, nil)
    simSetObjectPosition(tbox4, model, {-posX - 0.3 , posY , posZ})
    simSetObjectName(tbox4, fallingObjectNames[4])
    
    local tbox5 = simCreatePureShape(1, 8, {0.2, 0.1, 0.07}, 0, nil)
    simSetObjectPosition(tbox5, model, {posX + 0.5, posY , posZ})
    simSetObjectName(tbox5, fallingObjectNames[5])
end

removeFallingObjects=function()
    --simAddStatusbarMessage("Fall-remove"..fallingObjectNames[1])
    local tBoxHandle1 = simGetObjectHandle(fallingObjectNames[1])
    simRemoveObject(tBoxHandle1)

    local tBoxHandle2 = simGetObjectHandle(fallingObjectNames[2])
    simRemoveObject(tBoxHandle2)

    local tBoxHandle3 = simGetObjectHandle(fallingObjectNames[3])
    simRemoveObject(tBoxHandle3)

    local tBoxHandle4 = simGetObjectHandle(fallingObjectNames[4])
    simRemoveObject(tBoxHandle4)

    local tBoxHandle5 = simGetObjectHandle(fallingObjectNames[5])
    simRemoveObject(tBoxHandle5)
end

moveJoint=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle   = inInts[1]
    local jointName     = inStrings[1]
    local jointDeltaPos = inFloats[1]
    local fullRobotName = simGetObjectName(robotHandle)
 
    simSetStringSignal(fullRobotName..'moveJoint', simPackFloatTable(inFloats))
   
	return {},{},{},''
end

reloadFallingObjects=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle   = inInts[1]
    local fullRobotName = simGetObjectName(robotHandle)
    local objectNames   = inStrings  
    --for i=1,#objectNames,1 do
    --    simAddStatusbarMessage("Object Names:"..objectNames[i])                
    --end
    
    fallingObjectNames = objectNames
    posZ=posZ+0.5
    removeFallingObjects()
    loadFallingObjects() 
   
	return {},{},{},''
end

detectCollisionWith=function(inInts,inFloats,inStrings,inBuffer)
    local robotHandle  = inInts[1]
    local objectName   = inStrings[1]    
    local objectHandle = simGetObjectHandle(objectName) 
    local ret = simCheckCollision(robotHandle, objectHandle)
	return {ret},{},{},''
end

detectObjectsOnGround=function(inInts,inFloats,inStrings,inBuffer)
    local floorHandle = simGetObjectHandle('ResizableFloor_5_25')
    local pos = {0,0,0}
    local ret = 1
    for i=1,#fallingObjectNames,1 do
        --local objectHandle = simGetObjectHandle(fallingObjectNames[i])
        --ret = simCheckCollision(floorHandle, objectHandle)
        pos = getObjPos(fallingObjectNames[i])
        if(pos[3] > 0.1)then
            ret = 0
            break
        end
    end

	return {ret},{},{},''
end

if (sim_call_type==sim_childscriptcall_initialization) then

    -- Put some initialization code here
    simExtRemoteApiStart(19999)
    posX = 0.25
    posY = 0
    posZ = 0.5

    model=simGetObjectAssociatedWithScript(sim_handle_self)
    modelName=simGetObjectName(model)
    fallingObjectNames = {'Obj1', 'Obj2', 'Obj3', 'Obj4', 'Obj5'}

    -- Pre-load falling objects
    loadFallingObjects()

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