-- This is a threaded script!

loadFallingObjects=function()
    simAddStatusbarMessage("Fall-load"..fallingObjectNames[1])
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
    simAddStatusbarMessage("Fall-remove"..fallingObjectNames[1])
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


activateSuctionPad=function(active)
    if (active) then
        simSetScriptSimulationParameter(suctionPadScript,'active','true')
    else
        simSetScriptSimulationParameter(suctionPadScript,'active','false')
    end
end

getTargetPosVectorFromObjectPose=function(objectHandle)
    local p=simGetObjectPosition(objectHandle,targetBase)
    local o=simGetObjectQuaternion(objectHandle,targetBase)
    return p,o
end

getNextContainerIndex=function(index)
    index=index+1
    if index>3 then
        index=1
    end
    return index
end

getNextBoxIndex=function(index)
    index=index+1
    if index>2 then
        index=1
    end
    return index
end

doBoxOperation=function()
    -- 1. Pick-up a box:
    -- Go to approach pose near container X:
    targetP,targetO=getTargetPosVectorFromObjectPose(approaches[containerIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Go to grasp pose on box A:
    targetP,targetO=getTargetPosVectorFromObjectPose(boxes[boxIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Activate suction pad:
    activateSuctionPad(true)
    -- Go to approach pose near container X:
    targetP,targetO=getTargetPosVectorFromObjectPose(approaches[containerIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Go to initial pose:
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,initPos,initOr,nil)


    -- 2. Drop a box:
    -- Get the next container:
    containerIndex=getNextContainerIndex(containerIndex)
    -- Go to approach pose near container X+1:
    targetP,targetO=getTargetPosVectorFromObjectPose(approaches[containerIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Go to drop pose on container X+1:
    targetP,targetO=getTargetPosVectorFromObjectPose(drops[containerIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Deactivate suction pad:
    activateSuctionPad(false)
    -- Go to approach pose near container X+1:
    targetP,targetO=getTargetPosVectorFromObjectPose(approaches[containerIndex])
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,targetP,targetO,nil)
    -- Go to initial pose:
    simRMLMoveToPosition(target,targetBase,-1,nil,nil,maxVel,maxAccel,maxJerk,initPos,initOr,nil)


    -- 3. Now handle the other box:
    boxIndex=getNextBoxIndex(boxIndex)
    containerIndex=getNextContainerIndex(containerIndex)
end

onMoveJointSignal=function()
    local moveJointSignal=simGetStringSignal(modelName..'moveJoint')
    if moveJointSignal and #moveJointSignal>0 then
        local jointMoveInfo = moveJointSignal

        simSetThreadAutomaticSwitch(false)            
        ---------------------------------------------------------------
        --for i=1,#path/6,1 do
        --    for j=1,6,1 do
        --        simSetJointPosition(jh[j],path[(i-1)*6+j])
        --    end
        --    simSwitchThread()
        --end
        local jointName     = jointMoveInfo[1]
        local jointDeltaPos = jointMoveInfo[2]
        local jointHandle   = simGetObjectHandle(jointName)
        local jointCurPos   = simGetJointPosition(jointHandle)
        simAddStatusbarMessage("String Signal Rev"..jointName..jointDeltaPos)    
        simSetJointPosition(jointHandle, jointCurPos+jointDeltaPos)

        ---------------------------------------------------------------
        simSetThreadAutomaticSwitch(false)
        simClearStringSignal(modelName..'moveJoint')
    end
end

onReloadFallingObjectsSignal=function()
    local reloadObjectsSignal=simGetStringSignal(modelName..'reloadFallingObjects')
    if reloadObjectsSignal and #reloadObjectsSignal>0 then
        fallingObjectNames=reloadObjectsSignal
        simSetThreadAutomaticSwitch(false)            
        ---------------------------------------------------------------

        posZ=posZ+0.5
        removeFallingObjects()
        loadFallingObjects()    

        simSetThreadAutomaticSwitch(false)
        simClearStringSignal(modelName..'reloadFallingObjects')
    end
end

threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        
        -- Thread tasks
        doBoxOperation()
        ----------------------------------------------------------------------------------------------
        -- Signals from non-thread script
        --
        -- Move Joint Signal from non-thread script
        onMoveJointSignal()

        -- Reload Falling Objects Signal
        onReloadFallingObjectsSignal()        
    end
end

-- Initialization:
model=simGetObjectAssociatedWithScript(sim_handle_self)
modelName=simGetObjectName(model)

simSetThreadSwitchTiming(2) 

suctionPad=simGetObjectHandle('suctionPad')
suctionPadScript=simGetScriptAssociatedWithObject(suctionPad)

target=simGetObjectHandle('RobotTarget')
targetBase=simGetObjectHandle('Robot1')

box1=simGetObjectHandle('Cuboid1Grasp')
box2=simGetObjectHandle('Cuboid2Grasp')
boxes={box1,box2}

drop1=simGetObjectHandle('CuboidDrop1')
drop2=simGetObjectHandle('CuboidDrop2')
drop3=simGetObjectHandle('CuboidDrop3')
drops={drop1,drop2,drop3}

approach1=simGetObjectHandle('CuboidApproach1')
approach2=simGetObjectHandle('CuboidApproach2')
approach3=simGetObjectHandle('CuboidApproach3')
approaches={approach1,approach2,approach3}

-- targetSphere is the object that the robot's tooltip will try to follow via IK, which means that
-- if you change the position/orientation of targetSphere, then the robot's tooltip will try to follow
-- targetSphereBase is used so that all position and orientation values are always relative to the robot base
-- (e.g. so that if you move the robot to another position, you will not have to rewrite this code!)

-- Get the current position and orientation of the robot's tooltip:
initPos=simGetObjectPosition(target,targetBase)
initOr=simGetObjectQuaternion(target,targetBase)

-- Set-up some of the RML vectors:
maxVel={0.4,0.4,0.4,1.8}
maxAccel={0.3,0.3,0.3,0.9}
maxJerk={0.2,0.2,0.2,0.8}

activateSuctionPad(false)
boxIndex=2
containerIndex=2

posX = 0.25
posY = 0
posZ = 0.5

fallingObjectNames = {'Obj1', 'Obj2', 'Obj3', 'Obj4', 'Obj5'}

-- Pre-load falling objects
loadFallingObjects()

-- Here we execute the regular thread code:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

-- Clean-up:



