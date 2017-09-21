-- This is a threaded script, and is just an example!

onMoveJointSignal=function()
    local moveJointSignal=simGetStringSignal(modelName..'moveJoint')    
    if moveJointSignal and #moveJointSignal>0 then        
        moveJointSignal = simUnpackFloatTable(moveJointSignal)
        local jointDeltaPos = moveJointSignal[1]
        --simAddStatusbarMessage('ABC: '..jointDeltaPos)
        simSetThreadAutomaticSwitch(false)            
        ---------------------------------------------------------------
        --for i=1,#path/6,1 do
        --    for j=1,6,1 do
        --        simSetJointPosition(jh[j],path[(i-1)*6+j])
        --    end
        --    simSwitchThread()
        --end
        ---------------------------------------------------------------
        targetPos[1]=targetPos[1]+jointDeltaPos
        simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos,targetVel)
        
        --targetPos2={-90*math.pi/180,90*math.pi/180,180*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180,0}
        --simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)

        --targetPos3={0,0,0,0,0,0,0}
        --simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
        simSwitchThread()        
        ---------------------------------------------------------------
        simSetThreadAutomaticSwitch(false)
        simClearStringSignal(modelName..'moveJoint')
    end
    simSwitchThread()
end

threadFunction=function()
    while simGetSimulationState()~=sim_simulation_advancing_abouttostop do
        
        -- Thread tasks
        --doBoxOperation()
        ----------------------------------------------------------------------------------------------
        -- Signals from non-thread script
        --
        -- Move Joint Signal from non-thread script
        onMoveJointSignal()
    end
end

-- Initialization:
model=simGetObjectAssociatedWithScript(sim_handle_self)
modelName=simGetObjectName(model)

-- Set-up some of the RML vectors:
jointHandles={-1,-1,-1,-1,-1,-1,-1}
for i=1,7,1 do
    jointHandles[i]=simGetObjectHandle('LBR4p_joint'..i)
end

-- Set-up some of the RML vectors:
vel=110
accel=40
jerk=80
currentVel={0,0,0,0,0,0,0}
currentAccel={0,0,0,0,0,0,0}
maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
targetVel={0,0,0,0,0,0,0}
-- Always return 3 tables and a string, e.g.:

targetPos={90*math.pi/180,90*math.pi/180,170*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180,0}
simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos,targetVel)
  
-- Main function:
res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

