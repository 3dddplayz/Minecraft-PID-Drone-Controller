os.loadAPI("lib/quaternions.lua")

local pos = ship.getShipyardPosition()
local mass = ship.getMass()
local vel = ship.getVelocity()
local omega = ship.getOmega()
local targetVel = {}
local rot = ship.getQuaternion()
targetVel.x = 0
targetVel.y = 0
targetVel.z = 0
local roll,pitch,yaw = 0
local flyMode = false
local fPressed = false

local targetRot = {}
targetRot.r = 0
targetRot.p = 0
targetRot.y = 0

local targetRotAng = {}
targetRotAng.r = 0
targetRotAng.p = 0
targetRotAng.y = 0

local rotOffset = {}
rotOffset.r = 0
rotOffset.y = 0
rotOffset.p = 0

local err = 0;
local opt = {};
opt.x,opt.y,opt.z = 0
gravity = -10;
local lastT = 0


local pidcontrollers = require "ccvs.pidcontrollers"
local controller = peripheral.find("tweaked_controller")

local P = .65
local I = 0
local D = 0
local loopDelay = .05/3

--used for integral clamping--
local minimum_value = -500 
local maximum_value = 500 --create shaft limits: 256 redstone limits: 15

local pidX = pidcontrollers.PID_Continuous_Scalar(P, I, D, minimum_value, maximum_value)
local pidY = pidcontrollers.PID_Continuous_Scalar(P, I, D, minimum_value, maximum_value)
local pidZ = pidcontrollers.PID_Continuous_Scalar(P, I, D, minimum_value, maximum_value)

local Pr = 4
local Ir = 0
local Dr = 0

--used for integral clamping--
local minimum_value_r = -500 
local maximum_value_r = 500 --create shaft limits: 256 redstone limits: 15

local pidR = pidcontrollers.PID_Continuous_Scalar(Pr, Ir, Dr, minimum_value_r, maximum_value_r)
local pidP = pidcontrollers.PID_Continuous_Scalar(Pr, Ir, Dr, minimum_value_r, maximum_value_r)
local pidY = pidcontrollers.PID_Continuous_Scalar(Pr, Ir, Dr, minimum_value_r, maximum_value_r)

local PrA = 3 
local IrA = 0
local DrA = 0

--used for integral clamping--
local minimum_value_r = -500 
local maximum_value_r = 500 --create shaft limits: 256 redstone limits: 15

local pidRA = pidcontrollers.PID_Continuous_Scalar(PrA, IrA, DrA, minimum_value_r, maximum_value_r)
local pidPA = pidcontrollers.PID_Continuous_Scalar(PrA, IrA, DrA, minimum_value_r, maximum_value_r)
local pidYA = pidcontrollers.PID_Continuous_Scalar(PrA, IrA, DrA, minimum_value_r, maximum_value_r)

local time = os.clock()

function getRotError()
	local err = {}
	err.r = angleCheck(targetRot.z - omega.z)
	err.p = angleCheck(targetRot.x - omega.x)
	err.y = angleCheck(targetRot.y - omega.y)
	return err
end

function angleCheck(ang)
	local result = ang
	if(math.abs(ang)>math.abs(ang+360)) then
		result = ang+360
	elseif(math.abs(ang)>math.abs(ang-360)) then
		result = ang-360
	end
	return result
end

function angleCorrect(targetYaw)
	if(targetYaw>180) then
		targetYaw = targetYaw-360
	elseif(targetYaw<-180) then
		targetYaw = targetYaw+360
	end
	return targetYaw
end

function getTime()
	return os.clock()-time
end

function getOffsetDefaultShipOrientation(default_ship_orientation)
	offset_orientation = quaternions.Quaternion.fromRotation(default_ship_orientation:localPositiveX(), 0)*default_ship_orientation
	offset_orientation = quaternions.Quaternion.fromRotation(offset_orientation:localPositiveY(), 0)*offset_orientation
	offset_orientation = quaternions.Quaternion.fromRotation(offset_orientation:localPositiveZ(), 0)*offset_orientation
	return offset_orientation
end

--start button code
--while not redstone.getInput("top") do 
	--term.clear()
	--print("Waiting to Start")
	--sleep(0.05)
--end
--sleep(1.2)

--main loop
while not redstone.getInput("top") do
	local ti = getTime()
	local event, phys = os.pullEvent("physics_tick")
	local inertia = phys.getInertia()
	local poseVel = phys.getPoseVel()
	
	vel = poseVel.vel
	mass = inertia.mass
	roll = ship.getRoll()
	pitch = ship.getPitch()
	yaw = ship.getYaw()
	local rot = {}
	rot.r = angleCorrect(roll * (180/math.pi))
	rot.y = angleCorrect(yaw * (180/math.pi) + 0)
	rot.p = angleCorrect(pitch * (180/math.pi))
	
	quat = ship.getQuaternion()
	quat =  (quaternions.Quaternion.new(quat.w,quat.x,quat.y,quat.z))
	offQuat = quaternions.Quaternion.fromAngle(rotOffset)
	
	--Input Detection
	
	--input gains
	controller.setFullPrecision(true)
	local forwardGain = 50
	local backwardGain = 5
	local linearGain = 10
	local rotGain = 1
	local rotShutoff = 1
	
	local targetVeltemp = vector.new(0,0,0)
	local targetRotTemp = vector.new(0,0,0)
	
	if (controller.getButton(8) and not fPressed) then
		fPressed = true
		redstone.setAnalogOutput("right",15)
		if flyMode then flyMode = false 
		else flyMode = true end
	elseif controller.getButton(8) then controller.getButton(8)
	elseif not controller.getButton(5) then fPressed = false end
	
	
	
	
	if controller.getButton(1) then
		targetVeltemp.y = targetVeltemp.y + linearGain
	end
	if controller.getButton(2) then
		targetVeltemp.y = targetVeltemp.y - linearGain
	end
	
	if(controller.getAxis(4)>0) then forwardGain = backwardGain end
		
	targetVeltemp.x = -controller.getAxis(4) * forwardGain
	
	if not flyMode then 
		forwardGain = backwardGain
		rotShutoff = 0
		targetVeltemp.x = -controller.getAxis(2) * linearGain
		targetVeltemp.z = controller.getAxis(1) * linearGain
		targetRotTemp.x = pidPA:run(angleCheck(targetRotAng.p-pitch))
		targetRotTemp.z = pidRA:run(angleCheck(targetRotAng.r-roll))
	else
		targetRotTemp.x = controller.getAxis(1) * rotGain * rotShutoff
		targetRotTemp.z = controller.getAxis(2) * rotGain * rotShutoff
	end
	targetRotTemp.y = (controller.getAxis(5) - controller.getAxis(6)) * rotGain
	
	targetVel = offQuat:rotateVector3(quat:rotateVector3(targetVeltemp))
	
	
	
	
	local targetNewRot = offQuat:rotateVector3(quat:rotateVector3(targetRotTemp))
	
	targetRot = targetNewRot
	
	--movement code
	omega = ship.getOmega()
	local err = getRotError()
	optRt = {}
	optRt.r = pidR:run(err.r)
	optRt.p = pidP:run(err.p)
	optRt.y = pidY:run(err.y)
	
	optCalcRt = {}
	optCalcRt.r = optRt.r * inertia.momentOfInertiaTensor[1][1]
	optCalcRt.p = optRt.p * inertia.momentOfInertiaTensor[1][1]
	optCalcRt.y = optRt.y * inertia.momentOfInertiaTensor[1][1]
	
	ship.applyInvariantTorque(optCalcRt.p, optCalcRt.y, optCalcRt.r)
	
	opt.x = pidX:run(targetVel.x-vel.x)
	opt.y = pidY:run(targetVel.y-vel.y)
	opt.z = pidZ:run(targetVel.z-vel.z)
	local optCalc = {}
	optCalc.x = (opt.x) * mass
	optCalc.y = (opt.y-gravity) * mass
	optCalc.z = (opt.z) * mass
	ship.applyInvariantForce(optCalc.x, optCalc.y, optCalc.z)
	
	local coords = commands.getBlockPosition()
	--local blockInfo = commands.getBlockInfo(coords[1], coords[2], coords[3])

	print("X: " .. omega.x)
	print("Y: " .. omega.y)
	print("Z: " .. omega.z)
	
	
	print("Y vel: "..vel.y)
	print("rot err: "..textutils.serialize(err))
	print("err: "..targetVel.y-vel.y)
	print("last time: "..ti-lastT)
	
	print("time: "..getTime())
	print(redstone.getInput("front"))
	print("Vel: " .. textutils.serialize(targetVel))
	print("Rot: " .. textutils.serialize(targetRot))
	print("mass: "..mass)
	print("block: " .. textutils.serialize(coords))
	print("Rot: " .. textutils.serialize(rot))
	print(inertia.momentOfInertiaTensor[1][1])
	lastT = ti
	
	
	
end