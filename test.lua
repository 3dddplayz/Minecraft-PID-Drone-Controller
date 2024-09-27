local lastT = 0
local time = os.clock()
local function getTime()
	return os.clock()-time
end

while true do
	local ti = getTime()
    local event, phys = os.pullEvent("physics_tick")
	local inertia = phys.getInertia()
    ship.applyInvariantForce(0, 10 * ship.getMass(), 0)
	print("last time: "..ti-lastT)
	lastT = ti
	
	
end