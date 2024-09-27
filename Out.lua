local event, phys = os.pullEvent("physics_tick")

print("List of ForcesInducers:")
for _, inducer in pairs(phys.getForcesInducers()) do
    print("&nbsp" .. inducer)
end