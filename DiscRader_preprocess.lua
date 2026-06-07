pi = math.pi
function onTick()
    local mindist = math.huge
    local azi = 0
    local ele = 0
    for i = 1, 8 do
        if input.getBool(i) then
            local dist = input.getNumber(i * 4 - 3)

            if mindist > dist then
                mindist = dist
                azi = input.getNumber(i * 4 - 2)
                ele = input.getNumber(i * 4 - 1)
            end
        end
    end
    if mindist == math.huge then
        output.setNumber(1, 10000)
        output.setNumber(2, 10000)
        output.setNumber(3, 10000)
    else
        local aziRad = azi * pi * 2
        local eleRad = ele * pi * 2
    	local X = mindist * math.cos(aziRad) * math.cos(eleRad)
    	local Y = mindist * math.sin(aziRad) * math.cos(eleRad)
    	local Z = mindist * math.sin(eleRad)
        output.setNumber(1, X)
        output.setNumber(2, Y)
        output.setNumber(3, Z)
    end
end