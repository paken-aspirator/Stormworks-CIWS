s = math.sin
c = math.cos
his_x={}
his_y={}
his_z={}
xsums={0,0}--sumY,sumXY
ysums={0,0}
zsums={0,0}
pi=math.pi
rx, ry, rz, vx, vy, vz =0,0,0,0,0,0
maxHisLen=16
loge099 = -0.01005033585 -- log_e(0.99)
init_v = 1000.0 --Initial velocity of cannon
G = 30.0
rtol = 0.05
delay = 6 --[tick]
tick = 1/60.0
Z_offset = 2

azi_hist = azi_hist or {}
ele_hist = ele_hist or {}
cnt=0

function pushAngle(azi, ele)
	for i = 10, 1, -1 do
		azi_hist[i+1] = azi_hist[i]
		ele_hist[i+1] = ele_hist[i]
	end
	azi_hist[1] = azi
	ele_hist[1] = ele
end

function valid(x)
	return x == x and x < 1e30 and x > -1e30
end
function pushHis(x,y,z)
	if maxHisLen <= #his_x then
		xsums={xsums[1]-his_x[maxHisLen], xsums[2]-maxHisLen*his_x[maxHisLen]}
		ysums={ysums[1]-his_y[maxHisLen], ysums[2]-maxHisLen*his_y[maxHisLen]}
		zsums={zsums[1]-his_z[maxHisLen], zsums[2]-maxHisLen*his_z[maxHisLen]}
		table.remove(his_x, maxHisLen)
		table.remove(his_y, maxHisLen)
		table.remove(his_z, maxHisLen)
	end 
	for i = #his_x,1,-1 do
		his_x[i+1] = his_x[i]
		his_y[i+1] = his_y[i]
		his_z[i+1] = his_z[i]
	end
	his_x[1]=x
	his_y[1]=y
	his_z[1]=z
	xsums[1]=xsums[1]+x
	ysums[1]=ysums[1]+y
	zsums[1]=zsums[1]+z
	xsums[2]=xsums[2]+xsums[1]
	ysums[2]=ysums[2]+ysums[1]
	zsums[2]=zsums[2]+zsums[1]
end

function linReg3()
	local N=#his_x
	if N <=1 then
		return {0, his_x[1] or 0, 0, his_y[1] or 0, 0, his_z[1] or 0}
	end

	local sumX = N*(N+1)/2
	local sumX2 = N*(N+1)*(2*N+1)/6
	local den = N * sumX2 - sumX * sumX

	local xa = (N * xsums[2] - sumX * xsums[1]) / den
	local ya = (N * ysums[2] - sumX * ysums[1]) / den
	local za = (N * zsums[2] - sumX * zsums[1]) / den

	local xb = (xsums[1] - xa * sumX) / N
	local yb = (ysums[1] - ya * sumX) / N
	local zb = (zsums[1] - za * sumX) / N

	return {-xa * 60,xb,-ya * 60,yb,-za * 60,zb}
end

function deltaF(t)
	local dist = math.sqrt((rx+vx*t)*(rx+vx*t)+(ry+vy*t)*(ry+vy*t)+(rz+vz*t+G*t*t/2)*(rz+vz*t+G*t*t/2))
	if 60 *dist * loge099 / init_v < -1 then 
		return -100
	end
	return math.log(1 + 60.0 * dist * loge099 / init_v, 0.99)/60.0 - (t - delay*tick)
end

function dfdt(t)
	local dist = math.sqrt((rx+vx*t)*(rx+vx*t)+(ry+vy*t)*(ry+vy*t)+(rz+vz*t+G*t*t/2)*(rz+vz*t+G*t*t/2))
	if init_v + 60 * dist * loge099 <= 0 then
		return 0
	end
	local bunsi = (rx+vx*t)*vx + (ry+vy*t)*vy + (rz+vz*t+G*t*t/2)*(vz+G*t)
	return bunsi / ((init_v + 60 * dist * loge099) * dist) - 1
end

function solve_t(t)
	for i = 1, 8 do
		local error = deltaF(t)
		local slope = dfdt(t)

		if error == -100 or slope == 0 then
			return -1
		end

		if error ~= error or slope ~= slope then
			return -1
		end

		local dt = error / slope

		-- 
		if dt > 1 then dt = 1 end
		if dt < -1 then dt = -1 end

		t = t - dt
		-- 
		if t < delay * tick then
			return -1
		end

		if math.abs(dt) < 0.02 then
			return t
		end
	end

	return -1
end

function init_t(x)
	return math.max(5.0*x/6000.0, (x-350.0)/400.0) + delay * tick
end

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
        self_X, self_Y, self_Z = 0,0,0
    else
        local aziRad = azi * pi * 2
        local eleRad = ele * pi * 2
    	local X = mindist * math.cos(aziRad) * math.cos(eleRad)
    	local Y = mindist * math.sin(aziRad) * math.cos(eleRad)
    	local Z = mindist * math.sin(eleRad)
        self_X, self_Y, self_Z = X,Y,Z
    end

	local raw_azi = input.getNumber(4) * pi * 2.0
	local raw_ele = input.getNumber(8) * pi * 2.0

	pushAngle(raw_azi, raw_ele)
	local rad_delay = 0
	rader_azi = azi_hist[1+rad_delay] or raw_azi
	rader_ele = ele_hist[1+rad_delay] or raw_ele
	output.setNumber(12,rader_azi)
	output.setNumber(16,rader_ele)
	output.setNumber(13,self_X)
	output.setNumber(14,self_Y)
	output.setNumber(15,self_Z)
	
	if self_X==0 and self_Y==0 and self_Z==0 then
		his_x={}
		his_y={}
		his_z={}
		xsums={0,0}
		ysums={0,0}
		zsums={0,0}
		cnt=cnt+1
		output.setNumber(1,0)
		if cnt >= 60 then
			output.setNumber(2,-rader_azi/pi/2.0)
			output.setNumber(3,-rader_ele/pi/2.0)
		else
			output.setNumber(2,0)
			output.setNumber(3,0)
		end
		return
	else
		cnt = 0
	end
	ship_x = self_X * c(rader_azi) * c(rader_ele) - self_Y * s(rader_azi) - self_Z * c(rader_azi) * s(rader_ele)
	ship_y = self_X * s(rader_azi) * c(rader_ele) + self_Y * c(rader_azi) - self_Z * s(rader_azi) * s(rader_ele)
	ship_z = self_X * s(rader_ele) + self_Z * c(rader_ele) + Z_offset
	
	local diff = math.sqrt((rx-ship_x)^2 + (ry-ship_y)^2 + (rz-ship_z)^2)
	local prev_dist = math.sqrt(rx^2 + ry^2 + rz^2)
	if #his_x<4 or  diff / math.max(prev_dist, 1) < rtol then
		pushHis(ship_x, ship_y, ship_z)
	else
		his_x={0}
		his_y={0}
		his_z={0}
		xsums={0,0}
		ysums={0,0}
		zsums={0,0}
		pushHis(ship_x, ship_y, ship_z)
	end
	output.setNumber(5,100+#his_x)
	vx,rx,vy,ry,vz,rz = table.unpack(linReg3())
	local dist = math.sqrt(rx*rx + ry*ry + rz*rz)
	local velo = math.sqrt(vx*vx + vy*vy + vz*vz)
	
	if dist > 1000 then --track the target
		target_x, target_y, target_z = rx, ry, rz
		output.setNumber(4,403)
	else  --point at the future trajectory
		local t = solve_t(init_t(dist))
		if t < 0 then
			target_x, target_y, target_z = rx, ry, rz
			output.setNumber(4,404)
		else
			target_x, target_y, target_z = rx+vx*t, ry+vy*t, rz+vz*t+G*t*t/2.0
			output.setNumber(4,200)
		end
	end
	output.setNumber(6,target_x)
	output.setNumber(7,target_y)
	output.setNumber(8,target_z)
	
	local dist = math.sqrt(target_x*target_x + target_y*target_y + target_z*target_z)
	target_ele = math.asin(target_z/dist)
	target_azi = math.atan(target_y,target_x)
	output.setNumber(1,dist)
	output.setNumber(2,(target_azi-rader_azi)/pi/2.0)
	output.setNumber(3,(target_ele-rader_ele)/pi/2.0)
	output.setNumber(9,dist)
	output.setNumber(10,target_azi)
	output.setNumber(11,target_ele)
end
