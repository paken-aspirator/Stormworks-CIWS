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
delay = 4 --[tick]
tick = 1/60.0
Z_offset = 2
disc_offset = 0.75 --[m]
PD_P = 6
PD_D = 15

azi_hist = azi_hist or {}
ele_hist = ele_hist or {}
prev_azi_error = nil
prev_ele_error = nil
prev_target_azi = nil
prev_target_ele = nil
cnt=0

function valid(x)
	return x == x and x < 1e30 and x > -1e30
end

function clamp(a, lo, hi)
	return math.max(lo,math.min(a,hi))
end

function wrapAngle(a)
	return (a + pi) % (pi * 2) - pi
end

function updateControl(target_azi, target_ele, rader_azi, rader_ele)
	local azi_error = wrapAngle(target_azi - rader_azi) / pi / 2.0
	local ele_error = wrapAngle(target_ele - rader_ele) / pi / 2.0
	local azi_d = prev_azi_error and azi_error - prev_azi_error or 0
	local ele_d = prev_ele_error and ele_error - prev_ele_error or 0
	local azi_ff = prev_target_azi and wrapAngle(target_azi - prev_target_azi) / pi / 2.0 or 0
	local ele_ff = prev_target_ele and wrapAngle(target_ele - prev_target_ele) / pi / 2.0 or 0

	prev_azi_error = azi_error
	prev_ele_error = ele_error
	prev_target_azi = target_azi
	prev_target_ele = target_ele

	output.setNumber(2, PD_P * azi_error + PD_D * azi_d + azi_ff)
	output.setNumber(3, PD_P * ele_error + PD_D * ele_d + ele_ff)
end

function resetControl()
	prev_azi_error = nil
	prev_ele_error = nil
	prev_target_azi = nil
	prev_target_ele = nil
end

function xyzToPolar(x,y,z)
	local r = math.sqrt(x*x + y*y + z*z)
	if r == 0 then
		return 0, 0, 0
	end
	return r, math.atan(y,x), math.asin(clamp(z/r, -1, 1))
end

function pushAngle(azi, ele)
	for i = 10, 1, -1 do
		azi_hist[i+1] = azi_hist[i]
		ele_hist[i+1] = ele_hist[i]
	end
	azi_hist[1] = azi
	ele_hist[1] = ele
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

discRadDelay=3
function updateHis(x,y,z)
	if #his_x <= discRadDelay then
		return
	end
    local nr, na, ne = xyzToPolar(x, y, z)
local or_, oa, oe = xyzToPolar(
	his_x[1 + discRadDelay],
	his_y[1 + discRadDelay],
	his_z[1 + discRadDelay]
)

    if nr > 0 and math.abs(nr - or_) / nr < 0.041
	    and math.abs(na - oa) < 0.02513
	    and math.abs(ne - oe) < 0.02513 then
		local old_x, old_y, old_z = his_x[1 + discRadDelay], his_y[1 + discRadDelay], his_z[1 + discRadDelay]
		local new_x, new_y, new_z = (3*x + old_x)/4, (3*y + old_y)/4, (3*z + old_z)/4
		xsums={xsums[1]-old_x+new_x, xsums[2]+(new_x-old_x)*(1 + discRadDelay)}
        ysums={ysums[1]-old_y+new_y, ysums[2]+(new_y-old_y)*(1 + discRadDelay)}
        zsums={zsums[1]-old_z+new_z, zsums[2]+(new_z-old_z)*(1 + discRadDelay)}
		his_x[1 + discRadDelay]=new_x
		his_y[1 + discRadDelay]=new_y
		his_z[1 + discRadDelay]=new_z
	end
end

function linReg3() --linear regression
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

function travelScale(tf)
	-- tf: 発射後の弾の飛翔時間 [s]
	-- 空気抵抗込みで、初速方向成分がどれだけ距離に変換されるか
	local lambda = -60.0 * loge099
	return (1.0 - math.exp(-lambda * tf)) / lambda
end

function dragDrop(tf)
	-- tf: 発射後の弾の飛翔時間 [s]
	-- 空気抵抗込みの重力落下量
	local lambda = -60.0 * loge099
	return G * (tf / lambda - (1.0 - math.exp(-lambda * tf)) / (lambda * lambda))
end

function deltaF(t)
	-- t: レーダー測定時刻から命中までの総時間 [s]
	local d = delay * tick
	local tf = t - d

	if tf <= 0 then
		return 1e9
	end

	-- t秒後の目標位置
	local px = rx + vx * t
	local py = ry + vy * t
	local pz = rz + vz * t

	-- 弾がtf秒で進める空気抵抗込みの距離係数
	local A = travelScale(tf)
	local can = init_v * A

	-- 重力で落ちる分だけ上を狙う
	local drop = dragDrop(tf)

	-- 必要な発射方向ベクトルの長さに相当
	local need = math.sqrt(px*px + py*py + (pz + drop)*(pz + drop))

	return need - can
end

function solve_t(t)
	-- tは「測定時刻から命中までの総時間」
	local d = delay * tick

	-- 初期値が遅延以下だと物理的に不可能
	if t <= d then
		t = d + 0.05
	end
	-- 初期値が大きすぎないように抑える
	if t > 5.0 + d then
		t = 5.0 + d
	end

	-- 数値微分つきニュートン法
	for i = 1, 8 do
		local f = deltaF(t)

		if f ~= f or math.abs(f) > 1e30 then
			return -1
		end

		if math.abs(f) < 0.5 then
			return t
		end

		local h = 0.02
		local fp = deltaF(t + h)
		local fm = deltaF(t - h)

		if fp ~= fp or fm ~= fm then
			return -1
		end

		local slope = (fp - fm) / (2.0 * h)

		if math.abs(slope) < 0.0001 then
			return -1
		end

		local dt = f / slope

		-- 発散防止
		if dt > 1.0 then dt = 1.0 end
		if dt < -1.0 then dt = -1.0 end

		t = t - dt

		if t <= d then
			t = d + 0.01
		end

		if math.abs(dt) < 0.005 then
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
			updateControl(0, 0, rader_azi, rader_ele)
		else
			resetControl()
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

    local discX,discY,discZ = input.getNumber(12), input.getNumber(16), input.getNumber(20)
	local azi_past, ele_past = azi_hist[1+discRadDelay], ele_hist[1+discRadDelay]
    if azi_past and ele_past then
	updateHis(discX * c(azi_past) * c(ele_past) - discY * s(azi_past) - discZ * c(azi_past) * s(ele_past), discX * s(azi_past) * c(ele_past) + discY * c(azi_past) - discZ * s(azi_past) * s(ele_past), discX * s(ele_past) + discZ * c(ele_past) + disc_offset)
    end
	
	local diff = math.sqrt((rx-ship_x)^2 + (ry-ship_y)^2 + (rz-ship_z)^2)
	local prev_dist = math.sqrt(rx^2 + ry^2 + rz^2)
	if #his_x<4 or  diff / math.max(prev_dist, 1) < rtol then
		pushHis(ship_x, ship_y, ship_z)
	else
		his_x={}
		his_y={}
		his_z={}
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
	
    local dist, target_azi, target_ele = xyzToPolar(target_x, target_y, target_z)
	output.setNumber(1,dist)
	updateControl(target_azi, target_ele, rader_azi, rader_ele)
	output.setNumber(9,dist)
	output.setNumber(10,target_azi)
	output.setNumber(11,target_ele)
end
