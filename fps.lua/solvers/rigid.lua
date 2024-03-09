--[[                                                    
MIT License

Copyright (c) 2021 Shoelee

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
]]

return function(fps)
local vector3 = fps.vector3
local matrix3 = fps.matrix3
local matrix4 = fps.matrix4

-------------------------------------------------------------------------------

local math_sqrt = math.sqrt
local math_abs  = math.abs

local vector3_dot   = vector3.dot
local vector3_cross = vector3.cross

local matrix3_multiply         = matrix3.multiply
local matrix3_multiply_vector3 = matrix3.multiply_vector3

-------------------------------------------------------------------------------

return function(
	body_a,body_b,
	collider_a,collider_b,
	collision_x,collision_y,collision_z,
	normal_x,normal_y,normal_z,depth
)
	local av = body_a.velocity
	local bv = body_b.velocity
	
	local aav = body_a.angular_velocity
	local bav = body_b.angular_velocity
	
	local restitution=(
		collider_a:get_restitution()*
		collider_b:get_restitution()
	)
	
	local friction=1-(
		collider_a:get_friction()*
		collider_b:get_friction()
	)
	
	--Inverse mass
	local ima = body_a.mass^-1
	local imb = body_b.mass^-1
	
	--Total inverse mass
	local tim=ima+imb
	
	local at = body_a.transform
	local bt = body_b.transform
	
	--Seperate bodies
	if not body_a.static then
		local depth_ratio_a = depth*(ima/tim)
		
		at[4]  = at[4]-normal_x*depth_ratio_a
		at[8]  = at[8]-normal_y*depth_ratio_a
		at[12] = at[12]-normal_z*depth_ratio_a
	end
	if not body_b.static then
		local depth_ratio_b = depth*(imb/tim)
		
		bt[4]  = bt[4]+normal_x*depth_ratio_b
		bt[8]  = bt[8]+normal_y*depth_ratio_b
		bt[12] = bt[12]+normal_z*depth_ratio_b
	end
	
	--Relative position to collision point
	local arx = collision_x-at[4]
	local ary = collision_y-at[8]
	local arz = collision_z-at[12]
	
	local brx = collision_x-bt[4]
	local bry = collision_y-bt[8]
	local brz = collision_z-bt[12]
	
	--Angular velocity
	local aavx,aavy,aavz=vector3_cross(
		aav[1],aav[2],aav[3],
		arx,ary,arz
	)
	local bavx,bavy,bavz=vector3_cross(
		bav[1],bav[2],bav[3],
		brx,bry,brz
	)
	
	--Full velocity
	local afvx=av[1]+aavx
	local afvy=av[2]+aavy
	local afvz=av[3]+aavz
	
	local bfvx=bv[1]+bavx
	local bfvy=bv[2]+bavy
	local bfvz=bv[3]+bavz
	
	local cvx = bfvx-afvx
	local cvy = bfvy-afvy
	local cvz = bfvz-afvz
	
	local impulse_force=vector3_dot(
		bfvx-afvx,bfvy-afvy,bfvz-afvz,
		normal_x,normal_y,normal_z
	)
	
	--Inertia tensor
	local
	aiit11,aiit12,aiit13,
	aiit21,aiit22,aiit23,
	aiit31,aiit32,aiit33
	=body_a:get_inverse_inertia_tensor()
	
	local
	biit11,biit12,biit13,
	biit21,biit22,biit23,
	biit31,biit32,biit33
	=body_b:get_inverse_inertia_tensor()
	
	local aitx,aity,aitz=matrix3_multiply_vector3(
		aiit11,aiit12,aiit13,
		aiit21,aiit22,aiit23,
		aiit31,aiit32,aiit33,
		vector3_cross(
			arx,ary,arz,
			normal_x,normal_y,normal_z
		)
	)
	local bitx,bity,bitz=matrix3_multiply_vector3(
		biit11,biit12,biit13,
		biit21,biit22,biit23,
		biit31,biit32,biit33,
		vector3_cross(
			brx,bry,brz,
			normal_x,normal_y,normal_z
		)
	)
	
	--Inertia
	local aix,aiy,aiz=vector3_cross(
		aitx,aity,aitz,
		arx,ary,arz
	)
	local bix,biy,biz=vector3_cross(
		bitx,bity,bitz,
		brx,bry,brz
	)
	
	local angular_effect=vector3_dot(
		aix+bix,aiy+biy,aiz+biz,
		normal_x,normal_y,normal_z
	)
	
	local j=(-(1+restitution)*impulse_force)/(tim+angular_effect)
	
	--Full impulse
	local fix=normal_x*j
	local fiy=normal_y*j
	local fiz=normal_z*j
	
	--Apply impulse
	body_a:apply_linear_impulse(-fix,-fiy,-fiz)
	body_b:apply_linear_impulse(fix,fiy,fiz)
	
	body_a:apply_angular_impulse(
		vector3_cross(arx,ary,arz,-fix,-fiy,-fiz)
	)
	body_b:apply_angular_impulse(
		vector3_cross(brx,bry,brz,fix,fiy,fiz)
	)
end
end