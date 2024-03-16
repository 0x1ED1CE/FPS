--[[                                                    
MIT License

Copyright (c) 2024 Dice

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
local body={}; body.__index=body

-------------------------------------------------------------------------------

local vector3 = fps.vector3
local matrix3 = fps.matrix3
local matrix4 = fps.matrix4
local ngc     = fps.ngc

-------------------------------------------------------------------------------

local math_min  = math.min
local math_max  = math.max
local math_huge = math.huge
local math_sqrt = math.sqrt
local math_sin  = math.sin
local math_cos  = math.cos

local vector3_cross = vector3.cross
local vector3_dot   = vector3.dot
local vector3_unit  = vector3.unit

local matrix3_inverse          = matrix3.inverse
local matrix3_multiply         = matrix3.multiply
local matrix3_multiply_vector3 = matrix3.multiply_vector3

local matrix4_inverse  = matrix4.inverse
local matrix4_multiply = matrix4.multiply

-------------------------------------------------------------------------------

function body.new()
	return setmetatable({
		world                  = nil,
		sleeping               = false,
		static                 = true,
		responsive             = true,
		colliding              = false,
		mass                   = 0,
		mass_center            = {0,0,0},
		force                  = {0,0,0},
		torque                 = {0,0,0},
		velocity               = {0,0,0},
		angular_velocity       = {0,0,0},
		boundary               = {
			0,0,0,
			0,0,0
		},
		transform              = { --Column major
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,1
		},
		inverse_inertia_tensor = {
			1,0,0,
			0,1,0,
			0,0,1
		},
		inverse_inertia        = {0,0,0},
		colliders              = {}
	},body)
end

function body.set_static(body_,static)
	body_.static=static or false
end

function body.set_responsive(body_,responsive)
	body_.responsive=responsive or false
end

function body.set_force(body_,x,y,z)
	local f=body_.force
	
	f[1],f[2],f[3]=x,y,z
end

function body.set_velocity(body_,x,y,z)
	local v=body_.velocity
	
	v[1],v[2],v[3]=x,y,z
end

function body.set_angular_velocity(body_,x,y,z)
	local av=body_.angular_velocity
	
	av[1],av[2],av[3]=x,y,z
end

function body.set_transform(
	body_,
	a11,a12,a13,a14,
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44
)
	local t=body_.transform
	
	t[1],t[2],t[3],t[4]     = a11,a12,a13,a14
	t[5],t[6],t[7],t[8]     = a21,a22,a23,a24
	t[9],t[10],t[11],t[12]  = a31,a32,a33,a34
	t[13],t[14],t[15],t[16] = a41,a42,a43,a44
	
	body_:update_boundary()
end

function body.set_position(body_,x,y,z)
	local t=body_.transform
	
	t[4],t[8],t[12]=x,y,z
	
	body_:update_boundary()
end

function body.set_look(
	body_,
	front_x,front_y,front_z,
	up_x,up_y,up_z
)
	local t=body_.transform
	
	up_x,up_y,up_z=up_x or 0,up_y or 1,up_z or 0
	front_x,front_y,front_z=-front_x,-front_y,-front_z
	
	local right_x = up_y*front_z-up_z*front_y
	local right_y = up_z*front_x-up_x*front_z
	local right_z = up_x*front_y-up_y*front_x
	local right_m = math_sqrt(right_x^2+right_y^2+right_z^2)
	
	right_x = right_x/right_m
	right_y = right_y/right_m
	right_z = right_z/right_m
	
	local top_x = front_y*right_z-front_z*right_y
	local top_y = front_z*right_x-front_x*right_z
	local top_z = front_x*right_y-front_y*right_x
	
	t[1],t[2],t[3]   = right_x,top_x,front_x
	t[5],t[6],t[7]   = right_y,top_y,front_y
	t[9],t[10],t[11] = right_z,top_z,front_z
	
	body_:update_boundary()
end

function body.apply_force(body_,fx,fy,fz)
	if body_.static then
		return
	end
	
	local force=body_.force
	
	force[1]=force[1]+fx
	force[2]=force[2]+fy
	force[3]=force[3]+fz
end

function body.apply_offset_force(body_,fx,fy,fz,ox,oy,oz)
	if body_.static then
		return
	end
	
	local force=body_.force
	local torque=body_.torque
	
	local px,py,pz=body_:get_position()
	
	force[1]=force[1]+fx
	force[2]=force[2]+fy
	force[3]=force[3]+fz
	
	local fcpx,fcpy,fcpz=vector3_cross(
		fx,fy,fz,
		px-ox,py-oy,pz-oz
	)
	
	torque[1]=torque[1]+fcpx
	torque[2]=torque[2]+fcpy
	torque[3]=torque[3]+fcpz
end

function body.apply_torque(body_,tx,ty,tz)
	if body_.static then
		return
	end
	
	local torque=body_.torque
	
	torque[1]=torque[1]+tx
	torque[2]=torque[2]+ty
	torque[3]=torque[3]+tz
end

function body.apply_linear_impulse(body_,fx,fy,fz)
	if body_.static then
		return
	end
	
	local velocity=body_.velocity
	local inverse_mass=body_.mass^-1
	
	velocity[1]=velocity[1]+fx*inverse_mass
	velocity[2]=velocity[2]+fy*inverse_mass
	velocity[3]=velocity[3]+fz*inverse_mass
end

function body.apply_angular_impulse(body_,tx,ty,tz)
	if body_.static then
		return
	end
	
	local angular_velocity=body_.angular_velocity
	local iit=body_.inverse_inertia_tensor
	local ix,iy,iz=matrix3_multiply_vector3(
		iit[1],iit[2],iit[3],
		iit[4],iit[5],iit[6],
		iit[7],iit[8],iit[9],
		tx,ty,tz
	)
	
	angular_velocity[1]=(angular_velocity[1]+ix)*0.9
	angular_velocity[2]=(angular_velocity[2]+iy)*0.9
	angular_velocity[3]=(angular_velocity[3]+iz)*0.9
end

function body.get_static(body_)
	return body_.static
end

function body.get_responsive(body_)
	return body_.responsive
end

function body.get_force(body_)
	local f=body_.force
	
	return f[1],f[2],f[3]
end

function body.get_velocity(body_)
	local v=body_.velocity
	
	return v[1],v[2],v[3]
end

function body.get_angular_velocity(body_)
	local av=body_.angular_velocity
	
	return av[1],av[2],av[3]
end

function body.get_transform(body_)
	local t=body_.transform
	
	return
		t[1],t[2],t[3],t[4],
		t[5],t[6],t[7],t[8],
		t[9],t[10],t[11],t[12],
		t[13],t[14],t[15],t[16]
end

function body.get_position(body_)
	local t=body_.transform
	
	return t[4],t[8],t[12]
end

function body.get_mass(body_)
	return body_.mass
end

function body.get_inverse_inertia_tensor(body_)
	local iit=body_.inverse_inertia_tensor
	
	return
		iit[1],iit[2],iit[3],
		iit[4],iit[5],iit[6],
		iit[7],iit[8],iit[9]
end

function body.get_inverse_inertia(body_)
	local ii=body_.inverse_inertia
	
	return ii[1],ii[2],ii[3]
end

function body.add_collider(body_,collider_)
	assert(
		collider_.body==nil,
		"Collider is already parented to a body."
	)
	
	collider_.body=body_
	
	body_.colliders[#body_.colliders+1]=collider_
	
	body_:update_mass()
	body_:update_boundary()
end

function body.remove_collider(body_,collider_)
	assert(
		collider_.body==body_,
		"Cannot remove invalid collider from body."
	)
	
	collider_.body=nil
	
	local nc=#body_.colliders
	for i=nc,1,-1 do
		if body_.colliders[i]==collider_ then
			body_.colliders[i]  = body_.colliders[nc]
			body_.colliders[nc] = nil
			break
		end
	end
	
	body_:update_mass()
	body_:update_boundary()
end

function body.update_mass(body_)
	local colliders=body_.colliders
	local sc=#colliders
	
	local total_mass=0
	local cx,cy,cz=0,0,0
	
	for i=1,sc do
		local collider_=colliders[i]
		
		local ox,oy,oz=collider_:get_position_offset()
		local collider_mass=collider_:get_mass()
		
		total_mass=total_mass+collider_mass
		cx=cx+ox*collider_mass
		cy=cy+oy*collider_mass
		cz=cz+oz*collider_mass
	end
	
	body_.mass=total_mass
	body_.mass_center[1]=cx/total_mass
	body_.mass_center[2]=cy/total_mass
	body_.mass_center[3]=cz/total_mass
	
	body_:update_inverse_inertia()
end

function body.update_boundary(body_)
	local bb = body_.boundary
	local bt = body_.transform
	
	local colliders=body_.colliders
	
	local min_x,min_y,min_z
	local max_x,max_y,max_z	
	
	for i=1,#colliders do
		local collider_=colliders[i]
		
		local ct   = collider_.transform
		local size = collider_.size
		
		local
		t11,t12,t13,t14,
		t21,t22,t23,t24,
		t31,t32,t33,t34,
		t41,t42,t43,t44
		=matrix4.multiply(
			bt[1],bt[2],bt[3],bt[4],
			bt[5],bt[6],bt[7],bt[8],
			bt[9],bt[10],bt[11],bt[12],
			bt[13],bt[14],bt[15],bt[16],
			ct[1],ct[2],ct[3],ct[4],
			ct[5],ct[6],ct[7],ct[8],
			ct[9],ct[10],ct[11],ct[12],
			ct[13],ct[14],ct[15],ct[16]
		)
		
		for z=-0.5,0.5,1 do for y=-0.5,0.5,1 do for x=-0.5,0.5,1 do
			local vx,vy,vz=matrix4.multiply_vector3(
				t11,t12,t13,t14,
				t21,t22,t23,t24,
				t31,t32,t33,t34,
				t41,t42,t43,t44,
				x*size[1],y*size[2],z*size[3]
			)
			
			min_x = math_min(vx,min_x or vx)
			min_y = math_min(vy,min_y or vy)
			min_z = math_min(vz,min_z or vz)
			
			max_x = math_max(vx,max_x or vx)
			max_y = math_max(vy,max_y or vy)
			max_z = math_max(vz,max_z or vz)
		end end end
	end
	
	bb[1],bb[2],bb[3] = min_x,min_y,min_z
	bb[4],bb[5],bb[6] = max_x,max_y,max_z
	
	--Todo: Update spatial partition
end

function body.update_inverse_inertia(body_)
	local inverse_inertia=body_.inverse_inertia
	
	local colliders=body_.colliders
	
	local iix,iiy,iiz=0,0,0
	
	for i=1,#colliders do --Treats each mass as a cube
		local collider_=colliders[i]
		
		local cim=collider_:get_mass()^-1
		local sx,sy,sz=collider_:get_size()
		
		iix=iix+(12*cim)/(sx^2+sy^2)
		iiy=iiy+(12*cim)/(sx^2+sz^2)
		iiz=iiz+(12*cim)/(sx^2+sy^2)
	end
	
	inverse_inertia[1]=iix
	inverse_inertia[2]=iiy
	inverse_inertia[3]=iiz
	
	local iit=body_.inverse_inertia_tensor
	
	local
	t11,t12,t13,_,
	t21,t22,t23,_,
	t31,t32,t33
	=body_:get_transform()
	
	local
	it11,it12,it13,
	it21,it22,it23,
	it31,it32,it33
	=matrix3_inverse(
		t11,t12,t13,
		t21,t22,t23,
		t31,t32,t33
	)
	
	local
	ts11,ts12,ts13,
	ts21,ts22,ts23,
	ts31,ts32,ts33
	=matrix3_multiply(
		t11,t12,t13,
		t21,t22,t23,
		t31,t32,t33,
		iix,0,0,
		0,iiy,0,
		0,0,iiz
	)
	
	iit[1],iit[2],iit[3],
	iit[4],iit[5],iit[6],
	iit[7],iit[8],iit[9]
	=matrix3_multiply(
		ts11,ts12,ts13,
		ts21,ts22,ts23,
		ts31,ts32,ts33,
		it11,it12,it13,
		it21,it22,it23,
		it31,it32,it33
	)
end

function body.raycast(body_,x,y,z,dx,dy,dz)
	local t=body_.transform
	
	local 
	i11,i12,i13,i14,
	i21,i22,i23,i24,
	i31,i32,i33,i34,
	i41,i42,i43,i44
	=matrix4.inverse(
		t[1],t[2],t[3],t[4],
		t[5],t[6],t[7],t[8],
		t[9],t[10],t[11],t[12],
		t[13],t[14],t[15],t[16]
	)
	
	--Convert to body space
	x,y,z=matrix4.multiply_vector3(
		i11,i12,i13,i14,
		i21,i22,i23,i24,
		i31,i32,i33,i34,
		i41,i42,i43,i44,
		x,y,z
	)
	dx,dy,dz=matrix4.multiply_vector3(
		i11,i12,i13,0,
		i21,i22,i23,0,
		i31,i32,i33,0,
		i41,i42,i43,i44,
		dx,dy,dz
	)
	
	local nearest_collider,m
	local px,py,pz,snx,sny,snz
	
	for i=1,#body_.colliders do
		local collider_=body_.colliders[i]
		
		local cx,cy,cz,nx,ny,nz,l=collider_:raycast(
			x,y,z,
			dx,dy,dz,
			l
		)
		
		if l and (not nearest_collider or l<m) then
			nearest_collider=collider_
			px,py,pz=cx,cy,cz
			snx,sny,snz=nx,ny,nz
			m=l
		end
	end
	
	if nearest_collider then
		--Convert to world space
		px,py,pz=matrix4.multiply_vector3(
			t[1],t[2],t[3],t[4],
			t[5],t[6],t[7],t[8],
			t[9],t[10],t[11],t[12],
			t[13],t[14],t[15],t[16],
			px,py,pz
		)
		snx,sny,snz=matrix4.multiply_vector3(
			t[1],t[2],t[3],0,
			t[5],t[6],t[7],0,
			t[9],t[10],t[11],0,
			t[13],t[14],t[15],t[16],
			snx,sny,snz
		)
	end
	
	return
		nearest_collider,
		px,py,pz,
		snx,sny,snz,
		m
end

function body.resolve_collision(body_a,body_b,dt,solvers)
	local ab,bb=body_a.boundary,body_b.boundary
	
	if body_a.static and body_b.static then
		return
	end
	if not body_a.responsive or not body_b.responsive then
		return
	end
	
	if not (
		ab[4]>bb[1] and
		ab[1]<bb[4] and
		ab[5]>bb[2] and
		ab[2]<bb[5] and
		ab[6]>bb[3] and
		ab[3]<bb[6]
	) then
		return
	end
	
	for a=1,#body_a.colliders do
		local collider_a=body_a.colliders[a]
		
		for b=1,#body_b.colliders do
			local collider_b=body_b.colliders[b]
			
			local cx,cy,cz,sx,sy,sz,sd=ngc(
				collider_a,
				collider_b
			)
			
			if sd>0 then
				body_a.colliding = true
				body_b.colliding = true
				
				for _,solver in ipairs(solvers) do
					solver(
						body_a,body_b,
						collider_a,collider_b,
						cx,cy,cz,
						sx,sy,sz,sd,
						dt
					)
				end
			
				body_a:update_boundary()
				body_b:update_boundary()
			end
		end
	end
end

function body.step(body_,dt)
	local t                = body_.transform
	local mass             = body_.mass
	local force            = body_.force
	local torque           = body_.torque
	local velocity         = body_.velocity
	local angular_velocity = body_.angular_velocity
	local mass_center      = body_.mass_center
	
	if not body_.sleeping and not body_.static then
		velocity[1] = velocity[1]+force[1]/mass*dt
		velocity[2] = velocity[2]+force[2]/mass*dt
		velocity[3] = velocity[3]+force[3]/mass*dt
		
		angular_velocity[1] = angular_velocity[1]+torque[1]/mass*dt
		angular_velocity[2] = angular_velocity[2]+torque[2]/mass*dt
		angular_velocity[3] = angular_velocity[3]+torque[3]/mass*dt
		
		local
		t11,t12,t13,t14,
		t21,t22,t23,t24,
		t31,t32,t33,t34,
		t41,t42,t43,t44
		=matrix4.translate(
			t[1],t[2],t[3],t[4],
			t[5],t[6],t[7],t[8],
			t[9],t[10],t[11],t[12],
			t[13],t[14],t[15],t[16],
			velocity[1]*dt,
			velocity[2]*dt,
			velocity[3]*dt
		)
		
		local avx = angular_velocity[1]*dt
		local avy = angular_velocity[2]*dt
		local avz = angular_velocity[3]*dt
		
		body_:set_transform(
			matrix4.multiply(
				t11,t12,t13,t14,
				t21,t22,t23,t24,
				t31,t32,t33,t34,
				t41,t42,t43,t44,
				matrix4.from_euler(
					avx,avy,avz
				)
			)
		)
	end
	
	force[1],force[2],force[3]    = 0,0,0
	torque[1],torque[2],torque[3] = 0,0,0
end

-------------------------------------------------------------------------------

return body
end