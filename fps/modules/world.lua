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
local world={}; world.__index=world

-------------------------------------------------------------------------------

function world.new()
	return setmetatable({
		gravity = {0,-100,0},
		bodies  = {},
		solvers = {}
	},world)
end

function world.set_gravity(world_,x,y,z)
	local gravity=world_.gravity
	
	gravity[1],gravity[2],gravity[3]=x,y,z
end

function world.get_gravity(world_)
	local gravity=world_.gravity
	
	return gravity[1],gravity[2],gravity[3]
end

function world.add_body(world_,body_)
	assert(
		body_.world==nil,
		"Body is already parented to a world."
	)
	
	body_.world=world_
	world_.bodies[#world_.bodies+1]=body_
	
	--Todo: add to spatial partition
end

function world.remove_body(world_,body_)
	assert(
		body_.world==world_,
		"Body does not exist in this world."
	)
	
	body_.world=nil
	
	local nb=#world_.bodies
	for i=nb,1,-1 do
		if world_.bodies[i]==body_ then
			world_.bodies[i]  = world_.bodies[nb]
			world_.bodies[nb] = nil
			break
		end
	end
	
	--Todo: remove from spatial partition
end

function world.add_solver(world_,solver_)
	world_.solvers[#world_.solvers+1]=solver_
end

function world.remove_solver(world_,solver_)
	local ns=#world_.solvers
	for i=ns,1,-1 do
		if world_.solvers[i]==solver_ then
			world_.solvers[i]  = world_.solvers[ns]
			world_.solvers[ns] = nil
			break
		end
	end
end

function world.raycast(world_,x,y,z,dx,dy,dz,l,ignore)
	local closest_body
	local closest_collider
	local px,py,pz
	local snx,sny,snz
	local m
	
	for i=1,#world_.bodies do
		local body_=world_.bodies[i]
		local ignored=false
		
		if ignore then
			for j=1,#ignore do
				if ignore[j]==body_ then
					ignored=true
					break
				end
			end
		end
		
		if not ignored then
			local collider_,cx,cy,cz,nx,ny,nz,cl=body_:raycast(
				x,y,z,
				dx,dy,dz,
				l
			)
			
			if collider_ and (not closest_body or cl<m) then
				closest_body=body_
				closest_collider=collider_
				px,py,pz=cx,cy,cz
				snx,sny,snz=nx,ny,nz
				m=cl
			end
		end
	end
	
	return
		closest_body,
		closest_collider,
		px,py,pz,
		snx,sny,snz,
		m
end

function world.step(world_,dt)
	local gravity=world_.gravity
	
	--Apply gravity
	for _,body_ in ipairs(world_.bodies) do
		local force = body_.force
		local mass  = body_.mass
		
		force[1] = force[1]+gravity[1]*mass
		force[2] = force[2]+gravity[2]*mass
		force[3] = force[3]+gravity[3]*mass
	end
	
	--Resolve collisions
	for _,body_ in ipairs(world_.bodies) do
		body_.colliding=false
	end
	
	for _,body_a in ipairs(world_.bodies) do
		for _,body_b in ipairs(world_.bodies) do
			if body_a~=body_b then
				body_a:resolve_collision(
					body_b,
					dt,
					world_.solvers
				)
			end
		end
		
		body_a:step(dt)
	end
end

-------------------------------------------------------------------------------

return world
end