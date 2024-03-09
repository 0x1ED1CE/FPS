--[[
Naive Geometry Clipping

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

--[[
This algorithm is not SAT! It uses a different approach by clipping the 
colliding geometries and determines the collision based on what's left from 
the clipping. I don't know a proper name for this algorithm as I came up with 
it myself.
]]

return function(fps)
local vector3 = fps.vector3
local matrix4 = fps.matrix4

-------------------------------------------------------------------------------

local math_huge = math.huge
local math_sqrt = math.sqrt
local math_abs  = math.abs

local table_insert = table.insert
local table_remove = table.remove
local table_clear  = (
	table.clear or 
	function(t)
		for i=#t,1,-1 do
			t[i]=nil
		end
	end
)

local vector3_cross     = vector3.cross
local vector3_dot       = vector3.dot
local vector3_unit      = vector3.unit
local vector3_magnitude = vector3.magnitude

local matrix4_multiply_vector3 = matrix4.multiply_vector3

-------------------------------------------------------------------------------

local function plane_intersection(
	px,py,pz,    --Plane origin
	pnx,pny,pnz, --Plane normal
	lx,ly,lz,    --Edge origin
	lnx,lny,lnz  --Edge normal
)
	if vector3_dot(pnx,pny,pnz,lnx,lny,lnz)==0 then
		return
	end
	
	local t=(
		vector3_dot(pnx,pny,pnz,px,py,pz)-
		vector3_dot(pnx,pny,pnz,lx,ly,lz)
	)/vector3_dot(pnx,pny,pnz,lnx,lny,lnz)
	
	return lx+lnx*t,ly+lny*t,lz+lnz*t
end

local function clip_triangle(
	px,py,pz,    --Plane origin
	pnx,pny,pnz, --Plane normal
	ax,ay,az,    --Vertexes
	bx,by,bz,
	cx,cy,cz
)
	local rx,ry,rz
	
	local aex,aey,aez = vector3_unit(ax-px,ay-py,az-pz)
	local bex,bey,bez = vector3_unit(bx-px,by-py,bz-pz)
	local cex,cey,cez = vector3_unit(cx-px,cy-py,cz-pz)
	
	local a_dot = vector3_dot(aex,aey,aez,pnx,pny,pnz)
	local b_dot = vector3_dot(bex,bey,bez,pnx,pny,pnz)
	local c_dot = vector3_dot(cex,cey,cez,pnx,pny,pnz)
	
	if a_dot<0 then
		rx,ry,rz=ax,ay,az --Clip towards this point
	elseif b_dot<0 then
		rx,ry,rz=bx,by,bz
	elseif c_dot<0 then
		rx,ry,rz=cx,cy,cz
	else --All vertexes are behind the plane so no clipping needed
		return
	end
	
	if a_dot>0 then
		ax,ay,az=plane_intersection(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(ax-rx,ay-ry,az-rz)
		)
	end
	
	if b_dot>0 then
		bx,by,bz=plane_intersection(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(bx-rx,by-ry,bz-rz)
		)
	end
	
	if c_dot>0 then
		cx,cy,cz=plane_intersection(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(cx-rx,cy-ry,cz-rz)
		)
	end
	
	return ax,ay,az,bx,by,bz,cx,cy,cz
end

local function pull_vertexes(collider,vertexes)
	local vertices = collider.shape.vertices
	local faces    = collider.shape.faces
	
	local ct = collider.transform
	local bt = collider.body.transform
	
	local sx = collider.size[1]
	local sy = collider.size[2]
	local sz = collider.size[3]
	
	local ct11,ct12,ct13,ct14 = ct[1],ct[2],ct[3],ct[4]
	local ct21,ct22,ct23,ct24 = ct[5],ct[6],ct[7],ct[8]
	local ct31,ct32,ct33,ct34 = ct[9],ct[10],ct[11],ct[12]
	local ct41,ct42,ct43,ct44 = ct[13],ct[14],ct[15],ct[16]
	
	local bt11,bt12,bt13,bt14 = bt[1],bt[2],bt[3],bt[4]
	local bt21,bt22,bt23,bt24 = bt[5],bt[6],bt[7],bt[8]
	local bt31,bt32,bt33,bt34 = bt[9],bt[10],bt[11],bt[12]
	local bt41,bt42,bt43,bt44 = bt[13],bt[14],bt[15],bt[16]
	
	local vertex_count = 0
	
	for i=1,#faces,3 do
		local v1 = (faces[i]-1)*3
		local v2 = (faces[i+1]-1)*3
		local v3 = (faces[i+2]-1)*3
		
		vertexes[vertex_count+1],
		vertexes[vertex_count+2],
		vertexes[vertex_count+3]
		=matrix4_multiply_vector3(
			bt11,bt12,bt13,bt14,
			bt21,bt22,bt23,bt24,
			bt31,bt32,bt33,bt34,
			bt41,bt42,bt43,bt44,
			matrix4_multiply_vector3(
				ct11,ct12,ct13,ct14,
				ct21,ct22,ct23,ct24,
				ct31,ct32,ct33,ct34,
				ct41,ct42,ct43,ct44,
				vertices[v1+1]*sx,
				vertices[v1+2]*sy,
				vertices[v1+3]*sz
			)
		)
		
		vertexes[vertex_count+4],
		vertexes[vertex_count+5],
		vertexes[vertex_count+6]
		=matrix4_multiply_vector3(
			bt11,bt12,bt13,bt14,
			bt21,bt22,bt23,bt24,
			bt31,bt32,bt33,bt34,
			bt41,bt42,bt43,bt44,
			matrix4_multiply_vector3(
				ct11,ct12,ct13,ct14,
				ct21,ct22,ct23,ct24,
				ct31,ct32,ct33,ct34,
				ct41,ct42,ct43,ct44,
				vertices[v2+1]*sx,
				vertices[v2+2]*sy,
				vertices[v2+3]*sz
			)
		)
		
		vertexes[vertex_count+7],
		vertexes[vertex_count+8],
		vertexes[vertex_count+9]
		=matrix4_multiply_vector3(
			bt11,bt12,bt13,bt14,
			bt21,bt22,bt23,bt24,
			bt31,bt32,bt33,bt34,
			bt41,bt42,bt43,bt44,
			matrix4_multiply_vector3(
				ct11,ct12,ct13,ct14,
				ct21,ct22,ct23,ct24,
				ct31,ct32,ct33,ct34,
				ct41,ct42,ct43,ct44,
				vertices[v3+1]*sx,
				vertices[v3+2]*sy,
				vertices[v3+3]*sz
			)
		)
		
		vertex_count=vertex_count+9
	end
	
	for i=#vertexes,vertex_count+1,-1 do
		vertexes[i]=nil
	end
end

local function clip_convex(vertexes_a,vertexes_b,clipped)
	for i=1,#vertexes_b do
		clipped[#clipped+1]=vertexes_b[i]
	end
	
	local clip_limit=#clipped-#vertexes_b+1
	
	for a=#vertexes_a,1,-9 do
		local aax,aay,aaz = vertexes_a[a-8],vertexes_a[a-7],vertexes_a[a-6]
		local abx,aby,abz = vertexes_a[a-5],vertexes_a[a-4],vertexes_a[a-3]
		local acx,acy,acz = vertexes_a[a-2],vertexes_a[a-1],vertexes_a[a]
		
		local px = (aax+abx+acx)/3
		local py = (aay+aby+acy)/3
		local pz = (aaz+abz+acz)/3
		
		local pnx,pny,pnz = vector3_unit(vector3_cross(
			abx-aax,aby-aay,abz-aaz,
			acx-aax,acy-aay,acz-aaz
		))
		
		for b=#clipped,clip_limit,-9 do
			local
			bax,bay,baz,
			bbx,bby,bbz,
			bcx,bcy,bcz
			=clip_triangle(
				px,py,pz,
				pnx,pny,pnz,
				clipped[b-8],clipped[b-7],clipped[b-6],
				clipped[b-5],clipped[b-4],clipped[b-3],
				clipped[b-2],clipped[b-1],clipped[b]
			)
			
			if bax then
				clipped[b-8],clipped[b-7],clipped[b-6] = bax,bay,baz
				clipped[b-5],clipped[b-4],clipped[b-3] = bbx,bby,bbz
				clipped[b-2],clipped[b-1],clipped[b]   = bcx,bcy,bcz
			else --Remove vertexes outside of convex shape
				for i=0,8 do
					table_remove(clipped,b-i)
				end
			end
		end
	end
end

-------------------------------------------------------------------------------

local vertexes_a = {}
local vertexes_b = {}
local clipped    = {}

return function(collider_a,collider_b)
	for i=#clipped,1,-1 do
		clipped[i]=nil
	end
	
	pull_vertexes(collider_a,vertexes_a)
	pull_vertexes(collider_b,vertexes_b)
	
	clip_convex(vertexes_a,vertexes_b,clipped)
	clip_convex(vertexes_b,vertexes_a,clipped)
	
	local clip_count = #clipped/3
	
	local cx,cy,cz       = 0,0,0
	local sx,sy,sz,sd    = 0,0,0,0
	local fx,fy,fz,fd,fi = 0,0,0,0,0    --Furthest vertex
	local nx,ny,nz,nd = 0,0,0,math.huge --Nearest vertex
	local acx,acy,acz = collider_a:get_world_position()
	
	for i=1,#clipped,3 do
		cx = cx+clipped[i]
		cy = cy+clipped[i+1]
		cz = cz+clipped[i+2]
		
		--Find furthest vertex from collider
		local distance = vector3_magnitude(
			acx-clipped[i],
			acy-clipped[i+1],
			acz-clipped[i+2]
		)
		
		if distance>fd then
			fx = clipped[i]
			fy = clipped[i+1]
			fz = clipped[i+2]
			fd = distance
			fi = i
		end
	end
	
	for i=1,#clipped,3 do
		--Find nearest vertex from furthest vertex
		if i~=fi then
			local distance = vector3_magnitude(
				fx-clipped[i],
				fy-clipped[i+1],
				fz-clipped[i+2]
			)
			
			if distance>0.0001 and distance<nd then
				nx = clipped[i]
				ny = clipped[i+1]
				nz = clipped[i+2]
				nd = distance
			end
		end
	end
	
	cx = cx/clip_count
	cy = cy/clip_count
	cz = cz/clip_count
	
	sx = nx-fx
	sy = ny-fy
	sz = nz-fz
	
	sd = vector3_magnitude(sx,sy,sz)
	
	return 
		cx,cy,cz,
		-sx/sd,-sy/sd,-sz/sd,sd,
		clipped
end
end