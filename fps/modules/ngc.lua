--[[
Naive Geometry Clipping
                                                 
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

--[[
This algorithm is not SAT! It uses a different approach by clipping the 
overlapping geometries and determines the collision based on what's left from 
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
local math_max  = math.max

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

local matrix4_multiply         = matrix4.multiply
local matrix4_multiply_vector3 = matrix4.multiply_vector3

-------------------------------------------------------------------------------

local function clip_edge(
	px,py,pz,    --Plane origin
	pnx,pny,pnz, --Plane normal
	lx,ly,lz,    --Edge origin
	lnx,lny,lnz  --Edge normal
)
	--Return if line is perpendicular to plane
	if vector3_dot(pnx,pny,pnz,lnx,lny,lnz)==0 then
		return lx,ly,lz
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
	local rx,ry,rz = 0,0,0
	
	local aex,aey,aez = vector3_unit(ax-px,ay-py,az-pz)
	local bex,bey,bez = vector3_unit(bx-px,by-py,bz-pz)
	local cex,cey,cez = vector3_unit(cx-px,cy-py,cz-pz)
	
	local a_dot = vector3_dot(aex,aey,aez,pnx,pny,pnz)
	local b_dot = vector3_dot(bex,bey,bez,pnx,pny,pnz)
	local c_dot = vector3_dot(cex,cey,cez,pnx,pny,pnz)
	
	--Get vertex behind plane to use as origin
	if a_dot<0 then
		rx,ry,rz=ax,ay,az
	elseif b_dot<0 then
		rx,ry,rz=bx,by,bz
	elseif c_dot<0 then
		rx,ry,rz=cx,cy,cz
	else --All vertexes are in front of the plane so no clipping is done
		return
	end
	
	if a_dot>0 then
		ax,ay,az=clip_edge(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(ax-rx,ay-ry,az-rz)
		)
	end
	
	if b_dot>0 then
		bx,by,bz=clip_edge(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(bx-rx,by-ry,bz-rz)
		)
	end
	
	if c_dot>0 then
		cx,cy,cz=clip_edge(
			px,py,pz,
			pnx,pny,pnz,
			rx,ry,rz,
			vector3_unit(cx-rx,cy-ry,cz-rz)
		)
	end
	
	return ax,ay,az,bx,by,bz,cx,cy,cz
end

local function clip_convex(
	vertexes_a,
	vertexes_b,
	clipped
)
	local clip_count = 0
	
	for i=1,#vertexes_b do
		clipped[clip_count+1]=vertexes_b[i]
		
		clip_count=clip_count+1
	end
	
	local clip_limit=clip_count-#vertexes_b+1
	
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
		
		for b=clip_count,clip_limit,-9 do
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
			else --Discard vertexes outside of convex shape
				for i=0,8 do
					clipped[b-i] = clipped[clip_count-i]
				end
				
				clip_count=clip_count-9
			end
		end
	end
	
	for i=#clipped,clip_count+1,-1 do
		clipped[i]=nil
	end
end

local function pull_vertexes(
	collider,
	vertexes
)
	local vertices = collider.shape.vertices
	local faces    = collider.shape.faces
	
	local gt = collider.global_transform
	
	local sx = collider.size[1]
	local sy = collider.size[2]
	local sz = collider.size[3]
	
	local t11,t12,t13,t14 = gt[1],gt[2],gt[3],gt[4]
	local t21,t22,t23,t24 = gt[5],gt[6],gt[7],gt[8]
	local t31,t32,t33,t34 = gt[9],gt[10],gt[11],gt[12]
	local t41,t42,t43,t44 = gt[13],gt[14],gt[15],gt[16]
	
	local vertex_count = 0
	
	for i=1,#faces,3 do
		local v1 = (faces[i]-1)*3
		local v2 = (faces[i+1]-1)*3
		local v3 = (faces[i+2]-1)*3
		
		vertexes[vertex_count+1],
		vertexes[vertex_count+2],
		vertexes[vertex_count+3]
		=matrix4_multiply_vector3(
			t11,t12,t13,t14,
			t21,t22,t23,t24,
			t31,t32,t33,t34,
			t41,t42,t43,t44,
			vertices[v1+1]*sx,
			vertices[v1+2]*sy,
			vertices[v1+3]*sz
		)
		
		vertexes[vertex_count+4],
		vertexes[vertex_count+5],
		vertexes[vertex_count+6]
		=matrix4_multiply_vector3(
			t11,t12,t13,t14,
			t21,t22,t23,t24,
			t31,t32,t33,t34,
			t41,t42,t43,t44,
			vertices[v2+1]*sx,
			vertices[v2+2]*sy,
			vertices[v2+3]*sz
		)
		
		vertexes[vertex_count+7],
		vertexes[vertex_count+8],
		vertexes[vertex_count+9]
		=matrix4_multiply_vector3(
			t11,t12,t13,t14,
			t21,t22,t23,t24,
			t31,t32,t33,t34,
			t41,t42,t43,t44,
			vertices[v3+1]*sx,
			vertices[v3+2]*sy,
			vertices[v3+3]*sz
		)
		
		vertex_count=vertex_count+9
	end
	
	for i=#vertexes,vertex_count+1,-1 do
		vertexes[i]=nil
	end
end

-------------------------------------------------------------------------------

local clipped = {}

local function test_collision(
	vertexes_a,
	vertexes_b
)
	clip_convex(
		vertexes_b,
		vertexes_a,
		clipped
	)
	
	if #clipped==0 then
		return 0,0,0,0,0,0,0
	end
	
	local clip_count   = #clipped/3
	local total_weight = 0
	
	local cx,cy,cz    = 0,0,0   --Contact point
	local sx,sy,sz,sd = 0,0,0,0 --Separation normal
	
	--Calculate contact point and separation normal
	for i=1,#clipped,9 do
		local aax,aay,aaz = clipped[i],clipped[i+1],clipped[i+2]
		local abx,aby,abz = clipped[i+3],clipped[i+4],clipped[i+5]
		local acx,acy,acz = clipped[i+6],clipped[i+7],clipped[i+8]
		
		local weight = vector3_magnitude(
			aax-abx,aay-aby,aaz-abz
		)*vector3_magnitude(
			aax-acx,aay-acy,aaz-acz
		)*vector3_magnitude(
			abx-acx,aby-acy,abz-acz
		)
		
		total_weight = total_weight + weight
		
		local px = (aax+abx+acx)
		local py = (aay+aby+acy)
		local pz = (aaz+abz+acz)
		
		cx = cx+px
		cy = cy+py
		cz = cz+pz
		
		local pnx,pny,pnz = vector3_unit(vector3_cross(
			abx-aax,aby-aay,abz-aaz,
			acx-aax,acy-aay,acz-aaz
		))
		
		sx = sx+pnx*weight
		sy = sy+pny*weight
		sz = sz+pnz*weight
	end
	
	total_weight=math_max(total_weight,1) --Prevent divide by zero
	
	cx = cx/clip_count
	cy = cy/clip_count
	cz = cz/clip_count
	
	sx,sy,sz=vector3_unit(
		sx/total_weight,
		sy/total_weight,
		sz/total_weight
	)
	
	local fx,fy,fz,fd = 0,0,0,-math.huge
	local nx,ny,nz,nd = 0,0,0,math.huge
	
	--Calculate separation distance
	for i=1,#clipped,3 do
		local px,py,pz = clipped[i],clipped[i+1],clipped[i+2]
		
		local pd = vector3_dot(sx,sy,sz,px,py,pz)
		
		if pd>fd then
			fx,fy,fz,fd = px,py,pz,pd
		end
		
		if pd<nd then
			nx,ny,nz,nd = px,py,pz,pd
		end
	end
	
	sd = vector3_dot(
		fx-nx,fy-ny,fz-nz,
		sx,sy,sz
	)
		
	return cx,cy,cz,sx,sy,sz,sd
end

-------------------------------------------------------------------------------

local prev_collider
local vertexes_a = {}
local vertexes_b = {}

return function(
	collider_a,
	collider_b
)
	if collider_a~=prev_collider then --Reuse vertexes from previous call
		pull_vertexes(
			collider_a,
			vertexes_a
		)
		
		prev_collider = collider_a
	end
	
	pull_vertexes(
		collider_b,
		vertexes_b
	)
	
	local cx,cy,cz,sx,sy,sz,sd=test_collision(
		vertexes_a,
		vertexes_b
	)
	
	if sd==0 then --Try the other way around
		cx,cy,cz,sx,sy,sz,sd=test_collision(
			vertexes_b,
			vertexes_a
		)
		
		sx,sy,sz = -sx,-sy,-sz
	end
	
	return cx,cy,cz,sx,sy,sz,sd
end
end