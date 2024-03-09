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
local raycast={}

-------------------------------------------------------------------------------

local vector3 = fps.vector3

-------------------------------------------------------------------------------

local vector3_cross = vector3.cross
local vector3_dot   = vector3.dot

local math_sqrt = math.sqrt

-------------------------------------------------------------------------------

local function triangle(
	rpx,rpy,rpz,
	rdx,rdy,rdz,
	ax,ay,az,
	bx,by,bz,
	cx,cy,cz
)
	--Plane normal
	local nx,ny,nz=vector3_cross(
		bx-ax,by-ay,bz-az,
		cx-ax,cy-ay,cz-az
	)
	
	local n_dot_rd=vector3_dot(nx,ny,nz,rdx,rdy,rdz)
	if n_dot_rd==0 then
		return --Ray is parallel to triangle
	elseif n_dot_rd>0 then
		nx,ny,nz=-nx,-ny,-nz
		n_dot_rd=-n_dot_rd
	end
	
	local d=vector3_dot(nx,ny,nz,ax,ay,az)
	local t=(vector3_dot(nx,ny,nz,rpx,rpy,rpz)+d)/n_dot_rd
	
	if t<0 then
		return --Triangle is behind ray
	end
	
	--Intersection point
	local px=rpx-rdx*t
	local py=rpy-rdy*t
	local pz=rpz-rdz*t
	
	--Check if point is within edge 1
	if vector3_dot(
		nx,ny,nz,
		vector3_cross(
			bx-ax,by-ay,bz-az,
			px-ax,py-ay,pz-az
		)
	)<0 then
		return
	end
	
	--Check if point is within edge 2
	if vector3_dot(
		nx,ny,nz,
		vector3_cross(
			cx-bx,cy-by,cz-bz,
			px-bx,py-by,pz-bz
		)
	)<0 then
		return
	end
	
	--Check if point is within edge 3
	if vector3_dot(
		nx,ny,nz,
		vector3_cross(
			ax-cx,ay-cy,az-cz,
			px-cx,py-cy,pz-cz
		)
	)<0 then
		return
	end
	
	local nm=math_sqrt(nx^2+ny^2+nz^2)
	
	return px,py,pz,-nx/nm,-ny/nm,-nz/nm,t
end

-------------------------------------------------------------------------------

raycast.triangle = triangle

return raycast
end