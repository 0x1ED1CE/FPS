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
local shape={}; shape.__index=shape

-------------------------------------------------------------------------------

local math_min = math.min
local math_max = math.max

-------------------------------------------------------------------------------

function shape.new(vertices,faces) --Must be convex
	local vertices_,faces_={},{} --{x,y,z...},{v1,v2,v3...}
	
	local min_x,min_y,min_z = 0,0,0
	local max_x,max_y,max_z = 0,0,0
	
	--Calculate bounds
	for i=1,#vertices,3 do
		local vx = vertices[i]
		local vy = vertices[i+1]
		local vz = vertices[i+2]
		
		min_x = math_min(vx,min_x)
		min_y = math_min(vy,min_y)
		min_z = math_min(vz,min_z)
		
		max_x = math_max(vx,max_x)
		max_y = math_max(vy,max_y)
		max_z = math_max(vz,max_z)
	end
	
	local size_x = max_x-min_x
	local size_y = max_y-min_y
	local size_z = max_z-min_z
	
	local middle_x = (min_x+max_x)/2
	local middle_y = (min_y+max_y)/2
	local middle_z = (min_z+max_z)/2
	
	--Normalize and center the vertices
	for i=1,#vertices,3 do
		vertices_[i]   = (vertices[i]-middle_x)/size_x
		vertices_[i+1] = (vertices[i+1]-middle_y)/size_y
		vertices_[i+2] = (vertices[i+2]-middle_z)/size_z
	end
	
	for i=1,#faces do
		faces_[i]=faces[i]
	end
	
	return setmetatable({
		vertices = vertices_,
		faces    = faces_
	},shape)
end

-------------------------------------------------------------------------------

return shape
end