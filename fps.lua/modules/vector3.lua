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
local vector3={}

-------------------------------------------------------------------------------

local sqrt=math.sqrt

-------------------------------------------------------------------------------

local function cross(x1,y1,z1,x2,y2,z2)
	return y1*z2-z1*y2,z1*x2-x1*z2,x1*y2-y1*x2
end

local function dot(x1,y1,z1,x2,y2,z2)
	return (x1*x2)+(y1*y2)+(z1*z2)
end

local function magnitude(x,y,z)
	return sqrt(x^2+y^2+z^2)
end

local function unit(x,y,z)
	local m=sqrt(x^2+y^2+z^2)
	
	if m==0 then
		return x,y,z
	end
	
	return x/m,y/m,z/m
end

-------------------------------------------------------------------------------

vector3.cross     = cross
vector3.dot       = dot
vector3.magnitude = magnitude
vector3.unit      = unit

return vector3
end