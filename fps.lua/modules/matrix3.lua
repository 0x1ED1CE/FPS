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
local matrix3={}

-------------------------------------------------------------------------------

local math_cos = math.cos
local math_sin = math.sin

-------------------------------------------------------------------------------

local function multiply(
	a11,a12,a13,
	a21,a22,a23,
	a31,a32,a33,
	b11,b12,b13,
	b21,b22,b23,
	b31,b32,b33
)
	return
		a11*b11+a12*b21+a13*b31,
		a11*b12+a12*b22+a13*b32,
		a11*b13+a12*b23+a13*b33,
		a21*b11+a22*b21+a23*b31,
		a21*b12+a22*b22+a23*b32,
		a21*b13+a22*b23+a23*b33,
		a31*b11+a32*b21+a33*b31,
		a31*b12+a32*b22+a33*b32,
		a31*b13+a32*b23+a33*b33
end

local function inverse(
	a11,a12,a13,
	a21,a22,a23,
	a31,a32,a33
)
	local det=(
		a11*(a22*a33-a32*a23)-
		a12*(a21*a33-a23*a31)+
		a13*(a21*a32-a22*a31)
	)
	
	return
		(a22*a33-a32*a23)/det,
		(a13*a32-a12*a33)/det,
		(a12*a23-a13*a22)/det,
		(a23*a31-a21*a33)/det,
		(a11*a33-a13*a31)/det,
		(a21*a13-a11*a23)/det,
		(a21*a32-a31*a22)/det,
		(a31*a12-a11*a32)/det,
		(a11*a22-a21*a12)/det
end

local function multiply_vector3(
	a11,a12,a13,
	a21,a22,a23,
	a31,a32,a33,
	bx,by,bz
)
	return
		bx*a11+by*a12+bz*a13,
		bx*a21+by*a22+bz*a23,
		bx*a31+by*a32+bz*a33
end

-------------------------------------------------------------------------------

matrix3.multiply         = multiply
matrix3.inverse          = inverse
matrix3.multiply_vector3 = multiply_vector3

return matrix3
end