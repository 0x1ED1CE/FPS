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
local matrix4={}

-------------------------------------------------------------------------------

local math_cos = math.cos
local math_sin = math.sin

-------------------------------------------------------------------------------

local function inverse(
	a11,a12,a13,a14,
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44
)
	local c11 =  a22*a33*a44-a22*a34*a43-a32*a23*a44+a32*a24*a43+a42*a23*a34-a42*a24*a33
	local c12 = -a12*a33*a44+a12*a34*a43+a32*a13*a44-a32*a14*a43-a42*a13*a34+a42*a14*a33
	local c13 =  a12*a23*a44-a12*a24*a43-a22*a13*a44+a22*a14*a43+a42*a13*a24-a42*a14*a23
	local c14 = -a12*a23*a34+a12*a24*a33+a22*a13*a34-a22*a14*a33-a32*a13*a24+a32*a14*a23
	local c21 = -a21*a33*a44+a21*a34*a43+a31*a23*a44-a31*a24*a43-a41*a23*a34+a41*a24*a33
	local c22 =  a11*a33*a44-a11*a34*a43-a31*a13*a44+a31*a14*a43+a41*a13*a34-a41*a14*a33
	local c23 = -a11*a23*a44+a11*a24*a43+a21*a13*a44-a21*a14*a43-a41*a13*a24+a41*a14*a23
	local c24 =  a11*a23*a34-a11*a24*a33-a21*a13*a34+a21*a14*a33+a31*a13*a24-a31*a14*a23
	local c31 =  a21*a32*a44-a21*a34*a42-a31*a22*a44+a31*a24*a42+a41*a22*a34-a41*a24*a32
	local c32 = -a11*a32*a44+a11*a34*a42+a31*a12*a44-a31*a14*a42-a41*a12*a34+a41*a14*a32
	local c33 =  a11*a22*a44-a11*a24*a42-a21*a12*a44+a21*a14*a42+a41*a12*a24-a41*a14*a22
	local c34 = -a11*a22*a34+a11*a24*a32+a21*a12*a34-a21*a14*a32-a31*a12*a24+a31*a14*a22
	local c41 = -a21*a32*a43+a21*a33*a42+a31*a22*a43-a31*a23*a42-a41*a22*a33+a41*a23*a32
	local c42 =  a11*a32*a43-a11*a33*a42-a31*a12*a43+a31*a13*a42+a41*a12*a33-a41*a13*a32
	local c43 = -a11*a22*a43+a11*a23*a42+a21*a12*a43-a21*a13*a42-a41*a12*a23+a41*a13*a22
	local c44 =  a11*a22*a33-a11*a23*a32-a21*a12*a33+a21*a13*a32+a31*a12*a23-a31*a13*a22
	
	local det = a11*c11+a12*c21+a13*c31+a14*c41
	
	if det==0 then
		return
			a11,a12,a13,a14,
			a21,a22,a23,a24,
			a31,a32,a33,a34,
			a41,a42,a43,a44
	end
	
	return
		c11/det,c12/det,c13/det,c14/det,
		c21/det,c22/det,c23/det,c24/det,
		c31/det,c32/det,c33/det,c34/det,
		c41/det,c42/det,c43/det,c44/det
end

local function multiply(
	a11,a12,a13,a14, --A
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44,
	b11,b12,b13,b14, --B
	b21,b22,b23,b24,
	b31,b32,b33,b34,
	b41,b42,b43,b44
)
	return
		a11*b11+a12*b21+a13*b31+a14*b41,
		a11*b12+a12*b22+a13*b32+a14*b42,
		a11*b13+a12*b23+a13*b33+a14*b43,
		a11*b14+a12*b24+a13*b34+a14*b44,
		a21*b11+a22*b21+a23*b31+a24*b41,
		a21*b12+a22*b22+a23*b32+a24*b42,
		a21*b13+a22*b23+a23*b33+a24*b43,
		a21*b14+a22*b24+a23*b34+a24*b44,
		a31*b11+a32*b21+a33*b31+a34*b41,
		a31*b12+a32*b22+a33*b32+a34*b42,
		a31*b13+a32*b23+a33*b33+a34*b43,
		a31*b14+a32*b24+a33*b34+a34*b44,
		a41*b11+a42*b21+a43*b31+a44*b41,
		a41*b12+a42*b22+a43*b32+a44*b42,
		a41*b13+a42*b23+a43*b33+a44*b43,
		a41*b14+a42*b24+a43*b34+a44*b44
end

local function multiply_vector3(
	a11,a12,a13,a14,
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44,
	bx,by,bz
)
	return
		a14+bx*a11+by*a12+bz*a13,
		a24+bx*a21+by*a22+bz*a23,
		a34+bx*a31+by*a32+bz*a33
end

local function translate(
	a11,a12,a13,a14,
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44,
	x,y,z
)
	return
		a11,a12,a13,a14+x,
		a21,a22,a23,a24+y,
		a31,a32,a33,a34+z,
		a41,a42,a43,a44
end

local function set_euler(
	a11,a12,a13,a14,
	a21,a22,a23,a24,
	a31,a32,a33,a34,
	a41,a42,a43,a44,
	x,y,z
)
	local cx,sx=math_cos(x),math_sin(x)
	local cy,sy=math_cos(y),math_sin(y)
	local cz,sz=math_cos(z),math_sin(z)
	
	a11=cy*cz
	a12=-cy*sz
	a13=sy
	
	a21=cz*sx*sy+cx*sz
	a22=cx*cz-sx*sy*sz
	a23=-cy*sx
	
	a31=sx*sz-cx*cz*sy
	a32=cz*sx+cx*sy*sz
	a33=cx*cy
	
	return
		a11,a12,a13,a14,
		a21,a22,a23,a24,
		a31,a32,a33,a34,
		a41,a42,a43,a44
end

-------------------------------------------------------------------------------

matrix4.inverse          = inverse
matrix4.multiply         = multiply
matrix4.multiply_vector3 = multiply_vector3
matrix4.translate        = translate
matrix4.set_euler        = set_euler

return matrix4
end