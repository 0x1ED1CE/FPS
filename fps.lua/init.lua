--[[                                                    
Fast Physics Solver

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

local fps={
	version="0.0.6"
}

local cd=...

-------------------------------------------------------------------------------

fps.vector3  = require( cd..".modules.vector3"  )(fps)
fps.matrix3  = require( cd..".modules.matrix3"  )(fps)
fps.matrix4  = require( cd..".modules.matrix4"  )(fps)
fps.raycast  = require( cd..".modules.raycast"  )(fps)
fps.ngc      = require( cd..".modules.ngc"      )(fps)
fps.shape    = require( cd..".modules.shape"    )(fps)
fps.collider = require( cd..".modules.collider" )(fps)
fps.body     = require( cd..".modules.body"     )(fps)
fps.world    = require( cd..".modules.world"    )(fps)

fps.solvers={
	rigid = require( cd..".solvers.rigid"  )(fps)
}

-------------------------------------------------------------------------------

return fps