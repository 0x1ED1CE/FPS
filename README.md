

# Fast Physics Solver
![LICENSE](https://img.shields.io/badge/LICENSE-MIT-green.svg) ![WIP](https://img.shields.io/badge/WIP-yellow.svg)

FPS is a lite 3D physics engine intended to be used for game development. It is NOT physically accurate and makes heavy approximations.

This engine uses clipping for both collision detection and collision response. It does not rely on GJK/EPA or SAT.

## Screenshots
<img src="/screenshots/8sn78vd.gif?raw=true">

<img src="/screenshots/0OBoqeb.gif?raw=true">

## TODO:
- Add friction and dampening
- Add spatial partitions. This is suppose to be fast after all!
- Rewrite it in C99. Originally written in Lua to be used with [LÖVE](https://github.com/love2d/love)
- Constraints?

## License
This software is free to use. You can modify it and redistribute it under the terms of the 
MIT license. Check [LICENSE](LICENSE) for further details.
