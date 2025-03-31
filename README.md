# Free Physics Solver
![LICENSE](https://img.shields.io/badge/LICENSE-MIT-green.svg) ![WIP](https://img.shields.io/badge/WIP-yellow.svg)

FPS is a lite 3D physics engine intended to be used for game development.

<img src="/screenshots/mBAgMbm.gif?raw=true">

**It is recommended to use this library with [VML](https://github.com/0x1ED1CE/VML)**

## Example Code
```C
#define FPS_IMPLEMENTATION
#include "fps.h"

void main() {
	// Create dynamic bodies
	fps_body *body_a = fps_body_new();
	fps_body *body_b = fps_body_new();

	// Create (box) colliders
	fps_part *part_a = fps_part_new();
	fps_part *part_b = fps_part_new();

	// Add colliders to the bodies
	fps_body_part_add(body_a,part_a);
	fps_body_part_add(body_b,part_b);

	// Make body A a moving box
	fps_body_dynamic_set(body_a,FPS_TRUE);              // Body A is dynamic and can move
	fps_body_position_set(body_a,(ice_real[3]){0,5,0}); // You can also use fps_body_transform_set()
	fps_part_size_set(part_a,(fps_real[3]){1,1,1});     // Set the size to 1,1,1
	fps_part_solid_set(part_a,FPS_TRUE);                // Make it solid so it reacts to collisions
	fps_part_friction_set(part_a,0.7);                  // Friction 0 (slick) to 1 (doesn't slide)
	fps_part_restitution_set(part_a,0.5);               // Restitution 0 (no bounce) to 1 (bounces)
	fps_part_density_set(part_a,1.0);                   // Density affects the mass and inertia

	// Make body B a static floor
	fps_body_dynamic_set(body_b,FPS_FALSE);
	fps_body_position_set(body_b,(ice_real[3]){0,0,0});
	fps_part_size_set(part_b,(fps_real[3]){10,1,10});
	fps_part_solid_set(part_b,FPS_TRUE);
	fps_part_friction_set(part_b,0.7);
	fps_part_restitution_set(part_b,0.5);
	fps_part_density_set(part_b,1.0);

	while (1) {
		// Apply gravity to body A
		fps_real mass = fps_body_mass_get(body_a);
		fps_body_force_apply(body_a,(fps_real[3]){0,-10.0*mass,0});

		// Test collision between bodies A and B
		fps_collision_test(body_a,body_b);

		// Step the bodies 1/60th of a second
		fps_body_step(body_a,1.0/60.0);
		fps_body_step(body_b,1.0/60.0);
	}

	// Cleanup
	fps_part_free(part_a);
	fps_part_free(part_b);

	fps_body_free(body_a);
	fps_body_free(body_b);
}
```
Refer to [fps.h](fps.h) to see rest of the functions.

## License
This software is free to use. You can modify it and redistribute it under the terms of the 
MIT license. Check [LICENSE](LICENSE) for further details.
