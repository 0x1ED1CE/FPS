/*
Free Physics Solver

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
*/

#ifndef FPS_H
#define FPS_H

#define FPS_VERSION_MAJOR 1
#define FPS_VERSION_MINOR 0
#define FPS_VERSION_PATCH 0

typedef unsigned int fps_uint;
typedef signed   int fps_sint;
typedef float        fps_real;

// VEC3 MODULE

static inline void fps_vec3_neg(
	fps_real a[3],
	fps_real b[3]
);

static inline void fps_vec3_mov(
	fps_real a[3],
	fps_real b[3]
);

static inline void fps_vec3_add(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

static inline void fps_vec3_sub(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

static inline void fps_vec3_mul(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

static inline void fps_vec3_div(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

static inline void fps_vec3_dot(
	fps_real  a[3],
	fps_real  b[3],
	fps_real *c
);

static inline void fps_vec3_cross(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

static inline void fps_vec3_unit(
	fps_real a[3],
	fps_real b[3]
);

static inline void fps_vec3_mag(
	fps_real  a[3],
	fps_real *b
);

static inline void fps_vec3_num_mul(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

static inline void fps_vec3_num_div(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

// MAT3 MODULE

static inline void fps_mat3_mov(
	fps_real a[9],
	fps_real b[9]
);

void fps_mat3_vec3_mul(
	fps_real a[9],
	fps_real b[3],
	fps_real c[3]
);

void fps_mat3_mul(
	fps_real a[9],
	fps_real b[9],
	fps_real c[9]
);

void fps_mat3_inv(
	fps_real a[9],
	fps_real b[9]
);

void fps_mat3_scale(
	fps_real a[9],
	fps_real b[3],
	fps_real c[9]
);

// MAT4 MODULE

static inline void fps_mat4_mov(
	fps_real a[16],
	fps_real b[16]
);

void fps_mat4_vec3_mul(
	fps_real a[16],
	fps_real b[3],
	fps_real c[3]
);

void fps_mat4_mul(
	fps_real a[16],
	fps_real b[16],
	fps_real c[16]
);

void fps_mat4_inv(
	fps_real a[16],
	fps_real b[16]
);

void fps_mat4_euler(
	fps_real a[3],
	fps_real b[16]
);

void fps_mat4_mat3(
	fps_real a[16],
	fps_real b[9]
);

// Define these structs first

typedef struct fps_collider fps_collider;

typedef struct fps_body fps_body;

typedef struct fps_collider {
	fps_body     *body;
	fps_collider *next;
	fps_uint      solid;
	fps_real      size[3];
	fps_real      offset[16];
	fps_real      transform[16];
	fps_real      boundary[2][3];
	fps_real      density;
	fps_real      friction;
	fps_real      restitution;
	fps_real    (*vertexes)[3];
	fps_uint      vertex_count;
} fps_collider;

typedef struct fps_body {
	fps_uint      sleeping;
	fps_uint      anchored;
	fps_real      transform[16];
	fps_real      boundary[2][3];
	fps_real      mass;
	fps_real      mass_offset[3];
	fps_real      force[3];
	fps_real      torque[3];
	fps_real      velocity[3];
	fps_real      angular_velocity[3];
	fps_real      inverse_inertia[9];
	fps_real      post_translation[3];
	fps_collider *collider;
} fps_body;

// COLLIDER MODULE

void fps_collider_new(
	fps_collider **collider
);

void fps_collider_free(
	fps_collider *collider
);

void fps_collider_next(
	fps_collider  *collider,
	fps_collider **next
);

void fps_collider_solid_get(
	fps_collider *collider,
	fps_uint     *state
);

void fps_collider_solid_set(
	fps_collider *collider,
	fps_uint      state
);

void fps_collider_size_get(
	fps_collider *collider,
	fps_real      size[3]
);

void fps_collider_size_set(
	fps_collider *collider,
	fps_real      size[3]
);

void fps_collider_offset_get(
	fps_collider *collider,
	fps_real      offset[16]
);

void fps_collider_offset_set(
	fps_collider *collider,
	fps_real      offset[16]
);

void fps_collider_offset_position_get(
	fps_collider *collider,
	fps_real      position[3]
);

void fps_collider_transform_get(
	fps_collider *collider,
	fps_real      transform[16]
);

void fps_collider_global_position_get(
	fps_collider *collider,
	fps_real      position[3]
);

void fps_collider_boundary_get(
	fps_collider *collider,
	fps_real      point_a[3],
	fps_real      point_b[3]
);

void fps_collider_density_get(
	fps_collider *collider,
	fps_real     *density
);

void fps_collider_density_set(
	fps_collider *collider,
	fps_real      density
);

void fps_collider_friction_get(
	fps_collider *collider,
	fps_real     *friction
);

void fps_collider_friction_set(
	fps_collider *collider,
	fps_real      friction
);

void fps_collider_restitution_get(
	fps_collider *collider,
	fps_real     *restitution
);

void fps_collider_restitution_set(
	fps_collider *collider,
	fps_real      restitution
);
/*
void fps_collider_vertexes_get(
	fps_collider *collider,
	fps_real    *(*vertexes)[3],
	fps_real     *vertex_count
);

void fps_collider_vertexes_set(
	fps_collider *collider,
	fps_real    (*vertexes)[3],
	fps_real      vertex_count
);
*/
void fps_collider_mass_get(
	fps_collider *collider,
	fps_real     *mass
);

void fps_collider_transform_update(
	fps_collider *collider
);

void fps_collider_boundary_update(
	fps_collider *collider
);

// BODY MODULE

void fps_body_new(
	fps_body **body
);

void fps_body_free(
	fps_body *body
);

void fps_body_sleeping_get(
	fps_body *body,
	fps_uint *state
);

void fps_body_sleeping_set(
	fps_body *body,
	fps_uint  state
);

void fps_body_anchored_get(
	fps_body *body,
	fps_uint *state
);

void fps_body_anchored_set(
	fps_body *body,
	fps_uint  state
);

void fps_body_transform_get(
	fps_body *body,
	fps_real  transform[16]
);

void fps_body_transform_set(
	fps_body *body,
	fps_real  transform[16]
);

void fps_body_position_get(
	fps_body *body,
	fps_real  position[3]
);

void fps_body_position_set(
	fps_body *body,
	fps_real  position[3]
);

void fps_body_translation_apply(
	fps_body *body,
	fps_real translation[3]
);

void fps_body_boundary_get(
	fps_body *body,
	fps_real  point_a[3],
	fps_real  point_b[3]
);

void fps_body_mass_get(
	fps_body *body,
	fps_real *mass
);

void fps_body_mass_offset_get(
	fps_body *body,
	fps_real  mass_offset[3]
);

void fps_body_force_get(
	fps_body *body,
	fps_real  force[3]
);

void fps_body_force_set(
	fps_body *body,
	fps_real  force[3]
);

void fps_body_torque_get(
	fps_body *body,
	fps_real  torque[3]
);

void fps_body_torque_set(
	fps_body *body,
	fps_real  torque[3]
);

void fps_body_velocity_get(
	fps_body *body,
	fps_real  velocity[3]
);

void fps_body_velocity_set(
	fps_body *body,
	fps_real  velocity[3]
);

void fps_body_angular_velocity_get(
	fps_body *body,
	fps_real angular_velocity[3]
);

void fps_body_angular_velocity_set(
	fps_body *body,
	fps_real  angular_velocity[3]
);

void fps_body_inverse_inertia_get(
	fps_body *body,
	fps_real  inverse_inertia[9]
);

void fps_body_force_apply(
	fps_body *body,
	fps_real  force[3]
);

void fps_body_torque_apply(
	fps_body *body,
	fps_real  torque[3]
);

void fps_body_linear_impulse_apply(
	fps_body *body,
	fps_real force[3]
);

void fps_body_angular_impulse_apply(
	fps_body *body,
	fps_real torque[3]
);

void fps_body_mass_update(
	fps_body *body
);

void fps_body_boundary_update(
	fps_body *body
);

void fps_body_collider_add(
	fps_body     *body,
	fps_collider *collider
);

void fps_body_collider_remove(
	fps_body     *body,
	fps_collider *collider
);

void fps_body_collider_get(
	fps_body      *body,
	fps_collider **collider
);

void fps_body_step(
	fps_body *body,
	fps_real  dt
);

// AABB MODULE

void fps_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  c[3],
	fps_real  d[3],
	fps_uint *r
);

// NGC MODULE

void fps_ngc_clip_edge(
	fps_real po[3], // Plane origin
	fps_real pn[3], // Plane normal
	fps_real eo[3], // Edge origin
	fps_real en[3], // Edge normal
	fps_real cp[3]  // Clipped point
);

void fps_ngc_clip_triangle(
	fps_real  po[3], // Plane origin
	fps_real  pn[3], // Plane normal
	fps_real  ta[3], // Triangle point A
	fps_real  tb[3], // Triangle point B
	fps_real  tc[3], // Triangle point C
	fps_uint *cs     // Clip status
);

void fps_ngc_clip_convex(
	fps_real  va[][3], // Vertexes A
	fps_real  vb[][3], // Vertexes B
	fps_real  vc[][3], // Clipped vertexes
	fps_uint  va_len,  // Vertexes A length
	fps_uint  vb_len,  // Vertexes B length
	fps_uint *vc_len   // Clipped length
);

void fps_ngc_test_convex(
	fps_real  va[][3], // Vertexes A
	fps_real  vb[][3], // Vertexes B
	fps_uint  va_len,  // Vertexes A length
	fps_uint  vb_len,  // Vertexes B length
	fps_real  cp[3],   // Collision point
	fps_real  sn[3],   // Separation normal
	fps_real *sd       // Separation distance
);

// SOLVER MODULE

void fps_collision_resolve(
	fps_body     *body_a,
	fps_body     *body_b,
	fps_collider *collider_a,
	fps_collider *collider_b,
	fps_real      cp[3],
	fps_real      sn[3],
	fps_real      sd
);

void fps_collision_test(
	fps_body *a,
	fps_body *b
);

#endif

/************************[IMPLEMENTATION BEGINS HERE]*************************/

#ifdef FPS_IMPLEMENTATION

#include <math.h>
#include <float.h>
#include <malloc.h>
#include <stdio.h>

#ifndef FPS_MIN
#define FPS_MIN(A,B) ((A)<(B)?(A):(B))
#endif

#ifndef FPS_MAX
#define FPS_MAX(A,B) ((A)>(B)?(A):(B))
#endif

static fps_real fps_cube[36][3] = { // DO NOT CHANGE
	{-0.5,  0.5,  0.5},
	{-0.5, -0.5, -0.5},
	{-0.5, -0.5,  0.5},
	{-0.5,  0.5, -0.5},
	{ 0.5, -0.5, -0.5},
	{-0.5, -0.5, -0.5},
	{ 0.5,  0.5, -0.5},
	{ 0.5, -0.5,  0.5},
	{ 0.5, -0.5, -0.5},
	{ 0.5,  0.5,  0.5},
	{-0.5, -0.5,  0.5},
	{ 0.5, -0.5,  0.5},
	{ 0.5, -0.5, -0.5},
	{-0.5, -0.5,  0.5},
	{-0.5, -0.5, -0.5},
	{-0.5,  0.5, -0.5},
	{ 0.5,  0.5,  0.5},
	{ 0.5,  0.5, -0.5},
	{-0.5,  0.5,  0.5},
	{-0.5,  0.5, -0.5},
	{-0.5, -0.5, -0.5},
	{-0.5,  0.5, -0.5},
	{ 0.5,  0.5, -0.5},
	{ 0.5, -0.5, -0.5},
	{ 0.5,  0.5, -0.5},
	{ 0.5,  0.5,  0.5},
	{ 0.5, -0.5,  0.5},
	{ 0.5,  0.5,  0.5},
	{-0.5,  0.5,  0.5},
	{-0.5, -0.5,  0.5},
	{ 0.5, -0.5, -0.5},
	{ 0.5, -0.5,  0.5},
	{-0.5, -0.5,  0.5},
	{-0.5,  0.5, -0.5},
	{-0.5,  0.5,  0.5},
	{ 0.5,  0.5,  0.5}
};

// VEC3 MODULE

static inline void fps_vec3_neg(
	fps_real a[3],
	fps_real b[3]
) {
	b[0] = -a[0];
	b[1] = -a[1];
	b[2] = -a[2];
}

static inline void fps_vec3_mov(
	fps_real a[3],
	fps_real b[3]
) {
	b[0] = a[0];
	b[1] = a[1];
	b[2] = a[2];
}

static inline void fps_vec3_add(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]+b[0];
	c[1] = a[1]+b[1];
	c[2] = a[2]+b[2];
}

static inline void fps_vec3_sub(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]-b[0];
	c[1] = a[1]-b[1];
	c[2] = a[2]-b[2];
}

static inline void fps_vec3_mul(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]*b[0];
	c[1] = a[1]*b[1];
	c[2] = a[2]*b[2];
}

static inline void fps_vec3_div(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]/b[0];
	c[1] = a[1]/b[1];
	c[2] = a[2]/b[2];
}

static inline void fps_vec3_dot(
	fps_real  a[3],
	fps_real  b[3],
	fps_real *c
) {
	*c = (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2]);
}

static inline void fps_vec3_cross(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	fps_real r0 = a[1]*b[2]-a[2]*b[1];
	fps_real r1 = a[2]*b[0]-a[0]*b[2];
	fps_real r2 = a[0]*b[1]-a[1]*b[0];

	c[0] = r0;
	c[1] = r1;
	c[2] = r2;
}

static inline void fps_vec3_unit(
	fps_real a[3],
	fps_real b[3]
) {
	fps_real m=(fps_real)sqrtf(
		powf((float)a[0],2.0f)+
		powf((float)a[1],2.0f)+
		powf((float)a[2],2.0f)
	);

	if (m==0) {
		b[0] = 0;
		b[1] = 0;
		b[2] = 0;

		return;
	}

	b[0] = a[0]/m;
	b[1] = a[1]/m;
	b[2] = a[2]/m;
}

static inline void fps_vec3_mag(
	fps_real  a[3],
	fps_real *b
) {
	*b=(fps_real)sqrtf(
		powf((float)a[0],2.0f)+
		powf((float)a[1],2.0f)+
		powf((float)a[2],2.0f)
	);
}

static inline void fps_vec3_num_mul(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
) {
	c[0] = a[0]*b;
	c[1] = a[1]*b;
	c[2] = a[2]*b;
}

static inline void fps_vec3_num_div(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
) {
	c[0] = a[0]/b;
	c[1] = a[1]/b;
	c[2] = a[2]/b;
}

// MAT3 MODULE

static fps_real _fps_mat3_id[9] = { // DO NOT CHANGE
	1,0,0,
	0,1,0,
	0,0,1
};

static inline void fps_mat3_mov(
	fps_real a[9],
	fps_real b[9]
) {
	for (fps_uint i=0; i<9; i++) {
		b[i] = a[i];
	}
}

void fps_mat3_vec3_mul(
	fps_real a[9],
	fps_real b[3],
	fps_real c[3]
) {
	fps_real r0 = b[0]*a[0]+b[1]*a[1]+b[2]*a[2];
	fps_real r1 = b[0]*a[3]+b[1]*a[4]+b[2]*a[5];
	fps_real r2 = b[0]*a[6]+b[1]*a[7]+b[2]*a[8];

	c[0] = r0;
	c[1] = r1;
	c[2] = r2;
}

void fps_mat3_mul(
	fps_real a[9],
	fps_real b[9],
	fps_real c[9]
) {
	fps_real a00 = a[0];
	fps_real a01 = a[1];
	fps_real a02 = a[2];
	fps_real a10 = a[3];
	fps_real a11 = a[4];
	fps_real a12 = a[5];
	fps_real a20 = a[6];
	fps_real a21 = a[7];
	fps_real a22 = a[8];

	fps_real b00 = b[0];
	fps_real b01 = b[1];
	fps_real b02 = b[2];
	fps_real b10 = b[3];
	fps_real b11 = b[4];
	fps_real b12 = b[5];
	fps_real b20 = b[6];
	fps_real b21 = b[7];
	fps_real b22 = b[8];

	c[0] = a00*b00+a01*b10+a02*b20;
	c[1] = a00*b01+a01*b11+a02*b21;
	c[2] = a00*b02+a01*b12+a02*b22;
	c[3] = a10*b00+a11*b10+a12*b20;
	c[4] = a10*b01+a11*b11+a12*b21;
	c[5] = a10*b02+a11*b12+a12*b22;
	c[6] = a20*b00+a21*b10+a22*b20;
	c[7] = a20*b01+a21*b11+a22*b21;
	c[8] = a20*b02+a21*b12+a22*b22;
}

void fps_mat3_inv(
	fps_real a[9],
	fps_real b[9]
) {
	fps_real a00 = a[0];
	fps_real a01 = a[1];
	fps_real a02 = a[2];
	fps_real a10 = a[3];
	fps_real a11 = a[4];
	fps_real a12 = a[5];
	fps_real a20 = a[6];
	fps_real a21 = a[7];
	fps_real a22 = a[8];

	fps_real det=(
		a00*(a11*a22-a21*a12)-
		a01*(a10*a22-a12*a20)+
		a02*(a10*a21-a11*a20)
	);

	b[0] = (a11*a22-a21*a12)/det;
	b[1] = (a02*a21-a01*a22)/det;
	b[2] = (a01*a12-a02*a11)/det;
	b[3] = (a12*a20-a10*a22)/det;
	b[4] = (a00*a22-a02*a20)/det;
	b[5] = (a10*a02-a00*a12)/det;
	b[6] = (a10*a21-a20*a11)/det;
	b[7] = (a20*a01-a00*a21)/det;
	b[8] = (a00*a11-a10*a01)/det;
}

void fps_mat3_scale(
	fps_real a[9],
	fps_real b[3],
	fps_real c[9]
) {
	fps_mat3_mul(a,(fps_real[9]){
		b[0], 0,    0,
		0,    b[1], 0,
		0,    0,    b[2]
	},c);
}

// MAT4 MODULE

static fps_real _fps_mat4_id[16] = { // DO NOT CHANGE
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1
};

static inline void fps_mat4_mov(
	fps_real a[16],
	fps_real b[16]
) {
	for (fps_uint i=0; i<16; i++) {
		b[i] = a[i];
	}
}

void fps_mat4_vec3_mul(
	fps_real a[16],
	fps_real b[3],
	fps_real c[3]
) {
	fps_real r0 = a[3]+b[0]*a[0]+b[1]*a[1]+b[2]*a[2];
	fps_real r1 = a[7]+b[0]*a[4]+b[1]*a[5]+b[2]*a[6];
	fps_real r2 = a[11]+b[0]*a[8]+b[1]*a[9]+b[2]*a[10];

	c[0] = r0;
	c[1] = r1;
	c[2] = r2;
}

void fps_mat4_mul(
	fps_real a[16],
	fps_real b[16],
	fps_real c[16]
) {
	fps_real a00 = a[0];
	fps_real a01 = a[1];
	fps_real a02 = a[2];
	fps_real a03 = a[3];
	fps_real a10 = a[4];
	fps_real a11 = a[5];
	fps_real a12 = a[6];
	fps_real a13 = a[7];
	fps_real a20 = a[8];
	fps_real a21 = a[9];
	fps_real a22 = a[10];
	fps_real a23 = a[11];
	fps_real a30 = a[12];
	fps_real a31 = a[13];
	fps_real a32 = a[14];
	fps_real a33 = a[15];
	
	fps_real b00 = b[0];
	fps_real b01 = b[1];
	fps_real b02 = b[2];
	fps_real b03 = b[3];
	fps_real b10 = b[4];
	fps_real b11 = b[5];
	fps_real b12 = b[6];
	fps_real b13 = b[7];
	fps_real b20 = b[8];
	fps_real b21 = b[9];
	fps_real b22 = b[10];
	fps_real b23 = b[11];
	fps_real b30 = b[12];
	fps_real b31 = b[13];
	fps_real b32 = b[14];
	fps_real b33 = b[15];

	c[0]  = a00*b00+a01*b10+a02*b20+a03*b30;
	c[1]  = a00*b01+a01*b11+a02*b21+a03*b31;
	c[2]  = a00*b02+a01*b12+a02*b22+a03*b32;
	c[3]  = a00*b03+a01*b13+a02*b23+a03*b33;
	c[4]  = a10*b00+a11*b10+a12*b20+a13*b30;
	c[5]  = a10*b01+a11*b11+a12*b21+a13*b31;
	c[6]  = a10*b02+a11*b12+a12*b22+a13*b32;
	c[7]  = a10*b03+a11*b13+a12*b23+a13*b33;
	c[8]  = a20*b00+a21*b10+a22*b20+a23*b30;
	c[9]  = a20*b01+a21*b11+a22*b21+a23*b31;
	c[10] = a20*b02+a21*b12+a22*b22+a23*b32;
	c[11] = a20*b03+a21*b13+a22*b23+a23*b33;
	c[12] = a30*b00+a31*b10+a32*b20+a33*b30;
	c[13] = a30*b01+a31*b11+a32*b21+a33*b31;
	c[14] = a30*b02+a31*b12+a32*b22+a33*b32;
	c[15] = a30*b03+a31*b13+a32*b23+a33*b33;
}

void fps_mat4_inv(
	fps_real a[16],
	fps_real b[16]
) {
	fps_real a00 = a[0];
	fps_real a01 = a[1];
	fps_real a02 = a[2];
	fps_real a03 = a[3];
	fps_real a10 = a[4];
	fps_real a11 = a[5];
	fps_real a12 = a[6];
	fps_real a13 = a[7];
	fps_real a20 = a[8];
	fps_real a21 = a[9];
	fps_real a22 = a[10];
	fps_real a23 = a[11];
	fps_real a30 = a[12];
	fps_real a31 = a[13];
	fps_real a32 = a[14];
	fps_real a33 = a[15];

	fps_real b00 =  a11*a22*a33-a11*a23*a32-a21*a12*a33+a21*a13*a32+a31*a12*a23-a31*a13*a22;
	fps_real b01 = -a01*a22*a33+a01*a23*a32+a21*a02*a33-a21*a03*a32-a31*a02*a23+a31*a03*a22;
	fps_real b02 =  a01*a12*a33-a01*a13*a32-a11*a02*a33+a11*a03*a32+a31*a02*a13-a31*a03*a12;
	fps_real b03 = -a01*a12*a23+a01*a13*a22+a11*a02*a23-a11*a03*a22-a21*a02*a13+a21*a03*a12;
	fps_real b10 = -a10*a22*a33+a10*a23*a32+a20*a12*a33-a20*a13*a32-a30*a12*a23+a30*a13*a22;
	fps_real b11 =  a00*a22*a33-a00*a23*a32-a20*a02*a33+a20*a03*a32+a30*a02*a23-a30*a03*a22;
	fps_real b12 = -a00*a12*a33+a00*a13*a32+a10*a02*a33-a10*a03*a32-a30*a02*a13+a30*a03*a12;
	fps_real b13 =  a00*a12*a23-a00*a13*a22-a10*a02*a23+a10*a03*a22+a20*a02*a13-a20*a03*a12;
	fps_real b20 =  a10*a21*a33-a10*a23*a31-a20*a11*a33+a20*a13*a31+a30*a11*a23-a30*a13*a21;
	fps_real b21 = -a00*a21*a33+a00*a23*a31+a20*a01*a33-a20*a03*a31-a30*a01*a23+a30*a03*a21;
	fps_real b22 =  a00*a11*a33-a00*a13*a31-a10*a01*a33+a10*a03*a31+a30*a01*a13-a30*a03*a11;
	fps_real b23 = -a00*a11*a23+a00*a13*a21+a10*a01*a23-a10*a03*a21-a20*a01*a13+a20*a03*a11;
	fps_real b30 = -a10*a21*a32+a10*a22*a31+a20*a11*a32-a20*a12*a31-a30*a11*a22+a30*a12*a21;
	fps_real b31 =  a00*a21*a32-a00*a22*a31-a20*a01*a32+a20*a02*a31+a30*a01*a22-a30*a02*a21;
	fps_real b32 = -a00*a11*a32+a00*a12*a31+a10*a01*a32-a10*a02*a31-a30*a01*a12+a30*a02*a11;
	fps_real b33 =  a00*a11*a22-a00*a12*a21-a10*a01*a22+a10*a02*a21+a20*a01*a12-a20*a02*a11;

	fps_real det = a00*b00+a01*b10+a02*b20+a03*b30;

	b[0]  = b00/det;
	b[1]  = b01/det;
	b[2]  = b02/det;
	b[3]  = b03/det;
	b[4]  = b10/det;
	b[5]  = b11/det;
	b[6]  = b12/det;
	b[7]  = b13/det;
	b[8]  = b20/det;
	b[9]  = b21/det;
	b[10] = b22/det;
	b[11] = b23/det;
	b[12] = b30/det;
	b[13] = b31/det;
	b[14] = b32/det;
	b[15] = b33/det;
}

void fps_mat4_euler(
	fps_real a[3],
	fps_real b[16]
) {
	fps_real cx = cosf(a[0]);
	fps_real cy = cosf(a[1]);
	fps_real cz = cosf(a[2]);
	fps_real sx = sinf(a[0]);
	fps_real sy = sinf(a[1]);
	fps_real sz = sinf(a[2]);

	b[0]  = cy*cz;
	b[1]  = -cy*sz;
	b[2]  = sy;
	b[3]  = 0;
	b[4]  = cz*sx*sy+cx*sz;
	b[5]  = cx*cz-sx*sy*sz;
	b[6]  = -cy*sx;
	b[7]  = 0;
	b[8]  = sx*sz-cx*cz*sy;
	b[9]  = cz*sx+cx*sy*sz;
	b[10] = cx*cy;
	b[11] = 0;
	b[12] = 0;
	b[13] = 0;
	b[14] = 0;
	b[15] = 1;
}

void fps_mat4_mat3(
	fps_real a[16],
	fps_real b[9]
) {
	b[0] = a[0];
	b[1] = a[1];
	b[2] = a[2];
	b[3] = a[4];
	b[4] = a[5];
	b[5] = a[6];
	b[6] = a[8];
	b[7] = a[9];
	b[8] = a[10];
}

// COLLIDER MODULE

void fps_collider_new(
	fps_collider **collider
) {
	*collider = calloc(1,sizeof(fps_collider));

	if (*collider==NULL) return;

	fps_mat4_mov(_fps_mat4_id,(*collider)->offset);

	(*collider)->vertexes     = fps_cube;
	(*collider)->vertex_count = 36;
}

void fps_collider_free(
	fps_collider *collider
) {
	if (collider->body!=NULL) {
		fps_body_collider_remove(collider->body,collider);
	}

	free(collider);
}

void fps_collider_next(
	fps_collider  *collider,
	fps_collider **next
) {
	*next = collider->next;
}

void fps_collider_solid_get(
	fps_collider *collider,
	fps_uint     *state
) {
	*state = collider->solid;
}

void fps_collider_solid_set(
	fps_collider *collider,
	fps_uint      state
) {
	collider->solid = state;
}

void fps_collider_size_get(
	fps_collider *collider,
	fps_real      size[3]
) {
	fps_vec3_mov(collider->size,size);
}

void fps_collider_size_set(
	fps_collider *collider,
	fps_real      size[3]
) {
	fps_vec3_mov(size,collider->size);

	fps_collider_boundary_update(collider);

	if (collider->body!=NULL) {
		fps_body_mass_update(collider->body);
		fps_body_boundary_update(collider->body);
	}
}

void fps_collider_offset_get(
	fps_collider *collider,
	fps_real      offset[16]
) {
	fps_mat4_mov(collider->offset,offset);
}

void fps_collider_offset_set(
	fps_collider *collider,
	fps_real      offset[16]
) {
	fps_mat4_mov(offset,collider->offset);

	fps_collider_transform_update(collider);
	fps_collider_boundary_update(collider);

	if (collider->body!=NULL) {
		fps_body_mass_update(collider->body);
		fps_body_boundary_update(collider->body);
	}
}

void fps_collider_offset_position_get(
	fps_collider *collider,
	fps_real      position[3]
) {
	position[0] = collider->offset[3];
	position[1] = collider->offset[7];
	position[2] = collider->offset[11];
}

void fps_collider_transform_get(
	fps_collider *collider,
	fps_real      transform[16]
) {
	fps_mat4_mov(collider->transform,transform);
}

void fps_collider_global_position_get(
	fps_collider *collider,
	fps_real      position[3]
) {
	position[0] = collider->transform[3];
	position[1] = collider->transform[7];
	position[2] = collider->transform[11];
}

void fps_collider_boundary_get(
	fps_collider *collider,
	fps_real      point_a[3],
	fps_real      point_b[3]
) {
	fps_vec3_mov(collider->boundary[0],point_a);
	fps_vec3_mov(collider->boundary[1],point_b);
}

void fps_collider_density_get(
	fps_collider *collider,
	fps_real     *density
) {
	*density = collider->density;
}

void fps_collider_density_set(
	fps_collider *collider,
	fps_real      density
) {
	collider->density = density;

	if (collider->body!=NULL) {
		fps_body_mass_update(collider->body);
	}
}

void fps_collider_friction_get(
	fps_collider *collider,
	fps_real     *friction
) {
	*friction = collider->friction;
}

void fps_collider_friction_set(
	fps_collider *collider,
	fps_real      friction
) {
	collider->friction = friction;
}

void fps_collider_restitution_get(
	fps_collider *collider,
	fps_real     *restitution
) {
	*restitution = collider->restitution;
}

void fps_collider_restitution_set(
	fps_collider *collider,
	fps_real      restitution
) {
	collider->restitution = restitution;
}
/*
void fps_collider_vertexes_get(
	fps_collider *collider,
	fps_real    *(*vertexes)[3],
	fps_real     *vertex_count
) {
	*vertexes     = collider->vertexes;
	*vertex_count = collider->vertex_count;
}

void fps_collider_vertexes_set(
	fps_collider *collider,
	fps_real     (*vertexes)[3],
	fps_real      vertex_count
) {
	collider->vertexes     = vertexes;
	collider->vertex_count = vertex_count;
}
*/
void fps_collider_mass_get(
	fps_collider *collider,
	fps_real     *mass
) {
	*mass = (
		collider->size[0]*
		collider->size[1]*
		collider->size[2]*
		collider->density
	);
}

void fps_collider_transform_update(
	fps_collider *collider
) {
	fps_body *body = collider->body;

	if (body==NULL) return;

	fps_mat4_mul(body->transform,collider->offset,collider->transform);
}

void fps_collider_boundary_update(
	fps_collider *collider
) {
	fps_real min[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
	fps_real max[3] = {-FLT_MAX,-FLT_MAX,-FLT_MAX};

	for (fps_real z=-0.5; z<=0.5; z++) {
		for (fps_real y=-0.5; y<=0.5; y++) {
			for (fps_real x=-0.5; x<=0.5; x++) {
				fps_real point[3] = {x,y,z};

				fps_vec3_mul(point,collider->size,point);
				fps_mat4_vec3_mul(collider->transform,point,point);

				min[0] = FPS_MIN(point[0],min[0]);
				min[1] = FPS_MIN(point[1],min[1]);
				min[2] = FPS_MIN(point[2],min[2]);

				max[0] = FPS_MAX(point[0],max[0]);
				max[1] = FPS_MAX(point[1],max[1]);
				max[2] = FPS_MAX(point[2],max[2]);
			}
		}
	}

	fps_vec3_mov(min,collider->boundary[0]);
	fps_vec3_mov(max,collider->boundary[1]);
}

// BODY MODULE

void fps_body_new(
	fps_body **body
) {
	*body = calloc(1,sizeof(fps_body));

	if (*body==NULL) return;

	fps_mat4_mov(_fps_mat4_id,(*body)->transform);
	fps_mat3_mov(_fps_mat3_id,(*body)->inverse_inertia);
}

void fps_body_free(
	fps_body *body
) {
	fps_collider *collider = body->collider;

	while (collider!=NULL) {
		fps_collider *next = collider->next;
		fps_body_collider_remove(body,collider);
		collider = next;
	}

	free(body);
}

void fps_body_sleeping_get(
	fps_body *body,
	fps_uint *state
) {
	*state = body->sleeping;
}

void fps_body_sleeping_set(
	fps_body *body,
	fps_uint  state
) {
	body->sleeping = state;
}

void fps_body_anchored_get(
	fps_body *body,
	fps_uint *state
) {
	*state = body->anchored;
}

void fps_body_anchored_set(
	fps_body *body,
	fps_uint  state
) {
	body->anchored = state;
}

void fps_body_transform_get(
	fps_body *body,
	fps_real  transform[16]
) {
	fps_mat4_mov(body->transform,transform);
}

void fps_body_transform_set(
	fps_body *body,
	fps_real  transform[16]
) {
	fps_mat4_mov(transform,body->transform);

	fps_collider *collider = body->collider;

	while (collider!=NULL) {
		fps_collider_transform_update(collider);
		fps_collider_boundary_update(collider);

		collider = collider->next;
	}

	fps_body_boundary_update(body);
}

void fps_body_position_get(
	fps_body *body,
	fps_real  position[3]
) {
	position[0] = body->transform[3];
	position[1] = body->transform[7];
	position[2] = body->transform[11];
}

void fps_body_position_set(
	fps_body *body,
	fps_real  position[3]
) {
	body->transform[3]  = position[0];
	body->transform[7]  = position[1];
	body->transform[11] = position[2];

	fps_collider *collider = body->collider;

	while (collider!=NULL) {
		fps_collider_transform_update(collider);
		fps_collider_boundary_update(collider);

		collider = collider->next;
	}

	fps_body_boundary_update(body);
}

void fps_body_translation_apply(
	fps_body *body,
	fps_real translation[3]
) {
	fps_vec3_add(
		body->post_translation,
		translation,
		body->post_translation
	);
}

void fps_body_boundary_get(
	fps_body *body,
	fps_real  point_a[3],
	fps_real  point_b[3]
) {
	fps_vec3_mov(body->boundary[0],point_a);
	fps_vec3_mov(body->boundary[1],point_b);
}

void fps_body_mass_get(
	fps_body *body,
	fps_real *mass
) {
	*mass = body->mass;
}

void fps_body_mass_offset_get(
	fps_body *body,
	fps_real  mass_offset[3]
) {
	fps_vec3_mov(body->mass_offset,mass_offset);
}

void fps_body_force_get(
	fps_body *body,
	fps_real  force[3]
) {
	fps_vec3_mov(body->force,force);
}

void fps_body_force_set(
	fps_body *body,
	fps_real  force[3]
) {
	fps_vec3_mov(force,body->force);
}

void fps_body_torque_get(
	fps_body *body,
	fps_real  torque[3]
) {
	fps_vec3_mov(body->torque,torque);
}

void fps_body_torque_set(
	fps_body *body,
	fps_real  torque[3]
) {
	fps_vec3_mov(torque,body->torque);
}

void fps_body_velocity_get(
	fps_body *body,
	fps_real  velocity[3]
) {
	fps_vec3_mov(body->velocity,velocity);
}

void fps_body_velocity_set(
	fps_body *body,
	fps_real  velocity[3]
) {
	fps_vec3_mov(velocity,body->velocity);
}

void fps_body_angular_velocity_get(
	fps_body *body,
	fps_real angular_velocity[3]
) {
	fps_vec3_mov(body->angular_velocity,angular_velocity);
}

void fps_body_angular_velocity_set(
	fps_body *body,
	fps_real  angular_velocity[3]
) {
	fps_vec3_mov(angular_velocity,body->angular_velocity);
}

void fps_body_inverse_inertia_get(
	fps_body *body,
	fps_real  inverse_inertia[9]
) {
	fps_mat3_mov(body->inverse_inertia,inverse_inertia);
}

void fps_body_force_apply(
	fps_body *body,
	fps_real  force[3]
) {
	fps_vec3_add(body->force,force,body->force);
}

void fps_body_torque_apply(
	fps_body *body,
	fps_real  torque[3]
) {
	fps_vec3_add(body->torque,torque,body->torque);
}

void fps_body_linear_impulse_apply(
	fps_body *body,
	fps_real force[3]
) {
	fps_real impulse[3];

	fps_vec3_mov(force,impulse);
	fps_vec3_num_mul(impulse,1/body->mass,impulse);
	fps_vec3_add(body->velocity,impulse,body->velocity);
}

void fps_body_angular_impulse_apply(
	fps_body *body,
	fps_real torque[3]
) {
	fps_real impulse[3];

	fps_mat3_vec3_mul(body->inverse_inertia,torque,impulse);
	fps_vec3_add(body->angular_velocity,impulse,body->angular_velocity);
}

void fps_body_mass_update(
	fps_body *body
) {
	fps_real total_mass = 0;
	fps_real center[3]  = {0,0,0};
	fps_real ii[3]      = {0,0,0}; // Inverse inertia

	fps_collider *collider = body->collider;

	while (collider!=NULL) {
		fps_real offset[3];
		fps_real size[3];
		fps_real mass;

		fps_collider_offset_position_get(collider,offset);
		fps_collider_size_get(collider,size);
		fps_collider_mass_get(collider,&mass);

		fps_vec3_num_mul(offset,mass,offset);
		fps_vec3_add(center,offset,center);

		total_mass+=mass;

		fps_real im = 12*(1/mass);

		size[0] = powf(size[0],2);
		size[1] = powf(size[1],2);
		size[2] = powf(size[2],2);

		ii[0] += im/(size[0]+size[1]);
		ii[1] += im/(size[0]+size[2]);
		ii[2] += im/(size[0]+size[1]);

		collider = collider->next;
	}

	body->mass = total_mass;

	fps_vec3_num_div(center,total_mass,body->mass_offset);

	fps_real t[9];
	fps_real it[9]; // Inverse rotation
	fps_real ts[9];

	fps_mat4_mat3(body->transform,t);
	fps_mat3_inv(t,it);

	fps_mat3_scale(t,ii,ts);
	fps_mat3_mul(ts,it,body->inverse_inertia);
}

void fps_body_boundary_update(
	fps_body *body
) {
	fps_collider *collider = body->collider;

	if (collider==NULL) {
		fps_body_position_get(body,body->boundary[0]);
		fps_body_position_get(body,body->boundary[1]);

		return;
	}

	fps_real min[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
	fps_real max[3] = {-FLT_MAX,-FLT_MAX,-FLT_MAX};

	while (collider!=NULL) {
		min[0] = FPS_MIN(collider->boundary[0][0],min[0]);
		min[1] = FPS_MIN(collider->boundary[0][1],min[1]);
		min[2] = FPS_MIN(collider->boundary[0][2],min[2]);

		max[0] = FPS_MAX(collider->boundary[1][0],max[0]);
		max[1] = FPS_MAX(collider->boundary[1][1],max[1]);
		max[2] = FPS_MAX(collider->boundary[1][2],max[2]);

		collider = collider->next;
	}

	fps_vec3_mov(min,body->boundary[0]);
	fps_vec3_mov(max,body->boundary[1]);
}

void fps_body_collider_add(
	fps_body     *body,
	fps_collider *collider
) {
	if (collider->body==body) {
		return;
	} else if (collider->body!=NULL) {
		fps_body_collider_remove(collider->body,collider);
	}

	collider->body = body;
	collider->next = body->collider;

	body->collider = collider;

	fps_collider_transform_update(collider);
	fps_collider_boundary_update(collider);

	fps_body_mass_update(body);
	fps_body_boundary_update(body);
}

void fps_body_collider_remove(
	fps_body     *body,
	fps_collider *collider
) {
	if (
		collider->body!=body ||
		body->collider==NULL
	) {
		return;
	}

	fps_collider *current = body->collider;

	if (current==collider) {
		body->collider = current->next;

		current->body = NULL;
		current->next = NULL;
	} else {
		fps_collider *previous = current;
		current = current->next;

		while (current!=NULL) {
			if (current==collider) {
				previous->next = current->next;

				current->body = NULL;
				current->next = NULL;

				break;
			}

			previous = current;
			current  = current->next;
		}
	}

	fps_body_mass_update(body);
	fps_body_boundary_update(body);
}

void fps_body_collider_get(
	fps_body      *body,
	fps_collider **collider
) {
	*collider = body->collider;
}

void fps_body_step(
	fps_body *body,
	fps_real  dt
) {
	fps_uint sleeping;
	fps_uint anchored;

	fps_body_sleeping_get(body,&sleeping);
	fps_body_anchored_get(body,&anchored);

	if (!sleeping && !anchored) {
		fps_real dt_mass = body->mass*dt;

		// Apply forces
		body->velocity[0] += (body->force[0]/body->mass)*dt;
		body->velocity[1] += (body->force[1]/body->mass)*dt;
		body->velocity[2] += (body->force[2]/body->mass)*dt;

		body->angular_velocity[0] += (body->torque[0]/body->mass)*dt;
		body->angular_velocity[1] += (body->torque[1]/body->mass)*dt;
		body->angular_velocity[2] += (body->torque[2]/body->mass)*dt;

		// Apply movement
		body->transform[3]  += body->velocity[0]*dt;
		body->transform[7]  += body->velocity[1]*dt;
		body->transform[11] += body->velocity[2]*dt;

		body->transform[3]  += body->post_translation[0];
		body->transform[7]  += body->post_translation[1];
		body->transform[11] += body->post_translation[2];

		// Convert rotation to matrix
		fps_real delta_rotation[3] = {
			body->angular_velocity[0]*dt,
			body->angular_velocity[1]*dt,
			body->angular_velocity[2]*dt,
		};
		fps_real orientation[16];

		fps_mat4_euler(delta_rotation,orientation);

		// Remove translation from transform
		fps_real position[3] = {
			body->transform[3],
			body->transform[7],
			body->transform[11]
		};

		body->transform[3]  = 0;
		body->transform[7]  = 0;
		body->transform[11] = 0;

		// Apply rotation to transform
		fps_mat4_mul(orientation,body->transform,body->transform);

		// Restore translation to transform
		body->transform[3]  = position[0];
		body->transform[7]  = position[1];
		body->transform[11] = position[2];

		// Update collider
		fps_collider *collider = body->collider;

		while (collider!=NULL) {
			fps_collider_transform_update(collider);
			fps_collider_boundary_update(collider);

			collider = collider->next;
		}

		// Update boundary
		fps_body_boundary_update(body);
	}

	body->force[0] = 0;
	body->force[1] = 0;
	body->force[2] = 0;

	body->torque[0] = 0;
	body->torque[1] = 0;
	body->torque[2] = 0;

	body->post_translation[0] = 0;
	body->post_translation[1] = 0;
	body->post_translation[2] = 0;
}

// AABB MODULE

void fps_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  c[3],
	fps_real  d[3],
	fps_uint *r
) {
	*r=(
		b[0]>c[0] &&
		a[0]<d[0] &&
		b[1]>c[1] &&
		a[1]<d[1] &&
		b[2]>c[2] &&
		a[2]<d[2]
	);
}

// NGC MODULE

void fps_ngc_clip_edge(
	fps_real po[3], // Plane origin
	fps_real pn[3], // Plane normal
	fps_real eo[3], // Edge origin
	fps_real en[3], // Edge normal
	fps_real cp[3]  // Clip point
) {
	fps_real pn_po_dot;
	fps_real pn_en_dot;
	fps_real pn_eo_dot;

	fps_vec3_dot(pn,en,&pn_en_dot);

	// Edge is parallel to plane
	if (pn_en_dot==0) {
		fps_vec3_mov(eo,cp);

		return;
	}

	fps_vec3_dot(pn,po,&pn_po_dot);
	fps_vec3_dot(pn,eo,&pn_eo_dot);

	fps_real t = (pn_po_dot-pn_eo_dot)/pn_en_dot;

	cp[0] = eo[0]+en[0]*t;
	cp[1] = eo[1]+en[1]*t;
	cp[2] = eo[2]+en[2]*t;
}

void fps_ngc_clip_triangle(
	fps_real  po[3], // Plane origin
	fps_real  pn[3], // Plane normal
	fps_real  ta[3], // Triangle point A
	fps_real  tb[3], // Triangle point B
	fps_real  tc[3], // Triangle point C
	fps_uint *cs     // Clip status
) {
	fps_real co[3]; // Clip origin

	fps_real ta_po_unit[3];
	fps_real tb_po_unit[3];
	fps_real tc_po_unit[3];

	fps_real ta_pn_dot;
	fps_real tb_pn_dot;
	fps_real tc_pn_dot;

	*cs = 1;

	fps_vec3_sub(ta,po,ta_po_unit);
	fps_vec3_sub(tb,po,tb_po_unit);
	fps_vec3_sub(tc,po,tc_po_unit);

	fps_vec3_unit(ta_po_unit,ta_po_unit);
	fps_vec3_unit(tb_po_unit,tb_po_unit);
	fps_vec3_unit(tc_po_unit,tc_po_unit);

	fps_vec3_dot(ta_po_unit,pn,&ta_pn_dot);
	fps_vec3_dot(tb_po_unit,pn,&tb_pn_dot);
	fps_vec3_dot(tc_po_unit,pn,&tc_pn_dot);

	// Find vertex behind plane to use as origin
	if (ta_pn_dot<0) {
		fps_vec3_mov(ta,co);
	} else if (tb_pn_dot<0) {
		fps_vec3_mov(tb,co);
	} else if (tc_pn_dot<0) {
		fps_vec3_mov(tc,co);
	} else { // All vertexes are in front of the plane
		*cs = 0;

		return;
	}

	if (ta_pn_dot>0) {
		fps_real ta_co_unit[3];
		fps_vec3_sub(ta,co,ta_co_unit);
		fps_vec3_unit(ta_co_unit,ta_co_unit);
		fps_ngc_clip_edge(po,pn,co,ta_co_unit,ta);
	}

	if (tb_pn_dot>0) {
		fps_real tb_co_unit[3];
		fps_vec3_sub(tb,co,tb_co_unit);
		fps_vec3_unit(tb_co_unit,tb_co_unit);
		fps_ngc_clip_edge(po,pn,co,tb_co_unit,tb);
	}

	if (tc_pn_dot>0) {
		fps_real tc_co_unit[3];
		fps_vec3_sub(tc,co,tc_co_unit);
		fps_vec3_unit(tc_co_unit,tc_co_unit);
		fps_ngc_clip_edge(po,pn,co,tc_co_unit,tc);
	}
}

void fps_ngc_clip_convex(
	fps_real  va[][3], // Vertexes A
	fps_real  vb[][3], // Vertexes B
	fps_real  vc[][3], // Clipped vertexes
	fps_uint  va_len,  // Vertexes A length
	fps_uint  vb_len,  // Vertexes B length
	fps_uint *vc_len   // Clipped length
) {
	*vc_len = vb_len;

	for (fps_uint i=0; i<vb_len; i++) {
		fps_vec3_mov(vb[i],vc[i]);
	}

	for (fps_uint a=0; a<va_len; a+=3) {
		fps_real po[3];
		fps_real pn[3];

		fps_real ba[3];
		fps_real ca[3];

		fps_vec3_add(va[a],va[a+1],po);
		fps_vec3_add(va[a+2],po,po);
		fps_vec3_num_div(po,3,po);

		fps_vec3_sub(va[a+1],va[a],ba);
		fps_vec3_sub(va[a+2],va[a],ca);

		fps_vec3_cross(ba,ca,pn);
		fps_vec3_unit(pn,pn);

		for (fps_uint b=*vc_len; b>0; b-=3) {
			fps_uint cs;

			fps_ngc_clip_triangle(
				po,
				pn,
				vc[b-3],
				vc[b-2],
				vc[b-1],
				&cs
			);

			// Discard
			if (cs==0) {
				fps_vec3_mov(vc[*vc_len-3],vc[b-3]);
				fps_vec3_mov(vc[*vc_len-2],vc[b-2]);
				fps_vec3_mov(vc[*vc_len-1],vc[b-1]);

				*vc_len-=3;
			}
		}
	}
}

void fps_ngc_test_convex(
	fps_real  va[][3], // Vertexes A
	fps_real  vb[][3], // Vertexes B
	fps_uint  va_len,  // Vertexes A length
	fps_uint  vb_len,  // Vertexes B length
	fps_real  cp[3],   // Collision point
	fps_real  sn[3],   // Separation normal
	fps_real *sd       // Separation distance
) {
	fps_uint vc_len=vb_len;
	fps_real vc[vc_len][3];

	cp[0] = 0;
	cp[1] = 0;
	cp[2] = 0;
	sn[0] = 0;
	sn[1] = 0;
	sn[2] = 0;
	*sd   = 0;

	fps_ngc_clip_convex(
		va,
		vb,
		vc,
		va_len,
		vb_len,
		&vc_len
	);

	if (vc_len==0) {
		return;
	}

	// Calculate contact point and separation normal
	for (fps_uint i=0; i<vc_len; i+=3) {
		fps_real ab[3];
		fps_real ac[3];
		fps_real bc[3];

		fps_real ab_mag;
		fps_real ac_mag;
		fps_real bc_mag;

		fps_vec3_sub(vc[i],vc[i+1],ab);
		fps_vec3_sub(vc[i],vc[i+2],ac);
		fps_vec3_sub(vc[i+1],vc[i+2],bc);

		fps_vec3_mag(ab,&ab_mag);
		fps_vec3_mag(ac,&ac_mag);
		fps_vec3_mag(bc,&bc_mag);

		fps_vec3_add(cp,vc[i],cp);
		fps_vec3_add(cp,vc[i+1],cp);
		fps_vec3_add(cp,vc[i+2],cp);

		fps_real pn[3];

		fps_vec3_cross(ab,ac,pn);
		fps_vec3_unit(pn,pn);
		fps_vec3_num_mul(pn,ab_mag*ac_mag*bc_mag,pn);

		fps_vec3_add(sn,pn,sn);
	}

	fps_vec3_num_div(cp,vc_len,cp);
	fps_vec3_unit(sn,sn);

	// Calculate separation distance
	fps_real fp[3];
	fps_real np[3];
	fps_real fd = -FLT_MAX;
	fps_real nd = FLT_MAX;

	for (fps_uint i=0; i<vc_len; i++) {
		fps_real d;

		fps_vec3_dot(sn,vc[i],&d);

		if (d>fd) {
			fps_vec3_mov(vc[i],fp);
			fd=d;
		}

		if (d<nd) {
			fps_vec3_mov(vc[i],np);
			nd=d;
		}
	}

	fps_real fp_np[3];

	fps_vec3_sub(fp,np,fp_np);
	fps_vec3_dot(fp_np,sn,sd);
}

// SOLVER MODULE

void fps_collision_resolve(
	fps_body     *body_a,
	fps_body     *body_b,
	fps_collider *collider_a,
	fps_collider *collider_b,
	fps_real      cp[3],
	fps_real      sn[3],
	fps_real      sd
) {
	fps_real a_im = 1/body_a->mass;
	fps_real b_im = 1/body_b->mass;
	fps_real t_im = a_im+b_im;

	if (!body_a->anchored) {
		fps_real depth_ratio = sd*(a_im/t_im)*0.5;

		fps_body_translation_apply(
			body_a,
			(fps_real[3]){
				sn[0]*depth_ratio,
				sn[1]*depth_ratio,
				sn[2]*depth_ratio
			}
		);
	}

	if (!body_b->anchored) {
		fps_real depth_ratio = sd*(b_im/t_im)*0.5;

		fps_body_translation_apply(
			body_b,
			(fps_real[3]){
				-sn[0]*depth_ratio,
				-sn[1]*depth_ratio,
				-sn[2]*depth_ratio
			}
		);
	}

	fps_real r = collider_a->restitution*collider_b->restitution;
	fps_real f = collider_a->friction*collider_b->friction;

	// Velocity
	fps_real av[3];
	fps_real bv[3];

	fps_body_velocity_get(body_a,av);
	fps_body_velocity_get(body_b,bv);

	// Angular velocity
	fps_real aav[3];
	fps_real bav[3];

	fps_body_angular_velocity_get(body_a,aav);
	fps_body_angular_velocity_get(body_b,bav);

	// Relative offset
	fps_real ar[3];
	fps_real br[3];

	fps_body_position_get(body_a,ar);
	fps_body_position_get(body_b,br);

	fps_vec3_sub(cp,ar,ar);
	fps_vec3_sub(cp,br,br);

	// Tangential velocity
	fps_real atv[3];
	fps_real btv[3];

	fps_vec3_cross(aav,ar,atv);
	fps_vec3_cross(bav,br,btv);

	// Full velocity
	fps_real fv[3];

	fps_vec3_add(bv,btv,fv);
	fps_vec3_sub(fv,atv,fv);
	fps_vec3_sub(fv,av,fv);

	// Impulse
	fps_real i;
	fps_vec3_dot(fv,sn,&i);

	// Inverse inertia tensor
	fps_real aiit[9];
	fps_real biit[9];

	fps_body_inverse_inertia_get(body_a,aiit);
	fps_body_inverse_inertia_get(body_b,biit);

	// Inverse inertia vector
	fps_real aiiv[3];
	fps_real biiv[3];

	fps_vec3_cross(ar,sn,aiiv);
	fps_vec3_cross(br,sn,biiv);

	fps_mat3_vec3_mul(aiit,aiiv,aiiv);
	fps_mat3_vec3_mul(biit,biiv,biiv);
	
	// Moment of inertia
	fps_real ai[3];
	fps_real bi[3];
	fps_real ci[3];
	
	fps_vec3_cross(aiiv,ar,ai);
	fps_vec3_cross(biiv,br,bi);
	fps_vec3_add(ai,bi,ci);

	// Angular effect
	fps_real ae;

	fps_vec3_dot(ci,sn,&ae);

	// Full impulse
	fps_real j=(-(1+r)*i)/(t_im+ae);

	fps_real fip[3];
	fps_real fin[3];

	fps_vec3_num_mul(sn,j,fip);
	fps_vec3_num_mul(sn,-j,fin);

	// Angular full impulse
	fps_real aafi[3];
	fps_real bafi[3];

	fps_vec3_cross(ar,fin,aafi);
	fps_vec3_cross(br,fip,bafi);

	// Apply linear impulse
	fps_body_linear_impulse_apply(body_a,fin);
	fps_body_linear_impulse_apply(body_b,fip);

	// Apply angular impulse
	fps_body_angular_impulse_apply(body_a,aafi);
	fps_body_angular_impulse_apply(body_b,bafi);
}

void fps_collision_test(
	fps_body *body_a,
	fps_body *body_b
) {
	if (body_a->anchored && body_b->anchored) return;
	
	fps_uint body_aabb;

	fps_aabb_test(
		body_a->boundary[0],
		body_a->boundary[1],
		body_b->boundary[0],
		body_b->boundary[1],
		&body_aabb
	);

	if (!body_aabb) return;

	fps_collider *collider_a;
	fps_collider *collider_b;

	fps_body_collider_get(body_a,&collider_a);

	while (collider_a!=NULL) {
		fps_real vertexes_a[collider_a->vertex_count][3];
		fps_uint vertexes_a_gen = 0;

		fps_body_collider_get(body_b,&collider_b);

		while (collider_b!=NULL) {
			fps_uint collider_aabb;

			fps_aabb_test(
				collider_a->boundary[0],
				collider_a->boundary[1],
				collider_b->boundary[0],
				collider_b->boundary[1],
				&collider_aabb
			);

			if (collider_aabb) {
				fps_real vertexes_b[collider_b->vertex_count][3];
				
				if (!vertexes_a_gen) { // Only generate once
					for (fps_uint i=0; i<collider_a->vertex_count; i++) {
						fps_vec3_mov(collider_a->vertexes[i],vertexes_a[i]);
						fps_vec3_mul(vertexes_a[i],collider_a->size,vertexes_a[i]);
						fps_mat4_vec3_mul(
							collider_a->transform,
							vertexes_a[i],
							vertexes_a[i]
						);
					}

					vertexes_a_gen = 1;
				}

				for (fps_uint i=0; i<collider_b->vertex_count; i++) {
					fps_vec3_mov(collider_b->vertexes[i],vertexes_b[i]);
					fps_vec3_mul(vertexes_b[i],collider_b->size,vertexes_b[i]);
					fps_mat4_vec3_mul(
						collider_b->transform,
						vertexes_b[i],
						vertexes_b[i]
					);
				}

				fps_real cp[3];
				fps_real sn[3];
				fps_real sd;

				fps_ngc_test_convex(
					vertexes_a,
					vertexes_b,
					collider_a->vertex_count,
					collider_b->vertex_count,
					cp,
					sn,
					&sd
				);

				if (sd==0) {
					fps_ngc_test_convex(
						vertexes_b,
						vertexes_a,
						collider_b->vertex_count,
						collider_a->vertex_count,
						cp,
						sn,
						&sd
					);

					fps_vec3_neg(sn,sn);
				}

				fps_collision_resolve(
					body_a,
					body_b,
					collider_a,
					collider_b,
					cp,
					sn,
					sd
				);
			}

			fps_collider_next(collider_b,&collider_b);
		}

		fps_collider_next(collider_a,&collider_a);
	}
}

#endif
