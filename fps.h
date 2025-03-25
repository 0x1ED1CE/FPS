/*
MIT License

Copyright (c) 2025 Dice

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
#define FPS_VERSION_MINOR 2
#define FPS_VERSION_PATCH 4

typedef unsigned int fps_uint;
typedef float        fps_real;

// VEC3 MODULE

void fps_vec3_neg(
	fps_real a[3],
	fps_real b[3]
);

void fps_vec3_mov(
	fps_real a[3],
	fps_real b[3]
);

void fps_vec3_add(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

void fps_vec3_sub(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

void fps_vec3_mul(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

void fps_vec3_div(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

void fps_vec3_dot(
	fps_real  a[3],
	fps_real  b[3],
	fps_real *c
);

void fps_vec3_cross(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
);

void fps_vec3_unit(
	fps_real a[3],
	fps_real b[3]
);

void fps_vec3_mag(
	fps_real  a[3],
	fps_real *b
);

void fps_vec3_num_add(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

void fps_vec3_num_sub(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

void fps_vec3_num_mul(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

void fps_vec3_num_div(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
);

// MAT3 MODULE

void fps_mat3_mov(
	fps_real a[9],
	fps_real b[9]
);

void fps_mat3_transpose(
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

void fps_mat4_mov(
	fps_real a[16],
	fps_real b[16]
);

void fps_mat4_transpose(
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

#define FPS_PART_STATE_SOLID 1

#define FPS_BODY_STATE_DYNAMIC  1
#define FPS_BODY_STATE_SLEEPING 2

typedef struct fps_part fps_part;
typedef struct fps_body fps_body;

typedef struct fps_part {
	fps_body  *body;
	fps_part  *next;
	void      *user;
	fps_uint   solid;
	fps_real   size[3];
	fps_real   offset[16];
	fps_real   transform[16];
	fps_real   boundary[2][3];
	fps_real   density;
	fps_real   friction;
	fps_real   restitution;
	fps_real (*vertexes)[3];
	fps_uint   vertex_count;
} fps_part;

// COLLIDER MODULE

void fps_part_new(
	fps_part **part
);

void fps_part_free(
	fps_part *part
);

void fps_part_next(
	fps_part **part
);

void fps_part_user_get(
	fps_part *part,
	void    **user
);

void fps_part_user_set(
	fps_part *part,
	void     *user
);

void fps_part_solid_get(
	fps_part *part,
	fps_uint *state
);

void fps_part_solid_set(
	fps_part *part,
	fps_uint  state
);

void fps_part_size_get(
	fps_part *part,
	fps_real  size[3]
);

void fps_part_size_set(
	fps_part *part,
	fps_real  size[3]
);

void fps_part_offset_get(
	fps_part *part,
	fps_real  offset[16]
);

void fps_part_offset_set(
	fps_part *part,
	fps_real  offset[16]
);

void fps_part_offset_position_get(
	fps_part *part,
	fps_real  position[3]
);

void fps_part_transform_get(
	fps_part *part,
	fps_real  transform[16]
);

void fps_part_global_position_get(
	fps_part *part,
	fps_real  position[3]
);

void fps_part_boundary_get(
	fps_part *part,
	fps_real  point_a[3],
	fps_real  point_b[3]
);

void fps_part_density_get(
	fps_part *part,
	fps_real *density
);

void fps_part_density_set(
	fps_part *part,
	fps_real  density
);

void fps_part_friction_get(
	fps_part *part,
	fps_real *friction
);

void fps_part_friction_set(
	fps_part *part,
	fps_real  friction
);

void fps_part_restitution_get(
	fps_part *part,
	fps_real *restitution
);

void fps_part_restitution_set(
	fps_part *part,
	fps_real  restitution
);
/*
void fps_part_vertexes_get(
	fps_part *part,
	fps_real *(*vertexes)[3],
	fps_real *vertex_count
);

void fps_part_vertexes_set(
	fps_part  *part,
	fps_real (*vertexes)[3],
	fps_real   vertex_count
);
*/
void fps_part_mass_get(
	fps_part *part,
	fps_real *mass
);

void fps_part_transform_update(
	fps_part *part
);

void fps_part_boundary_update(
	fps_part *part
);

fps_uint fps_part_raycast(
	fps_part *part,
	fps_real  ro[3],
	fps_real  rn[3],
	fps_real  co[3],
	fps_real  cn[3]
);

// BODY MODULE

typedef struct fps_body {
	fps_part *part;
	void     *user;
	fps_uint  sleeping;
	fps_uint  dynamic;
	fps_real  transform[16];
	fps_real  boundary[2][3];
	fps_real  mass;
	fps_real  mass_offset[3];
	fps_real  force[3];
	fps_real  torque[3];
	fps_real  velocity[3];
	fps_real  angular_velocity[3];
	fps_real  inverse_inertia[9];
	fps_real  post_translation[3];
} fps_body;

void fps_body_new(
	fps_body **body
);

void fps_body_free(
	fps_body *body
);

void fps_body_user_get(
	fps_body *body,
	void    **user
);

void fps_body_user_set(
	fps_body *body,
	void     *user
);

void fps_body_sleeping_get(
	fps_body *body,
	fps_uint *state
);

void fps_body_sleeping_set(
	fps_body *body,
	fps_uint  state
);

void fps_body_dynamic_get(
	fps_body *body,
	fps_uint *state
);

void fps_body_dynamic_set(
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

void fps_body_rotation_set(
	fps_body *body,
	fps_real  rotation[3]
);

void fps_body_rotation_get(
	fps_body *body,
	fps_real  rotation[3]
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

void fps_body_part_add(
	fps_body *body,
	fps_part *part
);

void fps_body_part_remove(
	fps_body *body,
	fps_part *part
);

void fps_body_part_get(
	fps_body  *body,
	fps_part **part
);

void fps_body_step(
	fps_body *body,
	fps_real  dt
);

void fps_body_raycast(
	fps_body  *body,
	fps_real   ro[3],
	fps_real   rn[3],
	fps_real   co[3],
	fps_real   cn[3],
	fps_part **part
);

// RAYCAST MODULE

void fps_raycast_triangle(
	fps_real ta[3],
	fps_real tb[3],
	fps_real tc[3],
	fps_real ro[3],
	fps_real rn[3],
	fps_real co[3],
	fps_real cn[3]
);

// AABB MODULE

fps_uint fps_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  c[3],
	fps_real  d[3]
);

fps_uint fps_ray_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  ro[3],
	fps_real  rn[3]
);

// NGC MODULE

void fps_ngc_clip_edge(
	fps_real po[3], // Plane origin
	fps_real pn[3], // Plane normal
	fps_real ea[3], // Edge point A
	fps_real eb[3]  // Edge point B
);

fps_uint fps_ngc_clip_triangle(
	fps_real  po[3], // Plane origin
	fps_real  pn[3], // Plane normal
	fps_real  ta[3], // Triangle point A
	fps_real  tb[3], // Triangle point B
	fps_real  tc[3]  // Triangle point C
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
	fps_body *body_a,
	fps_body *body_b,
	fps_part *part_a,
	fps_part *part_b,
	fps_real  cp[3],
	fps_real  sn[3],
	fps_real  sd
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

#define FPS_MIN(A,B) ((A)<(B)?(A):(B))
#define FPS_MAX(A,B) ((A)>(B)?(A):(B))

#define FPS_MAX_VERTEXES 96

static fps_real fps_buffer_a[FPS_MAX_VERTEXES][3];
static fps_real fps_buffer_b[FPS_MAX_VERTEXES][3];
static fps_real fps_buffer_c[FPS_MAX_VERTEXES][3];

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

void fps_vec3_neg(
	fps_real a[3],
	fps_real b[3]
) {
	b[0] = -a[0];
	b[1] = -a[1];
	b[2] = -a[2];
}

void fps_vec3_mov(
	fps_real a[3],
	fps_real b[3]
) {
	b[0] = a[0];
	b[1] = a[1];
	b[2] = a[2];
}

void fps_vec3_add(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]+b[0];
	c[1] = a[1]+b[1];
	c[2] = a[2]+b[2];
}

void fps_vec3_sub(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]-b[0];
	c[1] = a[1]-b[1];
	c[2] = a[2]-b[2];
}

void fps_vec3_mul(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]*b[0];
	c[1] = a[1]*b[1];
	c[2] = a[2]*b[2];
}

void fps_vec3_div(
	fps_real a[3],
	fps_real b[3],
	fps_real c[3]
) {
	c[0] = a[0]/b[0];
	c[1] = a[1]/b[1];
	c[2] = a[2]/b[2];
}

void fps_vec3_dot(
	fps_real  a[3],
	fps_real  b[3],
	fps_real *c
) {
	*c = (a[0]*b[0])+(a[1]*b[1])+(a[2]*b[2]);
}

void fps_vec3_cross(
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

void fps_vec3_unit(
	fps_real a[3],
	fps_real b[3]
) {
	fps_real m=(fps_real)sqrtf(
		(float)
		a[0]*a[0]+
		a[1]*a[1]+
		a[2]*a[2]
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

void fps_vec3_mag(
	fps_real  a[3],
	fps_real *b
) {
	*b=(fps_real)sqrtf(
		(float)
		a[0]*a[0]+
		a[1]*a[1]+
		a[2]*a[2]
	);
}

void fps_vec3_num_add(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
) {
	c[0] = a[0]+b;
	c[1] = a[1]+b;
	c[2] = a[2]+b;
}

void fps_vec3_num_sub(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
) {
	c[0] = a[0]-b;
	c[1] = a[1]-b;
	c[2] = a[2]-b;
}

void fps_vec3_num_mul(
	fps_real a[3],
	fps_real b,
	fps_real c[3]
) {
	c[0] = a[0]*b;
	c[1] = a[1]*b;
	c[2] = a[2]*b;
}

void fps_vec3_num_div(
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

void fps_mat3_mov(
	fps_real a[9],
	fps_real b[9]
) {
	for (fps_uint i=0; i<9; i++) {
		b[i] = a[i];
	}
}

void fps_mat3_transpose(
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

	b[0] = a00;
	b[1] = a10;
	b[2] = a20;
	b[3] = a01;
	b[4] = a11;
	b[5] = a21;
	b[6] = a02;
	b[7] = a12;
	b[8] = a22;
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

	fps_real det=1.0/(
		a00*(a11*a22-a21*a12)-
		a01*(a10*a22-a12*a20)+
		a02*(a10*a21-a11*a20)
	);

	b[0] = (a11*a22-a21*a12)*det;
	b[1] = (a02*a21-a01*a22)*det;
	b[2] = (a01*a12-a02*a11)*det;
	b[3] = (a12*a20-a10*a22)*det;
	b[4] = (a00*a22-a02*a20)*det;
	b[5] = (a10*a02-a00*a12)*det;
	b[6] = (a10*a21-a20*a11)*det;
	b[7] = (a20*a01-a00*a21)*det;
	b[8] = (a00*a11-a10*a01)*det;
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

void fps_mat4_mov(
	fps_real a[16],
	fps_real b[16]
) {
	for (fps_uint i=0; i<16; i++) {
		b[i] = a[i];
	}
}

void fps_mat4_transpose(
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

	b[0]  = a00;
	b[1]  = a10;
	b[2]  = a20;
	b[3]  = a30;
	b[4]  = a01;
	b[5]  = a11;
	b[6]  = a21;
	b[7]  = a31;
	b[8]  = a02;
	b[9]  = a12;
	b[10] = a22;
	b[11] = a32;
	b[12] = a03;
	b[13] = a13;
	b[14] = a23;
	b[15] = a33;
}

void fps_mat4_vec3_mul(
	fps_real a[16],
	fps_real b[3],
	fps_real c[3]
) {
	fps_real r0 = b[0]*a[0]+b[1]*a[1]+b[2]*a[2]+a[3];
	fps_real r1 = b[0]*a[4]+b[1]*a[5]+b[2]*a[6]+a[7];
	fps_real r2 = b[0]*a[8]+b[1]*a[9]+b[2]*a[10]+a[11];

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

	fps_real det = 1.0/(a00*b00+a01*b10+a02*b20+a03*b30);

	b[0]  = b00*det;
	b[1]  = b01*det;
	b[2]  = b02*det;
	b[3]  = b03*det;
	b[4]  = b10*det;
	b[5]  = b11*det;
	b[6]  = b12*det;
	b[7]  = b13*det;
	b[8]  = b20*det;
	b[9]  = b21*det;
	b[10] = b22*det;
	b[11] = b23*det;
	b[12] = b30*det;
	b[13] = b31*det;
	b[14] = b32*det;
	b[15] = b33*det;
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
	// b[3]  = 0;
	b[4]  = cz*sx*sy+cx*sz;
	b[5]  = cx*cz-sx*sy*sz;
	b[6]  = -cy*sx;
	// b[7]  = 0;
	b[8]  = sx*sz-cx*cz*sy;
	b[9]  = cz*sx+cx*sy*sz;
	b[10] = cx*cy;
	// b[11] = 0;
	// b[12] = 0;
	// b[13] = 0;
	// b[14] = 0;
	// b[15] = 1;
}

void fps_mat4_scale(
	fps_real a[16],
	fps_real b[3],
	fps_real c[16]
) {
	fps_mat4_mul(a,(fps_real[16]){
		b[0], 0,    0,    0,
		0,    b[1], 0,    0,
		0,    0,    b[2], 0,
		0,    0,    0,    1
	},c);
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

void fps_part_new(
	fps_part **part
) {
	*part = calloc(1,sizeof(fps_part));

	if (*part==NULL) return;

	fps_mat4_mov(_fps_mat4_id,(*part)->offset);

	(*part)->vertexes     = fps_cube;
	(*part)->vertex_count = 36;
}

void fps_part_free(
	fps_part *part
) {
	if (part->body!=NULL) {
		fps_body_part_remove(part->body,part);
	}

	free(part);
}

void fps_part_next(
	fps_part **part
) {
	if (*part!=NULL) *part = (*part)->next;
}

void fps_part_user_get(
	fps_part *part,
	void    **user
) {
	*user = part->user;
}

void fps_part_user_set(
	fps_part *part,
	void     *user
) {
	part->user = user;
}

void fps_part_solid_get(
	fps_part *part,
	fps_uint *state
) {
	*state = part->solid;
}

void fps_part_solid_set(
	fps_part *part,
	fps_uint  state
) {
	part->solid = state;
}

void fps_part_size_get(
	fps_part *part,
	fps_real  size[3]
) {
	fps_vec3_mov(part->size,size);
}

void fps_part_size_set(
	fps_part *part,
	fps_real  size[3]
) {
	fps_vec3_mov(size,part->size);

	fps_part_boundary_update(part);

	if (part->body!=NULL) {
		fps_body_mass_update(part->body);
		fps_body_boundary_update(part->body);
	}
}

void fps_part_offset_get(
	fps_part *part,
	fps_real  offset[16]
) {
	fps_mat4_mov(part->offset,offset);
}

void fps_part_offset_set(
	fps_part *part,
	fps_real  offset[16]
) {
	fps_mat4_mov(offset,part->offset);

	fps_part_transform_update(part);
	fps_part_boundary_update(part);

	if (part->body!=NULL) {
		fps_body_mass_update(part->body);
		fps_body_boundary_update(part->body);
	}
}

void fps_part_offset_position_get(
	fps_part *part,
	fps_real  position[3]
) {
	position[0] = part->offset[3];
	position[1] = part->offset[7];
	position[2] = part->offset[11];
}

void fps_part_transform_get(
	fps_part *part,
	fps_real  transform[16]
) {
	fps_mat4_mov(part->transform,transform);
}

void fps_part_global_position_get(
	fps_part *part,
	fps_real  position[3]
) {
	position[0] = part->transform[3];
	position[1] = part->transform[7];
	position[2] = part->transform[11];
}

void fps_part_boundary_get(
	fps_part *part,
	fps_real  point_a[3],
	fps_real  point_b[3]
) {
	fps_vec3_mov(part->boundary[0],point_a);
	fps_vec3_mov(part->boundary[1],point_b);
}

void fps_part_density_get(
	fps_part *part,
	fps_real *density
) {
	*density = part->density;
}

void fps_part_density_set(
	fps_part *part,
	fps_real  density
) {
	part->density = density;

	if (part->body!=NULL) {
		fps_body_mass_update(part->body);
	}
}

void fps_part_friction_get(
	fps_part *part,
	fps_real *friction
) {
	*friction = part->friction;
}

void fps_part_friction_set(
	fps_part *part,
	fps_real  friction
) {
	part->friction = friction;
}

void fps_part_restitution_get(
	fps_part *part,
	fps_real *restitution
) {
	*restitution = part->restitution;
}

void fps_part_restitution_set(
	fps_part *part,
	fps_real  restitution
) {
	part->restitution = restitution;
}
/*
void fps_part_vertexes_get(
	fps_part *part,
	fps_real *(*vertexes)[3],
	fps_real *vertex_count
) {
	*vertexes     = part->vertexes;
	*vertex_count = part->vertex_count;
}

void fps_part_vertexes_set(
	fps_part  *part,
	fps_real (*vertexes)[3],
	fps_real   vertex_count
) {
	part->vertexes     = vertexes;
	part->vertex_count = vertex_count;
}
*/
void fps_part_mass_get(
	fps_part *part,
	fps_real *mass
) {
	*mass = (
		part->size[0]*
		part->size[1]*
		part->size[2]*
		part->density
	);
}

void fps_part_transform_update(
	fps_part *part
) {
	fps_body *body = part->body;

	if (body==NULL) return;

	fps_mat4_mul(body->transform,part->offset,part->transform);
}

void fps_part_boundary_update(
	fps_part *part
) {
	fps_real min[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
	fps_real max[3] = {-FLT_MAX,-FLT_MAX,-FLT_MAX};

	for (fps_real z=-0.5; z<=0.5; z++) {
		for (fps_real y=-0.5; y<=0.5; y++) {
			for (fps_real x=-0.5; x<=0.5; x++) {
				fps_real point[3] = {x,y,z};

				fps_vec3_mul(point,part->size,point);
				fps_mat4_vec3_mul(part->transform,point,point);

				min[0] = FPS_MIN(point[0],min[0]);
				min[1] = FPS_MIN(point[1],min[1]);
				min[2] = FPS_MIN(point[2],min[2]);

				max[0] = FPS_MAX(point[0],max[0]);
				max[1] = FPS_MAX(point[1],max[1]);
				max[2] = FPS_MAX(point[2],max[2]);
			}
		}
	}

	fps_vec3_mov(min,part->boundary[0]);
	fps_vec3_mov(max,part->boundary[1]);
}

fps_uint fps_part_raycast(
	fps_part *part,
	fps_real  ro[3],
	fps_real  rn[3],
	fps_real  co[3],
	fps_real  cn[3]
) {
	cn[0] = 0;
	cn[1] = 0;
	cn[2] = 0;

	if (!fps_ray_aabb_test(
		part->boundary[0],
		part->boundary[1],
		ro,
		rn
	)) return 0;

	for (fps_uint i=0; i<part->vertex_count; i+=3) {
		fps_real ta[3];
		fps_real tb[3];
		fps_real tc[3];

		fps_vec3_mul(part->size,part->vertexes[i],ta);
		fps_vec3_mul(part->size,part->vertexes[i+1],tb);
		fps_vec3_mul(part->size,part->vertexes[i+2],tc);

		fps_mat4_vec3_mul(part->transform,ta,ta);
		fps_mat4_vec3_mul(part->transform,tb,tb);
		fps_mat4_vec3_mul(part->transform,tc,tc);

		fps_raycast_triangle(ta,tb,tc,ro,rn,co,cn);

		if (
			cn[0]!=0 ||
			cn[1]!=0 ||
			cn[2]!=0
		) return 1;
	}

	return 0;
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
	fps_part *part = body->part;

	while (part!=NULL) {
		fps_part *next = part->next;
		fps_body_part_remove(body,part);
		part = next;
	}

	free(body);
}

void fps_body_user_get(
	fps_body *body,
	void    **user
) {
	*user = body->user;
}

void fps_body_user_set(
	fps_body *body,
	void     *user
) {
	body->user = user;
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

void fps_body_dynamic_get(
	fps_body *body,
	fps_uint *state
) {
	*state = body->dynamic;
}

void fps_body_dynamic_set(
	fps_body *body,
	fps_uint  state
) {
	body->dynamic = state;
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

	fps_part *part = body->part;

	while (part!=NULL) {
		fps_part_transform_update(part);
		fps_part_boundary_update(part);

		part = part->next;
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

	fps_part *part = body->part;

	while (part!=NULL) {
		fps_part_transform_update(part);
		fps_part_boundary_update(part);

		part = part->next;
	}

	fps_body_boundary_update(body);
}

void fps_body_rotation_set(
	fps_body *body,
	fps_real  rotation[3]
) {
	fps_mat4_euler(
		rotation,
		body->transform
	);

	fps_part *part = body->part;

	while (part!=NULL) {
		fps_part_transform_update(part);
		fps_part_boundary_update(part);

		part = part->next;
	}

	fps_body_boundary_update(body);
}

void fps_body_rotation_get(
	fps_body *body,
	fps_real  rotation[3]
) {
	fps_real *t = body->transform;

	rotation[0] = atan2f(-t[6],t[10]);
	rotation[1] = asinf(t[2]);
	rotation[2] = atan2f(-t[1],t[0]);
}

void fps_body_translation_apply(
	fps_body *body,
	fps_real translation[3]
) {
	fps_vec3_add(
		translation,
		body->post_translation,
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
	if (!body->dynamic) return;

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
	if (!body->dynamic) return;

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
	if (!body->dynamic) return;

	fps_vec3_mov(velocity,body->velocity);
}

void fps_body_angular_velocity_get(
	fps_body *body,
	fps_real angular_velocity[3]
) {
	fps_vec3_mov(
		body->angular_velocity,
		angular_velocity
	);
}

void fps_body_angular_velocity_set(
	fps_body *body,
	fps_real  angular_velocity[3]
) {
	if (!body->dynamic) return;

	fps_vec3_mov(
		angular_velocity,
		body->angular_velocity
	);
}

void fps_body_inverse_inertia_get(
	fps_body *body,
	fps_real  inverse_inertia[9]
) {
	fps_mat3_mov(
		body->inverse_inertia,
		inverse_inertia
	);
}

void fps_body_force_apply(
	fps_body *body,
	fps_real  force[3]
) {
	if (!body->dynamic) return;

	fps_vec3_add(
		body->force,
		force,
		body->force
	);
}

void fps_body_torque_apply(
	fps_body *body,
	fps_real  torque[3]
) {
	if (!body->dynamic) return;

	fps_vec3_add(
		body->torque,
		torque,
		body->torque
	);
}

void fps_body_linear_impulse_apply(
	fps_body *body,
	fps_real force[3]
) {
	if (!body->dynamic) return;

	fps_real impulse[3];

	fps_vec3_num_mul(
		force,
		1.0/body->mass,
		impulse
	);
	fps_vec3_add(
		body->velocity,
		impulse,
		body->velocity
	);
}

void fps_body_angular_impulse_apply(
	fps_body *body,
	fps_real torque[3]
) {
	if (!body->dynamic) return;

	fps_real impulse[3];

	fps_mat3_vec3_mul(
		body->inverse_inertia,
		torque,
		impulse
	);
	fps_vec3_add(
		body->angular_velocity,
		impulse,
		body->angular_velocity
	);
}

void fps_body_mass_update(
	fps_body *body
) {
	fps_real total_mass = 0;
	fps_real center[3]  = {0,0,0};
	fps_real ii[3]      = {0,0,0}; // Inverse inertia

	fps_part *part = body->part;

	while (part!=NULL) {
		fps_real offset[3];
		fps_real size[3];
		fps_real mass;

		fps_part_offset_position_get(part,offset);
		fps_part_size_get(part,size);
		fps_part_mass_get(part,&mass);

		fps_vec3_num_mul(offset,mass,offset);
		fps_vec3_add(center,offset,center);

		total_mass+=mass;

		fps_real im = 12.0*(1.0/mass);

		size[0] *= size[0];
		size[1] *= size[1];
		size[2] *= size[2];

		ii[0] += im/(size[0]+size[1]);
		ii[1] += im/(size[0]+size[2]);
		ii[2] += im/(size[0]+size[1]);

		part = part->next;
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
	fps_part *part = body->part;

	if (part==NULL) {
		fps_body_position_get(body,body->boundary[0]);
		fps_body_position_get(body,body->boundary[1]);

		return;
	}

	fps_real min[3] = {FLT_MAX,FLT_MAX,FLT_MAX};
	fps_real max[3] = {-FLT_MAX,-FLT_MAX,-FLT_MAX};

	while (part!=NULL) {
		min[0] = FPS_MIN(part->boundary[0][0],min[0]);
		min[1] = FPS_MIN(part->boundary[0][1],min[1]);
		min[2] = FPS_MIN(part->boundary[0][2],min[2]);

		max[0] = FPS_MAX(part->boundary[1][0],max[0]);
		max[1] = FPS_MAX(part->boundary[1][1],max[1]);
		max[2] = FPS_MAX(part->boundary[1][2],max[2]);

		part = part->next;
	}

	fps_vec3_mov(min,body->boundary[0]);
	fps_vec3_mov(max,body->boundary[1]);
}

void fps_body_part_add(
	fps_body *body,
	fps_part *part
) {
	if (part->body==body) {
		return;
	} else if (part->body!=NULL) {
		fps_body_part_remove(part->body,part);
	}

	part->body = body;
	part->next = body->part;
	body->part = part;

	fps_part_transform_update(part);
	fps_part_boundary_update(part);

	fps_body_mass_update(body);
	fps_body_boundary_update(body);
}

void fps_body_part_remove(
	fps_body *body,
	fps_part *part
) {
	if (
		part->body!=body ||
		body->part==NULL
	) {
		return;
	}

	fps_part *current = body->part;

	if (current==part) {
		body->part = current->next;

		current->body = NULL;
		current->next = NULL;
	} else {
		fps_part *previous = current;
		current = current->next;

		while (current!=NULL) {
			if (current==part) {
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

void fps_body_part_get(
	fps_body  *body,
	fps_part **part
) {
	*part = body->part;
}

void fps_body_step(
	fps_body *body,
	fps_real  dt
) {
	if (!body->sleeping && body->dynamic) {
		fps_real dt_im = dt/body->mass;

		// Dampen velocity
		fps_vec3_num_mul(
			body->velocity,
			0.999,
			body->velocity
		);
		fps_vec3_num_mul(
			body->angular_velocity,
			0.999,
			body->angular_velocity
		);

		// Apply acceleration
		body->velocity[0] += body->force[0]*dt_im;
		body->velocity[1] += body->force[1]*dt_im;
		body->velocity[2] += body->force[2]*dt_im;

		body->angular_velocity[0] += body->torque[0]*dt_im;
		body->angular_velocity[1] += body->torque[1]*dt_im;
		body->angular_velocity[2] += body->torque[2]*dt_im;

		// Apply movement
		body->transform[3]  += body->velocity[0]*dt;
		body->transform[7]  += body->velocity[1]*dt;
		body->transform[11] += body->velocity[2]*dt;

		body->transform[3]  += body->post_translation[0];
		body->transform[7]  += body->post_translation[1];
		body->transform[11] += body->post_translation[2];

		// Remove translation from transform
		fps_real position[3] = {
			body->transform[3],
			body->transform[7],
			body->transform[11]
		};

		body->transform[3]  = 0;
		body->transform[7]  = 0;
		body->transform[11] = 0;

		// Convert rotation to matrix
		fps_real delta_rotation[3] = {
			body->angular_velocity[0]*dt,
			body->angular_velocity[1]*dt,
			body->angular_velocity[2]*dt,
		};
		fps_real orientation[16];

		fps_mat4_mov(
			_fps_mat4_id,
			orientation
		);
		fps_mat4_euler(
			delta_rotation,
			orientation
		);

		// Apply rotation to transform
		fps_mat4_mul(
			orientation,
			body->transform,
			body->transform
		);

		// Restore translation to transform
		body->transform[3]  = position[0];
		body->transform[7]  = position[1];
		body->transform[11] = position[2];

		// Update part
		fps_part *part = body->part;

		while (part!=NULL) {
			fps_part_transform_update(part);
			fps_part_boundary_update(part);

			part = part->next;
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

void fps_body_raycast(
	fps_body  *body,
	fps_real   ro[3],
	fps_real   rn[3],
	fps_real   co[3],
	fps_real   cn[3],
	fps_part **part
) {
	cn[0] = 0;
	cn[1] = 0;
	cn[2] = 0;
	*part = NULL;

	if (!fps_ray_aabb_test(
		body->boundary[0],
		body->boundary[1],
		ro,
		rn
	)) return;

	fps_real cd = FLT_MAX;

	fps_part *current = body->part;

	while (current!=NULL) {
		fps_real cco[3];
		fps_real ccn[3];

		if (fps_part_raycast(
			current,
			ro,
			rn,
			cco,
			ccn
		)) {
			fps_real cco_ro[3];
			fps_real ccd;

			fps_vec3_sub(cco,ro,cco_ro);
			fps_vec3_mag(cco_ro,&ccd);

			if (ccd<cd) {
				fps_vec3_mov(cco,co);
				fps_vec3_mov(ccn,cn);

				cd        = ccd;
				*part = current;
			}
		}

		current = current->next;
	}
}

// RAYCAST MODULE

void fps_raycast_triangle(
	fps_real ta[3],
	fps_real tb[3],
	fps_real tc[3],
	fps_real ro[3],
	fps_real rn[3],
	fps_real co[3],
	fps_real cn[3]
) {
	cn[0] = 0;
	cn[1] = 0;
	cn[2] = 0;

	fps_real ba[3];
	fps_real ca[3];
	fps_real tn[3];

	fps_real tn_rn_dot;
	fps_real tn_ta_dot;
	fps_real tn_ro_dot;

	fps_vec3_sub(tb,ta,ba);
	fps_vec3_sub(tc,ta,ca);

	fps_vec3_cross(ba,ca,tn);
	fps_vec3_unit(tn,tn);

	fps_vec3_dot(tn,rn,&tn_rn_dot);

	// Ray direction does not face triangle
	if (tn_rn_dot>=0) return;

	// Calculate surface point
	fps_vec3_dot(tn,ta,&tn_ta_dot);
	fps_vec3_dot(tn,ro,&tn_ro_dot);

	fps_real t = (tn_ta_dot-tn_ro_dot)/tn_rn_dot;

	// Triangle is behind ray
	if (t<=0) return;

	co[0] = ro[0]+rn[0]*t;
	co[1] = ro[1]+rn[1]*t;
	co[2] = ro[2]+rn[2]*t;

	// Check if surface point is within triangle
	fps_real a[3];
	fps_real b[3];
	fps_real c[3];

	fps_real u[3];
	fps_real v[3];
	fps_real w[3];

	fps_real uv;
	fps_real uw;

	fps_vec3_sub(ta,co,a);
	fps_vec3_sub(tb,co,b);
	fps_vec3_sub(tc,co,c);

	fps_vec3_cross(b,c,u);
	fps_vec3_cross(c,a,v);

	fps_vec3_dot(u,v,&uv);

	if (uv<0) return;

	fps_vec3_cross(a,b,w);
	fps_vec3_dot(u,w,&uw);

	if (uw<0) return;

	fps_vec3_mov(tn,cn);
}

// AABB MODULE

fps_uint fps_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  c[3],
	fps_real  d[3]
) {
	return (
		b[0]>c[0] &&
		a[0]<d[0] &&
		b[1]>c[1] &&
		a[1]<d[1] &&
		b[2]>c[2] &&
		a[2]<d[2]
	);
}

fps_uint fps_ray_aabb_test(
	fps_real  a[3],
	fps_real  b[3],
	fps_real  ro[3],
	fps_real  rn[3]
) {
	fps_real t_min[3];
	fps_real t_max[3];

	t_min[0] = (a[0]-ro[0])/rn[0];
	t_max[0] = (b[0]-ro[0])/rn[0];

	if (t_min[0]>t_max[0]) {
		fps_real temp = t_min[0];
		t_min[0]      = t_max[0];
		t_max[0]      = temp;
	}

	t_min[1] = (a[1]-ro[1])/rn[1];
	t_max[1] = (b[1]-ro[1])/rn[1];

	if (t_min[1]>t_max[1]) {
		fps_real temp = t_min[1];
		t_min[1]      = t_max[1];
		t_max[1]      = temp;
	}

	if (t_min[0]>t_max[1] || t_min[1]>t_max[0]) return 0;
	if (t_min[1]>t_min[0]) t_min[0] = t_min[1];
	if (t_max[1]<t_max[0]) t_max[0] = t_max[1];

	t_min[2] = (a[2]-ro[2])/rn[2];
	t_max[2] = (b[2]-ro[2])/rn[2];

	if (t_min[2]>t_max[2]) {
		fps_real temp = t_min[2];
		t_min[2]      = t_max[2];
		t_max[2]      = temp;
	}

	if (t_min[0]>t_max[2] || t_min[2]>t_max[0]) return 0;
	// if (t_min[2]>t_min[0]) t_min[0] = t_min[2];
	// if (t_max[2]<t_max[0]) t_max[0] = t_min[2];

	return 1;
}

// NGC MODULE

void fps_ngc_clip_edge(
	fps_real po[3], // Plane origin
	fps_real pn[3], // Plane normal
	fps_real ea[3], // Edge point A
	fps_real eb[3]  // Edge point B
) {
	fps_real en[3];

	fps_real pn_po_dot;
	fps_real pn_ea_dot;
	fps_real pn_en_dot;

	fps_vec3_sub(eb,ea,en);
	fps_vec3_unit(en,en);

	fps_vec3_dot(pn,po,&pn_po_dot);
	fps_vec3_dot(pn,ea,&pn_ea_dot);
	fps_vec3_dot(pn,en,&pn_en_dot);

	fps_real t = (pn_po_dot-pn_ea_dot)/pn_en_dot;

	eb[0] = ea[0]+en[0]*t;
	eb[1] = ea[1]+en[1]*t;
	eb[2] = ea[2]+en[2]*t;
}

fps_uint fps_ngc_clip_triangle(
	fps_real  po[3], // Plane origin
	fps_real  pn[3], // Plane normal
	fps_real  ta[3], // Triangle point A
	fps_real  tb[3], // Triangle point B
	fps_real  tc[3]  // Triangle point C
) {
	fps_real ta_po[3];
	fps_real tb_po[3];
	fps_real tc_po[3];

	fps_real ta_pn_dot;
	fps_real tb_pn_dot;
	fps_real tc_pn_dot;

	fps_vec3_sub(ta,po,ta_po);
	fps_vec3_sub(tb,po,tb_po);
	fps_vec3_sub(tc,po,tc_po);

	fps_vec3_unit(ta_po,ta_po);
	fps_vec3_unit(tb_po,tb_po);
	fps_vec3_unit(tc_po,tc_po);

	fps_vec3_dot(ta_po,pn,&ta_pn_dot);
	fps_vec3_dot(tb_po,pn,&tb_pn_dot);
	fps_vec3_dot(tc_po,pn,&tc_pn_dot);

	// Find vertex furthest behind plane to use as origin
	if (ta_pn_dot<0.0) {
		if (tb_pn_dot>0.01) fps_ngc_clip_edge(po,pn,ta,tb);
		if (tc_pn_dot>0.01) fps_ngc_clip_edge(po,pn,ta,tc);
	} else if (tb_pn_dot<0.0) {
		if (ta_pn_dot>0.01) fps_ngc_clip_edge(po,pn,tb,ta);
		if (tc_pn_dot>0.01) fps_ngc_clip_edge(po,pn,tb,tc);
	} else if (tc_pn_dot<0.0) {
		if (ta_pn_dot>0.01) fps_ngc_clip_edge(po,pn,tc,ta);
		if (tb_pn_dot>0.01) fps_ngc_clip_edge(po,pn,tc,tb);
	// All vertexes are in front of the plane
	} else {
		return 1;
	}

	return 0;
}

void fps_ngc_clip_convex(
	fps_real  va[][3], // Vertexes A
	fps_real  vb[][3], // Vertexes B
	fps_real  vc[][3], // Clipped vertexes
	fps_uint  va_len,  // Vertexes A length
	fps_uint  vb_len,  // Vertexes B length
	fps_uint *vc_len   // Clipped length
) {
	for (fps_uint i=0; i<*vc_len; i++) {
		fps_vec3_mov(vb[i],vc[i]);
	}

	for (fps_uint a=0; a<va_len; a+=3) {
		fps_real ba[3];
		fps_real ca[3];
		fps_real pn[3];

		fps_vec3_sub(va[a+1],va[a],ba);
		fps_vec3_sub(va[a+2],va[a],ca);

		fps_vec3_cross(ba,ca,pn);
		fps_vec3_unit(pn,pn);

		for (fps_uint b=*vc_len; b>0; b-=3) {
			if (fps_ngc_clip_triangle(
				va[a],
				pn,
				vc[b-3],
				vc[b-2],
				vc[b-1]
			)) { // Discard
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
	cp[0] = 0.0;
	cp[1] = 0.0;
	cp[2] = 0.0;
	sn[0] = 0.0;
	sn[1] = 0.0;
	sn[2] = 0.0;
	*sd   = 0.0;

	// Clip intersecting convex volumes
	fps_uint vc_len   = vb_len;
	fps_real (*vc)[3] = fps_buffer_c;

	fps_ngc_clip_convex(
		va,
		vb,
		vc,
		va_len,
		vb_len,
		&vc_len
	);

	// Return if volumes do not intersect
	if (vc_len==0) return;

	// Approximate contact point and separation normal
	for (fps_uint i=0; i<vc_len; i+=3) {
		fps_real ba[3];
		fps_real ca[3];
		fps_real pn[3];

		fps_vec3_add(cp,vc[i],cp);
		fps_vec3_add(cp,vc[i+1],cp);
		fps_vec3_add(cp,vc[i+2],cp);

		fps_vec3_sub(vc[i+1],vc[i],ba);
		fps_vec3_sub(vc[i+2],vc[i],ca);

		fps_vec3_cross(ba,ca,pn);
		fps_vec3_add(sn,pn,sn);
	}

	fps_vec3_num_div(cp,vc_len,cp);
	fps_vec3_unit(sn,sn);

	// Calculate separation distance
	fps_real fd = -FLT_MAX;
	fps_real nd = FLT_MAX;

	for (fps_uint i=0; i<vc_len; i++) {
		fps_real d;

		fps_vec3_dot(sn,vc[i],&d);

		fd = FPS_MAX(fd,d);
		nd = FPS_MIN(nd,d);
	}

	// printf("wtf\n"); // For some reason this fixes the bug

	*sd = fd-nd;
}

// SOLVER MODULE

void fps_collision_resolve(
	fps_body *body_a,
	fps_body *body_b,
	fps_part *part_a,
	fps_part *part_b,
	fps_real  cp[3],
	fps_real  sn[3],
	fps_real  sd
) {
	if (sd==0.0) return;

	fps_real a_im = (1.0/body_a->mass)*(body_a->dynamic?1.0:0.0);
	fps_real b_im = (1.0/body_b->mass)*(body_b->dynamic?1.0:0.0);
	fps_real t_im = a_im+b_im;

	if (body_a->dynamic) {
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

	if (body_b->dynamic) {
		fps_real depth_ratio = -sd*(b_im/t_im)*0.5;

		fps_body_translation_apply(
			body_b,
			(fps_real[3]){
				sn[0]*depth_ratio,
				sn[1]*depth_ratio,
				sn[2]*depth_ratio
			}
		);
	}

	fps_real r = part_a->restitution*part_b->restitution;
	fps_real f = part_a->friction*part_b->friction;

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
	fps_real fav[3];
	fps_real fbv[3];

	fps_vec3_add(av,atv,fav);
	fps_vec3_add(bv,btv,fbv);

	// Relative velocity
	fps_real rv[3];

	fps_vec3_sub(fbv,fav,rv);

	// Impulse
	fps_real i;
	fps_vec3_dot(rv,sn,&i);

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
	fps_real j=(-(1.0+r)*i)/(t_im+ae);

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

	// Now do the same for friction

	// Convert normal to tangent
	fps_real sm;

	fps_vec3_num_mul(sn,i,sn);
	fps_vec3_sub(rv,sn,sn);
	fps_vec3_mag(sn,&sm);

	if (sm>0.001) {
		fps_vec3_unit(sn,sn);

		// Impulse
		fps_vec3_dot(rv,sn,&i);

		// Inverse inertia vector
		fps_vec3_cross(ar,sn,aiiv);
		fps_vec3_cross(br,sn,biiv);

		fps_mat3_vec3_mul(aiit,aiiv,aiiv);
		fps_mat3_vec3_mul(biit,biiv,biiv);

		// Moment of inertia
		fps_vec3_cross(aiiv,ar,ai);
		fps_vec3_cross(biiv,br,bi);
		fps_vec3_add(ai,bi,ci);

		// Angular effect
		fps_vec3_dot(ci,sn,&ae);

		// Full impulse
		j=(-f*i)/(t_im+ae);

		fps_vec3_num_mul(sn,j,fip);
		fps_vec3_num_mul(sn,-j,fin);

		// Angular full impulse
		fps_vec3_cross(ar,fin,aafi);
		fps_vec3_cross(br,fip,bafi);

		// Apply linear impulse
		fps_body_linear_impulse_apply(body_a,fin);
		fps_body_linear_impulse_apply(body_b,fip);

		// Apply angular impulse
		fps_body_angular_impulse_apply(body_a,aafi);
		fps_body_angular_impulse_apply(body_b,bafi);
	}
}

void fps_collision_test(
	fps_body *body_a,
	fps_body *body_b
) {
	if (!body_a->dynamic && !body_b->dynamic) return;

	if (!fps_aabb_test(
		body_a->boundary[0],
		body_a->boundary[1],
		body_b->boundary[0],
		body_b->boundary[1]
	)) return;

	fps_part *part_a = NULL;
	fps_part *part_b = NULL;

	fps_real (*va)[3] = NULL;
	fps_real (*vb)[3] = NULL;

	fps_body_part_get(body_a,&part_a);

	while (part_a!=NULL) {
		fps_body_part_get(body_b,&part_b);

		while (part_b!=NULL) {
			if (fps_aabb_test(
				part_a->boundary[0],
				part_a->boundary[1],
				part_b->boundary[0],
				part_b->boundary[1]
			)) {
				if (va==NULL) {
					va = fps_buffer_a;

					for (fps_uint i=0; i<part_a->vertex_count; i++) {
						fps_vec3_mul(
							part_a->vertexes[i],
							part_a->size,
							va[i]
						);
						fps_mat4_vec3_mul(
							part_a->transform,
							va[i],
							va[i]
						);
					}
				}

				vb = fps_buffer_b;

				for (fps_uint i=0; i<part_b->vertex_count; i++) {
					fps_vec3_mul(
						part_b->vertexes[i],
						part_b->size,
						vb[i]
					);
					fps_mat4_vec3_mul(
						part_b->transform,
						vb[i],
						vb[i]
					);
				}

				fps_real cp[3];
				fps_real sn[3];
				fps_real sd;

				fps_ngc_test_convex(
					va,
					vb,
					part_a->vertex_count,
					part_b->vertex_count,
					cp,
					sn,
					&sd
				);

				if (sd==0.0) {
					fps_ngc_test_convex(
						vb,
						va,
						part_b->vertex_count,
						part_a->vertex_count,
						cp,
						sn,
						&sd
					);

					fps_vec3_neg(sn,sn);
				}

				fps_collision_resolve(
					body_a,
					body_b,
					part_a,
					part_b,
					cp,
					sn,
					sd
				);
			}

			fps_part_next(&part_b);
		}

		fps_part_next(&part_a);
	}
}

#endif
