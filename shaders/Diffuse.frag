#version 330

// The camera's position in world-space
uniform vec3 u_cam_pos;

// Color
uniform vec4 u_color;

// Properties of the single point light
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// We also get the uniform texture we want to use.
uniform sampler2D u_texture_1;

// These are the inputs which are the outputs of the vertex shader.
in vec4 v_position;
in vec4 v_normal;

// This is where the final pixel color is output.
// Here, we are only interested in the first 3 dimensions (xyz).
// The 4th entry in this vector is for "alpha blending" which we
// do not require you to know about. For now, just set the alpha
// to 1.
out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float kd = 1.0;
  vec4 ll = vec4(u_light_pos, 1) - v_position;
  float r2 = pow(ll[0], 2) + pow(ll[1], 2) + pow(ll[2], 2) + pow(ll[3], 2);
			
  vec4 Ld = kd * (vec4(u_light_intensity, 1) / r2) * max(0.0, dot(v_normal, normalize(ll)));

  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color = Ld;
  out_color.a = 1;
}
