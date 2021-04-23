#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
  vec4 tex = texture(u_texture_2, uv);
  return tex.r;

  // return 0.0;
}

void main() {
  // YOUR CODE HERE
  float kh = u_height_scaling;
  float u = in_uv[0];
  float v = in_uv[1];

  v_position = in_position + in_normal * h(vec2(u, v)) * kh;

  // (Placeholder code. You will want to replace it.)
  // v_position = u_model * in_position;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);
  gl_Position = u_view_projection * u_model * v_position;
}