#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec4 wo = normalize(vec4(u_cam_pos, 1) - v_position);
  vec4 wi = normalize(v_normal - wo);

  out_color = texture(u_texture_cubemap, vec3(wi));

  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}
