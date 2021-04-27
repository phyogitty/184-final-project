#version 330


uniform vec3 u_cam_pos;

uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;

out vec4 out_color;

void main() {
  // similar to ray tracing, we find the unit direction vector from the vertex to the camera
  vec4 w_out = normalize(v_position - vec4(u_cam_pos, 0.0));
  // then we calculate the direction which light has to originally come from to reflect at that vertex to our camera
  vec4 w_in = normalize(w_out - (2 * dot(w_out, v_normal) * v_normal));
  out_color = texture(u_texture_cubemap, vec3(w_in));
  out_color.a = 1;
}
