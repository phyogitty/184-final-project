#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  vec4 tex = texture(u_texture_2, uv);
  return tex.r;

  // return 0.0;
}

void main() {
  // YOUR CODE HERE
  float kh = u_height_scaling;
  float kn = u_normal_scaling;
  float w = u_texture_2_size[0];
  float h = u_texture_2_size[1];
  float u = v_uv[0];
  float v = v_uv[1];
  
  mat3 TBN = mat3(normalize(v_tangent.xyz), normalize(cross(v_normal.xyz, v_tangent.xyz)) , normalize(v_normal.xyz));
  float dU = kh * kn * (h(vec2(u + 1.0 / w, v)) - h(vec2(u, v)));
  float dV = kh * kn * (h(vec2(u, v + 1.0 / h)) - h(vec2(u, v)));
  vec3 no = normalize(vec3(-dU, -dV, 1));
  vec3 nd = normalize(TBN * no);

  // Phong
  float ka = 0.3, kd = .8, ks = 0.5;
  float p = 20.0;
  vec3 Ia = vec3(0.05, 0.05, 0.05);

  vec3 ll = u_light_pos - v_position.xyz;
  vec3 hh = normalize(ll + u_cam_pos - v_position.xyz);
  float r2 = pow(ll.x, 2) + pow(ll.y, 2) + pow(ll.z, 2);

  vec3 La = ka * Ia;
  vec3 Ld = kd * (u_light_intensity / r2) * max(0.0, dot(nd, normalize(ll)));
  vec3 Ls = ks * (u_light_intensity / r2) * pow(max(0.0, dot(nd, hh)), p);
  vec3 L = La + Ld + Ls;
  
  out_color = vec4(L, 1);

  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color.a = 1;
}

