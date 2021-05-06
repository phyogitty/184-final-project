#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float ka = 0.3, kd = .8, ks = 0.5;
  float p = 20.0;
  vec4 Ia = vec4(0.5, 0.5, 0.5, 1);

  vec4 ll = vec4(u_light_pos, 1) - v_position;
  vec4 h = normalize(ll + (vec4(u_cam_pos, 1) - v_position));
  float r2 = pow(ll[0], 2) + pow(ll[1], 2) + pow(ll[2], 2) + pow(ll[3], 2);

  vec4 La = ka * Ia;
  vec4 Ld = kd * (vec4(u_light_intensity, 1) / r2) * max(0.0, dot(v_normal, normalize(ll)));
  vec4 Ls = ks * (vec4(u_light_intensity, 1) / r2) * pow(max(0.0, dot(v_normal, h)), p);
  vec4 L = La + Ld + Ls;

  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  out_color = L;
  out_color.a = 1;
}

