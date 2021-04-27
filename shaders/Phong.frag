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
  // parameters/coefficients governing light reflection
  vec4 i_a = vec4(1.0);
  vec4 k_a = vec4(0.1);
  vec4 k_d = u_color;
  vec4 k_s = vec4(0.5);
  int p = 100;

  vec4 cam_pos = vec4(u_cam_pos, 1.0);
  vec4 light_pos = vec4(u_light_pos, 1);
  vec4 point_to_light = normalize(light_pos - v_position);
  vec4 light_intensity = (vec4(u_light_intensity, 1));
  vec4 illumination = light_intensity / (distance(v_position, light_pos) * distance(v_position, light_pos));

  out_color = k_a * i_a;

  // DIFFUSE SHADING
  float diffuse_scale = max(0, dot(normalize(v_normal), normalize(light_pos - v_position)));
  out_color += k_d * illumination * diffuse_scale;

  // BLINN-PHONG SHADING
  vec4 h = (cam_pos - v_position) + point_to_light;   // vector halfway between the light and camera
  float blinnPhong_scale = pow(max(0, dot(normalize(v_normal), normalize(h))), p);
  out_color += k_s * illumination * blinnPhong_scale;
  out_color.a = 1;
}

