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

// this function returns the height of the object at texture coordinates (u,v)
float h(vec2 uv) {
  return texture(u_texture_2, uv)[0];
}

void main() {
  // BUMP MAPPING
  vec3 t = normalize(vec3(v_tangent));
  vec3 n = normalize(vec3(v_normal));
  vec3 b = normalize(cross(n, t));
  mat3 TBN = mat3(t, b, n);  // This matrix wil transform a vector from object space to model space.

  // Finding rate of change in height at (u, v) on heightmap
  float dU = (h(v_uv + vec2(1/u_texture_2_size[1], 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dV = (h(v_uv + vec2(0.0, 1/u_texture_2_size[0])) - h(v_uv)) * u_height_scaling * u_normal_scaling;

  // Local space normal is displaced by the rate of change of height at that point then converted to model space
  vec3 normal = normalize(vec3(-dU, -dV, 1.0));
  vec4 displacedModelSpaceNormal = vec4(TBN * normal, 1.0);

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
  float diffuse_scale = max(0, dot(displacedModelSpaceNormal, normalize(light_pos - v_position)));
  out_color += k_d * illumination * diffuse_scale;

  // BLINN-PHONG SHADING
  vec4 h = (cam_pos - v_position) + point_to_light;   // vector halfway between the light and camera
  float blinnPhong_scale = pow(max(0, dot(displacedModelSpaceNormal, normalize(h))), p);
  out_color += k_s * illumination * blinnPhong_scale;
  out_color.a = 1;
}
