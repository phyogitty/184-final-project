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
  vec4 light_pos = vec4(u_light_pos, 1);
  vec4 light_intensity = vec4(u_light_intensity, 1);
  // The illumination falling on a point from a point-light source is equal to the intensity of the light divided by
  // the distance from the point to the light squared
  vec4 illumination = light_intensity / (distance(v_position, light_pos) * distance(v_position, light_pos));
  // The intensity of the light coming from a point on a diffuse surface depends on the
  // angle between the point and the light source. i.e.
  float scale = max(0, dot(normalize(v_normal), normalize(light_pos - v_position)));
  out_color = u_color * illumination * scale;
  out_color.a = 1;
}
