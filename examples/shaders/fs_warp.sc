$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_utils.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_time;

vec3 hsvToRgb(vec3 c) {
  vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);
  vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
  return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

vec4 hueRainbow(vec2 coordinates) {
  float hue = atan2(coordinates.y, coordinates.x) / (2.0 * 3.14159265) - u_time.x * 0.1;

  vec3 color = hsvToRgb(vec3(hue, 1.0, 1.0));
  return vec4(color, 1.0);
}

vec4 passthrough(vec2 coordinates) {
  return texture2D(s_texture, coordinates);
}

vec4 warp(vec2 coordinates) {
  coordinates = coordinates + 0.01 * sin(u_time.x + coordinates * 20.0);
  vec4 color = texture2D(s_texture, coordinates)  * hueRainbow(coordinates + vec2(-0.5, -0.5));
  return color;
}

void main() {
  gl_FragColor = warp(v_coordinates);
  // gl_FragColor = passthrough(v_coordinates);
}
