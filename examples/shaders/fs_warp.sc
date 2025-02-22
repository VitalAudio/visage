$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_time;
uniform vec4 u_center_position;
uniform vec4 u_atlas_scale;
uniform vec4 u_dimensions;

vec3 hsvToRgb(vec3 c) {
  vec4 k = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);
  vec3 p = abs(fract(c.xxx + k.xyz) * 6.0 - k.www);
  return c.z * mix(k.xxx, clamp(p - k.xxx, 0.0, 1.0), c.y);
}

vec4 hueRainbow(vec2 coordinates) {
  float hue = atan2(coordinates.y, coordinates.x) / (2.0 * 3.14159265) - u_time.x * 0.1;
  vec3 color = hsvToRgb(vec3(hue, 1.0, 1.0));
  return vec4(color, 1.0);
}

vec4 passthrough(vec2 coordinates) {
  return texture2D(s_texture, coordinates);
}

vec4 warp(vec2 texture_uv) {
  vec2 coordinates = ((texture_uv / u_atlas_scale.xy) - u_center_position.xy) / u_dimensions.xy;
  coordinates = coordinates + 0.01 * sin(u_time.x + coordinates * 20.0);
  texture_uv = ((coordinates * u_dimensions.xy) + u_center_position.xy) * u_atlas_scale.xy;
  vec4 color = texture2D(s_texture, texture_uv) * hueRainbow(coordinates);
  return color;
}

void main() {
  gl_FragColor = warp(v_texture_uv);

  // Uncomment to see raw shapes without post effect
  // gl_FragColor = passthrough(v_texture_uv);
}

