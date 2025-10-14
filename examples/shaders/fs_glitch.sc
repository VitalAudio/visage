$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_time;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;
uniform vec4 u_dimensions;

float random(vec2 uv) {
  return fract(sin(dot(uv, vec2(12.9898, 78.233))) * 43758.5453);
}

vec3 glitch1(vec2 value) {
  float random_size = random(floor(value * vec2(5.0, 0.1)));
  float random_off = random(floor(value * vec2(25.0 * random_size + 5.0, 3.0)));
  float offset = random(floor(value * vec2(0.2 * mod(random_off, 1.0), random_off)));
  offset = floor(offset * 1.05);

  float random_color_off1 = random(floor(value * vec2(15.0, 5.0)));
  float random_color_off2 = random(floor(value * vec2(17.0, 5.0)));
  float random_color_off3 = random(floor(value * vec2(19.0, 5.0)));
  return vec3(offset, offset, offset) * 0.018 * vec3(random_color_off1, random_color_off2, random_color_off3);
}

float noise(vec2 uv) {
  vec2 i = floor(uv);
  vec2 f = fract(uv);
  float a = random(i);
  float b = random(i + vec2(1.0, 0.0));
  float c = random(i + vec2(0.0, 1.0));
  float d = random(i + vec2(1.0, 1.0));
  vec2 u = f * f * (3.0 - 2.0 * f);
  return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
}

vec3 glitch2(vec2 value, float time) {
  float r = pow((noise(value * vec2(0.0, 3.0) + time * 0.250100001 + 5.0) - 0.5) * 0.8, 5.0);
  float g = pow((noise(value * vec2(0.0, 3.0) + time * 0.2509999888 + 4.0) - 0.5) * 0.8, 5.0);
  float b = pow((noise(value * vec2(0.0, 3.0) + time * 0.25) - 0.5) * 0.8, 5.0);
  return vec3(r, g, b);
}

void main() {
  vec3 g = glitch1(vec2(v_texture_uv.y, u_time.x + 150.9)) + glitch2(v_texture_uv, u_time.x);
  gl_FragColor.r = texture2D(s_texture, clamp(v_texture_uv + vec2(g.r, 0.0), u_texture_clamp.xy, u_texture_clamp.zw)).r;
  gl_FragColor.g = texture2D(s_texture, clamp(v_texture_uv + vec2(g.b, 0.0), u_texture_clamp.xy, u_texture_clamp.zw)).g;
  gl_FragColor.b = texture2D(s_texture, clamp(v_texture_uv + vec2(g.g, 0.0), u_texture_clamp.xy, u_texture_clamp.zw)).b;
  gl_FragColor.rgb = gl_FragColor.rgb * u_color_mult.rgb;
  gl_FragColor.a = 1.0;
}

