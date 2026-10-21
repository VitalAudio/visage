$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

float random(vec2 position) {
  return fract(sin(dot(position, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
  vec2 texel = v_texture_uv / u_atlas_scale.xy;
  vec2 cell = floor(texel);
  vec2 jitter = vec2(random(cell), random(cell + vec2(17.0, 31.0))) * 2.0 - 1.0;
  gl_FragColor = sampleTexel(texel + jitter * u_amount.x);
}
