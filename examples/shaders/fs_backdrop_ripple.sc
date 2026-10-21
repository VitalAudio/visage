$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_time;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;
uniform vec4 u_center_position;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

void main() {
  vec2 texel = v_texture_uv / u_atlas_scale.xy;
  vec2 delta = texel - u_center_position.xy;
  float distance = length(delta);
  float wave = sin(distance * 0.08 - u_time.x * 4.0);
  vec2 direction = delta / max(distance, 0.0001);
  gl_FragColor = sampleTexel(texel + direction * wave * u_amount.x);
}
