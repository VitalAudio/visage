$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;
uniform vec4 u_center_position;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

void main() {
  vec2 texel = v_texture_uv / u_atlas_scale.xy;
  float size = max(u_amount.x, 1.0);
  vec2 snapped = (floor((texel - u_center_position.xy) / size) + 0.5) * size + u_center_position.xy;
  gl_FragColor = sampleTexel(snapped);
}
