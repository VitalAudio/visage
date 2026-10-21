$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;
uniform vec4 u_center_position;
uniform vec4 u_dimensions;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

void main() {
  vec2 texel = v_texture_uv / u_atlas_scale.xy;
  vec2 delta = texel - u_center_position.xy;
  vec2 normalized = delta / max(0.5 * u_dimensions.xy, vec2(1.0, 1.0));
  float radius = length(normalized);
  float factor = mix(1.0 / max(u_amount.x, 1.0), 1.0, smoothstep(0.0, 1.0, radius * radius));
  gl_FragColor = sampleTexel(u_center_position.xy + delta * factor);
}
