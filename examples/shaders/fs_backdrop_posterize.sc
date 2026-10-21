$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

void main() {
  vec4 color = sampleTexel(v_texture_uv / u_atlas_scale.xy);
  float levels = max(u_amount.x, 2.0) - 1.0;
  vec3 straight = color.rgb / max(color.a, 0.0001);
  vec3 stepped = floor(straight * levels + 0.5) / levels;
  gl_FragColor = vec4(stepped * color.a, color.a);
}
