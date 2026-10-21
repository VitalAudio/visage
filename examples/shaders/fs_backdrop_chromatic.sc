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
  vec2 offset = (texel - u_center_position.xy) * u_amount.x;
  vec4 color = sampleTexel(texel);
  float red = sampleTexel(texel + offset).r;
  float blue = sampleTexel(texel - offset).b;
  gl_FragColor = vec4(red, color.g, blue, color.a);
}
