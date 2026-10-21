$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

void main() {
  vec4 color = sampleTexel(v_texture_uv / u_atlas_scale.xy);
  gl_FragColor = vec4(vec3(color.a, color.a, color.a) - color.rgb, color.a);
}
