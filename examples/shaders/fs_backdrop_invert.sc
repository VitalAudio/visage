$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_texture_clamp;

void main() {
  vec4 color = texture2D(s_texture, clamp(v_texture_uv, u_texture_clamp.xy, u_texture_clamp.zw));
  gl_FragColor = vec4(vec3(color.a, color.a, color.a) - color.rgb, color.a);
}
