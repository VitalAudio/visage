$input v_texture_uv, v_shader_values

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

void main() {
  gl_FragColor = v_shader_values.x * max(texture2D(s_texture, v_texture_uv), vec4_splat(0.0));
}
