$input v_coordinates, v_shader_values

#include <shader_include.sh>

SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

void main() {
  vec2 cell = floor((v_coordinates * 0.5 + vec2(0.5, 0.5)) * v_shader_values.zw);
  cell = min(cell, v_shader_values.zw - vec2(1.0, 1.0));
  float value = texture2DLod(s_texture, (v_shader_values.xy + cell) * u_atlas_scale.xy, 0.0).r;
  gl_FragColor = vec4(value, 0.0, 1.0 - value, 1.0);
}
