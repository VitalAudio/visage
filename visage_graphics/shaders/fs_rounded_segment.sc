$input v_coordinates, v_dimensions, v_color0, v_shader_values, v_shader_values1

#include <shader_utils.sh>

void main() {
  gl_FragColor = v_color0;
  gl_FragColor.a = v_color0.a * segment(v_coordinates, v_dimensions, v_shader_values.xy, v_shader_values.zw, v_shader_values1.x);
}
