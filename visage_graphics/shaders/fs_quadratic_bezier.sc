$input v_coordinates, v_dimensions, v_color0, v_shader_values, v_shader_values1

#include <shader_include.sh>

void main() {
  gl_FragColor = v_color0;
  gl_FragColor.a = v_color0.a * quadraticBezier(v_coordinates, v_dimensions, v_shader_values.zw, v_shader_values1.xy, v_shader_values1.zw, v_shader_values.x);
}
