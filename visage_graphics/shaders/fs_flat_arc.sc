$input v_coordinates, v_dimensions, v_color0, v_color1, v_shader_values, v_shader_values1, v_position, v_gradient_pos

#include <shader_include.sh>

void main() {
  gl_FragColor = gradient(v_color0, v_color1, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  gl_FragColor.a = gl_FragColor.a * flatArc(v_coordinates, v_shader_values1.xy, v_shader_values1.zw, v_dimensions.x, v_shader_values.x);
}
