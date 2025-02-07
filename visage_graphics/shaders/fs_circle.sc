$input v_coordinates, v_dimensions, v_color0, v_color1, v_shader_values, v_position, v_gradient_pos

#include <shader_include.sh>

void main() {
  gl_FragColor = gradient(v_color0, v_color1, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  gl_FragColor.a = gl_FragColor.a * circle(v_coordinates, v_dimensions.x, v_shader_values.x, v_shader_values.y);
}
