$input v_coordinates, v_dimensions, v_shader_values, v_shader_values1, v_position, v_gradient_pos, v_gradient_color_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);

void main() {
  vec2 gradient_pos = gradient(v_gradient_color_pos.xy, v_gradient_color_pos.zw, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  gl_FragColor = texture2D(s_gradient, gradient_pos);
  gl_FragColor.a = gl_FragColor.a * segment(v_coordinates, v_dimensions, v_shader_values.zw, v_shader_values1.xy, v_shader_values.x);
}
