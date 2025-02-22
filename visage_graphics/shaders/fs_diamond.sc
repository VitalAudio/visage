$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_color_pos

#include <shader_include.sh>

uniform vec4 u_color_mult;

SAMPLER2D(s_gradient, 0);

void main() {
  vec2 gradient_pos = gradient(v_gradient_color_pos.xy, v_gradient_color_pos.zw, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  gl_FragColor = u_color_mult * texture2D(s_gradient, gradient_pos);
  gl_FragColor.a = gl_FragColor.a * roundedDiamond(v_coordinates, v_dimensions, 2.0 * v_shader_values.z, v_shader_values.x, v_shader_values.y);
}
