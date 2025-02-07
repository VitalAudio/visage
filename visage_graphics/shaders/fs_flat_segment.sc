$input v_coordinates, v_dimensions, v_color0, v_color1, v_shader_values, v_shader_values1, v_position, v_gradient_pos

#include <shader_include.sh>

uniform vec4 u_origin_flip;

void main() {
  gl_FragColor = gradient(v_color0, v_color1, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  vec2 flip_mult = vec2(1.0, u_origin_flip.x);
  gl_FragColor.a = gl_FragColor.a * flatSegment(v_coordinates, v_dimensions, v_shader_values.zw * flip_mult, v_shader_values1.xy * flip_mult, v_shader_values.x);
}
