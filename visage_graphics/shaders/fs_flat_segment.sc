$input v_coordinates, v_dimensions, v_color0, v_shader_values, v_shader_values1

#include <shader_utils.sh>

uniform vec4 u_origin_flip;

void main() {
  gl_FragColor = v_color0;
  vec2 flip_mult = vec2(1.0, u_origin_flip.x);
  gl_FragColor.a = v_color0.a * flatSegment(v_coordinates, v_dimensions, v_shader_values.zw * flip_mult, v_shader_values1.xy * flip_mult, v_shader_values.x);
}
