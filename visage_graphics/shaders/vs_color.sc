$input a_position, a_texcoord0, a_texcoord1, a_texcoord2, a_texcoord3
$output v_position, v_gradient_pos, v_gradient_color_pos

#include <shader_include.sh>

uniform vec4 u_color_mult;
uniform vec4 u_bounds;

void main() {
  vec2 min = a_texcoord3.xy;
  vec2 max = a_texcoord3.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord2.xy, min, max);
  vec2 delta = clamped - a_position.xy;

  v_position = a_position.xy;
  v_gradient_color_pos = a_texcoord0;
  v_gradient_pos = a_texcoord1;
  gl_Position = vec4(clamped * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
