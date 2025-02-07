$input a_position, a_texcoord0, a_texcoord1, a_texcoord2, a_color0, a_color1, a_color2
$output v_color0, v_color1, v_position, v_gradient_pos

#include <shader_include.sh>

uniform vec4 u_color_mult;
uniform vec4 u_bounds;

void main() {
  vec2 min = a_texcoord2.xy;
  vec2 max = a_texcoord2.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord1.xy, min, max);
  vec2 delta = clamped - a_position.xy;

  v_position = a_position.xy;
  v_gradient_pos = a_texcoord0;
  gl_Position = vec4(clamped * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
  v_color0 = (a_color0 * a_color2.x) * u_color_mult.x;
  v_color0.a = a_color0.a;
  v_color1 = (a_color1 * a_color2.x) * u_color_mult.x;
  v_color1.a = a_color1.a;
}
