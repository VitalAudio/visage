$input a_position, a_color0, a_color1, a_color2, a_texcoord0, a_texcoord1
$output v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

uniform vec4 u_bounds;

void main() {
  vec2 min = a_texcoord1.xy;
  vec2 max = a_texcoord1.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord0.xy, min, max);
  vec2 delta = clamped - a_position.xy;

  v_position = clamped;
  v_gradient_texture_pos = a_color0;
  v_gradient_pos = a_color1;
  v_gradient_pos2 = a_color2;
  gl_Position = vec4(clamped * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
