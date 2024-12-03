$input a_position, a_texcoord0, a_texcoord1, a_texcoord2, a_color0, a_color1
$output v_color0

#include <bgfx_shader.sh>

uniform vec4 u_color_mult;
uniform vec4 u_bounds;

void main() {
  vec2 min = a_texcoord1.xy;
  vec2 max = a_texcoord1.zw;
  vec2 clamped = clamp(a_position.xy + a_texcoord0.xy, min, max);
  vec2 delta = clamped - a_position.xy;

  gl_Position = vec4(clamped * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
  v_color0 = (a_color0 * a_color1.x) * u_color_mult.x;
  v_color0.a = a_color0.a;
}
