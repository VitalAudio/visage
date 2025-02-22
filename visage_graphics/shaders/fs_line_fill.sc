$input v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_gradient_color_position;
uniform vec4 u_gradient_position;
uniform vec4 u_time;

SAMPLER2D(s_gradient, 0);

void main() {
  vec2 gradient_pos = gradient(u_gradient_color_position.xy, u_gradient_color_position.zw, u_gradient_position.xy, u_gradient_position.zw, v_position);
  gl_FragColor = texture2D(s_gradient, gradient_pos);
  gl_FragColor.a = (v_shader_values.y + 1.0) * gl_FragColor.a;
}
