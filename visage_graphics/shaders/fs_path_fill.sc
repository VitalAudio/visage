$input v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_gradient_texture_position;
uniform vec4 u_gradient_position;
uniform vec4 u_gradient_position2;

SAMPLER2D(s_gradient, 0);

void main() {
  vec4 color = gradient(s_gradient, u_gradient_texture_position, u_gradient_position, u_gradient_position2, v_position);
  color.a = color.a * v_shader_values.x;
  gl_FragColor = color;
}
