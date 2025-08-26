$input v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_gradient_texture_position;
uniform vec4 u_radial_gradient;
uniform vec4 u_gradient_position;
uniform vec4 u_gradient_position2;
uniform vec4 u_color_mult;

SAMPLER2D(s_gradient, 0);

void main() {
  vec2 gradient_pos = gradient(u_radial_gradient.x != 0.0, u_gradient_texture_position, u_gradient_position, u_gradient_position2, v_position);
  gl_FragColor = u_color_mult * texture2D(s_gradient, gradient_pos);
  gl_FragColor.a = (v_shader_values.y + 1.0) * gl_FragColor.a;
}
