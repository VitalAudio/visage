$input v_shader_values

#include <shader_utils.sh>

uniform vec4 u_time;
uniform vec4 u_color_mult;

uniform vec4 u_top_left_color;
uniform vec4 u_top_right_color;
uniform vec4 u_bottom_left_color;
uniform vec4 u_bottom_right_color;

void main() {
  float x_t = v_shader_values.z;
  float y_t = v_shader_values.w;
  vec4 top_color = u_top_left_color + (u_top_right_color - u_top_left_color) * x_t;
  vec4 bottom_color = u_bottom_left_color + (u_bottom_right_color - u_bottom_left_color) * x_t;
  gl_FragColor = bottom_color + (top_color - bottom_color) * y_t;
  gl_FragColor.rgb = gl_FragColor.rgb * u_color_mult.x;
  gl_FragColor.a = (v_shader_values.y + 1.0) * gl_FragColor.a;
}
