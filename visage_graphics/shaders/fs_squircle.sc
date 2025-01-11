$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_include.sh>

void main() {
  gl_FragColor = v_color0;
  gl_FragColor.a = v_color0.a * squircle(v_coordinates, v_dimensions, v_shader_values.z, v_shader_values.x, v_shader_values.y);
}
