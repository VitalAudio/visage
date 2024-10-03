$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_utils.sh>

void main() {
  gl_FragColor = v_color0;
  gl_FragColor.a = gl_FragColor.a * ring(v_coordinates, v_dimensions.x, v_shader_values.x);
}
