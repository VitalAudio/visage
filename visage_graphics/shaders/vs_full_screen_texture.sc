$input a_position
$output v_coordinates

#include <shader_include.sh>

void main() {
  gl_Position = vec4(a_position.xy, 0.5, 1.0);
  v_coordinates = a_position.zw;
}
