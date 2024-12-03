$input a_position, a_texcoord0
$output v_coordinates

#include <bgfx_shader.sh>

void main() {
  gl_Position = vec4(a_position.xy, 0.5, 1.0);
  v_coordinates = a_texcoord0.xy;
}
