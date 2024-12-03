$input a_position, a_texcoord0
$output v_coordinates

#include <bgfx_shader.sh>

uniform vec4 u_resample_values;

void main() {
  gl_Position = vec4(a_position.xy, 0.5, 1.0);
  v_coordinates = u_resample_values.xy * a_texcoord0.xy;
}
