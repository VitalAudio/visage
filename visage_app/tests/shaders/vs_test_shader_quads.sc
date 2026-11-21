$input a_position, a_texcoord0, a_texcoord1, a_texcoord2
$output v_coordinates, v_shader_values

#include <shader_include.sh>

uniform vec4 u_bounds;

void main() {
  vec2 corner = a_texcoord0.xy * 0.5 + vec2(0.5, 0.5);
  vec2 dimensions = a_texcoord0.zw;
  vec2 origin = a_position.xy - corner * dimensions;
  float quad = a_position.z;
  vec2 position = origin + dimensions * vec2((quad + corner.x) * 0.25,
                                             mix(0.75 - quad * 0.25, 1.0, corner.y));
  vec2 clamped = clamp(position, a_texcoord1.xy, a_texcoord1.zw);

  v_coordinates = (clamped - origin) / dimensions * 2.0 - vec2(1.0, 1.0);
  v_shader_values = a_texcoord2;
  gl_Position = vec4(clamped * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
