$input a_position, a_texcoord0, a_texcoord1
$output v_shader_values, v_shader_values1, v_position

uniform vec4 u_bounds;

void main() {
  vec2 points[3];
  points[0] = a_position.zw;
  points[1] = a_texcoord0.xy;
  points[2] = a_texcoord0.zw;
  int index = int(a_position.x);
  vec2 position = points[index];

  v_shader_values.xy = a_texcoord0.xy;
  v_shader_values.zw = a_texcoord0.zw;
  v_shader_values1.xy = a_position.zw;
  v_shader_values1.zw = a_texcoord1.xy;
  v_position = a_texcoord1.zw;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
