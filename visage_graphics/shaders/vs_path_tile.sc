$input a_position, a_texcoord0, a_texcoord1
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;

void main() {
  v_shader_values = a_texcoord0;
  v_shader_values1 = vec4(a_position.zw, a_texcoord1.x, 0.0);
  gl_Position = vec4(a_position.xy * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
