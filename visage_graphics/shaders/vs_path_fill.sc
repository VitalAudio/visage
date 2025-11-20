$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;

float implicitFunction(vec2 p, vec2 a, vec2 b) {
  vec2 ba = b - a;
  vec2 pa = p - a;
  return (ba.x * pa.y - ba.y * pa.x) / max(0.000001, length(ba));
}

void main() {
  v_shader_values.x = implicitFunction(a_position.xy, a_position.zw, a_texcoord0.xy);
  v_shader_values.y = implicitFunction(a_position.xy, a_position.zw, a_texcoord0.zw);
  v_shader_values1 = a_texcoord0;
  gl_Position = vec4(a_position.xy * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
