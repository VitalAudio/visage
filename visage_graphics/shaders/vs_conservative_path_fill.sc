$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;

float implicitFunction(vec2 p, vec2 a, vec2 b) {
  vec2 ba = b - a;
  vec2 pa = p - a;
  return (ba.x * pa.y - ba.y * pa.x) / max(0.000001, length(ba));
}

void main() {
  vec2 points[3];
  points[0] = a_position.zw;
  points[1] = a_texcoord0.xy;
  points[2] = a_texcoord0.zw;
  int index = a_position.x;
  vec2 position = points[index];

  v_shader_values.x = implicitFunction(position, a_position.zw, a_texcoord0.xy);
  v_shader_values.y = implicitFunction(position, a_position.zw, a_texcoord0.zw);
  v_shader_values1 = a_texcoord0;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
