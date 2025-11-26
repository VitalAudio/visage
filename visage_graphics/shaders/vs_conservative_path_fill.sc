$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;

float lineFunctionDeltas(vec2 pa, vec2 ba) {
  return (ba.x * pa.y - ba.y * pa.x) / dot(ba, ba);
}

float lineFunction(vec2 p, vec2 a, vec2 b) {
  vec2 pa = p - a;
  vec2 ba = b - a;
  return lineFunctionDeltas(pa, ba);
}

float lineFunctionScaled(vec2 p, vec2 a, vec2 b) {
  vec2 pa = p - a;
  vec2 ba = b - a;
  return lineFunctionDeltas(pa, ba) * length(ba);
}

void main() {
  vec2 points[3];
  points[0] = a_position.zw;
  points[1] = a_texcoord0.xy;
  points[2] = a_texcoord0.zw;
  int index = int(a_position.x);
  vec2 position = points[index];

  v_shader_values.x = lineFunctionScaled(position, a_position.zw, a_texcoord0.xy);
  v_shader_values.y = lineFunctionScaled(position, a_position.zw, a_texcoord0.zw);
  v_shader_values.z = lineFunctionScaled(position, a_texcoord0.zw, a_texcoord0.xy);
  vec2 delta = a_texcoord0.zw - a_texcoord0.xy;
  v_shader_values.w = lineFunction(position, a_texcoord0.xy, a_texcoord0.xy + vec2(delta.y, -delta.x));
  v_shader_values1.xy = a_position.zw;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
