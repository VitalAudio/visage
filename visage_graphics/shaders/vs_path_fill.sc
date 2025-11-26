$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;
uniform vec4 u_origin_flip;

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
  int index = int(a_position.x);
  vec2 p, next, prev;

  if (index == 0) {
    p = a_position.zw;
    next = a_texcoord0.xy;
    prev = a_texcoord0.zw;
  }
  else if (index == 1) {
    p = a_texcoord0.xy;
    next = a_texcoord0.zw;
    prev = a_position.zw;
  }
  else {
    p = a_texcoord0.zw;
    next = a_position.zw;
    prev = a_texcoord0.xy;
  }

  vec2 prev_tangent = normalize(p - prev);
  vec2 next_tangent = normalize(next - p);
  vec2 tangent_diff = prev_tangent - next_tangent;
  vec2 point_normal = normalize(tangent_diff);
  vec2 prev_normal = vec2(prev_tangent.y, -prev_tangent.x);
  prev_normal = dot(prev_normal, next_tangent) > 0.0 ? -prev_normal : prev_normal;
  vec2 square_normal = normalize(prev_normal + point_normal);

  vec2 extended = prev_normal + dot(prev_tangent, square_normal) * prev_tangent;
  float blend = a_position.y;
  vec2 offset = point_normal + blend * (extended - point_normal);
  vec2 position = p + 0.51 * offset;

  v_shader_values.x = lineFunctionScaled(position, a_position.zw, a_texcoord0.xy);
  v_shader_values.y = lineFunctionScaled(position, a_position.zw, a_texcoord0.zw);
  v_shader_values.z = lineFunctionScaled(position, a_texcoord0.zw, a_texcoord0.xy) * u_origin_flip.x;
  vec2 delta = a_texcoord0.zw - a_texcoord0.xy;
  v_shader_values.w = lineFunction(position, a_texcoord0.xy, a_texcoord0.xy + vec2(delta.y, -delta.x));
  v_shader_values1.xy = a_position.zw;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
