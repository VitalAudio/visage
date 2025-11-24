$input a_position, a_texcoord0
$output v_shader_values, v_shader_values1

uniform vec4 u_bounds;

float lineFunction(vec2 p, vec2 a, vec2 ba, float ba_inv_len) {
  vec2 pa = p - a;
  return (ba.x * pa.y - ba.y * pa.x) * ba_inv_len;
}

void main() {
  int index = int(a_position.x);
  vec2 p, next, prev;

  if (index == 0) {
    p = a_position.zw;
    next = a_texcoord0.xy;
    prev = a_texcoord0.zw;
  } else if (index == 1) {
    p = a_texcoord0.xy;
    next = a_texcoord0.zw;
    prev = a_position.zw;
  } else {
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

  vec2 edge1 = a_texcoord0.xy - a_position.zw;
  vec2 edge2 = a_texcoord0.zw - a_position.zw;
  float edge1_inv_len = 1.0 / max(0.000001, length(edge1));
  float edge2_inv_len = 1.0 / max(0.000001, length(edge2));

  v_shader_values.x = lineFunction(position, a_position.zw, edge1, edge1_inv_len);
  v_shader_values.y = lineFunction(position, a_position.zw, edge2, edge2_inv_len);
  v_shader_values1 = a_texcoord0;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
