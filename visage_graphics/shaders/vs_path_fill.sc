$input a_position, a_texcoord0, a_texcoord1
$output v_shader_values, v_shader_values1, v_position

uniform vec4 u_bounds;

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
  float half_turn_cos = dot(point_normal, prev_normal);
  if (half_turn_cos > 0.966)
    offset = point_normal / half_turn_cos;
  vec2 position = p + 0.71 * offset;

  v_shader_values.xy = a_texcoord0.xy;
  v_shader_values.zw = a_texcoord0.zw;
  v_shader_values1.xy = a_position.zw;
  v_shader_values1.zw = a_texcoord1.xy;
  v_position = a_texcoord1.zw;
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
