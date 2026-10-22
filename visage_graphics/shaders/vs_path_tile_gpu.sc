$output v_shader_values, v_shader_values1

#include <shader_include.sh>

SAMPLER2D(s_texture2, 1);

uniform vec4 u_bounds;
uniform vec4 u_dimensions;

void main() {
  int width = int(u_dimensions.x + 0.5);
  int vertex = int(gl_VertexID);
  int tile = vertex / 6;
  int corner = vertex - tile * 6;
  int index = tile * 3;
  vec4 record = texelFetch(s_texture2, ivec2(index % width, index / width), 0);
  vec4 rect = texelFetch(s_texture2, ivec2((index + 1) % width, (index + 1) / width), 0);
  vec4 rule = texelFetch(s_texture2, ivec2((index + 2) % width, (index + 2) / width), 0);
  float right = corner == 1 || corner == 3 || corner == 4 ? 1.0 : 0.0;
  float bottom = corner == 2 || corner == 4 || corner == 5 ? 1.0 : 0.0;
  vec2 position = mix(rect.xy, rect.zw, vec2(right, bottom));

  v_shader_values = record;
  v_shader_values1 = vec4(rect.xy, rule.x, 0.0);
  gl_Position = vec4(position * u_bounds.xy + u_bounds.zw, 0.5, 1.0);
}
