$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

float distanceSquared(vec2 position, vec2 point1, vec2 point2) {
  vec2 position_delta = position - point1;
  vec2 line_delta = point2 - point1;
  float t = clamp(dot(position_delta, line_delta) / dot(line_delta, line_delta), 0.0, 1.0);
  vec2 delta = position_delta - line_delta * t;
  delta = delta * delta;
  return delta.x + delta.y;
}

void main() {
  gl_FragColor = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  float thickness = 0.5 * v_shader_values.x;
  int last_index = int(v_shader_values.y + 0.5);
  vec2 start_data = v_shader_values.zw * u_atlas_scale.xy;

  vec2 dimensions = v_dimensions - vec2(1.0, 1.0);
  vec2 pos = (v_coordinates * 0.5 + vec2(0.5, 0.5)) * v_dimensions - vec2(0.5, 0.5);
  float segments = float(max(last_index, 1));
  float convert = segments / dimensions.x;
  float range = thickness + 0.5;
  int range_start = int(clamp(floor((pos.x - range) * convert), 0.0, segments));
  int range_end = int(clamp(ceil((pos.x + range) * convert), 0.0, segments));

  float last_x = float(range_start) / convert;
  float last_y = dimensions.y * texture2DLod(s_texture, start_data + vec2(u_atlas_scale.x * float(min(range_start, last_index)), 0.0), 0.0).r;

  float distance = (range + 1.0) * (range + 1.0);
  for (int i = range_start + 1; i <= range_end; ++i) {
    float x = float(i) / convert;
    float y = dimensions.y * texture2DLod(s_texture, start_data + vec2(u_atlas_scale.x * float(min(i, last_index)), 0.0), 0.0).r;
    distance = min(distance, distanceSquared(pos, vec2(last_x, last_y), vec2(x, y)));
    last_x = x;
    last_y = y;
  }

  float center_distance = sqrt(distance);
  float alpha = clamp(min(center_distance + 0.5, thickness) - max(center_distance - 0.5, -thickness), 0.0, 1.0);
  gl_FragColor = clampToAlpha(gl_FragColor * alpha);
}
