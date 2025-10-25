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
  float resolution = v_shader_values.y;
  vec2 start_data = v_shader_values.zw * u_atlas_scale.xy;
  float data_width = resolution * u_atlas_scale.x;

  vec2 percent = v_coordinates * 0.5 + vec2(0.5, 0.5);
  vec2 pos = percent * v_dimensions;
  float convert = resolution / v_dimensions.x;
  float range = thickness * convert;
  float range_start = floor(percent.x * resolution - range);
  float range_end = ceil(percent.x * resolution + range);

  float last_x = range_start * v_dimensions.x / resolution;
  float last_y = v_dimensions.y * texture2D(s_texture, start_data + vec2(clamp(u_atlas_scale.x * range_start, 0.0, data_width), 0.0)).r;

  float distance = (thickness + 1.0) * (thickness + 1.0);
  // TODO: Could define the range at compile time
  float span = min(20.0, max(1.0, range_end - range_start));
  for (float i = 0.0; i <= span; ++i) {
    float sample_index = range_start + i;
    float x = sample_index / convert;
    float y = v_dimensions.y * texture2D(s_texture, start_data + vec2(clamp(u_atlas_scale.x * sample_index, 0.0, data_width), 0.0)).r;
    distance = min(distance, distanceSquared(pos, vec2(last_x, last_y), vec2(x, y)));
    last_x = x;
    last_y = y;
  }

  float alpha = 1.0 - smoothed(-0.5, 0.5, sqrt(distance) - thickness);
  gl_FragColor.a = gl_FragColor.a * alpha;
}
