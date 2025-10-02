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
  float center_y = v_shader_values.x * v_dimensions.y;
  float thickness = 1.0;
  float resolution = v_shader_values.y;
  vec2 start_data = v_shader_values.zw * u_atlas_scale.xy;

  vec2 percent = v_coordinates * 0.5 + vec2(0.5, 0.5);
  vec2 pos = percent * v_dimensions;

  vec2 sample_pos = start_data + vec2(u_atlas_scale.x * percent.x * resolution, 0.0);
  float line_y1 = v_dimensions.y * texture2D(s_texture, sample_pos + vec2(-0.0001, 0.0)).r;
  float line_y2 = v_dimensions.y * texture2D(s_texture, sample_pos + vec2(0.0001, 0.0)).r;
  float line_y = 0.5 * (line_y1 + line_y2);
  float adjust = abs(line_y1 - line_y2) / (v_dimensions.x * 0.0002);
  float fade = sqrt(1.0 + adjust * adjust);

  float mult = sign(pos.y - center_y);
  float alpha = 1.0 - smoothed(-0.5 * fade, 0.5 * fade, mult * (pos.y - line_y)) * smoothed(-0.5, 0.5, mult * (pos.y - center_y));
  gl_FragColor.a = gl_FragColor.a * alpha;
}
