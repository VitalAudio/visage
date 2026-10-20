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

  float index = percent.x * resolution;
  float index0 = floor(index);
  float value0 = texture2D(s_texture, start_data + vec2(u_atlas_scale.x * index0, 0.0)).r;
  float value1 = texture2D(s_texture, start_data + vec2(u_atlas_scale.x * min(index0 + 1.0, resolution), 0.0)).r;
  float line_y = v_dimensions.y * mix(value0, value1, index - index0);
  float adjust = v_dimensions.y * abs(value1 - value0) / (u_atlas_scale.x * v_dimensions.x);
  float fade = sqrt(1.0 + adjust * adjust);

  float mult = sign(pos.y - center_y);
  float alpha = 1.0 - smoothed(-0.5 * fade, 0.5 * fade, mult * (pos.y - line_y)) * smoothed(-0.5, 0.5, mult * (pos.y - center_y));
  gl_FragColor = clampToAlpha(gl_FragColor * alpha);
}
