$input v_coordinates, v_dimensions, v_color0, v_color1, v_color2, v_shader_values

#include <shader_include.sh>

uniform vec4 u_time;

float shape(vec2 position, vec2 dimensions, float fade_mult, float hover_amount) {
  const float kRoundingMult = 0.8;

  float rounding = (hover_amount * kRoundingMult) * dimensions.y;
  return roundedRectangle(position, dimensions, rounding, dimensions.x + dimensions.y, fade_mult);
}

vec2 sphereWarp(vec2 coordinates, float hover_amount) {
  const float kShapeXSquish = 0.95;

  float y2 = coordinates.y * coordinates.y;
  float z2 = 1.0 - coordinates.x * coordinates.x - y2;
  float projected = sqrt(y2 + z2);
  float squish = kShapeXSquish + 0.13 * hover_amount;
  return vec2(coordinates.x * squish, acos(coordinates.y / projected));
}

void main() {
  const float kCycles = 2.5;
  const float kDimRatio = 0.65;
  const float kSpread = 0.66;
  const float kRingWidth = 0.00;
  
  float raw_value = v_shader_values.x;
  float hover_amount = v_shader_values.y;
  float thickness = v_shader_values.z;

  float max_arc = v_shader_values.w;

  float unipolar = floor(raw_value * 0.5);
  float current_value = raw_value - 2.0 * unipolar;
  float value = current_value * max_arc * 2.0 - max_arc;
  
  float dim_ratio = kDimRatio + 0.06 * hover_amount;
  float min_dimension = min(v_dimensions.x, v_dimensions.y);
  float base_width = min_dimension * kDimRatio;
  float width = min_dimension * dim_ratio;
  vec2 adjusted_coordinates = v_coordinates * v_dimensions / width;

  float time_position = -5.0 * current_value;
  vec2 pos = sphereWarp(adjusted_coordinates, hover_amount);
  vec2 dims = v_dimensions * vec2(1.0, 0.5 / kCycles);
  pos.y = (kSpread + 0.55 * hover_amount) * (mod((pos.y + 1.0 + time_position) * kCycles, 2.0) - 1.0);
  float dist = length(adjusted_coordinates * width);
  float dist_value = 1.0 - clamp((dist - width) * 0.5 + 2.0, 0.0, 1.0);
  float shape_alpha = shape(pos, dims, kCycles, hover_amount);
  gl_FragColor.rgb = v_color1.rgb * shape_alpha;
  float ring = width * kRingWidth;
  gl_FragColor.a = shape_alpha * dist_value;

  vec3 arc_rads_distance = roundedArcRadsDistance(v_coordinates, v_dimensions.x, thickness, 0.0, max_arc);
  float rads = arc_rads_distance.y;

  float delta_rads = -rads - value;
  float color_step1 = step(0.0, delta_rads);
  float color_step = abs(color_step1);
  float blend = arc_rads_distance.x;

  vec4 arc_color = blend * (v_color0 * (1.0 - color_step));
  float color_switch = step(dist_value, 0.0);
  gl_FragColor = gl_FragColor * (1.0 - color_switch) + arc_color * color_switch;

  float full_radius = 0.5 * v_dimensions.x;
  float center_arc = full_radius - thickness * 0.5;
  float delta_center = arc_rads_distance.z;

  float thumb_x = sin(delta_rads) * dist * 0.5;
  float thumb_y = cos(delta_rads) * dist * 0.5 - center_arc;
  float adjusted_thumb_y = min(thumb_y, 0.0);
  float outside_arc_step = step(0.0, thumb_y);
  float thumb_y_distance = thumb_y * outside_arc_step + adjusted_thumb_y * (1.0 - outside_arc_step);
  float thumb_distance = length(vec2(thumb_x, thumb_y_distance));
  float thumb_alpha = clamp(thickness * 0.5 - thumb_distance, 0.0, 1.0);
  gl_FragColor = gl_FragColor * (1.0 - thumb_alpha) + v_color2 * thumb_alpha;
}

