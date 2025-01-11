$input v_coordinates, v_dimensions, v_color0, v_color1, v_color2, v_shader_values

#include <shader_include.sh>

uniform vec4 u_time;
uniform vec4 u_settings;

void main() {
  float raw_value = v_shader_values.x;
  float hover_amount = v_shader_values.y;

  float thickness = u_settings.x * (1.0 + u_settings.y * hover_amount);
  float thumb_amount = u_settings.z;
  float max_arc = u_settings.w;

  float start_pos = floor(raw_value * 0.5);
  float value = (2.0 * (raw_value - start_pos * 2.0) - 1.0) * max_arc;

  vec3 arc_rads_distance = roundedArcRadsDistance(v_coordinates, v_dimensions.x, thickness, 0.0, max_arc);
  float rads = arc_rads_distance.y;

  float delta_rads = -rads - value;
  float color_step1 = step(0.0, delta_rads);
  float color_step2 = step(0.0, start_pos + rads);
  float color_step = abs(color_step2 - color_step1);
  gl_FragColor = v_color1 * color_step + v_color0 * (1.0 - color_step);
  gl_FragColor.a = gl_FragColor.a * arc_rads_distance.x;


  float full_radius = 0.5 * v_dimensions.x;
  float center_arc = full_radius - thickness * 0.5;
  float delta_center = arc_rads_distance.z;

  float thumb_length = full_radius * thumb_amount;
  float thumb_x = sin(delta_rads) * delta_center;
  float thumb_y = cos(delta_rads) * delta_center - center_arc;
  float adjusted_thumb_y = min(thumb_y + thumb_length, 0.0);
  float outside_arc_step = step(0.0, thumb_y);
  float thumb_y_distance = thumb_y * outside_arc_step + adjusted_thumb_y * (1.0 - outside_arc_step);
  float thumb_distance = length(vec2(thumb_x, thumb_y_distance));
  float thumb_alpha = clamp(thickness * 0.5 - thumb_distance, 0.0, 1.0);
  gl_FragColor = gl_FragColor * (1.0 - thumb_alpha) + v_color2 * thumb_alpha;
}
