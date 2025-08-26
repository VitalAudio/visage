$input v_shader_values, v_position

#include <shader_include.sh>

uniform vec4 u_gradient_texture_position;
uniform vec4 u_gradient_position;
uniform vec4 u_gradient_position2;
uniform vec4 u_radial_gradient;
uniform vec4 u_line_width;
uniform vec4 u_time;

SAMPLER2D(s_gradient, 0);

float radialGradient2(vec2 position, vec2 focal_point, vec4 coefficient) {
  vec2 position2 = position * position;
  float a = 1.0 - coefficient.x * focal_point.x * focal_point.x -
                  coefficient.y * focal_point.y * focal_point.y -
                  coefficient.z * focal_point.x * focal_point.y;
  float b = 2.0 * (coefficient.x * focal_point.x + coefficient.y * focal_point.y - 1.0) +
            coefficient.z * (position.x * focal_point.y + position.y * focal_point.x);
  float c = 1.0 - coefficient.x * position2.x - coefficient.y * position2.y - coefficient.z * position.x * position.y;
  float s = sqrt(b * b - 4.0 * a * c);

  return (s - b) / (2.0 * a);
}

vec2 gradientA(bool radial, vec4 texture_pos, vec4 gradient_pos1, vec4 gradient_pos2, vec2 position) {
  float t = 0.0;
  if (radial)
    t = radialGradient2(position - gradient_pos1.xy, gradient_pos1.zw, gradient_pos2);
  else
    t = linearGradient(gradient_pos1.xy, gradient_pos1.zw, position);

  bool should_clamp = texture_pos.x != 0.0;
  t = should_clamp ? clamp(t, 0.0, 1.0) : t;
  return mix(texture_pos.xy, texture_pos.zw, t);
}

void main() {

  vec2 gradient_pos = gradientA(u_radial_gradient.x != 0.0, u_gradient_texture_position, u_gradient_position, u_gradient_position2, v_position);
  vec4 color = texture2D(s_gradient, gradient_pos);

  float depth_out = v_shader_values.x;
  float dist_from_edge = min(depth_out, 1.0 - depth_out);
  float mult = 1.0 + max(dist_from_edge - 2.0 / u_line_width.x, 0.0);
  vec4 result = min(1.0, mult) * color;
  float scale = u_line_width.x * dist_from_edge;
  result.a = min(result.a * scale * 0.5, 1.0);
  result.rgb = result.rgb * v_shader_values.y;
  gl_FragColor = result;
}
