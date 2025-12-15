$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

void main() {
  float y = v_coordinates.y * 0.5 + 0.5;
  float base_pow = pow(2.0, -v_shader_values.y);
  float numerator = pow(base_pow, y) - 1.0;
  float denominator = base_pow - 1.0;
  y = denominator == 0.0 ? y : numerator / denominator;
  y = v_shader_values.x * y;
  vec2 texture_position = (v_shader_values.zw + vec2(0.0, y)) * u_atlas_scale.xy;
  float value = texture2D(s_texture, texture_position).r;
  gl_FragColor = sampleGradient(s_gradient, v_gradient_texture_pos.xy, v_gradient_texture_pos.zw, clamp(value, 0.0, 1.0));
}
