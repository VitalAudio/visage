$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

void main() {
  vec2 texture_position = v_shader_values.zw * u_atlas_scale.xy;
  float value = texture2D(s_texture, texture_position).r;
  gl_FragColor = sampleGradient(s_gradient, v_gradient_texture_pos.xy, v_gradient_texture_pos.zw, clamp(value, 0.0, 1.0));
}
