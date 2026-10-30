$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

float sampleData(vec2 texel) {
  return texture2D(s_texture, (texel + vec2(0.5, 0.5)) * u_atlas_scale.xy).r;
}

void main() {
  float y = v_coordinates.y * 0.5 + 0.5;
  float base_pow = pow(2.0, -v_shader_values.y);
  float numerator = pow(base_pow, y) - 1.0;
  float denominator = base_pow - 1.0;
  y = denominator == 0.0 ? y : numerator / denominator;
  y = v_shader_values.x * y;
  vec2 texel = v_shader_values.zw + vec2(0.0, y) - vec2(0.5, 0.5);
  vec2 base = floor(texel);
  vec2 t = texel - base;
  float bottom = mix(sampleData(base), sampleData(base + vec2(1.0, 0.0)), t.x);
  float top = mix(sampleData(base + vec2(0.0, 1.0)), sampleData(base + vec2(1.0, 1.0)), t.x);
  float value = mix(bottom, top, t.y);
  gl_FragColor = clampToAlpha(sampleGradient(s_gradient, v_gradient_texture_pos.xy, v_gradient_texture_pos.zw, clamp(value, 0.0, 1.0)));
}
