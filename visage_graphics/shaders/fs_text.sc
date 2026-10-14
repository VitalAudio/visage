$input v_coordinates, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

void main() {
  vec4 color = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  vec4 glyph = texture2D(s_texture, v_coordinates);
  glyph.rgb = glyph.rgb / max(glyph.a, 0.0001);
  gl_FragColor = color * glyph;
}
