$input v_coordinates

#include <shader_include.sh>

uniform vec4 u_pixel_size;
SAMPLER2D(s_texture, 0);

void main() {
  vec2 offset = 1.3333333333333333 * u_pixel_size.xy;
  gl_FragColor = texture2D(s_texture, v_coordinates) * 0.29411764705882354;
  gl_FragColor += texture2D(s_texture, v_coordinates + offset) * 0.35294117647058826;
  gl_FragColor += texture2D(s_texture, v_coordinates - offset) * 0.35294117647058826;
}
