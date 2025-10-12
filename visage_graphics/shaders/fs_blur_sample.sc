$input v_coordinates

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_pixel_size;

void main() {
  gl_FragColor = texture2D(s_texture, v_coordinates);
  vec2 offset = 0.5 * u_pixel_size.xy;
  vec4 c = texture2D(s_texture, v_coordinates + vec2(-offset.x, -offset.y)) +
           texture2D(s_texture, v_coordinates + vec2(offset.x, -offset.y)) +
           texture2D(s_texture, v_coordinates + vec2(-offset.x, offset.y)) +
           texture2D(s_texture, v_coordinates + vec2(offset.x, offset.y));
  gl_FragColor = c * 0.25;
}

