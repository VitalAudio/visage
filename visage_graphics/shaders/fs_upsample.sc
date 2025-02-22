$input v_coordinates

#include <shader_include.sh>

uniform vec4 u_pixel_size;

SAMPLER2D(s_texture, 0);

void main() {
  vec2 dimensions = u_pixel_size.zw;
  vec2 position = dimensions * v_coordinates;
  vec2 pixel_t = fract(position * 0.5);
  vec2 pixel_position = v_coordinates - pixel_t * u_pixel_size.xy;

  vec2 pixel_offset = u_pixel_size.xy;
  vec4 top1 = texture2D(s_texture, pixel_position);
  vec4 top2 = texture2D(s_texture, pixel_position + vec2(pixel_offset.x, 0.0));
  vec4 bottom1 = texture2D(s_texture, pixel_position + vec2(0.0, pixel_offset.y));
  vec4 bottom2 = texture2D(s_texture, pixel_position + pixel_offset);
  gl_FragColor = mix(mix(top1, top2, pixel_t.x), mix(bottom1, bottom2, pixel_t.x), pixel_t.y);
}

