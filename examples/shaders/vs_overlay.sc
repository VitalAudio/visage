$input a_position, a_texcoord0, a_texcoord1
$output v_texture_uv

#include <shader_include.sh>

uniform vec4 u_bounds;
uniform vec4 u_atlas_scale;
uniform vec4 u_zoom;

void main() {
  vec2 min = a_texcoord1.xy;
  vec2 max = a_texcoord1.zw;
  vec2 clamped = clamp(a_position.xy, min, max);
  vec2 delta = clamped - a_position.xy;

  v_texture_uv = (a_texcoord0.xy + delta) * u_atlas_scale.xy;
  vec2 adjusted_position = clamped * u_bounds.xy + u_bounds.zw;
  gl_Position = vec4(adjusted_position * u_zoom.x, 0.5, 1.0);
}
