$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_time;
uniform vec4 u_atlas_scale;
uniform vec4 u_texture_clamp;

vec4 sampleTexel(vec2 texel) {
  return texture2D(s_texture, clamp(texel * u_atlas_scale.xy, u_texture_clamp.xy, u_texture_clamp.zw));
}

vec3 rotateHue(vec3 color, float angle) {
  vec3 axis = vec3(0.57735, 0.57735, 0.57735);
  float cos_angle = cos(angle);
  return color * cos_angle + cross(axis, color) * sin(angle) + axis * dot(axis, color) * (1.0 - cos_angle);
}

void main() {
  vec4 color = sampleTexel(v_texture_uv / u_atlas_scale.xy);
  gl_FragColor = vec4(clamp(rotateHue(color.rgb, u_time.x * u_amount.x), 0.0, 1.0) * step(0.0, color.a), color.a);
}
