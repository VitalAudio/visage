$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);
uniform vec4 u_amount;
uniform vec4 u_time;
uniform vec4 u_texture_clamp;

vec3 rotateHue(vec3 color, float angle) {
  vec3 axis = vec3(0.57735, 0.57735, 0.57735);
  float cos_angle = cos(angle);
  return color * cos_angle + cross(axis, color) * sin(angle) + axis * dot(axis, color) * (1.0 - cos_angle);
}

void main() {
  vec4 color = texture2D(s_texture, clamp(v_texture_uv, u_texture_clamp.xy, u_texture_clamp.zw));
  gl_FragColor = vec4(clamp(rotateHue(color.rgb, u_time.x * u_amount.x), 0.0, 1.0) * step(0.0, color.a), color.a);
}
