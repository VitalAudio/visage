$input v_shader_values, v_shader_values1

#include <shader_include.sh>

vec2 distanceToLine(vec2 p, vec2 a, vec2 b) {
  vec2 ba = b - a;
  vec2 pa = p - a;
  float t = dot(pa, ba) / dot(ba, ba);
  float offset = max(t - 1.0, -t);
  float ba_length = length(ba);
  return vec2((ba.x * pa.y - ba.y * pa.x) / ba_length, clamp(offset * ba_length, 0.0, 1.0));
}

void main() {
  float mult = gl_FrontFacing ? -1.0 : 1.0;
  vec3 delta = clamp(mult * v_shader_values.xyz + vec3(0.5, 0.5, 0.5), 0.0, 1.0);
  vec2 outer_result = distanceToLine(gl_FragCoord.xy, v_shader_values1.zw, v_shader_values1.xy);
  float outer_alpha = clamp(mult * outer_result.x + 0.5, 0.0, 1.0) * (1.0 - outer_result.y) + outer_result.y;
  gl_FragColor.r = mult * clamp(delta.y - delta.x, 0.0, 1.0) * outer_alpha;
}
