$input v_shader_values, v_shader_values1

#include <shader_include.sh>

uniform vec4 u_color;

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
  float d1 = clamp(mult * distanceToLine(gl_FragCoord.xy, v_shader_values.zw, v_shader_values1.xy).x + 0.5, 0.0, 1.0);
  float d2 = clamp(mult *  distanceToLine(gl_FragCoord.xy, v_shader_values.zw, v_shader_values1.zw).x + 0.5, 0.0, 1.0);
  vec2 d3_result = distanceToLine(gl_FragCoord.xy, v_shader_values1.zw, v_shader_values1.xy);
  float d3 = clamp(mult * d3_result.x + 0.5, 0.0, 1.0) * (1.0 - d3_result.y) + d3_result.y;
  gl_FragColor.r = mult * clamp(d2 - d1, 0.0, 1.0) * d3;
}
