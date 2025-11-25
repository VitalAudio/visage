$input v_shader_values, v_shader_values1

#include <shader_include.sh>

void main() {
  float mult = gl_FrontFacing ? -1.0 : 1.0;
  vec3 delta = clamp(mult * v_shader_values.xyz + vec3(0.5, 0.5, 0.5), 0.0, 1.0);
  float outer_amount = clamp((abs(v_shader_values.w - 0.5) - 0.5) / fwidth(v_shader_values.w), 0.0, 1.0);
  float outer_alpha = delta.z * (1.0 - outer_amount) + outer_amount;
  float point_alpha = v_shader_values1.y < gl_FragCoord.y + 1.0 ? 0.0 : 1.0;
  gl_FragColor.r = mult * (delta.y - delta.x) * outer_alpha * point_alpha;
}
