$input v_texture_uv, v_shader_values

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

void main() {
  const vec3 kLuminanceWeights = vec3(0.2125, 0.7154, 0.0721);
  vec4 color = v_shader_values.x * u_color_mult * texture2D(s_texture, v_texture_uv);
  gl_FragColor = vec4_splat(dot(color.rgb, kLuminanceWeights));
}
