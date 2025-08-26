$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

void main() {
  vec4 color = texture2D(s_texture, v_texture_uv);
  gl_FragColor.r = dot(color.rgb, vec3(0.393, 0.769, 0.189)) * u_color_mult.x;
  gl_FragColor.g = dot(color.rgb, vec3(0.349, 0.686, 0.168)) * u_color_mult.x;
  gl_FragColor.b  = dot(color.rgb, vec3(0.272, 0.534, 0.131)) * u_color_mult.x;
  gl_FragColor.a = color.a;
}

