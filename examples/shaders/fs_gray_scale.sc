$input v_texture_uv

#include <shader_include.sh>

SAMPLER2D(s_texture, 0);

void main() {
  vec4 color = texture2D(s_texture, v_texture_uv);
  float gray = dot(color.rgb, vec3(0.299, 0.587, 0.114)) * u_color_mult.x;
  gl_FragColor = vec4(gray, gray, gray, color.a);
}

