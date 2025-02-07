$input v_color0, v_color1, v_position, v_gradient_pos

#include <shader_include.sh>

void main() {
  gl_FragColor = gradient(v_color0, v_color1, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
}
