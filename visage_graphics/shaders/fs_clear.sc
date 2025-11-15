$input v_position

#include <shader_include.sh>

void main() {
  gl_FragColor = vec4(v_position.x, 0.5, 0.5, 0.5);
}
