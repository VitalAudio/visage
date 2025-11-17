$input a_position

#include <shader_include.sh>

void main() {
  gl_Position = a_position;
  gl_Position.z = 0.5;
  gl_Position.w = 1.0;
}
