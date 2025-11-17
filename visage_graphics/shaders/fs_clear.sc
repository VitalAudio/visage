#include <shader_include.sh>

uniform vec4 u_color;

void main() {
  gl_FragColor.r = u_color.r;
}
