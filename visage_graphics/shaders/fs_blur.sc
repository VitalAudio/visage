$input v_coordinates

#include <shader_include.sh>

uniform vec4 u_pixel_size;
SAMPLER2D(s_texture, 0);

void main() {
  vec2 step = u_pixel_size.xy;
  gl_FragColor =
    texture2D(s_texture, v_coordinates + step * -10.0) * 0.001331642384180115 +
    texture2D(s_texture, v_coordinates + step * -9.0)  * 0.0031311897862488025 +
    texture2D(s_texture, v_coordinates + step * -8.0)  * 0.006728909236626558 +
    texture2D(s_texture, v_coordinates + step * -7.0)  * 0.01321579963304265 +
    texture2D(s_texture, v_coordinates + step * -6.0)  * 0.02372224120933465 +
    texture2D(s_texture, v_coordinates + step * -5.0)  * 0.038916294930399935 +
    texture2D(s_texture, v_coordinates + step * -4.0)  * 0.0583472982820951 +
    texture2D(s_texture, v_coordinates + step * -3.0)  * 0.07995092874022597 +
    texture2D(s_texture, v_coordinates + step * -2.0)  * 0.10012436424202198 +
    texture2D(s_texture, v_coordinates + step * -1.0)  * 0.11459601788478359 +
    texture2D(s_texture, v_coordinates)                * 0.11987062734208122 +
    texture2D(s_texture, v_coordinates + step * 1.0)   * 0.11459601788478359 +
    texture2D(s_texture, v_coordinates + step * 2.0)   * 0.10012436424202198 +
    texture2D(s_texture, v_coordinates + step * 3.0)   * 0.07995092874022597 +
    texture2D(s_texture, v_coordinates + step * 4.0)   * 0.0583472982820951 +
    texture2D(s_texture, v_coordinates + step * 5.0)   * 0.038916294930399935 +
    texture2D(s_texture, v_coordinates + step * 6.0)   * 0.02372224120933465 +
    texture2D(s_texture, v_coordinates + step * 7.0)   * 0.01321579963304265 +
    texture2D(s_texture, v_coordinates + step * 8.0)   * 0.006728909236626558 +
    texture2D(s_texture, v_coordinates + step * 9.0)   * 0.0031311897862488025 +
    texture2D(s_texture, v_coordinates + step * 10.0)  * 0.001331642384180115;
}
