$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_utils.sh>

void main() {
  gl_FragColor = v_color0;
  float distance = sdCircle(v_coordinates * v_dimensions.x, v_dimensions.x);
  float alpha = smoothed(-0.5, 0.5, distance);
  gl_FragColor.a = v_color0.a * alpha;
}
