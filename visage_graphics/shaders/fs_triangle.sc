$input v_coordinates, v_dimensions, v_color0, v_shader_values

#include <shader_utils.sh>

void main() {
  vec2 dimensions = mix(v_dimensions, v_dimensions.yx, v_shader_values.w) + vec2(0.5, 0.5);
  vec2 coordinates = mix(v_coordinates, v_coordinates.yx, v_shader_values.w) * vec2(v_shader_values.z, 1.0);
  gl_FragColor = v_color0;
  gl_FragColor.a = v_color0.a * max(0.0, isosceles(coordinates, dimensions));
}
