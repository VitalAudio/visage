$input v_coordinates, v_dimensions, v_color0, v_color1, v_shader_values, v_position, v_gradient_pos

#include <shader_include.sh>

void main() {
  vec2 dimensions = mix(v_dimensions, v_dimensions.yx, v_shader_values.w) - vec2(0.5, 0.5);
  vec2 coordinates = mix(v_coordinates, v_coordinates.yx, v_shader_values.w) * vec2(v_shader_values.z, 1.0);
  gl_FragColor = gradient(v_color0, v_color1, v_gradient_pos.xy, v_gradient_pos.zw, v_position);
  gl_FragColor.a = gl_FragColor.a * max(0.0, isosceles(coordinates, dimensions));
}
