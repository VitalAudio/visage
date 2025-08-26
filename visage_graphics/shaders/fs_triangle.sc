$input v_coordinates, v_dimensions, v_shader_values, v_shader_values1, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

SAMPLER2D(s_gradient, 0);

void main() {
  vec2 dimensions = mix(v_dimensions, v_dimensions.yx, v_shader_values.w) - vec2(0.5, 0.5);
  vec2 coordinates = mix(v_coordinates, v_coordinates.yx, v_shader_values.w) * vec2(v_shader_values.z, 1.0);
  gl_FragColor = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  gl_FragColor.a = gl_FragColor.a * trianglePoints(v_coordinates, v_dimensions, v_shader_values.zw, v_shader_values1.xy, v_shader_values1.zw, v_shader_values.x, v_shader_values.y);
}
