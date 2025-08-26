$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>

uniform vec4 u_time;

SAMPLER2D(s_gradient, 0);

float fcos(float x) {
  return cos(x) * smoothstep(6.28, 0.0, fwidth(x));
}

float getColor(in float t) {
  float col = 0.3;
  col += 0.12 * fcos(6.28318 * t *   1.0);
  col += 0.11 * fcos(6.28318 * t *   3.1);
  col += 0.10 * fcos(6.28318 * t *   5.1);
  col += 0.09 * fcos(6.28318 * t *   9.1);
  col += 0.08 * fcos(6.28318 * t *  17.1);
  col += 0.07 * fcos(6.28318 * t *  31.1);
  col += 0.06 * fcos(6.28318 * t *  65.1);
  col += 0.06 * fcos(6.28318 * t * 115.1);
  col += 0.09 * fcos(6.28318 * t * 265.1);
  return col;
}

vec2 deform(float time, in vec2 p) {
  p *= 4.0;
  p = 0.5 * p / dot(p, p);
  p.x += time * 0.05;

  p += 0.2 * cos(1.5 * p.yx + 0.03 * 1.0 * time + vec2(0.1, 1.1));
  p += 0.2 * cos(2.4 * p.yx + 0.03 * 1.6 * time + vec2(4.5, 2.6));
  p += 0.2 * cos(3.3 * p.yx + 0.03 * 1.2 * time + vec2(3.2, 3.4));
  p += 0.2 * cos(4.2 * p.yx + 0.03 * 1.7 * time + vec2(1.8, 5.2));
  p += 0.2 * cos(9.1 * p.yx + 0.03 * 1.1 * time + vec2(6.3, 3.9));

  return p;
}

void main() {
  vec2 p = v_coordinates * 0.3;
  vec2 w = p;
  p = deform(u_time.x, p);

  float col = 1.5 * getColor(0.5 * length(p));

  col *= 1.4 - 0.14 / length(w);
  vec4 color = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  gl_FragColor = max(0.0, col) * color;
}

