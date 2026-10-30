$input v_coordinates, v_dimensions, v_shader_values, v_position, v_gradient_pos, v_gradient_pos2, v_gradient_texture_pos

#include <shader_include.sh>
#include <path_area.sh>

SAMPLER2D(s_gradient, 0);
SAMPLER2D(s_texture, 1);

uniform vec4 u_atlas_scale;

float pieceArea(float width, float y0, float y1, float center) {
  return width * abs(clampedAverage(y0, y1) - center);
}

void main() {
  gl_FragColor = gradient(s_gradient, v_gradient_texture_pos, v_gradient_pos, v_gradient_pos2, v_position);
  int last_index = int(v_shader_values.y + 0.5);
  vec2 start_data = v_shader_values.zw * u_atlas_scale.xy;

  vec2 dimensions = v_dimensions - vec2(1.0, 1.0);
  vec2 pixel = (v_coordinates * 0.5 + vec2(0.5, 0.5)) * v_dimensions - vec2(1.0, 1.0);
  float center_y = v_shader_values.x * dimensions.y - pixel.y;
  float clamped_center = clamp(center_y, 0.0, 1.0);
  float left = max(pixel.x, 0.0);
  float right = min(pixel.x + 1.0, dimensions.x);

  float segments = float(max(last_index, 1));
  float convert = segments / dimensions.x;
  int range_start = int(clamp(floor(left * convert), 0.0, segments));
  int range_end = int(clamp(ceil(right * convert), 0.0, segments));

  float last_x = float(range_start) / convert;
  float last_y = dimensions.y * texture2DLod(s_texture, start_data + vec2(u_atlas_scale.x * float(min(range_start, last_index)), 0.0), 0.0).r - pixel.y;

  float area = 0.0;
  for (int i = range_start + 1; i <= range_end; ++i) {
    float x = float(i) / convert;
    float y = dimensions.y * texture2DLod(s_texture, start_data + vec2(u_atlas_scale.x * float(min(i, last_index)), 0.0), 0.0).r - pixel.y;
    float x0 = max(last_x, left);
    float x1 = min(x, right);
    if (x1 > x0) {
      float slope = (y - last_y) / (x - last_x);
      float y0 = last_y + slope * (x0 - last_x);
      float y1 = last_y + slope * (x1 - last_x);
      if ((y0 - center_y) * (y1 - center_y) < 0.0) {
        float cross_x = x0 + (center_y - y0) / (y1 - y0) * (x1 - x0);
        area += pieceArea(cross_x - x0, y0, center_y, clamped_center);
        area += pieceArea(x1 - cross_x, center_y, y1, clamped_center);
      }
      else
        area += pieceArea(x1 - x0, y0, y1, clamped_center);
    }
    last_x = x;
    last_y = y;
  }

  gl_FragColor = clampToAlpha(gl_FragColor * clamp(area, 0.0, 1.0));
}
