$input v_shader_values, v_shader_values1

#include <shader_include.sh>
#include <path_area.sh>

SAMPLER2D(s_texture, 0);

uniform vec4 u_dimensions;

#define kAreaScale 1048576.0
#define kMaxExactSegments 1024

void main() {
  vec2 pixel = floor(gl_FragCoord.xy) - v_shader_values1.xy;
  int start = int(v_shader_values.x + 0.5);
  int count = int(v_shader_values.y + 0.5);
  int end = start + count;
  int width = int(u_dimensions.x + 0.5);
  float coverage = floor(v_shader_values.z + 0.5);
  if (v_shader_values.w >= 0.0) {
    int column = int(pixel.x);
    int index = int(v_shader_values.w + 0.5) + column / 4;
    vec4 cover = texelFetch(s_texture, ivec2(index % width, index / width), 0);
    int component = column % 4;
    coverage += component == 0 ? cover.x : component == 1 ? cover.y : component == 2 ? cover.z : cover.w;
  }

  int shift = 0;
  while ((count >> shift) > kMaxExactSegments)
    ++shift;

  float area_scale = kAreaScale / float(1 << shift);
  int area = 0;
  for (int i = start; i < end; ++i) {
    vec4 segment = texelFetch(s_texture, ivec2(i % width, i / width), 0);
    area += int(round(edgeArea(segment.xy - pixel, segment.zw - pixel) * area_scale));
  }

  coverage = abs(coverage + float(area) / area_scale);
  float alpha = v_shader_values1.z > 0.5 ? 1.0 - abs(mod(coverage, 2.0) - 1.0) : min(coverage, 1.0);
  gl_FragColor = vec4(alpha, 0.0, 0.0, 1.0);
}
