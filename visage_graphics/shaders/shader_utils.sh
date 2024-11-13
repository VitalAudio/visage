#include <bgfx_shader.sh>

#define kPi 3.141592653589793238462643383279
#define kArcRounded 1.5
#define kArcFlat 0.5

float smoothed(float from, float to, float value) {
  float t = clamp((value - from) / (to - from), 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

float border(float distance, float thickness, float fade) {
  return smoothed(0.0, 2.0 * fade, thickness - abs(distance + thickness));
}

float sdCircle(vec2 position, float radius) {
  return length(position) - radius;
}

float sdSquircle(vec2 position, vec2 rectangle, float power) {
  vec2 normalized = abs(position) / rectangle;
  float distance = pow(pow(normalized.x, power) + pow(normalized.y, power), 1.0 / power);
  float len = length(position);
  return distance * len - len;
}

float sdRectangle(vec2 position, vec2 rectangle) {
  vec2 offset = abs(position) - rectangle;
  return min(max(offset.x, offset.y), 0.0) + length(max(offset, 0.0));
}

float sdRoundedRectangle(vec2 position, vec2 rectangle, float rounding) {
  vec2 offset = abs(position) - rectangle + rounding;
  return min(max(offset.x, offset.y), 0.0) + length(max(offset, 0.0)) - rounding;
}

float sdRoundedDiamond(vec2 position, vec2 diamond, float rounding) {
  vec2 rotated = vec2(position.x + position.y, position.y - position.x);
  vec2 offset = abs(rotated) - diamond + rounding;
  return min(max(offset.x, offset.y), 0.0) + length(max(offset, 0.0)) - rounding;
}

float sdSegment(vec2 position, vec2 point1, vec2 point2) {
  vec2 position_delta = position - point1;
  vec2 line_delta = point2 - point1;
  float t = clamp(dot(position_delta, line_delta) / dot(line_delta, line_delta), 0.0, 1.0);
  return length(position_delta - line_delta * t);
}

float sdFlatSegment(vec2 position, vec2 point1, vec2 point2, float thickness) {
  float size = length(point2 - point1);
  vec2 normalized_delta = (point2 - point1) / size;
  vec2 delta = (position - (point1 + point2) * 0.5);
  delta = mul(delta, mat2(normalized_delta.x, -normalized_delta.y, normalized_delta.y, normalized_delta.x));
  delta = abs(delta) - vec2(size, thickness) * 0.5;
  return length(max(delta, 0.0)) + min(max(delta.x, delta.y), 0.0);
}

float sdArc(vec2 position, vec2 middle_coords, vec2 curve_coords, float radius, float thickness) {
  position = mul(position, mat2(middle_coords.x, middle_coords.y, -middle_coords.y, middle_coords.x));
  position.x = abs(position.x);
  float k = (curve_coords.y * position.x > curve_coords.x * position.y) ? dot(position.xy, curve_coords) : length(position);
  return sqrt(max(0.0, dot(position, position) + radius * radius - 2.0 * radius * k)) - thickness;
}

float sdFlatArc(vec2 position, vec2 middle_coords, vec2 curve_coords, float radius, float thickness) {
  float dist = length(position);
  position = mul(position, mat2(-middle_coords.x, -middle_coords.y, middle_coords.y, -middle_coords.x));
  position.x = abs(position.x);
  position = mul(position, mat2(curve_coords.y, curve_coords.x, curve_coords.x, -curve_coords.y));
  position = vec2((position.y > 0.0) ? position.x : dist * sign(-curve_coords.x), (position.x > 0.0) ? position.y : dist);
  position = vec2(position.x - 1.0, abs(position.y - radius) - thickness);
  return length(max(position, 0.0)) + min(0.0, max(position.x, position.y));
}

float gaussian(float x, float sigma) {
  return exp(-(x * x) / (2.0 * sigma * sigma)) / (2.50662827463 * sigma);
}

vec2 erf(vec2 x) {
  vec2 s = sign(x);
  vec2 a = abs(x);
  x = 1.0 + (0.278393 + (0.230389 + 0.078108 * (a * a)) * a) * a;
  x *= x;
  return s - s / (x * x);
}

vec4 erf(vec4 x) {
  vec4 s = sign(x);
  vec4 a = abs(x);
  x = 1.0 + (0.278393 + (0.230389 + 0.078108 * (a * a)) * a) * a;
  x *= x;
  return s - s / (x * x);
}

float boxShadow(vec2 coordinates, vec2 dimensions, float sigma) {
  vec2 half_size = dimensions * 0.5;
  vec2 position = coordinates * half_size * (vec2(1.0, 1.0) + 2.0 * vec2(sigma, sigma) / half_size);
  vec4 query = vec4(position - half_size, half_size + position);
  vec4 integral = 0.5 + 0.5 * erf(query * (sqrt(0.5) / sigma));
  return (integral.z - integral.x) * (integral.w - integral.y);
}

float roundedBoxShadowX(float x, float y, float sigma, float corner, vec2 half_size) {
  float delta = min(half_size.y - corner - abs(y), 0.0);
  float curved = half_size.x - corner + sqrt(max(0.0, corner * corner - delta * delta));
  vec2 integral = 0.5 + 0.5 * erf((x + vec2(-curved, curved)) * (sqrt(0.5) / sigma));
  return integral.y - integral.x;
}

float roundedBoxShadow(vec2 coordinates, vec2 dimensions, float sigma, float corner) {
  vec2 half_size = dimensions * 0.5;
  vec2 position = coordinates * half_size;
  float low = position.y - half_size.y;
  float high = position.y + half_size.y;
  float start = clamp(-3.0 * sigma, low, high);
  float end = clamp(3.0 * sigma, low, high);

  float step = (end - start) / 4.0;
  float y = start + step * 0.5;
  float value = 0.0;
  for (int i = 0; i < 4; i++) {
    value += roundedBoxShadowX(position.x, position.y - y, sigma, corner, half_size) * gaussian(y, sigma) * step;
    y += step;
  }

  return value;
}

float rectangle(vec2 coordinates, vec2 dimensions, float thickness, float fade) {
  float distance = sdRectangle(coordinates * dimensions, dimensions);
  return border(distance, thickness, fade);
}

float circle(vec2 coordinates, float width, float thickness, float fade) {
  float distance = sdCircle(coordinates * width, width);
  return border(distance, thickness + 1.0, fade);
}

float segment(vec2 coordinates, vec2 dimensions, vec2 point1, vec2 point2, float thickness) {
  float distance = abs(sdSegment(coordinates * dimensions, point1 * dimensions, point2 * dimensions)) - thickness;
  return 1.0 - smoothed(-2.0, 0.0, distance);
}

float flatSegment(vec2 coordinates, vec2 dimensions, vec2 point1, vec2 point2, float thickness) {
  float distance = sdFlatSegment(coordinates * dimensions, point1 * dimensions, point2 * dimensions, thickness * 2.0);
  return 1.0 - smoothed(-2.0, 0.0, distance);
}

float arc(vec2 coordinates, vec2 middle_coords, vec2 curve_coords, float width, float thickness) {
  float distance = sdArc(coordinates * width, middle_coords, curve_coords, width - thickness, thickness * 0.5);
  return 1.0 - smoothed(-2.0, 0.0, distance - thickness * 0.5);
}

float flatArc(vec2 coordinates, vec2 middle_coords, vec2 curve_coords, float width, float thickness) {
  float distance = sdFlatArc(coordinates * width, middle_coords, curve_coords, width - thickness, thickness);
  return 1.0 - smoothed(-2.0, 0.0, distance);
}

float roundedRectangle(vec2 coordinates, vec2 dimensions, float rounding, float thickness, float fade) {
  float distance = sdRoundedRectangle(coordinates * dimensions, dimensions, rounding);
  return border(distance, thickness + 1.0, fade);
}

float squircle(vec2 coordinates, vec2 dimensions, float power, float thickness, float fade) {
  float distance = sdSquircle(coordinates * dimensions, dimensions, power);
  return border(distance, thickness, fade);
}

float roundedDiamond(vec2 coordinates, vec2 dimensions, float rounding, float thickness, float fade) {
  float distance = sdRoundedDiamond(coordinates * dimensions, dimensions, rounding);
  return border(distance, thickness, fade);
}

float diamond(vec2 coordinates, float width) {
  float radius = width * 0.5;
  vec2 distance = abs(coordinates);
  float delta_center = (distance.x + distance.y) * radius;
  return clamp(radius - delta_center, 0.0, 1.0);
}

vec3 arcRadsDistance(vec2 coordinates, float width, float thickness, float center_radians, float radian_radius, float shape) {
  float rounding = 1.0;
  float pixel_scale = 1.0;

  float radius = 0.5 * width;
  float delta_center = length(coordinates) * radius;

  float scaled_thickness = 0.5 * thickness;
  float center_arc = radius - scaled_thickness;
  float delta_arc = delta_center - center_arc;
  float distance_arc = abs(delta_arc);

  float rads = mod(atan2(coordinates.x, coordinates.y) + center_radians, 2.0 * kPi) - kPi;
  float dist_curve_mult = min(delta_center, center_arc);
  float dist_curve_left = max(dist_curve_mult * (rads - radian_radius), 0.0);
  float dist_curve = max(dist_curve_mult * (-rads - radian_radius), dist_curve_left);
  float distance_from_thickness = mix(distance_arc, length(vec2(distance_arc, dist_curve)), rounding);
  float radians_alpha = mix(max(1.0 - dist_curve * pixel_scale, 0.0), 1.0, rounding);
  return vec3(clamp((scaled_thickness - distance_from_thickness) * pixel_scale, 0.0, 1.0) * radians_alpha, rads, delta_center);
}

float isosceles(vec2 coordinates, vec2 dimensions) {
  float edge_length = length(vec2(dimensions.x, dimensions.y * 0.5));
  float slope = dimensions.x * 2.0 / dimensions.y;
  vec2 position = (coordinates * 0.5 + vec2(0.5, 0.5)) * dimensions;
  float delta_top = position.x * slope - position.y;
  float delta_bottom = position.x * slope - dimensions.y + position.y;
  float scale = dimensions.x / edge_length;
  float alpha_top = clamp(0.5 - scale * delta_top, 0.0, 1.0);
  float alpha_bottom = clamp(0.5 - scale * delta_bottom, 0.0, 1.0);
  float alpha_side = min(1.0, position.x);
  return alpha_top * alpha_bottom * alpha_side;
}
