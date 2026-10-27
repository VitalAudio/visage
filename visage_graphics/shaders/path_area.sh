float clampedArea(float y) {
  float c = clamp(y, 0.0, 1.0);
  return 0.5 * c * c + max(y - 1.0, 0.0);
}

float clampedAverage(float y0, float y1) {
  float dy = y1 - y0;
  return abs(dy) > 0.0001 ? (clampedArea(y1) - clampedArea(y0)) / dy : clamp(0.5 * (y0 + y1), 0.0, 1.0);
}

float edgeArea(vec2 a, vec2 b) {
  float x0 = clamp(min(a.x, b.x), 0.0, 1.0);
  float x1 = clamp(max(a.x, b.x), 0.0, 1.0);
  if (x1 <= x0)
    return 0.0;

  float dx = b.x - a.x;
  float slope = (b.y - a.y) / dx;
  float y0 = a.y + slope * (x0 - a.x);
  float y1 = a.y + slope * (x1 - a.x);
  return sign(dx) * (x1 - x0) * clampedAverage(y0, y1);
}
