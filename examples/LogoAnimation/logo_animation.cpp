/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>
#include <visage/app.h>

namespace {
  constexpr float kPi = 3.14159265358979323846f;
  constexpr float kLogoSize = 1701.0f;
  constexpr float kCenter = kLogoSize * 0.5f;
  constexpr float kViewWidth = 5145.0f;
  constexpr float kViewHeight = 3619.0f;
  constexpr float kCenterHeightRatio = 0.472f;

  constexpr float kRingRadius = 587.25f;
  constexpr float kRingWidth = 85.5f;
  constexpr float kRingGap = 100.0f;
  constexpr float kBodyRadius = 700.0f;
  constexpr float kOuterRingRadius = 717.5f;
  constexpr float kOuterRingWidth = 24.5f;
  constexpr float kPointerStart = 290.0f;
  constexpr float kPointerLength = 297.0f;
  constexpr float kHookLength = 2220.0f;
  constexpr float kKnobAngle = 144.0f;
  constexpr float kEdgeWidth = 42.0f;
  constexpr float kShadowWidth = 140.0f;
  constexpr float kRevealDistance = 1400.0f;
  constexpr float kRevealSoftness = 175.0f;

  constexpr float kAnimationLength = 2.1f;
  constexpr float kHoldLength = 1.9f;

  constexpr unsigned int kBackgroundColor = 0xff24282c;
  constexpr unsigned int kBodyColor = 0xff1b1f23;
  constexpr unsigned int kTrackColor = 0xff4c4e53;
  constexpr unsigned int kShadowColor = 0xff0e0f12;
  constexpr unsigned int kPointerColor = 0xfff3f2f6;
  constexpr unsigned int kRingTopColor = 0xffa686fb;
  constexpr unsigned int kRingBottomColor = 0xff8463d2;
  constexpr unsigned int kVTopColor = 0xfff9f9f9;
  constexpr unsigned int kVBottomColor = 0xffe8e8ea;
  constexpr unsigned int kTextColor = 0xffe7e7eb;

  constexpr char kVPath[] =
      "M1545.679,448.976c-103.096,36.788-281.918,58.519-449.179,69.215c-80.995,106.668-180.985,"
      "271.027-246,381.296c-65.016-110.27-165.007-274.632-246.003-381.299c-167.261-10.697-346.081-"
      "32.424-449.176-69.213l-0.141,0.244C283.39,558.187,470.26,839.905,569.908,1012.5s250.188,"
      "475.288,280.452,640.805h0.281c30.264-165.517,180.804-468.21,280.452-640.805s286.517-454.313,"
      "414.728-563.281L1545.679,448.976z";

  class Track {
  public:
    struct Key {
      float time = 0.0f;
      float value = 0.0f;
    };

    Track(std::initializer_list<Key> keys) : keys_(keys), slopes_(keys_.size(), 0.0f) {
      int num_keys = keys_.size();
      std::vector<float> secants(std::max(0, num_keys - 1));
      for (int i = 0; i < num_keys - 1; ++i)
        secants[i] = (keys_[i + 1].value - keys_[i].value) / (keys_[i + 1].time - keys_[i].time);

      for (int i = 1; i < num_keys - 1; ++i) {
        if (secants[i - 1] * secants[i] > 0.0f)
          slopes_[i] = 0.5f * (secants[i - 1] + secants[i]);
      }

      for (int i = 0; i < num_keys - 1; ++i) {
        if (secants[i] == 0.0f)
          continue;
        float a = slopes_[i] / secants[i];
        float b = slopes_[i + 1] / secants[i];
        float magnitude = a * a + b * b;
        if (magnitude > 9.0f) {
          float scale = 3.0f / std::sqrt(magnitude);
          slopes_[i] = scale * a * secants[i];
          slopes_[i + 1] = scale * b * secants[i];
        }
      }
    }

    float operator()(float time) const {
      if (time <= keys_.front().time)
        return keys_.front().value;
      if (time >= keys_.back().time)
        return keys_.back().value;

      int index = 0;
      while (keys_[index + 1].time < time)
        ++index;

      const Key& from = keys_[index];
      const Key& to = keys_[index + 1];
      float duration = to.time - from.time;
      float t = (time - from.time) / duration;
      float t2 = t * t;
      float t3 = t2 * t;
      return (2.0f * t3 - 3.0f * t2 + 1.0f) * from.value +
             (t3 - 2.0f * t2 + t) * duration * slopes_[index] + (3.0f * t2 - 2.0f * t3) * to.value +
             (t3 - t2) * duration * slopes_[index + 1];
    }

  private:
    std::vector<Key> keys_;
    std::vector<float> slopes_;
  };

  const Track kBodyRadiusTrack = { { 0.0f, 24.0f },        { 0.033f, 108.0f }, { 0.1f, 378.0f },
                                   { 0.167f, 539.0f },     { 0.233f, 640.0f }, { 0.3f, 689.0f },
                                   { 0.367f, kBodyRadius } };
  const Track kOuterRingRadiusTrack = { { 0.0f, 854.0f },   { 0.067f, 836.5f },
                                        { 0.133f, 817.0f }, { 0.217f, 793.0f },
                                        { 0.3f, 770.0f },   { 0.367f, 754.0f },
                                        { 0.483f, 728.0f }, { 0.583f, kOuterRingRadius } };
  const Track kOuterRingEndTrack = { { 0.0f, 176.0f }, { 0.067f, 148.0f }, { 0.133f, 116.0f },
                                     { 0.2f, 86.0f },  { 0.267f, 57.0f },  { 0.333f, 38.0f },
                                     { 0.4f, 22.0f },  { 0.5f, 7.0f },     { 0.583f, 0.0f } };
  const Track kOuterRingStartTrack = { { 0.0f, 168.0f },  { 0.067f, 82.0f },   { 0.133f, -14.0f },
                                       { 0.2f, -102.0f }, { 0.267f, -191.0f }, { 0.333f, -250.0f },
                                       { 0.4f, -294.0f }, { 0.5f, -342.0f },   { 0.583f, -360.0f } };
  const Track kBandInnerTrack = { { 0.983f, kOuterRingRadius - 0.5f * kOuterRingWidth },
                                  { 1.0f, 696.5f },
                                  { 1.033f, 651.0f },
                                  { 1.067f, 577.5f },
                                  { 1.1f, 514.5f },
                                  { 1.133f, 420.0f },
                                  { 1.167f, 315.0f },
                                  { 1.2f, 241.5f },
                                  { 1.233f, 147.0f },
                                  { 1.267f, 94.5f },
                                  { 1.3f, 24.5f },
                                  { 1.317f, 0.0f } };
  const Track kBandOuterTrack = { { 1.167f, kOuterRingRadius + 0.5f * kOuterRingWidth },
                                  { 1.2f, 661.5f },
                                  { 1.25f, 399.0f },
                                  { 1.3f, 126.0f },
                                  { 1.333f, 0.0f } };

  const Track kTrackStartTrack = { { 0.15f, 0.0f },  { 0.217f, 16.0f },
                                   { 0.3f, 40.0f },  { 0.4f, 88.0f },
                                   { 0.5f, 140.0f }, { 0.6f, 185.0f },
                                   { 0.7f, 210.0f }, { 0.767f, 360.0f - kKnobAngle } };
  const Track kTrackEndTrack = {
    { 0.0f, 0.0f },     { 0.067f, 9.0f },   { 0.133f, 34.0f },  { 0.2f, 76.0f },
    { 0.267f, 145.0f }, { 0.333f, 214.0f }, { 0.4f, 288.0f },   { 0.467f, 356.0f },
    { 0.533f, 416.0f }, { 0.6f, 467.0f },   { 0.667f, 488.0f }, { 0.75f, 360.0f + kKnobAngle }
  };
  const Track kTrackWidthTrack = { { 0.0f, 17.5f },     { 0.067f, 31.5f }, { 0.133f, 44.0f },
                                   { 0.2f, 58.0f },     { 0.267f, 73.5f }, { 0.333f, 84.0f },
                                   { 0.4f, kRingWidth } };
  const Track kPointerAngleTrack = { { 0.033f, -109.0f }, { 0.133f, -70.0f },
                                     { 0.233f, -12.0f },  { 0.333f, 64.0f },
                                     { 0.45f, 167.0f },   { 0.55f, 266.0f },
                                     { 0.65f, 365.0f },   { 0.75f, 440.0f },
                                     { 0.833f, 493.0f },  { 0.9f, 360.0f + kKnobAngle } };
  const Track kPointerWidthTrack = { { 0.033f, 0.0f },
                                     { 0.083f, 42.0f },
                                     { 0.133f, kRingWidth },
                                     { 1.433f, kRingWidth },
                                     { 1.517f, 45.0f } };
  const Track kRingCloseTrack = { { 1.15f, 0.0f },
                                  { 1.2f, 11.0f },
                                  { 1.25f, 22.0f },
                                  { 1.3f, 32.0f },
                                  { 1.333f, 180.0f - kKnobAngle } };

  const Track kHookHeadTrack = {
    { 0.133f, 0.0f },  { 0.15f, 45.0f },           { 0.217f, 150.0f },         { 0.267f, 213.0f },
    { 0.3f, 248.0f },  { 0.367f, kPointerLength }, { 0.983f, kPointerLength }, { 1.017f, 254.0f },
    { 1.05f, 229.0f }, { 1.083f, 215.0f },         { 1.117f, 289.0f },         { 1.167f, 548.0f },
    { 1.2f, 674.0f },  { 1.25f, 922.0f },          { 1.3f, 1164.0f },          { 1.35f, 1419.0f },
    { 1.4f, 1685.0f }, { 1.45f, 1951.0f },         { 1.483f, kHookLength }
  };
  const Track kHookTailTrack = { { 1.267f, 0.0f },    { 1.3f, 96.0f },     { 1.35f, 303.0f },
                                 { 1.4f, 590.0f },    { 1.45f, 915.0f },   { 1.483f, 1174.0f },
                                 { 1.517f, 1400.0f }, { 1.533f, 1520.0f }, { 1.6f, kHookLength } };

  const Track kOuterEdgeTrack = { { 1.4f, 0.0f },    { 1.417f, 0.08f }, { 1.433f, 0.2f },
                                  { 1.45f, 0.33f },  { 1.467f, 0.47f }, { 1.483f, 0.6f },
                                  { 1.533f, 0.85f }, { 1.567f, 1.0f } };
  const Track kInnerEdgeTrack = { { 1.483f, 0.0f }, { 1.517f, 0.65f }, { 1.54f, 1.0f } };
  const Track kTopEdgeTrack = { { 1.525f, 0.0f }, { 1.55f, 0.6f }, { 1.567f, 1.0f } };
  const Track kEdgeWidthTrack = { { 1.62f, kEdgeWidth }, { 1.7f, 0.0f } };
  const Track kRevealTrack = { { 1.483f, 0.0f },   { 1.517f, 520.0f },
                               { 1.533f, 680.0f }, { 1.55f, 790.0f },
                               { 1.567f, 900.0f }, { 1.6f, 1080.0f },
                               { 1.65f, 1350.0f }, { 1.7f, kRevealDistance + kRevealSoftness } };
  const Track kRingGapTrack = { { 1.675f, 0.0f }, { 1.7f, 45.0f }, { 1.75f, kRingGap } };

  const Track kLongStreakHeadTrack = { { 1.567f, 0.0f },
                                       { 1.617f, 340.0f },
                                       { 1.667f, 742.0f },
                                       { 1.717f, 1043.0f },
                                       { 1.767f, 1172.0f } };
  const Track kLongStreakTailTrack = { { 1.667f, 0.0f }, { 1.7f, 330.0f }, { 1.75f, 770.0f }, { 1.817f, 1172.0f } };
  const Track kShortStreakHeadTrack = { { 1.567f, 0.0f },  { 1.583f, 105.0f }, { 1.617f, 203.0f },
                                        { 1.65f, 399.0f }, { 1.683f, 528.0f }, { 1.717f, 707.0f },
                                        { 1.75f, 864.0f }, { 1.783f, 930.0f } };
  const Track kShortStreakTailTrack = { { 1.633f, 0.0f },   { 1.65f, 157.0f }, { 1.683f, 325.0f },
                                        { 1.717f, 560.0f }, { 1.75f, 763.0f }, { 1.783f, 930.0f } };

  struct Letter {
    float start_time = 0.0f;
    std::vector<std::vector<visage::Point>> strokes;
  };

  constexpr float kTextTop = 1974.0f;
  constexpr float kTextBottom = 2166.5f;
  constexpr float kTextStroke = 24.5f;
  constexpr float kTextDrop = 98.0f;
  constexpr float kTextFadeLength = 0.23f;

  std::vector<Letter> wordmark() {
    float top = kTextTop;
    float bottom = kTextBottom;
    float bar = kTextStroke * 0.5f;
    float cross = top + (bottom - top) * 0.68f;
    return {
      { 1.65f, { { { 246.75f, top }, { 320.25f, bottom }, { 393.75f, top } } } },
      { 1.467f, { { { 581.0f, top }, { 581.0f, bottom } } } },
      { 1.583f,
        { { { 750.25f, top + bar }, { 900.75f, top + bar } }, { { 826.0f, top + bar }, { 826.0f, bottom } } } },
      { 1.517f,
        { { { 1029.0f, bottom }, { 1106.0f, top }, { 1183.0f, bottom } },
          { { 1029.0f + 77.0f * (bottom - cross) / (bottom - top), cross },
            { 1183.0f - 77.0f * (bottom - cross) / (bottom - top), cross } } } },
      { 1.4f, { { { 1361.5f, top }, { 1361.5f, bottom - bar }, { 1457.75f, bottom - bar } } } },
    };
  }

  float clamp01(float value) {
    return std::clamp(value, 0.0f, 1.0f);
  }

  float easeOut(float t) {
    t = clamp01(t);
    return 1.0f - (1.0f - t) * (1.0f - t) * (1.0f - t);
  }

  visage::Point polar(float radius, float degrees) {
    float radians = degrees * kPi / 180.0f;
    return { kCenter + radius * std::sin(radians), kCenter - radius * std::cos(radians) };
  }

  visage::Point mirror(const visage::Point& point) {
    return { kLogoSize - point.x, point.y };
  }

  visage::Path hookPath() {
    visage::Point direction = polar(1.0f, kKnobAngle) - visage::Point(kCenter, kCenter);
    visage::Point center(kCenter, kCenter);
    float u_radius = 262.5f;
    float u_center_x = kCenter + u_radius;
    float u_center_y = 1634.5f;

    visage::Path path;
    path.moveTo(center + direction * kPointerStart);
    path.lineTo(center + direction * 780.0f);
    path.bezierTo(center + direction * 860.0f, { u_center_x + u_radius, 1580.0f },
                  { u_center_x + u_radius, u_center_y });
    path.arcTo(u_radius, u_radius, 0.0f, false, true, { kCenter, u_center_y });
    path.lineTo(kCenter, 899.487f);
    return path;
  }

  visage::Path outerEdgePath(bool right) {
    auto map = [right](visage::Point point) { return right ? mirror(point) : point; };
    visage::Path path;
    path.moveTo(map({ kCenter, 1450.0f }));
    path.bezierTo(map({ 810.0f, 1330.0f }), map({ 629.9f, 1116.4f }), map({ 569.908f, 1012.5f }));
    path.bezierTo(map({ 470.26f, 839.905f }), map({ 283.39f, 558.187f }), map({ 155.18f, 449.219f }));
    return path;
  }

  visage::Path innerEdgePath(bool right) {
    auto map = [right](visage::Point point) { return right ? mirror(point) : point; };
    visage::Path path;
    path.moveTo(map({ 850.5f, 899.487f }));
    path.bezierTo(map({ 785.484f, 789.217f }), map({ 685.493f, 624.855f }), map({ 604.497f, 518.188f }));
    return path;
  }

  visage::Path topEdgePath(bool right) {
    auto map = [right](visage::Point point) { return right ? mirror(point) : point; };
    visage::Path path;
    path.moveTo(map({ 604.497f, 518.188f }));
    path.bezierTo(map({ 437.236f, 507.491f }), map({ 258.416f, 485.764f }), map({ 155.321f, 448.975f }));
    return path;
  }

  visage::Path longStreakPath(bool right) {
    auto map = [right](visage::Point point) { return right ? mirror(point) : point; };
    visage::Path path;
    path.moveTo(map({ 155.321f, 448.975f }));
    path.bezierTo(map({ -232.75f, 385.0f }), map({ -621.25f, 38.5f }), map({ -778.75f, -234.5f }));
    return path;
  }

  visage::Path shortStreakPath(bool right) {
    auto map = [right](visage::Point point) { return right ? mirror(point) : point; };
    visage::Path path;
    path.moveTo(map({ 155.321f, 448.975f }));
    path.bezierTo(map({ -1.75f, 280.0f }), map({ -281.75f, 105.0f }), map({ -649.0f, 24.5f }));
    return path;
  }

  class Polyline {
  public:
    explicit Polyline(const visage::Path& path) {
      if (path.subPaths().empty())
        return;

      points_ = path.subPaths().front().points;
      lengths_.reserve(points_.size());
      float length = 0.0f;
      for (size_t i = 0; i < points_.size(); ++i) {
        if (i > 0)
          length += (points_[i] - points_[i - 1]).length();
        lengths_.push_back(length);
      }
    }

    float length() const { return lengths_.empty() ? 0.0f : lengths_.back(); }

    visage::Point point(float distance, visage::Point* direction = nullptr) const {
      if (points_.size() < 2)
        return points_.empty() ? visage::Point() : points_.front();

      size_t index = 1;
      while (index + 1 < points_.size() && lengths_[index] < distance)
        ++index;

      visage::Point from = points_[index - 1];
      visage::Point to = points_[index];
      float segment = lengths_[index] - lengths_[index - 1];
      float t = segment > 0.0f ? std::clamp((distance - lengths_[index - 1]) / segment, 0.0f, 1.0f) : 0.0f;
      if (direction)
        *direction = (to - from).normalized();
      return from + (to - from) * t;
    }

  private:
    std::vector<visage::Point> points_;
    std::vector<float> lengths_;
  };

  class RingCutter {
  public:
    static constexpr int kSamples = 720;
    static constexpr float kInnerRadius = kRingRadius - 0.5f * kRingWidth;
    static constexpr float kOuterRadius = kRingRadius + 0.5f * kRingWidth;

    struct Range {
      float start = 0.0f;
      float end = 0.0f;
    };

    struct Piece {
      Range outer;
      Range inner;
    };

    RingCutter() {
      visage::Path v;
      v.loadSvgPath(kVPath);
      const std::vector<visage::Point>& outline = v.subPaths().front().points;
      for (int i = 0; i < kSamples; ++i) {
        float degrees = 360.0f * i / kSamples;
        inner_distances_[i] = signedDistance(outline, polar(kInnerRadius, degrees));
        outer_distances_[i] = signedDistance(outline, polar(kOuterRadius, degrees));
      }
    }

    std::vector<Piece> pieces(float gap) const {
      std::vector<Range> outer_ranges = keptRanges(outer_distances_, gap);
      std::vector<Range> inner_ranges = keptRanges(inner_distances_, gap);
      std::vector<Piece> result;
      for (const Range& outer : outer_ranges) {
        float middle = 0.5f * (outer.start + outer.end);
        for (const Range& inner : inner_ranges) {
          if (contains(inner, middle)) {
            result.push_back({ outer, inner });
            break;
          }
        }
      }
      return result;
    }

  private:
    static bool contains(const Range& range, float degrees) {
      float offset = std::fmod(std::fmod(degrees - range.start, 360.0f) + 360.0f, 360.0f);
      return offset <= range.end - range.start;
    }

    static float signedDistance(const std::vector<visage::Point>& outline, const visage::Point& point) {
      float min_distance = std::numeric_limits<float>::max();
      bool inside = false;
      for (size_t i = 0, j = outline.size() - 1; i < outline.size(); j = i++) {
        visage::Point from = outline[j];
        visage::Point delta = outline[i] - from;
        float t = std::clamp((point - from).dot(delta) / std::max(delta.dot(delta), 0.000001f), 0.0f, 1.0f);
        min_distance = std::min(min_distance, (from + delta * t - point).length());
        if ((from.y > point.y) != (outline[i].y > point.y) &&
            point.x < from.x + (point.y - from.y) * delta.x / delta.y)
          inside = !inside;
      }
      return inside ? -min_distance : min_distance;
    }

    static std::vector<Range> keptRanges(const std::array<float, kSamples>& distances, float gap) {
      int first_cut = -1;
      for (int i = 0; i < kSamples && first_cut < 0; ++i) {
        if (distances[i] <= gap)
          first_cut = i;
      }
      if (first_cut < 0)
        return { { 0.0f, 360.0f } };

      auto crossing = [&](int from) {
        float a = distances[from % kSamples];
        float b = distances[(from + 1) % kSamples];
        return 360.0f * (from + (gap - a) / (b - a)) / kSamples;
      };

      std::vector<Range> ranges;
      Range range;
      bool kept = false;
      for (int i = first_cut + 1; i <= first_cut + kSamples; ++i) {
        bool keep = distances[i % kSamples] > gap;
        if (keep && !kept)
          range.start = crossing(i - 1);
        else if (!keep && kept) {
          range.end = crossing(i - 1);
          ranges.push_back(range);
        }
        kept = keep;
      }
      return ranges;
    }

    std::array<float, kSamples> inner_distances_ {};
    std::array<float, kSamples> outer_distances_ {};
  };

  class Scene {
  public:
    Scene(visage::Canvas& canvas, float width, float height) :
        canvas_(canvas), width_(width), height_(height) {
      scale_ = std::min(width / kViewWidth, height / kViewHeight);
      offset_ = { width * 0.5f - kCenter * scale_, height * kCenterHeightRatio - kCenter * scale_ };
    }

    void draw(float time) {
      time_ = time;
      canvas_.setColor(kBackgroundColor);
      canvas_.fill(0, 0, width_, height_);

      drawBody();
      drawOuterRing();
      drawTrack();
      drawRing();
      drawHook();
      drawV();
      drawEdges();
      drawStreaks();
      drawWordmark();
    }

  private:
    visage::Point toScreen(const visage::Point& point) const { return offset_ + point * scale_; }

    visage::Path toScreen(const visage::Path& path) const {
      return path.transformed(visage::Transform::translation(offset_) *
                              visage::Transform::scale(scale_, scale_));
    }

    void addArc(visage::Path& path, float radius, float from_degrees, float to_degrees) const {
      int segments = std::max(1, static_cast<int>(std::ceil(std::abs(to_degrees - from_degrees) / 90.0f)));
      for (int i = 1; i <= segments; ++i) {
        float degrees = from_degrees + (to_degrees - from_degrees) * i / segments;
        path.arcTo(radius * scale_, radius * scale_, 0.0f, false, to_degrees > from_degrees,
                   toScreen(polar(radius, degrees)));
      }
    }

    visage::Path arc(float radius, float from_degrees, float to_degrees) const {
      visage::Path path;
      path.moveTo(toScreen(polar(radius, from_degrees)));
      addArc(path, radius, from_degrees, to_degrees);
      return path;
    }

    visage::Path annulus(float inner_radius, float outer_radius) const {
      visage::Point center = toScreen({ kCenter, kCenter });
      visage::Path path;
      path.addCircle(center.x, center.y, outer_radius * scale_);
      if (inner_radius > 0.0f) {
        visage::Path inner;
        inner.addCircle(center.x, center.y, inner_radius * scale_);
        path = path.combine(inner.reversed(), visage::Path::FillRule::NonZero);
      }
      return path;
    }

    void dot(const visage::Point& center, float diameter) {
      if (diameter <= 0.0f)
        return;
      visage::Point position = toScreen(center);
      float width = diameter * scale_;
      canvas_.circle(position.x - 0.5f * width, position.y - 0.5f * width, width);
    }

    void strokeArc(float radius, float from_degrees, float to_degrees, float width) {
      if (width <= 0.0f)
        return;
      if (to_degrees - from_degrees < 0.05f) {
        dot(polar(radius, from_degrees), width);
        return;
      }
      canvas_.stroke(arc(radius, from_degrees, to_degrees), 0, 0, width_, height_, width * scale_);
    }

    void strokeTrimmed(const visage::Path& path, float from, float to, float width,
                       visage::Path::Join join = visage::Path::Join::Round) {
      if (width <= 0.0f || to <= from)
        return;
      float length = path.length();
      canvas_.stroke(path, 0, 0, width_, height_, width, join, visage::Path::EndCap::Round,
                     { to - from, length + width }, -from);
    }

    visage::Brush ringBrush() const {
      return visage::Brush::linear(visage::Gradient(kRingTopColor, kRingBottomColor),
                                   toScreen({ kCenter, kCenter - kRingRadius - 0.5f * kRingWidth }),
                                   toScreen({ kCenter, kCenter + kRingRadius + 0.5f * kRingWidth }));
    }

    visage::Path vShape() const {
      visage::Path path;
      path.loadSvgPath(kVPath);
      return toScreen(path);
    }

    void drawBody() {
      float radius = std::min(kBodyRadiusTrack(time_), kBandInnerTrack(time_));
      if (time_ > 1.35f || radius <= 0.0f)
        return;
      canvas_.setColor(kBodyColor);
      dot({ kCenter, kCenter }, 2.0f * radius);
    }

    void drawOuterRing() {
      canvas_.setColor(kTrackColor);
      if (time_ < 0.983f) {
        float start = kOuterRingStartTrack(time_);
        float end = kOuterRingEndTrack(time_);
        float radius = kOuterRingRadiusTrack(time_);
        if (end - start >= 359.9f)
          canvas_.fill(annulus(radius - 0.5f * kOuterRingWidth, radius + 0.5f * kOuterRingWidth));
        else
          strokeArc(radius, start, end, kOuterRingWidth);
        return;
      }

      float outer = kBandOuterTrack(time_);
      float inner = std::min(kBandInnerTrack(time_), outer);
      if (outer > 0.0f)
        canvas_.fill(annulus(inner, outer));
    }

    void drawTrack() {
      if (time_ > 0.983f)
        return;

      float start = kTrackStartTrack(time_);
      float end = kTrackEndTrack(time_);
      float width = kTrackWidthTrack(time_);
      float diameter = (2.0f * kRingRadius + width) * scale_;
      visage::Point corner = toScreen({ kCenter, kCenter }) -
                             visage::Point(0.5f * diameter, 0.5f * diameter);
      float center_radians = (0.5f * (start + end) - 90.0f) * kPi / 180.0f;
      float radians = std::max(0.0f, 0.5f * (end - start) * kPi / 180.0f);
      canvas_.setColor(kShadowColor);
      canvas_.roundedArcShadow(corner.x, corner.y, diameter, width * scale_, center_radians,
                               radians, kShadowWidth * scale_);

      canvas_.setColor(kTrackColor);
      strokeArc(kRingRadius, start, end, width);
    }

    void drawRing() {
      canvas_.setColor(ringBrush());
      if (time_ < 1.333f) {
        float start = kTrackStartTrack(time_);
        float end = std::min(kPointerAngleTrack(time_), kTrackEndTrack(time_));
        float close = kRingCloseTrack(time_);
        if (end > start)
          strokeArc(kRingRadius, start - close, end + close, kRingWidth);
        return;
      }

      float gap = kRingGapTrack(time_);
      if (gap <= 0.0f) {
        canvas_.fill(annulus(RingCutter::kInnerRadius, RingCutter::kOuterRadius));
        return;
      }

      static const RingCutter kCutter;
      for (const RingCutter::Piece& piece : kCutter.pieces(gap)) {
        visage::Path path = arc(RingCutter::kOuterRadius, piece.outer.start, piece.outer.end);
        path.lineTo(toScreen(polar(RingCutter::kInnerRadius, piece.inner.end)));
        addArc(path, RingCutter::kInnerRadius, piece.inner.end, piece.inner.start);
        path.close();
        canvas_.fill(path);
      }
    }

    void drawHook() {
      float head = kHookHeadTrack(time_);
      float tail = kHookTailTrack(time_);
      float width = kPointerWidthTrack(time_);
      if (time_ < 0.033f || width <= 0.0f)
        return;

      canvas_.setColor(kPointerColor);
      float angle = kPointerAngleTrack(time_);
      if (head < 0.5f) {
        dot(polar(kPointerStart, angle), width);
        return;
      }
      if (tail >= head)
        return;

      static const visage::Path kHook = hookPath();
      float rotation = angle - 360.0f - kKnobAngle;
      visage::Path hook = kHook.transformed(visage::Transform::rotation(rotation, { kCenter, kCenter }));
      strokeTrimmed(toScreen(hook), tail * scale_, head * scale_, width * scale_);
    }

    void drawV() {
      float reveal = kRevealTrack(time_);
      if (reveal <= 0.0f)
        return;

      visage::Gradient gradient = visage::Gradient::fromSampleFunction(128, [reveal](float t) {
        float distance = t * kRevealDistance;
        float alpha = clamp01((reveal - distance) / kRevealSoftness);
        return visage::Color(kVBottomColor).interpolateWith(visage::Color(kVTopColor), t).withAlpha(alpha);
      });
      canvas_.setColor(visage::Brush::radial(gradient, toScreen({ kCenter, 1653.305f }),
                                             kRevealDistance * scale_));
      canvas_.fill(vShape());
    }

    void drawEdges() {
      float width = kEdgeWidthTrack(time_) * scale_;
      if (width <= 0.0f)
        return;

      float outer = kOuterEdgeTrack(time_);
      float inner = kInnerEdgeTrack(time_);
      float top = kTopEdgeTrack(time_);
      canvas_.setColor(kPointerColor);
      for (bool right : { false, true }) {
        visage::Path outer_path = toScreen(outerEdgePath(right));
        visage::Path inner_path = toScreen(innerEdgePath(right));
        visage::Path top_path = toScreen(topEdgePath(right));
        strokeTrimmed(outer_path, 0.0f, outer * outer_path.length(), width);
        strokeTrimmed(inner_path, 0.0f, inner * inner_path.length(), width);
        strokeTrimmed(top_path, 0.0f, top * top_path.length(), width);
      }
    }

    void drawStreak(const visage::Path& curve, float tail, float head, float max_width) {
      static constexpr int kSamples = 48;
      static constexpr float kMinTaperLength = 525.0f;
      static constexpr int kCapSamples = 12;
      if (head - tail <= 0.0f)
        return;

      Polyline line(toScreen(curve));
      head = std::min(head * scale_, line.length());
      tail = std::min(tail * scale_, head);
      if (head - tail <= 0.0f)
        return;

      max_width *= scale_;
      float taper_length = std::max(head - tail, kMinTaperLength * scale_);
      std::vector<visage::Point> left;
      std::vector<visage::Point> right;
      visage::Point direction;
      for (int i = 0; i <= kSamples; ++i) {
        float t = i / static_cast<float>(kSamples);
        visage::Point point = line.point(tail + (head - tail) * t, &direction);
        visage::Point normal(-direction.y, direction.x);
        float half_width = 0.5f * max_width * (1.0f - (1.0f - t) * (head - tail) / taper_length);
        left.push_back(point + normal * half_width);
        right.push_back(point - normal * half_width);
      }

      visage::Path shape;
      shape.moveTo(left.front());
      for (size_t i = 1; i < left.size(); ++i)
        shape.lineTo(left[i]);

      visage::Point tip = line.point(head, &direction);
      visage::Point normal(-direction.y, direction.x);
      for (int i = 1; i < kCapSamples; ++i) {
        float angle = kPi * i / kCapSamples;
        shape.lineTo(tip + normal * (0.5f * max_width * std::cos(angle)) +
                     direction * (0.5f * max_width * std::sin(angle)));
      }
      for (auto it = right.rbegin(); it != right.rend(); ++it)
        shape.lineTo(*it);
      shape.close();
      shape.setFillRule(visage::Path::FillRule::NonZero);
      canvas_.fill(shape);
    }

    void drawStreaks() {
      canvas_.setColor(kPointerColor);
      for (bool right : { false, true }) {
        drawStreak(longStreakPath(right), kLongStreakTailTrack(time_), kLongStreakHeadTrack(time_), 42.0f);
        drawStreak(shortStreakPath(right), kShortStreakTailTrack(time_),
                   kShortStreakHeadTrack(time_), 35.0f);
      }
    }

    void drawWordmark() {
      static const std::vector<Letter> kLetters = wordmark();
      for (const Letter& letter : kLetters) {
        float t = (time_ - letter.start_time) / kTextFadeLength;
        if (t <= 0.0f)
          continue;

        float amount = easeOut(t);
        float drop = (1.0f - amount) * kTextDrop;
        canvas_.setColor(visage::Color(kTextColor).withAlpha(clamp01(t * 1.4f)));
        for (const auto& stroke : letter.strokes) {
          visage::Path path;
          path.moveTo(toScreen({ stroke.front().x, stroke.front().y - drop }));
          for (size_t i = 1; i < stroke.size(); ++i)
            path.lineTo(toScreen({ stroke[i].x, stroke[i].y - drop }));
          canvas_.stroke(path, 0, 0, width_, height_, kTextStroke * scale_,
                         visage::Path::Join::Miter, visage::Path::EndCap::Butt);
        }
      }
    }

    visage::Canvas& canvas_;
    float width_ = 0.0f;
    float height_ = 0.0f;
    float scale_ = 1.0f;
    visage::Point offset_;
    float time_ = 0.0f;
  };
}

class LogoAnimation : public visage::Frame {
public:
  static void drawAtTime(visage::Canvas& canvas, float width, float height, float time) {
    Scene(canvas, width, height).draw(time);
  }

  void draw(visage::Canvas& canvas) override {
    if (start_time_ < 0.0)
      start_time_ = canvas.time();

    double elapsed = std::fmod(canvas.time() - start_time_, kAnimationLength + kHoldLength);
    drawAtTime(canvas, width(), height(), static_cast<float>(elapsed));
    redraw();
  }

  void mouseDown(const visage::MouseEvent&) override { start_time_ = -1.0; }

private:
  double start_time_ = -1.0;
};

int runExample() {
  visage::ApplicationWindow app;
  LogoAnimation animation;
  app.addChild(&animation);
  animation.layout().setMargin(0);

  app.setTitle("Visage Logo Animation Example");
  if (visage::isMobileDevice())
    app.showMaximized();
  else
    app.show(735, 517);
  app.runEventLoop();
  return 0;
}
