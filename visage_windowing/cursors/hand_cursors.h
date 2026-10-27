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

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace visage::hand_cursors {
  static constexpr int kDesignSize = 32;
  static constexpr float kHotSpotX = 16.0f;
  static constexpr float kHotSpotY = 16.0f;

  struct Capsule {
    float x1, y1, x2, y2, radius;
  };

  struct Point2 {
    float x, y;
  };

  static constexpr float kPalmRadius = 1.5f;
  static constexpr Point2 kPalm[] = { { 10.0f, 15.0f }, { 25.0f, 15.0f }, { 24.6f, 22.0f },
                                      { 21.5f, 26.5f }, { 13.0f, 26.5f }, { 9.8f, 21.8f } };

  static constexpr Capsule kOpenHand[] = { { 10.0f, 7.0f, 10.0f, 17.0f, 2.0f },
                                           { 15.0f, 4.5f, 15.0f, 16.0f, 2.0f },
                                           { 20.0f, 5.5f, 20.0f, 16.0f, 2.0f },
                                           { 24.75f, 9.0f, 24.75f, 17.0f, 1.75f },
                                           { 5.2f, 14.2f, 9.6f, 21.0f, 2.0f } };

  static constexpr Capsule kClosedHand[] = { { 10.0f, 12.0f, 10.0f, 15.0f, 2.0f },
                                             { 15.0f, 11.0f, 15.0f, 15.0f, 2.0f },
                                             { 20.0f, 11.0f, 20.0f, 15.0f, 2.0f },
                                             { 24.75f, 12.5f, 24.75f, 16.0f, 1.75f },
                                             { 7.6f, 15.2f, 10.2f, 19.5f, 2.0f } };

  inline float segmentDistance(float x, float y, float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    float length_squared = dx * dx + dy * dy;
    float t = length_squared > 0.0f ? ((x - x1) * dx + (y - y1) * dy) / length_squared : 0.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    return std::hypot(x - x1 - t * dx, y - y1 - t * dy);
  }

  inline float palmDistance(float x, float y) {
    constexpr int kNumPoints = sizeof(kPalm) / sizeof(kPalm[0]);
    bool inside = true;
    float distance = 1e9f;
    for (int i = 0; i < kNumPoints; ++i) {
      Point2 a = kPalm[i];
      Point2 b = kPalm[(i + 1) % kNumPoints];
      distance = std::min(distance, segmentDistance(x, y, a.x, a.y, b.x, b.y));
      if ((b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x) < 0.0f)
        inside = false;
    }
    return (inside ? -distance : distance) - kPalmRadius;
  }

  template<int kNumCapsules>
  float handDistance(const Capsule (&capsules)[kNumCapsules], float x, float y) {
    float distance = palmDistance(x, y);
    for (const Capsule& capsule : capsules) {
      float capsule_distance = segmentDistance(x, y, capsule.x1, capsule.y1, capsule.x2, capsule.y2) -
                               capsule.radius;
      distance = std::min(distance, capsule_distance);
    }
    return distance;
  }

  inline std::vector<uint32_t> render(bool closed, int size) {
    static constexpr int kSamples = 4;
    float scale = size / static_cast<float>(kDesignSize);
    float outline = std::max(1.0f, std::round(scale)) / scale;

    std::vector<uint32_t> pixels(size * size, 0);
    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        int covered = 0;
        int filled = 0;
        for (int sample_y = 0; sample_y < kSamples; ++sample_y) {
          for (int sample_x = 0; sample_x < kSamples; ++sample_x) {
            float design_x = (x + (sample_x + 0.5f) / kSamples) / scale;
            float design_y = (y + (sample_y + 0.5f) / kSamples) / scale;
            float distance = closed ? handDistance(kClosedHand, design_x, design_y) :
                                      handDistance(kOpenHand, design_x, design_y);
            covered += distance < outline;
            filled += distance < 0.0f;
          }
        }

        if (covered == 0)
          continue;

        uint32_t alpha = (255 * covered + kSamples * kSamples / 2) / (kSamples * kSamples);
        uint32_t gray = (255 * filled + covered / 2) / covered;
        pixels[y * size + x] = (alpha << 24) | (gray << 16) | (gray << 8) | gray;
      }
    }
    return pixels;
  }

  inline int hotSpot(float design_position, int size) {
    return static_cast<int>(design_position * size / kDesignSize);
  }
}
