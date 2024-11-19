/* Copyright Vital Audio, LLC
 *
 * visage is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * visage is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with visage.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <memory>

namespace visage {
  class EmojiRasterizerImpl;

  class EmojiRasterizer {
  public:
    static EmojiRasterizer& instance() {
      static EmojiRasterizer instance;
      return instance;
    }

    void drawIntoBuffer(char32_t emoji, int font_size, int write_width, unsigned int* dest,
                        int dest_width, int x, int y);

  private:
    EmojiRasterizer();
    ~EmojiRasterizer();

    std::unique_ptr<EmojiRasterizerImpl> impl_;
  };
}