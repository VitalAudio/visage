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

#include "color.h"

#include <iomanip>
#include <sstream>

namespace visage {
  std::string Color::encode() const {
    std::ostringstream stream;
    encode(stream);
    return stream.str();
  }

  void Color::encode(std::ostringstream& stream) const {
    stream << std::setprecision(std::numeric_limits<float>::max_digits10);

    for (auto value : values_)
      stream << value << " ";

    stream << std::to_string(hdr_) << std::endl;
  }

  void Color::decode(const std::string& data) {
    std::istringstream stream(data);
    decode(stream);
  }

  void Color::decode(std::istringstream& stream) {
    for (auto& value : values_)
      stream >> value;

    stream >> hdr_;
  }
}