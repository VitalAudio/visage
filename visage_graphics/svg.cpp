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

#include "svg.h"

#include <unordered_map>

namespace visage {
  struct TagData {
    std::string name;
    std::string text;
    std::map<std::string, std::string> attributes;
    bool is_closing = false;
    bool is_self_closing = false;
    bool ignored = false;
  };

  struct Tag {
    TagData data;
    std::vector<Tag> children;
  };

  std::string unescape(std::string input, const std::string& from, const std::string& to) {
    size_t pos = 0;

    while ((pos = input.find(from, pos)) != std::string::npos) {
      input.replace(pos, from.length(), to);
      pos += to.length();
    }

    return input;
  }

  std::vector<std::string> splitArguments(const std::string& str) {
    std::string with_spaces = unescape(str, ",", " ");

    std::vector<std::string> tokens;
    std::string token;
    std::istringstream stream(with_spaces);
    while (std::getline(stream, token, ' ')) {
      if (!token.empty())
        tokens.push_back(token);

      std::string part;
      while (stream >> part)
        tokens.push_back(part);
    }
    return tokens;
  }

  struct CssSelector {
    std::string tag_name;
    std::string id;
    bool direct_child = false;
    std::vector<std::string> classes;
    std::vector<CssSelector> parents;
    // TODO support attributes, pseudo-classes, and chaining selectors

    bool matches(const Tag& tag) const {
      if (!tag_name.empty() && tag.data.name != tag_name)
        return false;

      if (!id.empty() && (tag.data.attributes.find("id") == tag.data.attributes.end() ||
                          tag.data.attributes.at("id") != id))
        return false;

      if (!classes.empty()) {
        if (tag.data.attributes.find("class") == tag.data.attributes.end())
          return false;
        std::vector<std::string> tag_classes = splitArguments(tag.data.attributes.at("class"));
        for (const auto& match_class : classes) {
          if (std::find(tag_classes.begin(), tag_classes.end(), match_class) == tag_classes.end())
            return false;
        }
      }

      return true;
    }
  };

  void SvgDrawable::draw(Canvas& canvas, float x, float y, float width, float height) const {
    if (!state.visible || state.opacity <= 0.0f)
      return;

    if (state.fill_opacity > 0.0f && !fill_brush.isNone())
      fill(canvas, x, y, width, height);
    if (state.stroke_opacity > 0.0f && state.stroke_width > 0.0f && !stroke_brush.isNone())
      stroke(canvas, x, y, width, height);
  }

  void SvgDrawable::fill(Canvas& canvas, float x, float y, float width, float height) const {
    canvas.setColor(fill_brush);
    canvas.fill(&path, x, y, width, height);
  }

  void SvgDrawable::stroke(Canvas& canvas, float x, float y, float width, float height) const {
    canvas.setColor(stroke_brush);
    canvas.fill(&stroke_path, x, y, width, height);
  }

  inline void tryReadFloat(float& result, const std::string& string) {
    try {
      result = std::stof(string);
    }
    catch (...) {
    }
  }

  void consumeWhiteSpace(const std::string& str, int& i) {
    while (i < str.size() && (str[i] == ' ' || str[i] == '\t' || str[i] == '\n' || str[i] == '\r'))
      i++;
  }

  void consumeTillEndTag(const std::string& str, int& i) {
    int size = str.size();
    while (i < size && str[i] != '>') {
      if (str[i] == '"' || str[i] == '\'') {
        char quote = str[i];
        i++;
        while (i < size && str[i] != quote)
          i++;
      }

      i++;
    }
    i++;
  }

  std::string consumeNonXmlTillNextTag(const std::string& str, int& i) {
    std::string result;
    int current = i;
    int size = str.size();
    while (i < size && str[i] != '<') {
      if (str[i] == '/' && i + 1 < size && str[i + 1] == '*') {
        auto end_pos = str.find("*/", i);
        if (end_pos == std::string::npos) {
          VISAGE_ASSERT(false);
          i = size;
          return result;
        }
        result += str.substr(current, end_pos - current);
        i = end_pos + 2;
        current = i;
      }
      i++;
    }
    if (current < i)
      result += str.substr(current, i - current);

    while (i < size && str[i] != '>')
      i++;
    i++;
    return result;
  }

  std::pair<std::string, std::string> parseAttribute(const std::string& str, int& i) {
    std::string key, value;
    consumeWhiteSpace(str, i);
    while (i < str.size() && str[i] != '=' && str[i] != ' ' && str[i] != '\t' && str[i] != '>' &&
           str[i] != '/')
      key += str[i++];

    if (key.empty() || i >= str.size() || str[i] != '=')
      return { "", "" };

    i++;
    char quote_char = str[i++];
    VISAGE_ASSERT(quote_char == '"' || quote_char == '\'');

    int end = str.find(quote_char, i);
    if (end == std::string::npos) {
      VISAGE_ASSERT(false);
      return { "", "" };
    }

    value = str.substr(i, end - i);
    i = end + 1;
    value = unescape(value, "&quot;", "\"");
    value = unescape(value, "&apos;", "'");
    value = unescape(value, "&lt;", "<");
    value = unescape(value, "&gt;", ">");
    value = unescape(value, "&amp;", "&");

    return { key, value };
  }

  TagData parseTag(const std::string& str, int& i) {
    TagData tag_data;
    if (i >= str.size())
      return tag_data;

    i = str.find('<', i);
    if (i == std::string::npos)
      return tag_data;

    i++;
    if (i >= str.size()) {
      VISAGE_ASSERT(false);
      return tag_data;
    }

    if (str[i] == '!' || str[i] == '?') {
      tag_data.ignored = true;
      consumeTillEndTag(str, i);
      return tag_data;
    }

    if (str[i] == '/') {
      tag_data.is_closing = true;
      i++;
    }

    while (i < str.size() && str[i] != ' ' && str[i] != '\t' && str[i] != '\n' && str[i] != '\r' &&
           str[i] != '>' && str[i] != '/')
      tag_data.name += str[i++];

    if (tag_data.is_closing) {
      VISAGE_ASSERT(!tag_data.name.empty() && i < str.size() && str[i] == '>');
      return tag_data;
    }

    auto attribute = parseAttribute(str, i);
    while (!attribute.first.empty()) {
      tag_data.attributes[attribute.first] = attribute.second;
      attribute = parseAttribute(str, i);
    }

    if (i >= str.size())
      return tag_data;

    if (str[i] == '/') {
      tag_data.is_self_closing = true;
      i++;
    }
    if (str[i] == '>')
      i++;
    else
      VISAGE_ASSERT(false);

    return tag_data;
  }

  bool isNonXmlTag(const std::string& tag_name) {
    return tag_name == "script" || tag_name == "style" || tag_name == "title" || tag_name == "desc";
  }

  Tag parseTagTree(const std::string& str, int& i) {
    Tag tag;
    tag.data = parseTag(str, i);

    if (tag.data.ignored || tag.data.is_self_closing || tag.data.is_closing || tag.data.name.empty())
      return tag;

    if (isNonXmlTag(tag.data.name)) {
      tag.data.text = consumeNonXmlTillNextTag(str, i);
      return tag;
    }

    Tag child = parseTagTree(str, i);
    while (!child.data.is_closing) {
      if (child.data.ignored || !child.data.name.empty())
        tag.children.push_back(child);

      child = parseTagTree(str, i);
    }

    VISAGE_ASSERT(tag.data.name == child.data.name);
    return tag;
  }

  Color translateColor(const std::string& color) {
    static const std::unordered_map<std::string, Color> named_colors = {
      { "aliceblue", Color(1, 0.941f, 0.973f, 1.0f) },
      { "antiquewhite", Color(1, 0.980f, 0.922f, 0.843f) },
      { "aqua", Color(1, 0.0f, 1.0f, 1.0f) },
      { "aquamarine", Color(1, 0.498f, 1.0f, 0.831f) },
      { "azure", Color(1, 0.941f, 1.0f, 1.0f) },
      { "beige", Color(1, 0.961f, 0.961f, 0.863f) },
      { "bisque", Color(1, 1.0f, 0.894f, 0.769f) },
      { "black", Color(1, 0.0f, 0.0f, 0.0f) },
      { "blanchedalmond", Color(1, 1.0f, 0.922f, 0.804f) },
      { "blue", Color(1, 0.0f, 0.0f, 1.0f) },
      { "blueviolet", Color(1, 0.541f, 0.169f, 0.886f) },
      { "brown", Color(1, 0.647f, 0.165f, 0.165f) },
      { "burlywood", Color(1, 0.871f, 0.722f, 0.529f) },
      { "cadetblue", Color(1, 0.373f, 0.620f, 0.627f) },
      { "chartreuse", Color(1, 0.498f, 1.0f, 0.0f) },
      { "chocolate", Color(1, 0.824f, 0.412f, 0.118f) },
      { "coral", Color(1, 1.0f, 0.498f, 0.314f) },
      { "cornflowerblue", Color(1, 0.392f, 0.584f, 0.929f) },
      { "cornsilk", Color(1, 1.0f, 0.973f, 0.863f) },
      { "crimson", Color(1, 0.863f, 0.078f, 0.235f) },
      { "cyan", Color(1, 0.0f, 1.0f, 1.0f) },
      { "darkblue", Color(1, 0.0f, 0.0f, 0.545f) },
      { "darkcyan", Color(1, 0.0f, 0.545f, 0.545f) },
      { "darkgoldenrod", Color(1, 0.722f, 0.525f, 0.043f) },
      { "darkgray", Color(1, 0.663f, 0.663f, 0.663f) },
      { "darkgrey", Color(1, 0.663f, 0.663f, 0.663f) },
      { "darkgreen", Color(1, 0.0f, 0.392f, 0.0f) },
      { "darkkhaki", Color(1, 0.741f, 0.718f, 0.420f) },
      { "darkmagenta", Color(1, 0.545f, 0.0f, 0.545f) },
      { "darkolivegreen", Color(1, 0.333f, 0.420f, 0.184f) },
      { "darkorange", Color(1, 1.0f, 0.549f, 0.0f) },
      { "darkorchid", Color(1, 0.600f, 0.196f, 0.800f) },
      { "darkred", Color(1, 0.545f, 0.0f, 0.0f) },
      { "darksalmon", Color(1, 0.914f, 0.588f, 0.478f) },
      { "darkseagreen", Color(1, 0.561f, 0.737f, 0.561f) },
      { "darkslateblue", Color(1, 0.282f, 0.239f, 0.545f) },
      { "darkslategray", Color(1, 0.184f, 0.310f, 0.310f) },
      { "darkslategrey", Color(1, 0.184f, 0.310f, 0.310f) },
      { "darkturquoise", Color(1, 0.0f, 0.808f, 0.820f) },
      { "darkviolet", Color(1, 0.580f, 0.0f, 0.827f) },
      { "deeppink", Color(1, 1.0f, 0.078f, 0.576f) },
      { "deepskyblue", Color(1, 0.0f, 0.749f, 1.0f) },
      { "dimgray", Color(1, 0.412f, 0.412f, 0.412f) },
      { "dimgrey", Color(1, 0.412f, 0.412f, 0.412f) },
      { "dodgerblue", Color(1, 0.118f, 0.565f, 1.0f) },
      { "firebrick", Color(1, 0.698f, 0.133f, 0.133f) },
      { "floralwhite", Color(1, 1.0f, 0.980f, 0.941f) },
      { "forestgreen", Color(1, 0.133f, 0.545f, 0.133f) },
      { "fuchsia", Color(1, 1.0f, 0.0f, 1.0f) },
      { "gainsboro", Color(1, 0.863f, 0.863f, 0.863f) },
      { "ghostwhite", Color(1, 0.973f, 0.973f, 1.0f) },
      { "gold", Color(1, 1.0f, 0.843f, 0.0f) },
      { "goldenrod", Color(1, 0.855f, 0.647f, 0.125f) },
      { "gray", Color(1, 0.502f, 0.502f, 0.502f) },
      { "grey", Color(1, 0.502f, 0.502f, 0.502f) },
      { "green", Color(1, 0.0f, 0.502f, 0.0f) },
      { "greenyellow", Color(1, 0.678f, 1.0f, 0.184f) },
      { "honeydew", Color(1, 0.941f, 1.0f, 0.941f) },
      { "hotpink", Color(1, 1.0f, 0.412f, 0.706f) },
      { "indianred", Color(1, 0.804f, 0.361f, 0.361f) },
      { "indigo", Color(1, 0.294f, 0.0f, 0.510f) },
      { "ivory", Color(1, 1.0f, 1.0f, 0.941f) },
      { "khaki", Color(1, 0.941f, 0.902f, 0.549f) },
      { "lavender", Color(1, 0.902f, 0.902f, 0.980f) },
      { "lavenderblush", Color(1, 1.0f, 0.941f, 0.961f) },
      { "lawngreen", Color(1, 0.486f, 0.988f, 0.0f) },
      { "lemonchiffon", Color(1, 1.0f, 0.980f, 0.804f) },
      { "lightblue", Color(1, 0.678f, 0.847f, 0.902f) },
      { "lightcoral", Color(1, 0.941f, 0.502f, 0.502f) },
      { "lightcyan", Color(1, 0.878f, 1.0f, 1.0f) },
      { "lightgoldenrodyellow", Color(1, 0.980f, 0.980f, 0.824f) },
      { "lightgray", Color(1, 0.827f, 0.827f, 0.827f) },
      { "lightgrey", Color(1, 0.827f, 0.827f, 0.827f) },
      { "lightgreen", Color(1, 0.565f, 0.933f, 0.565f) },
      { "lightpink", Color(1, 1.0f, 0.714f, 0.757f) },
      { "lightsalmon", Color(1, 1.0f, 0.627f, 0.478f) },
      { "lightseagreen", Color(1, 0.125f, 0.698f, 0.667f) },
      { "lightskyblue", Color(1, 0.529f, 0.808f, 0.980f) },
      { "lightslategray", Color(1, 0.467f, 0.533f, 0.600f) },
      { "lightslategrey", Color(1, 0.467f, 0.533f, 0.600f) },
      { "lightsteelblue", Color(1, 0.690f, 0.769f, 0.871f) },
      { "lightyellow", Color(1, 1.0f, 1.0f, 0.878f) },
      { "lime", Color(1, 0.0f, 1.0f, 0.0f) },
      { "limegreen", Color(1, 0.196f, 0.804f, 0.196f) },
      { "linen", Color(1, 0.980f, 0.941f, 0.902f) },
      { "magenta", Color(1, 1.0f, 0.0f, 1.0f) },
      { "maroon", Color(1, 0.502f, 0.0f, 0.0f) },
      { "mediumaquamarine", Color(1, 0.400f, 0.804f, 0.667f) },
      { "mediumblue", Color(1, 0.0f, 0.0f, 0.804f) },
      { "mediumorchid", Color(1, 0.729f, 0.333f, 0.827f) },
      { "mediumpurple", Color(1, 0.576f, 0.439f, 0.859f) },
      { "mediumseagreen", Color(1, 0.235f, 0.702f, 0.443f) },
      { "mediumslateblue", Color(1, 0.482f, 0.408f, 0.933f) },
      { "mediumspringgreen", Color(1, 0.0f, 0.980f, 0.604f) },
      { "mediumturquoise", Color(1, 0.282f, 0.820f, 0.800f) },
      { "mediumvioletred", Color(1, 0.780f, 0.082f, 0.522f) },
      { "midnightblue", Color(1, 0.098f, 0.098f, 0.439f) },
      { "mintcream", Color(1, 0.961f, 1.0f, 0.980f) },
      { "mistyrose", Color(1, 1.0f, 0.894f, 0.882f) },
      { "moccasin", Color(1, 1.0f, 0.894f, 0.710f) },
      { "navajowhite", Color(1, 1.0f, 0.871f, 0.678f) },
      { "navy", Color(1, 0.0f, 0.0f, 0.502f) },
      { "oldlace", Color(1, 0.992f, 0.961f, 0.902f) },
      { "olive", Color(1, 0.502f, 0.502f, 0.0f) },
      { "olivedrab", Color(1, 0.420f, 0.557f, 0.137f) },
      { "orange", Color(1, 1.0f, 0.647f, 0.0f) },
      { "orangered", Color(1, 1.0f, 0.271f, 0.0f) },
      { "orchid", Color(1, 0.855f, 0.439f, 0.839f) },
      { "palegoldenrod", Color(1, 0.933f, 0.910f, 0.667f) },
      { "palegreen", Color(1, 0.596f, 0.984f, 0.596f) },
      { "paleturquoise", Color(1, 0.686f, 0.933f, 0.933f) },
      { "palevioletred", Color(1, 0.859f, 0.439f, 0.576f) },
      { "papayawhip", Color(1, 1.0f, 0.937f, 0.835f) },
      { "peachpuff", Color(1, 1.0f, 0.855f, 0.725f) },
      { "peru", Color(1, 0.804f, 0.522f, 0.247f) },
      { "pink", Color(1, 1.0f, 0.753f, 0.796f) },
      { "plum", Color(1, 0.867f, 0.627f, 0.867f) },
      { "powderblue", Color(1, 0.690f, 0.878f, 0.902f) },
      { "purple", Color(1, 0.502f, 0.0f, 0.502f) },
      { "red", Color(1, 1.0f, 0.0f, 0.0f) },
      { "rosybrown", Color(1, 0.737f, 0.561f, 0.561f) },
      { "royalblue", Color(1, 0.255f, 0.412f, 0.882f) },
      { "saddlebrown", Color(1, 0.545f, 0.271f, 0.075f) },
      { "salmon", Color(1, 0.980f, 0.502f, 0.447f) },
      { "sandybrown", Color(1, 0.957f, 0.643f, 0.376f) },
      { "seagreen", Color(1, 0.180f, 0.545f, 0.341f) },
      { "seashell", Color(1, 1.0f, 0.961f, 0.933f) },
      { "sienna", Color(1, 0.627f, 0.322f, 0.176f) },
      { "silver", Color(1, 0.753f, 0.753f, 0.753f) },
      { "skyblue", Color(1, 0.529f, 0.808f, 0.922f) },
      { "slateblue", Color(1, 0.416f, 0.353f, 0.804f) },
      { "slategray", Color(1, 0.439f, 0.502f, 0.565f) },
      { "slategrey", Color(1, 0.439f, 0.502f, 0.565f) },
      { "snow", Color(1, 1.0f, 0.980f, 0.980f) },
      { "springgreen", Color(1, 0.0f, 1.0f, 0.498f) },
      { "steelblue", Color(1, 0.275f, 0.510f, 0.706f) },
      { "tan", Color(1, 0.824f, 0.706f, 0.549f) },
      { "teal", Color(1, 0.0f, 0.502f, 0.502f) },
      { "thistle", Color(1, 0.847f, 0.749f, 0.847f) },
      { "tomato", Color(1, 1.0f, 0.388f, 0.278f) },
      { "turquoise", Color(1, 0.251f, 0.878f, 0.816f) },
      { "violet", Color(1, 0.933f, 0.510f, 0.933f) },
      { "wheat", Color(1, 0.961f, 0.871f, 0.702f) },
      { "white", Color(1, 1.0f, 1.0f, 1.0f) },
      { "whitesmoke", Color(1, 0.961f, 0.961f, 0.961f) },
      { "yellow", Color(1, 1.0f, 1.0f, 0.0f) },
      { "yellowgreen", Color(1, 0.604f, 0.804f, 0.196f) },
      { "transparent", Color(0, 0.0f, 0.0f, 0.0f) }
    };

    auto it = named_colors.find(color);
    if (it != named_colors.end())
      return it->second;

    return Color::fromHexString(color);
  }

  float parseNumber(const std::string& str, float max) {
    auto percent_pos = str.find('%');
    if (percent_pos != std::string::npos)
      max = 100.0f;

    try {
      return std::stof(str) / max;
    }
    catch (...) {
      return 0.0f;
    }
  }

  Color parseStopColor(Tag& tag) {
    Color color;
    if (tag.data.attributes.count("stop-color"))
      color = translateColor(tag.data.attributes.at("stop-color"));
    if (tag.data.attributes.count("stop-opacity"))
      color = color.withAlpha(parseNumber(tag.data.attributes.at("stop-opacity"), 1.0f));
    if (tag.data.attributes.count("style")) {
      std::string style = tag.data.attributes.at("style");
      std::stringstream ss(style);
      std::string item;
      while (std::getline(ss, item, ';')) {
        auto pos = item.find(':');
        if (pos != std::string::npos) {
          std::string key = item.substr(0, pos);
          std::string value = item.substr(pos + 1);
          if (key == "stop-color")
            color = translateColor(value);
          else if (key == "stop-opacity")
            color = color.withAlpha(parseNumber(value, 1.0f));
        }
      }
    }
    return color;
  }

  void removeWhitespace(std::string& string) {
    constexpr auto is_whitespace = [](char c) { return std::isspace(c); };
    string.erase(std::remove_if(string.begin(), string.end(), is_whitespace), string.end());
  }

  std::vector<std::string> parseFunctionTokens(const std::string& function_string, int& pos) {
    size_t start = function_string.find_first_not_of(" \t\n\r", pos);
    if (start == std::string::npos)
      return {};

    size_t end = function_string.find('(', start);
    if (end == std::string::npos)
      return { function_string };

    std::string function_name = function_string.substr(start, end - start);
    size_t close = function_string.find(')', end);
    if (close == std::string::npos)
      return {};

    pos = close + 1;
    auto result = splitArguments(function_string.substr(end + 1, close - end - 1));
    result.insert(result.begin(), function_name);
    return result;
  }

  Matrix parseTransform(const std::string& transform_string) {
    Matrix matrix;
    std::istringstream ss(transform_string);
    int pos = 0;
    while (pos < transform_string.size()) {
      auto tokens = parseFunctionTokens(transform_string, pos);
      if (tokens.size() < 2)
        break;

      std::vector<float> args;
      for (int i = 1; i < tokens.size(); ++i) {
        try {
          args.push_back(std::stof(tokens[i]));
        }
        catch (...) {
          return matrix;
        }
      }

      if (tokens[0] == "translate" && args.size() > 0) {
        float y = args.size() > 1 ? args[1] : args[0];
        matrix = matrix * Matrix::translation(args[0], y);
      }
      else if (tokens[0] == "scale" && args.size() > 0) {
        float y = args.size() > 1 ? args[1] : args[0];
        matrix = matrix * Matrix::scale(args[0], y);
      }
      else if (tokens[0] == "rotate" && args.size() > 0) {
        if (args.size() > 2)
          matrix = matrix * Matrix::rotation(args[0], { args[1], args[2] });
        else
          matrix = matrix * Matrix::rotation(args[0]);
      }
      else if (tokens[0] == "skewX" && args.size() > 0)
        matrix = matrix * Matrix::skewX(args[0]);
      else if (tokens[0] == "skewY" && args.size() > 0)
        matrix = matrix * Matrix::skewY(args[0]);
      else if (tokens[0] == "matrix" && args.size() > 5)
        matrix = matrix * Matrix(args[0], args[2], args[4], args[1], args[3], args[5]);
    }
    return matrix;
  }

  GradientDef parseGradient(const Svg::ViewSettings& view, Tag& tag) {
    GradientDef gradient_def;
    if (tag.data.attributes.count("x1")) {
      gradient_def.x1 = parseNumber(tag.data.attributes.at("x1"), 1.0f);
      gradient_def.x1_ratio = tag.data.attributes.at("x1").find('%') != std::string::npos;
    }
    if (tag.data.attributes.count("y1")) {
      gradient_def.y1 = parseNumber(tag.data.attributes.at("y1"), 1.0f);
      gradient_def.y1_ratio = tag.data.attributes.at("y1").find('%') != std::string::npos;
    }
    if (tag.data.attributes.count("x2")) {
      gradient_def.x2 = parseNumber(tag.data.attributes.at("x2"), 1.0f);
      gradient_def.x2_ratio = tag.data.attributes.at("x2").find('%') != std::string::npos;
    }
    if (tag.data.attributes.count("y2")) {
      gradient_def.y2 = parseNumber(tag.data.attributes.at("y2"), 1.0f);
      gradient_def.y2_ratio = tag.data.attributes.at("y2").find('%') != std::string::npos;
    }
    if (tag.data.attributes.count("spreadMethod")) {
      std::string spread_method = tag.data.attributes.at("spreadMethod");
      gradient_def.gradient.setRepeat(spread_method == "repeat" || spread_method == "reflect");
      gradient_def.gradient.setReflect(spread_method == "reflect");
    }
    if (tag.data.attributes.count("gradientTransform"))
      gradient_def.transform = parseTransform(tag.data.attributes.at("gradientTransform"));
    if (tag.data.attributes.count("gradientUnits"))
      gradient_def.user_space = tag.data.attributes.at("gradientUnits") == "userSpaceOnUse";

    for (auto& tag : tag.children) {
      if (tag.data.name != "stop" || !tag.data.attributes.count("offset"))
        continue;

      float offset = parseNumber(tag.data.attributes.at("offset"), 1.0f);
      gradient_def.gradient.addColorStop(parseStopColor(tag), offset);
    }

    return gradient_def;
  }

  void collectDefs(std::vector<Tag>& tags, std::map<std::string, Tag>& defs) {
    for (auto& tag : tags) {
      if (tag.data.attributes.count("id") && !tag.data.attributes.at("id").empty()) {
        std::string id = "#" + tag.data.attributes.at("id");
        defs[id] = tag;
        defs[id].data.attributes.erase("id");
      }
      collectDefs(tag.children, defs);
    }
  }

  void collectGradients(const Svg::ViewSettings& view, std::vector<Tag>& tags,
                        std::map<std::string, GradientDef>& gradients) {
    for (auto& tag : tags) {
      if (tag.data.attributes.count("id") && !tag.data.attributes.at("id").empty()) {
        std::string id = tag.data.attributes.at("id");
        if (tag.data.name == "linearGradient" || tag.data.name == "radialGradient")
          gradients[id] = parseGradient(view, tag);
      }
      collectGradients(view, tag.children, gradients);
    }
  }

  void resolveUses(std::vector<Tag>& tags, const std::map<std::string, Tag> defs) {
    auto use_tag = [&defs](Tag& target, const std::string& reference_id) {
      if (defs.count(reference_id) == 0)
        return;

      const Tag& reference = defs.at(reference_id);
      for (const auto& attr : reference.data.attributes) {
        if (!target.data.attributes.count(attr.first))
          target.data.attributes[attr.first] = attr.second;
      }
      target.children.insert(target.children.begin(), reference.children.begin(),
                             reference.children.end());

      if (target.data.name.empty())
        target.data.name = reference.data.name;
    };

    for (auto& tag : tags) {
      if (tag.data.name == "use") {
        Tag child;
        if (tag.data.attributes.count("href"))
          use_tag(child, tag.data.attributes.at("href"));
        else if (tag.data.attributes.count("xlink:href"))
          use_tag(child, tag.data.attributes.at("xlink:href"));
        tag.children.push_back(child);
      }
      else if (tag.data.attributes.count("xlink:href"))
        use_tag(tag, tag.data.attributes.at("xlink:href"));

      resolveUses(tag.children, defs);
    }
  }

  CssSelector parseCssSelector(const std::string& selectors) {
    auto edited = unescape(selectors, ">", " > ");
    bool direct_child = false;
    std::vector<CssSelector> chained_selectors;
    std::stringstream ss(edited);
    std::string selector_text;

    while (std::getline(ss, selector_text, ' ')) {
      removeWhitespace(selector_text);
      if (selector_text.empty())
        continue;
      if (edited == ">") {
        direct_child = true;
        continue;
      }
      chained_selectors.push_back({});
      auto& selector = chained_selectors.back();
      selector.direct_child = direct_child;

      selector_text = unescape(unescape(selector_text, "#", " #"), ".", " .");
      std::stringstream selector_stream(selector_text);
      std::string item;
      while (std::getline(selector_stream, item, ' ')) {
        removeWhitespace(item);
        if (item[0] == '#')
          selector.id = item.substr(1);
        else if (item[0] == '.')
          selector.classes.push_back(item.substr(1));
        else
          selector.tag_name = item;
      }

      direct_child = false;
    }

    for (int i = 0; i + 1 < chained_selectors.size(); ++i)
      chained_selectors[i + 1].parents.push_back(chained_selectors[i]);

    return chained_selectors.back();
  }

  void parseCssStyle(std::string& style, std::vector<std::pair<CssSelector, std::string>>& style_lookup) {
    size_t pos = 0;
    while (pos < style.size()) {
      size_t brace_open = style.find('{', pos);
      if (brace_open == std::string::npos)
        break;

      size_t brace_close = style.find('}', brace_open);
      if (brace_close == std::string::npos)
        break;

      std::string selectors = style.substr(pos, brace_open - pos);
      removeWhitespace(selectors);
      std::stringstream ss(selectors);

      std::string rules = style.substr(brace_open + 1, brace_close - brace_open - 1);
      std::string item;
      while (std::getline(ss, item, ',')) {
        if (item.empty())
          continue;

        style_lookup.emplace_back(parseCssSelector(item), rules);
      }
      pos = brace_close + 1;
    }
  }

  void loadStyleTags(std::vector<Tag>& tags, std::vector<std::pair<CssSelector, std::string>>& style_lookup) {
    for (auto& tag : tags) {
      if (tag.data.name == "style")
        parseCssStyle(tag.data.text, style_lookup);
      else
        loadStyleTags(tag.children, style_lookup);
    }
  }

  GradientDef parseColor(DrawableState& state, std::string& color,
                         const std::map<std::string, GradientDef>& gradients) {
    removeWhitespace(color);

    if (color == "none")
      return {};
    if (color[0] == '#')
      return GradientDef(Color::fromHexString(color.substr(1)));

    int pos = 0;
    auto tokens = parseFunctionTokens(color, pos);
    if (tokens.size() == 0)
      return {};
    if (tokens.size() == 1)
      return GradientDef(translateColor(tokens[0]));

    if (tokens[0].substr(0, 3) == "rgb" && tokens.size() > 3) {
      float alpha = tokens.size() > 4 ? parseNumber(tokens[4], 1.0f) : 1.0f;
      return GradientDef(Color(alpha, parseNumber(tokens[1], 255.0f),
                               parseNumber(tokens[2], 255.0f), parseNumber(tokens[3], 255.0f)));
    }
    if (tokens[0].substr(0, 3) == "hsl" && tokens.size() > 3) {
      float alpha = tokens.size() > 4 ? parseNumber(tokens[4], 1.0f) : 1.0f;
      return GradientDef(Color::fromAHSV(alpha, parseNumber(tokens[1], 360.0f),
                                         parseNumber(tokens[2], 100.0f), parseNumber(tokens[3], 100.0f)));
    }
    if (tokens[0].substr(0, 3) == "url" && tokens.size() > 1) {
      if (tokens[1].size() > 1 && tokens[1][0] == '#') {
        std::string id = tokens[1].substr(1);
        if (gradients.count(id) > 0)
          return gradients.at(id);
      }
    }
    return {};
  }

  Path::EndCap parseStrokeEndCap(const std::string& value) {
    if (value == "round")
      return Path::EndCap::Round;
    if (value == "square")
      return Path::EndCap::Square;

    return Path::EndCap::Butt;
  }

  Path::Join parseStrokeJoin(const std::string& value) {
    if (value == "round")
      return Path::Join::Round;
    if (value == "bevel")
      return Path::Join::Bevel;

    return Path::Join::Miter;
  }

  std::vector<std::pair<float, bool>> parseStrokeDashArray(const std::string& value) {
    std::vector<std::pair<float, bool>> array;
    auto args = splitArguments(value);
    if (args.empty() || args[0] == "none")
      return array;

    for (const auto& arg : args) {
      if (arg.empty())
        continue;

      bool is_ratio = arg.find('%') != std::string::npos;
      float number = parseNumber(arg, 1.0f);
      array.emplace_back(number, is_ratio);
    }

    return array;
  }

  void parseStyleAttribute(DrawableState& state, const std::string& style,
                           const std::map<std::string, GradientDef>& gradients) {
    std::stringstream stream(style);
    std::string line;

    while (std::getline(stream, line, ';')) {
      auto pos = line.find(':');
      if (pos != std::string::npos) {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        removeWhitespace(key);
        if (key == "opacity")
          tryReadFloat(state.opacity, value);
        else if (key == "fill")
          state.fill_gradient = parseColor(state, value, gradients);
        else if (key == "fill-rule")
          state.non_zero_fill = value == "nonzero";
        else if (key == "fill-opacity")
          tryReadFloat(state.fill_opacity, value);
        else if (key == "stroke")
          state.stroke_gradient = parseColor(state, value, gradients);
        else if (key == "stroke-opacity")
          tryReadFloat(state.stroke_opacity, value);
        else if (key == "stroke-width")
          tryReadFloat(state.stroke_width, value);
        else if (key == "visibility")
          state.visible = value != "hidden";
        else if (key == "display")
          state.visible = value != "none";
        else if (key == "transform")
          state.local_transform = parseTransform(value);
        else if (key == "transform-origin") {
          auto args = splitArguments(value);
          if (!args.empty()) {
            for (auto& arg : args) {
              if (arg == "center")
                arg = "50%";
            }

            state.tranform_origin_x = parseNumber(args[0], 1.0f);
            if (args.size() > 1)
              state.tranform_origin_y = parseNumber(args[1], 1.0f);

            state.transform_ratio_x = args[0].find('%') != std::string::npos;
            state.transform_ratio_y = args.size() > 1 && args[1].find('%') != std::string::npos;
          }
        }
        else if (key == "stroke-linecap")
          state.stroke_end_cap = parseStrokeEndCap(value);
        else if (key == "stroke-linejoin")
          state.stroke_join = parseStrokeJoin(value);
        else if (key == "stroke-dasharray")
          state.stroke_dasharray = parseStrokeDashArray(value);
        else if (key == "stroke-dashoffset") {
          state.stroke_dashoffset = parseNumber(value, 1.0f);
          state.stroke_dashoffset_ratio = value.find('%') != std::string::npos;
        }
        else if (key == "stroke-miterlimit")
          tryReadFloat(state.stroke_miter_limit, value);
      }
    }

    state.scale_transform = state.scale_transform * state.local_transform.withNoTranslation();
  }

  void loadDrawableState(Tag& tag, std::vector<DrawableState>& state_stack,
                         const std::map<std::string, GradientDef>& gradients) {
    auto& state = state_stack.back();
    state.local_transform = Matrix::identity();
    state.tranform_origin_x = 0;
    state.tranform_origin_y = 0;

    float x = 0.0, y = 0.0f;
    for (auto& attribute : tag.data.attributes) {
      if (attribute.first == "transform")
        state.local_transform = parseTransform(attribute.second);
      else if (attribute.first == "x")
        tryReadFloat(x, attribute.second);
      else if (attribute.first == "y")
        tryReadFloat(y, attribute.second);
      else if (attribute.first == "fill")
        state.fill_gradient = parseColor(state, attribute.second, gradients);
      else if (attribute.first == "fill-rule")
        state.non_zero_fill = attribute.second == "nonzero";
      else if (attribute.first == "fill-opacity")
        tryReadFloat(state.fill_opacity, attribute.second);
      else if (attribute.first == "stroke")
        state.stroke_gradient = parseColor(state, attribute.second, gradients);
      else if (attribute.first == "stroke-opacity")
        tryReadFloat(state.stroke_opacity, attribute.second);
      else if (attribute.first == "stroke-width")
        tryReadFloat(state.stroke_width, attribute.second);
      else if (attribute.first == "stroke-linecap")
        state.stroke_end_cap = parseStrokeEndCap(attribute.second);
      else if (attribute.first == "stroke-linejoin")
        state.stroke_join = parseStrokeJoin(attribute.second);
      else if (attribute.first == "stroke-dasharray")
        state.stroke_dasharray = parseStrokeDashArray(attribute.second);
      else if (attribute.first == "stroke-dashoffset") {
        state.stroke_dashoffset = parseNumber(attribute.second, 1.0f);
        state.stroke_dashoffset_ratio = attribute.second.find('%') != std::string::npos;
      }
      else if (attribute.first == "stroke-miterlimit")
        tryReadFloat(state.stroke_miter_limit, attribute.second);
      else if (attribute.first == "visibility")
        state.visible = attribute.second != "hidden";
      else if (attribute.first == "style")
        parseStyleAttribute(state, attribute.second, gradients);
      else if (attribute.first == "opacity")
        tryReadFloat(state.opacity, attribute.second);
    }

    if (x || y)
      state.local_transform = state.local_transform * Matrix::translation(x, y);
    state.scale_transform = state.scale_transform * state.local_transform.withNoTranslation();
  }

  std::unique_ptr<SvgDrawable> loadDrawable(const Svg::ViewSettings& view, Tag& tag,
                                            std::vector<DrawableState>& state_stack,
                                            const std::map<std::string, GradientDef>& gradients) {
    auto& state = state_stack.back();
    float width = 0.0f;
    float height = 0.0f;
    if (tag.data.attributes.count("width"))
      width = parseNumber(tag.data.attributes.at("width"), 1.0f);
    if (tag.data.attributes.count("height"))
      height = parseNumber(tag.data.attributes.at("height"), 1.0f);

    Path path;
    path.setResolutionTransform(state.scale_transform);
    if (state.non_zero_fill)
      path.setFillRule(Path::FillRule::NonZero);
    else
      path.setFillRule(Path::FillRule::EvenOdd);

    if (tag.data.name == "path" && tag.data.attributes.count("d"))
      path.parseSvgPath(tag.data.attributes.at("d"));
    else if (tag.data.name == "line") {
      float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
      if (tag.data.attributes.count("x1"))
        x1 = parseNumber(tag.data.attributes.at("x1"), 1.0f);
      if (tag.data.attributes.count("y1"))
        y1 = parseNumber(tag.data.attributes.at("y1"), 1.0f);
      if (tag.data.attributes.count("x2"))
        x2 = parseNumber(tag.data.attributes.at("x2"), 1.0f);
      if (tag.data.attributes.count("y2"))
        y2 = parseNumber(tag.data.attributes.at("y2"), 1.0f);
      path.moveTo(x1, y1);
      path.lineTo(x2, y2);
    }
    else if (tag.data.name == "polygon" || tag.data.name == "polyline") {
      auto points = splitArguments(tag.data.attributes.at("points"));
      if (points.size() < 2)
        return nullptr;
      path.moveTo(parseNumber(points[0], 1.0f), parseNumber(points[1], 1.0f));
      for (size_t i = 3; i < points.size(); i += 2)
        path.lineTo(parseNumber(points[i - 1], 1.0f), parseNumber(points[i], 1.0f));

      if (tag.data.name == "polygon")
        path.close();
    }
    else if (tag.data.name == "rect") {
      float rx = 0.0f, ry = 0.0f;
      if (tag.data.attributes.count("rx"))
        rx = parseNumber(tag.data.attributes.at("rx"), 1.0f);
      if (tag.data.attributes.count("ry"))
        ry = parseNumber(tag.data.attributes.at("ry"), 1.0f);
      if (rx > 0.0f || ry > 0.0f)
        path.addRoundedRectangle(0, 0, width, height, rx, ry);
      else
        path.addRectangle(0, 0, width, height);
    }
    else if (tag.data.name == "circle" || tag.data.name == "ellipse") {
      float cx = 0.0f, cy = 0.0f, rx = 0.0f, ry = 0.0f;
      if (tag.data.attributes.count("cx"))
        cx = parseNumber(tag.data.attributes.at("cx"), 1.0f);
      if (tag.data.attributes.count("cy"))
        cy = parseNumber(tag.data.attributes.at("cy"), 1.0f);
      if (tag.data.attributes.count("r"))
        rx = ry = parseNumber(tag.data.attributes.at("r"), 1.0f);
      if (tag.data.attributes.count("rx"))
        rx = parseNumber(tag.data.attributes.at("rx"), 1.0f);
      if (tag.data.attributes.count("ry"))
        ry = parseNumber(tag.data.attributes.at("ry"), 1.0f);

      state.local_transform = state.local_transform * Matrix::translation(cx - rx, cy - ry);
      path.addEllipse(rx, ry, rx, ry);
    }
    else
      return nullptr;

    auto drawable = std::make_unique<SvgDrawable>();
    drawable->state = state;
    auto start_bounding_box = path.boundingBox();

    Matrix transform;
    for (int i = state_stack.size() - 1; i >= 0; --i) {
      if (state_stack[i].tranform_origin_x || state_stack[i].tranform_origin_y) {
        Point origin(state_stack[i].tranform_origin_x, state_stack[i].tranform_origin_y);
        if (state_stack[i].transform_ratio_x || state_stack[i].transform_ratio_x) {
          auto bounding_box = path.transformed(transform).boundingBox();
          if (state_stack[i].transform_ratio_x)
            origin.x *= bounding_box.width();
          if (state_stack[i].transform_ratio_y)
            origin.y *= bounding_box.height();
        }
        transform = Matrix::translation(origin) * state_stack[i].local_transform *
                    Matrix::translation(-origin) * transform;
      }
      else
        transform = state_stack[i].local_transform * transform;
    }

    std::vector<float> dashes;
    float view_width = view.width ? view.width : start_bounding_box.width();
    float view_height = view.height ? view.height : start_bounding_box.height();
    float dash_scale = std::sqrtf(0.5f * (view_width * view_width + view_height * view_height));
    float dash_offset = state.stroke_dashoffset;
    if (state.stroke_dashoffset_ratio)
      dash_offset *= dash_scale;

    for (const auto& dash : state.stroke_dasharray) {
      if (dash.second) {
        dashes.push_back(dash.first * dash_scale);
      }
      else
        dashes.push_back(dash.first);
    }

    drawable->path = path.transformed(transform);
    drawable->stroke_path = path.stroke(state.stroke_width, state.stroke_join, state.stroke_end_cap,
                                        dashes, dash_offset, state.stroke_miter_limit);
    drawable->stroke_path.transform(transform);

    width = width ? width : start_bounding_box.width();
    height = height ? height : start_bounding_box.height();
    float x = 0.0f, y = 0.0f;
    if (tag.data.attributes.count("x"))
      x = parseNumber(tag.data.attributes.at("x"), 1.0f);
    if (tag.data.attributes.count("y"))
      y = parseNumber(tag.data.attributes.at("y"), 1.0f);
    Matrix local_translation = Matrix::translation(x, y);
    drawable->fill_brush = state.fill_gradient.toBrush(width, height, transform, x, y);
    drawable->fill_brush = drawable->fill_brush.withMultipliedAlpha(state.fill_opacity * state.opacity);

    drawable->stroke_brush = state.stroke_gradient.toBrush(width, height, transform, x, y);
    drawable->stroke_brush = drawable->stroke_brush.withMultipliedAlpha(state.stroke_opacity * state.opacity);
    return drawable;
  }

  void computeDrawables(const Svg::ViewSettings& view, Tag& tag, std::vector<DrawableState>& state_stack,
                        std::vector<std::unique_ptr<SvgDrawable>>& drawables,
                        const std::map<std::string, Tag>& defs,
                        const std::map<std::string, GradientDef>& gradients,
                        const std::vector<std::pair<CssSelector, std::string>>& style_lookup) {
    if (tag.data.name == "defs")
      return;

    state_stack.push_back(state_stack.back());

    for (const auto& style : style_lookup) {
      if (style.first.matches(tag))
        parseStyleAttribute(state_stack.back(), style.second, gradients);
    }

    loadDrawableState(tag, state_stack, gradients);
    auto drawable = loadDrawable(view, tag, state_stack, gradients);
    if (drawable) {
      drawables.push_back(std::move(drawable));
      state_stack.pop_back();
      return;
    }

    for (auto& child : tag.children)
      computeDrawables(view, child, state_stack, drawables, defs, gradients, style_lookup);

    state_stack.pop_back();
  }

  static Svg::ViewSettings loadSvgViewSettings(const Tag& tag) {
    Svg::ViewSettings result;
    if (tag.data.attributes.count("width"))
      result.width = parseNumber(tag.data.attributes.at("width"), 1.0f);
    if (tag.data.attributes.count("height"))
      result.height = parseNumber(tag.data.attributes.at("height"), 1.0f);
    if (tag.data.attributes.count("viewBox")) {
      std::vector<std::string> tokens = splitArguments(tag.data.attributes.at("viewBox"));
      if (tokens.size() >= 4) {
        result.view_box_x = parseNumber(tokens[0], 1.0f);
        result.view_box_y = parseNumber(tokens[1], 1.0f);
        result.view_box_width = parseNumber(tokens[2], 1.0f);
        result.view_box_height = parseNumber(tokens[3], 1.0f);
      }
    }

    result.align = "xMidYMid";
    result.scale = "meet";
    if (tag.data.attributes.count("preserveAspectRatio")) {
      auto aspect_ratio_settings = tag.data.attributes.at("preserveAspectRatio");
      std::vector<std::string> tokens = splitArguments(aspect_ratio_settings);
      if (tokens.size() > 0) {
        if (tokens[0][0] == 'x' || tokens[0][0] == 'X')
          result.align = tokens[0];
        else
          result.scale = tokens[0];
      }
      if (tokens.size() > 1) {
        if (tokens[1][0] == 'x' || tokens[1][0] == 'X')
          result.align = tokens[1];
        else
          result.scale = tokens[1];
      }
    }
    return result;
  }

  Matrix initialTransform(const Svg::ViewSettings& view) {
    Matrix transform;

    float extra_width = 0.0f;
    float extra_height = 0.0f;
    if (view.width > 0 && view.height > 0 && view.view_box_width > 0 && view.view_box_height > 0) {
      float scale_x = view.width / view.view_box_width;
      float scale_y = view.height / view.view_box_height;
      if (view.scale == "meet")
        scale_x = scale_y = std::min(scale_x, scale_y);
      else if (view.scale == "slice")
        scale_x = scale_y = std::max(scale_x, scale_y);

      transform = Matrix::scale(scale_x, scale_y) * Matrix::translation(-view.view_box_x, -view.view_box_y);
      extra_width = view.width - (view.view_box_width * scale_x);
      extra_height = view.height - (view.view_box_height * scale_y);
    }

    if (view.align == "xMidYMid")
      transform = Matrix::translation(extra_width / 2, extra_height / 2) * transform;
    else if (view.align == "xMaxYMax")
      transform = Matrix::translation(extra_width, extra_height) * transform;
    else if (view.align == "xMinYMax")
      transform = Matrix::translation(0, extra_height) * transform;
    else if (view.align == "xMaxYMin")
      transform = Matrix::translation(extra_width, 0) * transform;
    else if (view.align == "xMidYMin")
      transform = Matrix::translation(extra_width / 2, 0) * transform;
    else if (view.align == "xMidYMax")
      transform = Matrix::translation(extra_width / 2, extra_height) * transform;
    else if (view.align == "xMinYMid")
      transform = Matrix::translation(0, extra_height / 2) * transform;
    else if (view.align == "xMaxYMid")
      transform = Matrix::translation(extra_width, extra_height / 2) * transform;

    return transform;
  }

  void Svg::parseData(const unsigned char* data, int data_size) {
    drawables_.clear();
    std::string str(reinterpret_cast<const char*>(data), data_size);

    std::vector<Tag> tags;
    int i = 0;
    Tag root = parseTagTree(str, i);
    while (root.data.ignored || !root.data.name.empty()) {
      if (!root.data.ignored)
        tags.push_back(root);

      root = parseTagTree(str, i);
    }

    DrawableState state;
    for (auto& tag : tags) {
      if (tag.data.name == "svg") {
        view_ = loadSvgViewSettings(tag);
        state.local_transform = initialTransform(view_);
        state.scale_transform = state.local_transform.withNoTranslation();
      }
    }

    std::map<std::string, Tag> defs;
    collectDefs(tags, defs);
    resolveUses(tags, defs);
    std::vector<std::pair<CssSelector, std::string>> style_lookup;
    loadStyleTags(tags, style_lookup);
    std::map<std::string, GradientDef> gradients;
    collectGradients(view_, tags, gradients);

    std::vector<DrawableState> state_stack;
    state_stack.push_back(state);
    for (auto& tag : tags)
      computeDrawables(view_, tag, state_stack, drawables_, defs, gradients, style_lookup);
  }
}