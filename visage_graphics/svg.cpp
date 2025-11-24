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

#include "canvas.h"

#include <unordered_map>

namespace visage {
  static std::string unescape(std::string input, const std::string& from, const std::string& to) {
    size_t pos = 0;

    while ((pos = input.find(from, pos)) != std::string::npos) {
      input.replace(pos, from.length(), to);
      pos += to.length();
    }

    return input;
  }

  static std::vector<std::string> splitArguments(const std::string& str) {
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

  static std::string removeWhitespace(const std::string& string) {
    constexpr auto is_whitespace = [](char c) { return std::isspace(c); };
    std::string result = string;
    result.erase(std::remove_if(result.begin(), result.end(), is_whitespace), result.end());
    return result;
  }

  static std::string trimWhitespace(const std::string& string) {
    constexpr auto is_whitespace = [](char c) { return std::isspace(c); };
    std::string result = string;
    result.erase(result.begin(), std::find_if_not(result.begin(), result.end(), is_whitespace));
    result.erase(std::find_if_not(result.rbegin(), result.rend(), is_whitespace).base(), result.end());
    return result;
  }

  static std::vector<std::string> parseFunctionTokens(const std::string& function_string, int& pos) {
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

  static std::string urlId(const std::string& url) {
    int pos = 0;
    auto tokens = parseFunctionTokens(url, pos);
    if (tokens.size() <= 1)
      return "";

    std::string id = unescape(unescape(tokens[1], "\'", ""), "\"", "");
    if (!id.empty() && id[0] == '#')
      id = id.substr(1);
    return id;
  }

  bool CssSelector::matches(const Tag& tag) const {
    // TODO support attributes, pseudo-classes, and chaining selectors

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

  void SvgDrawable::draw(Canvas& canvas, ColorContext* context, float x, float y, float width,
                         float height) const {
    if (!state.visible || opacity <= 0.0f || is_defines)
      return;

    if (opacity < 1.0f)
      canvas.setBlendMode(BlendMode::Composite);

    fill(canvas, context, x, y, width, height);
    stroke(canvas, context, x, y, width, height);
  }

  void SvgDrawable::drawAll(Canvas& canvas, ColorContext* context, float x, float y, float width,
                            float height) const {
    draw(canvas, context, x, y, width, height);
    for (const auto& child : children)
      child->drawAll(canvas, context, x, y, width, height);
  }

  bool SvgDrawable::setContextColor(Canvas& canvas, const ColorContext* context,
                                    const GradientDef& gradient, float color_opacity) const {
    const Brush* context_brush = nullptr;
    if (gradient.type == GradientDef::Type::CurrentColor)
      context_brush = context->current_color;
    else if (gradient.type == GradientDef::Type::ContextFill)
      context_brush = context->fill_color;
    else if (gradient.type == GradientDef::Type::ContextStroke)
      context_brush = context->stroke_color;

    if (context_brush) {
      if (color_opacity == 1.0f)
        canvas.setColor(*context_brush);
      else
        canvas.setColor(context_brush->withMultipliedAlpha(color_opacity));
      return true;
    }

    return false;
  }

  void SvgDrawable::fill(Canvas& canvas, ColorContext* context, float x, float y, float width,
                         float height) const {
    if (state.fill_opacity <= 0.0f)
      return;

    if (!setContextColor(canvas, context, state.fill_gradient, state.fill_opacity)) {
      if (fill_brush.isNone())
        return;

      auto brush = fill_brush;
      brush.transform(Transform::translation(x, y));
      canvas.setColor(brush);
      context->fill_color = &fill_brush;
    }

    canvas.fill(path, x, y, width, height);
  }

  void SvgDrawable::stroke(Canvas& canvas, ColorContext* context, float x, float y, float width,
                           float height) const {
    if (state.stroke_opacity <= 0.0f || state.stroke_width <= 0.0f)
      return;

    if (!setContextColor(canvas, context, state.stroke_gradient, state.stroke_opacity)) {
      if (stroke_brush.isNone())
        return;

      auto brush = stroke_brush;
      brush.transform(Transform::translation(x, y));
      canvas.setColor(brush);
      context->stroke_color = &stroke_brush;
    }

    canvas.fill(stroke_path, x, y, width, height);
  }

  inline void tryReadFloat(float& result, const std::string& string) {
    try {
      result = std::stof(string);
    }
    catch (...) {
    }
  }

  inline float parseNumber(const std::string& str, float max) {
    auto suffix_pos = str.find_first_not_of("0123456789+-.eE");
    auto units = suffix_pos != std::string::npos ? trimWhitespace(str.substr(suffix_pos)) : "";

    float mult = 1.0f;
    if (units == "%")
      mult = 0.01f * max;
    else if (units == "in")
      mult = 96.0f;
    else if (units == "cm")
      mult = 96.0f / 2.54f;
    else if (units == "mm")
      mult = 96.0f / 25.4f;
    else if (units == "pt")
      mult = 96.0f / 72.0f;
    else if (units == "pc")
      mult = 16.0f;

    float result = 0.0f;
    tryReadFloat(result, str);
    return result * mult;
  }

  inline float parsePositionValue(const std::string& token, float range) {
    if (token == "top" || token == "left")
      return 0.0f;
    if (token == "bottom" || token == "right")
      return range;
    return parseNumber(token, range);
  }

  static Point parseEllipseRadius(const std::string& token1, const std::string& token2,
                                  const Point& center, float max_x, float max_y) {
    auto parse_ellipse_dimension = [](const std::string& token, float center, float range) {
      if (token == "closest-side")
        return std::min(center, range - center);
      if (token == "farthest-side")
        return std::max(center, range - center);
      return parseNumber(token, range);
    };

    return { parse_ellipse_dimension(token1, center.x, max_x),
             parse_ellipse_dimension(token2, center.y, max_y) };
  }

  inline float parseCircleRadius(const std::string& token, Point center, float max_x, float max_y) {
    if (token == "closest-side") {
      float dx = std::min(center.x, max_x - center.x);
      float dy = std::min(center.y, max_y - center.y);
      return std::min(dx, dy);
    }
    if (token == "farthest-side") {
      float dx = std::max(center.x, max_x - center.x);
      float dy = std::max(center.y, max_y - center.y);
      return std::max(dx, dy);
    }
    if (token == "closest-corner") {
      float dx = std::min(center.x, max_x - center.x);
      float dy = std::min(center.y, max_y - center.y);
      return std::sqrt(dx * dx + dy * dy);
    }
    if (token == "farthest-corner") {
      float dx = std::max(center.x, max_x - center.x);
      float dy = std::max(center.y, max_y - center.y);
      return std::sqrt(dx * dx + dy * dy);
    }
    if (token.find('%') != std::string::npos)
      return parseNumber(token, std::sqrt(0.5f * (max_x * max_x + max_y * max_y)));

    return parseNumber(token, std::sqrt(0.5f * (max_x * max_x + max_y * max_y)));
  }

  Path::CommandList parseEllipseShape(std::vector<std::string>& tokens, const Bounds& bounding_box) {
    auto next_or_end = [&tokens](const std::vector<std::string>::iterator& it) {
      return it < tokens.end() ? std::next(it) : tokens.end();
    };

    auto at_it = std::find(tokens.begin(), tokens.end(), "at");

    Point center(bounding_box.width() / 2, bounding_box.height() / 2);
    auto center_it = next_or_end(at_it);
    if (center_it < tokens.end()) {
      center.x = parsePositionValue(*center_it, bounding_box.width());
      center_it++;
      if (center_it < tokens.end())
        center.y = parsePositionValue(*center_it, bounding_box.height());
    }

    auto radius_it = next_or_end(tokens.begin());
    auto radius2_it = next_or_end(radius_it);
    Point radius;
    if (radius2_it < at_it) {
      radius = parseEllipseRadius(*radius_it, *radius2_it, center, bounding_box.width(),
                                  bounding_box.height());
    }
    else {
      const std::string& radius_token = radius_it < at_it ? *radius_it : "closest-side";
      radius.x = radius.y = parseCircleRadius(radius_token, center, bounding_box.width(),
                                              bounding_box.height());
    }

    Path::CommandList path;
    path.addEllipse(bounding_box.x() + center.x, bounding_box.y() + center.y, radius.x, radius.y);
    return path;
  }

  Path::CommandList parsePolygonShape(std::vector<std::string>& tokens, int start_index,
                                      const Bounds& bounding_box) {
    Path::CommandList path;
    if (tokens.size() > start_index + 1) {
      float x = parseNumber(tokens[start_index], bounding_box.width());
      float y = parseNumber(tokens[start_index + 1], bounding_box.height());
      path.moveTo(bounding_box.x() + x, bounding_box.y() + y);
    }
    int index = start_index + 2;
    while (index + 1 < tokens.size()) {
      float x = parseNumber(tokens[index], bounding_box.width());
      float y = parseNumber(tokens[index + 1], bounding_box.height());
      path.lineTo(bounding_box.x() + x, bounding_box.y() + y);
      index += 2;
    }
    return path;
  }

  bool parseBorderRadius(float* results, std::vector<std::string>& tokens, const Bounds& bounding_box) {
    auto r_it = std::find(tokens.begin(), tokens.end(), "round");
    if (r_it == tokens.end())
      return false;
    r_it++;
    if (r_it == tokens.end())
      return false;

    auto y_it = std::find(tokens.begin(), tokens.end(), "/");
    int x_range = std::distance(r_it, y_it);
    int x_index = std::distance(tokens.begin(), r_it);
    for (int i = 0; i < 4; ++i) {
      int index = x_index + i % x_range;
      float dim = (i % 2) ? bounding_box.height() : bounding_box.width();
      results[4 + i] = results[i] = parseNumber(tokens[index], dim);
    }

    if (y_it != tokens.end() && std::next(y_it) != tokens.end()) {
      y_it = std::next(y_it);
      int y_range = std::distance(y_it, tokens.end());
      int y_index = std::distance(tokens.begin(), y_it);
      for (int i = 0; i < 4; ++i) {
        int index = y_index + i % y_range;
        float dim = (i % 2) ? bounding_box.height() : bounding_box.width();
        results[4 + i] = parseNumber(tokens[index], dim);
      }
    }
    return true;
  }

  Path::CommandList parseRectShape(std::vector<std::string>& tokens, const Bounds& bounding_box) {
    static constexpr int kNumInsets = 4;
    float insets[kNumInsets] {};
    for (int i = 0; i < tokens.size() - 1 && i < kNumInsets; ++i) {
      float dim = (i % 2) ? bounding_box.width() : bounding_box.height();
      insets[i] = parseNumber(tokens[i + 1], dim);
    }

    Path::CommandList path;
    float rs[8] {};
    float x = bounding_box.x() + insets[3];
    float y = bounding_box.y() + insets[0];
    float width = bounding_box.width() - insets[1] - insets[3];
    float height = bounding_box.height() - insets[2] - insets[0];
    if (parseBorderRadius(rs, tokens, bounding_box)) {
      path.addRoundedRectangle(x, y, width, height, rs[0], rs[4], rs[1], rs[5], rs[2], rs[6], rs[3],
                               rs[7]);
    }
    else
      path.addRectangle(x, y, width, height);
    return path;
  }

  void SvgDrawable::checkPathClipping(const Matrix& scale_matrix, const Bounds& view_box,
                                      std::map<std::string, SvgDrawable*>& clip_paths) {
    clipping_paths.clear();

    if (is_clip_path)
      clip_paths[id] = this;

    int position = 0;
    auto tokens = parseFunctionTokens(clip_path_shape, position);
    if (tokens.empty())
      return;

    std::string remaining = clip_path_shape.substr(position);
    Bounds bounding_box;
    if (remaining == "fill-box")
      bounding_box = boundingFillBox();
    else if (remaining == "stroke-box")
      bounding_box = boundingStrokeBox();
    else if (remaining == "view-box")
      bounding_box = view_box;
    else
      bounding_box = boundingBox();

    if (tokens[0] == "url") {
      auto url_id = urlId(clip_path_shape);
      if (clip_paths.count(url_id)) {
        auto& clip_drawable = clip_paths.at(url_id);
        if (clip_drawable->is_clip_bounding_box) {
          auto scale = scale_matrix * Matrix::scale(bounding_box.width(), bounding_box.height());
          clip_drawable->initPaths(scale, view_box);
          clip_drawable->adjustPaths(scale, view_box, clip_paths);

          clip_drawable->gatherPaths(clipping_paths);
          for (auto& clip_path : clipping_paths) {
            clip_path.transform(Transform::translation(bounding_box.x(), bounding_box.y()) *
                                Transform::scale(bounding_box.width(), bounding_box.height()));
          }
        }
        else
          clip_drawable->gatherPaths(clipping_paths);
      }

      return;
    }

    Path clip_path;
    clip_path.setResolutionMatrix(scale_matrix);
    clip_path.setFillRule(Path::FillRule::NonZero);
    if (tokens[0] == "inset" || tokens[0] == "rect")
      clip_path.loadCommands(parseRectShape(tokens, bounding_box));
    else if (tokens[0] == "circle" || tokens[0] == "ellipse")
      clip_path.loadCommands(parseEllipseShape(tokens, bounding_box));
    else if (tokens[0] == "polygon" || tokens[0] == "polyline")
      clip_path.loadCommands(parsePolygonShape(tokens, 1, bounding_box));

    clipping_paths.push_back(clip_path);
  }

  void SvgDrawable::initPaths(const Matrix& scale_matrix, const Bounds& view_box) {
    auto scale = scale_matrix * local_transform.matrix;
    for (auto& child : children)
      child->initPaths(scale, view_box);

    if (!command_list.empty()) {
      path.clear();
      path.setFillRule(state.non_zero_fill ? Path::FillRule::NonZero : Path::FillRule::EvenOdd);
      path.setResolutionMatrix(scale_matrix);
      path.loadCommands(command_list);

      Bounds bounding_box = path.boundingBox();
      auto fill_box = state.fill_gradient.user_space ? view_box : bounding_box;
      fill_brush = state.fill_gradient.toBrush(fill_box);
      fill_brush = fill_brush.withMultipliedAlpha(state.fill_opacity);

      strokePath(scale_matrix, view_box);
    }
  }

  void SvgDrawable::strokePath(const Matrix& scale_matrix, const Bounds& view_box) {
    if (state.stroke_width <= 0.0f || !state.visible || state.stroke_opacity <= 0.0f)
      return;
    if (state.stroke_gradient.isNone() && state.stroke_gradient.type != GradientDef::Type::CurrentColor &&
        state.stroke_gradient.type != GradientDef::Type::ContextFill &&
        state.stroke_gradient.type != GradientDef::Type::ContextStroke)
      return;

    std::vector<float> dashes;
    float dash_scale = std::sqrt(0.5f * (view_box.width() * view_box.width() +
                                         view_box.height() * view_box.height()));
    float dash_offset = state.stroke_dashoffset;
    if (state.stroke_dashoffset_ratio)
      dash_offset *= dash_scale;

    for (const auto& dash : state.stroke_dasharray) {
      if (dash.second)
        dashes.push_back(dash.first * dash_scale);
      else
        dashes.push_back(dash.first);
    }

    float stroke_opacity = state.stroke_opacity;
    if (state.non_scaling_stroke) {
      auto inverted = path.transformed(scale_matrix);
      stroke_path = inverted.stroke(state.stroke_width, state.stroke_join, state.stroke_end_cap,
                                    dashes, dash_offset, state.stroke_miter_limit);
      stroke_path.transform(scale_matrix.inversed());
    }
    else {
      float multiplier = std::sqrt(std::abs(scale_matrix.determinant()));
      float width = std::max(1.0f, state.stroke_width * multiplier) / multiplier;
      stroke_opacity = state.stroke_opacity * state.stroke_width / width;
      stroke_path = path.stroke(width, state.stroke_join, state.stroke_end_cap, dashes, dash_offset,
                                state.stroke_miter_limit);
    }
    auto stroke_box = state.stroke_gradient.user_space ? view_box : path.boundingBox();
    stroke_brush = state.stroke_gradient.toBrush(stroke_box);
    stroke_brush = stroke_brush.withMultipliedAlpha(stroke_opacity);
  }

  void SvgDrawable::adjustPaths(const Matrix& scale_matrix, const Bounds& view_box,
                                std::map<std::string, SvgDrawable*>& clip_paths) {
    for (auto& child : children)
      child->adjustPaths(scale_matrix * local_transform.matrix, view_box, clip_paths);

    auto transform = local_transform;
    if (transform_origin_x || transform_origin_y) {
      transform = Transform::translation(-transform_origin_x, -transform_origin_y) * transform *
                  Transform::translation(transform_origin_x, transform_origin_y);
    }

    if (!transform.isIdentity())
      transformPaths(transform);

    checkPathClipping(scale_matrix * local_transform.matrix, view_box, clip_paths);
  }

  void consumeWhiteSpace(const std::string& str, int& i) {
    while (i < str.size() && (str[i] == ' ' || str[i] == '\t' || str[i] == '\n' || str[i] == '\r'))
      i++;
  }

  void consumeTillEndTag(const std::string& str, int& i) {
    auto end_pos = str.find('>', i);
    if (end_pos == std::string::npos)
      i = str.size();
    else
      i = end_pos + 1;
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
        result += str.substr(current, i - current);
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
    std::string key;
    consumeWhiteSpace(str, i);
    while (i < str.size() && str[i] != '=' && str[i] != ' ' && str[i] != '\t' && str[i] != '>' &&
           str[i] != '/')
      key += str[i++];

    if (key.empty() || i >= str.size() - 1 || str[i] != '=')
      return { "", "" };

    i++;
    char quote_char = str[i++];
    VISAGE_ASSERT(quote_char == '"' || quote_char == '\'');

    int end = str.find(quote_char, i);
    if (end == std::string::npos) {
      VISAGE_ASSERT(false);
      return { "", "" };
    }

    std::string value = str.substr(i, end - i);
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
    while (!child.data.is_closing && i < str.size()) {
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

  Color parseStopColor(const Tag& tag) {
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

  Transform parseTransform(const std::string& transform_string) {
    Transform matrix;
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

      if (args.empty())
        return matrix;

      if (tokens[0] == "translate") {
        float y = args.size() > 1 ? args[1] : args[0];
        matrix = matrix * Transform::translation(args[0], y);
      }
      else if (tokens[0] == "scale") {
        float y = args.size() > 1 ? args[1] : args[0];
        matrix = matrix * Transform::scale(args[0], y);
      }
      else if (tokens[0] == "rotate") {
        if (args.size() > 2)
          matrix = matrix * Transform::rotation(args[0], { args[1], args[2] });
        else
          matrix = matrix * Transform::rotation(args[0]);
      }
      else if (tokens[0] == "skewX")
        matrix = matrix * Transform::skewX(args[0]);
      else if (tokens[0] == "skewY")
        matrix = matrix * Transform::skewY(args[0]);
      else if (tokens[0] == "matrix" && args.size() > 5)
        matrix = matrix * Transform(args[0], args[2], args[4], args[1], args[3], args[5]);
    }
    return matrix;
  }

  GradientDef parseGradientTag(const Tag& tag) {
    GradientDef gradient_def;
    if (tag.data.name == "linearGradient") {
      gradient_def.type = GradientDef::Type::Linear;
      if (tag.data.attributes.count("x1"))
        gradient_def.point1.x = parseNumber(tag.data.attributes.at("x1"), 1.0f);
      if (tag.data.attributes.count("y1"))
        gradient_def.point1.y = parseNumber(tag.data.attributes.at("y1"), 1.0f);
      if (tag.data.attributes.count("x2"))
        gradient_def.point2.x = parseNumber(tag.data.attributes.at("x2"), 1.0f);
      if (tag.data.attributes.count("y2"))
        gradient_def.point2.y = parseNumber(tag.data.attributes.at("y2"), 1.0f);
    }
    else {
      gradient_def.type = GradientDef::Type::Radial;
      gradient_def.point1.x = 0.5f;
      gradient_def.point1.y = 0.5f;
      gradient_def.point2.x = 0.5f;
      gradient_def.point2.y = 0.5f;
      if (tag.data.attributes.count("cx"))
        gradient_def.point2.x = gradient_def.point1.x = parseNumber(tag.data.attributes.at("cx"), 1.0f);
      if (tag.data.attributes.count("cy"))
        gradient_def.point2.y = gradient_def.point1.y = parseNumber(tag.data.attributes.at("cy"), 1.0f);
      if (tag.data.attributes.count("fx"))
        gradient_def.point2.x = parseNumber(tag.data.attributes.at("fx"), 1.0f);
      if (tag.data.attributes.count("fy"))
        gradient_def.point2.y = parseNumber(tag.data.attributes.at("fy"), 1.0f);
      if (tag.data.attributes.count("r"))
        gradient_def.radius = parseNumber(tag.data.attributes.at("r"), 1.0f);
      if (tag.data.attributes.count("fr"))
        gradient_def.focal_radius = parseNumber(tag.data.attributes.at("fr"), 1.0f);
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

    for (auto& child : tag.children) {
      if (child.data.name != "stop" || !child.data.attributes.count("offset"))
        continue;

      float offset = parseNumber(child.data.attributes.at("offset"), 1.0f);
      gradient_def.gradient.addColorStop(parseStopColor(child), offset);
    }

    return gradient_def;
  }

  CssSelector parseCssSelector(const std::string& selectors) {
    auto edited = unescape(selectors, ">", " > ");
    bool direct_child = false;
    std::vector<CssSelector> chained_selectors;
    std::stringstream ss(edited);
    std::string selector_text;

    while (std::getline(ss, selector_text, ' ')) {
      selector_text = removeWhitespace(selector_text);
      if (selector_text.empty())
        continue;
      if (selector_text == ">") {
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
        item = removeWhitespace(item);
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

  Color parseColor(const std::string& color_string) {
    std::string color = removeWhitespace(color_string);

    if (color == "none")
      return {};
    if (color[0] == '#')
      return Color::fromHexString(color.substr(1));

    int pos = 0;
    auto tokens = parseFunctionTokens(color, pos);
    if (tokens.empty())
      return {};
    if (tokens.size() == 1)
      return translateColor(tokens[0]);

    if (tokens[0].substr(0, 3) == "rgb" && tokens.size() > 3) {
      float alpha = tokens.size() > 4 ? parseNumber(tokens[4], 1.0f) : 1.0f;
      return { alpha, parseNumber(tokens[1], 255.0f) / 255.0f,
               parseNumber(tokens[2], 255.0f) / 255.0f, parseNumber(tokens[3], 255.0f) / 255.0f };
    }
    if (tokens[0].substr(0, 3) == "hsl" && tokens.size() > 3) {
      float alpha = tokens.size() > 4 ? parseNumber(tokens[4], 1.0f) : 1.0f;
      return Color::fromAHSV(alpha, parseNumber(tokens[1], 360.0f) / 360.0f,
                             parseNumber(tokens[2], 100.0f) * 0.01f,
                             parseNumber(tokens[3], 100.0f) * 0.01f);
    }
    return {};
  }

  SvgViewSettings loadSvgViewSettings(const Tag& tag) {
    SvgViewSettings result;
    if (tag.data.attributes.count("width")) {
      result.width = parseNumber(tag.data.attributes.at("width"), 1.0f);
      result.view_box.setWidth(result.width);
    }
    if (tag.data.attributes.count("height")) {
      result.height = parseNumber(tag.data.attributes.at("height"), 1.0f);
      result.view_box.setHeight(result.height);
    }
    if (tag.data.attributes.count("viewBox")) {
      std::vector<std::string> tokens = splitArguments(tag.data.attributes.at("viewBox"));
      if (tokens.size() >= 4) {
        result.view_box = Bounds(parseNumber(tokens[0], 1.0f), parseNumber(tokens[1], 1.0f),
                                 parseNumber(tokens[2], 1.0f), parseNumber(tokens[3], 1.0f));
      }
    }

    result.align = "xMidYMid";
    result.scale = "meet";
    if (tag.data.attributes.count("preserveAspectRatio")) {
      auto aspect_ratio_settings = tag.data.attributes.at("preserveAspectRatio");
      std::vector<std::string> tokens = splitArguments(aspect_ratio_settings);
      if (!tokens.empty()) {
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

  bool SvgParser::loadDrawable(const Tag& tag, SvgDrawable* drawable) const {
    int view_width = view_.view_box.width() > 0.0f ? view_.view_box.width() : 1.0f;
    int view_height = view_.view_box.height() > 0.0f ? view_.view_box.height() : 1.0f;

    float width = 0.0f;
    float height = 0.0f;
    if (tag.data.attributes.count("width"))
      width = parseNumber(tag.data.attributes.at("width"), view_width);
    if (tag.data.attributes.count("height"))
      height = parseNumber(tag.data.attributes.at("height"), view_height);

    if (tag.data.name == "path" && tag.data.attributes.count("d"))
      drawable->command_list = Path::parseSvgPath(tag.data.attributes.at("d"));
    else if (tag.data.name == "line") {
      float x1 = 0.0f, y1 = 0.0f, x2 = 0.0f, y2 = 0.0f;
      if (tag.data.attributes.count("x1"))
        x1 = parseNumber(tag.data.attributes.at("x1"), view_width);
      if (tag.data.attributes.count("y1"))
        y1 = parseNumber(tag.data.attributes.at("y1"), view_height);
      if (tag.data.attributes.count("x2"))
        x2 = parseNumber(tag.data.attributes.at("x2"), view_width);
      if (tag.data.attributes.count("y2"))
        y2 = parseNumber(tag.data.attributes.at("y2"), view_height);
      drawable->command_list.moveTo(x1, y1);
      drawable->command_list.lineTo(x2, y2);
    }
    else if (tag.data.name == "polygon" || tag.data.name == "polyline") {
      auto points = splitArguments(tag.data.attributes.at("points"));
      drawable->command_list = parsePolygonShape(points, 0, view_.view_box);

      if (tag.data.name == "polygon")
        drawable->command_list.close();
    }
    else if (tag.data.name == "rect") {
      float x = 0.0f, y = 0.0f, rx = 0.0f, ry = 0.0f;
      if (tag.data.attributes.count("x"))
        x = parseNumber(tag.data.attributes.at("x"), view_width);
      if (tag.data.attributes.count("y"))
        y = parseNumber(tag.data.attributes.at("y"), view_height);
      if (tag.data.attributes.count("rx"))
        rx = parseNumber(tag.data.attributes.at("rx"), view_width);
      if (tag.data.attributes.count("ry")) {
        ry = parseNumber(tag.data.attributes.at("ry"), view_height);
        if (tag.data.attributes.count("rx") == 0)
          rx = ry;
      }
      else
        ry = rx;

      if (rx > 0.0f || ry > 0.0f)
        drawable->command_list.addRoundedRectangle(x, y, width, height, rx, ry);
      else
        drawable->command_list.addRectangle(x, y, width, height);
    }
    else if (tag.data.name == "circle" || tag.data.name == "ellipse") {
      float x = 0.0f, y = 0.0f, cx = 0.0f, cy = 0.0f, rx = 0.0f, ry = 0.0f;
      if (tag.data.attributes.count("x"))
        x = parseNumber(tag.data.attributes.at("x"), view_width);
      if (tag.data.attributes.count("y"))
        y = parseNumber(tag.data.attributes.at("y"), view_height);
      if (tag.data.attributes.count("cx"))
        cx = parseNumber(tag.data.attributes.at("cx"), view_width);
      if (tag.data.attributes.count("cy"))
        cy = parseNumber(tag.data.attributes.at("cy"), view_height);
      if (tag.data.attributes.count("r")) {
        float normalized = std::sqrt(0.5f * (view_width * view_width + view_height * view_height));
        rx = ry = parseNumber(tag.data.attributes.at("r"), normalized);
      }
      if (tag.data.attributes.count("rx"))
        rx = parseNumber(tag.data.attributes.at("rx"), view_width);
      if (tag.data.attributes.count("ry"))
        ry = parseNumber(tag.data.attributes.at("ry"), view_height);

      drawable->command_list.addEllipse(x + cx, y + cy, rx, ry);
    }
    else
      return false;

    if (drawable->command_list.empty())
      return false;

    Marker* marker_current = drawable->marker_start;
    int end_index = drawable->command_list.size() - 1;
    for (int i = 0; i < drawable->command_list.size(); ++i) {
      if (i == drawable->command_list.size() - 1)
        marker_current = drawable->marker_end;

      if (marker_current) {
        auto point = drawable->command_list[i].end;
        auto marker = std::make_unique<SvgDrawable>(marker_current->drawable);
        Transform rotation;
        if (marker_current->use_angle)
          rotation = Transform::rotation(marker_current->marker_angle);
        else {
          auto direction = drawable->command_list.direction(i);
          if (i == 0 && marker_current->reverse_start_marker)
            direction = -direction;

          rotation = Transform(direction.x, -direction.y, 0.0f, direction.y, direction.x, 0.0f);
        }

        marker->local_transform = Transform::translation(point) * rotation * marker->local_transform;
        drawable->children.push_back(std::move(marker));
      }

      if (drawable->marker_mid)
        marker_current = drawable->marker_mid;
      else
        i = std::max(end_index - 1, i);
    }

    if (tag.data.attributes.count("id"))
      drawable->id = tag.data.attributes.at("id");
    return true;
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

  void loadOffset(const Tag& tag, SvgDrawable* drawable) {
    if (tag.data.name != "svg" && tag.data.name != "use")
      return;

    float x = 0.0f, y = 0.0f;
    for (auto& attribute : tag.data.attributes) {
      if (attribute.first == "x")
        tryReadFloat(x, attribute.second);
      else if (attribute.first == "y")
        tryReadFloat(y, attribute.second);
    }

    if (x || y)
      drawable->local_transform = drawable->local_transform * Transform::translation(x, y);
  }

  void SvgParser::collectDefs(std::vector<Tag>& tags) {
    for (auto& tag : tags) {
      if (tag.data.attributes.count("id") && !tag.data.attributes.at("id").empty()) {
        std::string id = "#" + tag.data.attributes.at("id");
        defs_[id] = tag;
        defs_[id].data.attributes.erase("id");
      }
      collectDefs(tag.children);
    }
  }

  void SvgParser::collectGradients(std::vector<Tag>& tags) {
    for (auto& tag : tags) {
      if (tag.data.name == "linearGradient" || tag.data.name == "radialGradient") {
        if (tag.data.attributes.count("id") && !tag.data.attributes.at("id").empty()) {
          std::string id = tag.data.attributes.at("id");
          gradients_[id] = parseGradientTag(tag);
        }
      }
      collectGradients(tag.children);
    }
  }

  void SvgParser::collectMarkers(std::vector<Tag>& tags) {
    for (auto& tag : tags) {
      if (tag.data.name == "marker") {
        if (tag.data.attributes.count("id") && !tag.data.attributes.at("id").empty()) {
          std::string id = tag.data.attributes.at("id");
          auto view = loadSvgViewSettings(tag);

          std::vector<DrawableState> state_stack;
          state_stack.push_back({});
          std::unique_ptr<Marker> marker = std::make_unique<Marker>();
          auto& drawable = marker->drawable;
          for (auto& child_tag : tag.children) {
            auto child = computeDrawables(child_tag, state_stack);
            if (child)
              drawable.children.push_back(std::move(child));
          }

          if (tag.data.attributes.count("markerWidth"))
            view.width = parseNumber(tag.data.attributes.at("markerWidth"), view.width);
          if (tag.data.attributes.count("markerHeight"))
            view.height = parseNumber(tag.data.attributes.at("markerHeight"), view.height);

          float x_offset = 0.0f;
          float y_offset = 0.0f;
          if (tag.data.attributes.count("refX"))
            x_offset = parseNumber(tag.data.attributes.at("refX"), view.width);
          if (tag.data.attributes.count("refY"))
            y_offset = parseNumber(tag.data.attributes.at("refY"), view.height);

          if (tag.data.attributes.count("orient")) {
            std::string orient = tag.data.attributes.at("orient");
            marker->reverse_start_marker = orient == "auto-start-reverse";
            marker->use_angle = orient != "auto" && orient != "auto-start-reverse";
            if (marker->use_angle)
              tryReadFloat(marker->marker_angle, orient);
          }

          drawable.local_transform = drawable.initialTransform(view, view.width, view.height) *
                                     Transform::translation(-x_offset, -y_offset);
          markers_[id] = std::move(marker);
        }
      }
      collectMarkers(tag.children);
    }
  }

  void SvgParser::resolveUses(std::vector<Tag>& tags) {
    auto use_tag = [this](Tag& target, const std::string& reference_id) {
      if (defs_.count(reference_id) == 0)
        return;

      const Tag& reference = defs_.at(reference_id);
      for (const auto& attr : reference.data.attributes) {
        if (!target.data.attributes.count(attr.first))
          target.data.attributes[attr.first] = attr.second;
      }
      if (target.children.empty())
        target.children = reference.children;

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

      resolveUses(tag.children);
    }
  }

  void SvgParser::parseCssStyle(const std::string& style) {
    size_t pos = 0;
    while (pos < style.size()) {
      size_t brace_open = style.find('{', pos);
      if (brace_open == std::string::npos)
        break;

      size_t brace_close = style.find('}', brace_open);
      if (brace_close == std::string::npos)
        break;

      std::string selectors = removeWhitespace(style.substr(pos, brace_open - pos));
      std::stringstream ss(selectors);

      std::string rules = style.substr(brace_open + 1, brace_close - brace_open - 1);
      std::string item;
      while (std::getline(ss, item, ',')) {
        if (item.empty())
          continue;

        style_lookup_.emplace_back(parseCssSelector(item), rules);
      }
      pos = brace_close + 1;
    }
  }

  void SvgParser::loadStyleTags(std::vector<Tag>& tags) {
    for (auto& tag : tags) {
      if (tag.data.name == "style")
        parseCssStyle(tag.data.text);
      else
        loadStyleTags(tag.children);
    }
  }

  GradientDef SvgParser::parseGradient(const std::string& color_string) {
    std::string color = removeWhitespace(color_string);
    if (color.substr(0, 3) == "url") {
      auto id = urlId(color_string);
      if (gradients_.count(id) > 0)
        return gradients_.at(id);
    }

    if (color == "currentColor")
      return GradientDef::Type::CurrentColor;
    if (color == "context-fill")
      return GradientDef::Type::ContextFill;
    if (color == "context-stroke")
      return GradientDef::Type::ContextStroke;

    return parseColor(color);
  }

  void SvgParser::parseStyleDefinition(const std::string& key, const std::string& value,
                                       DrawableState& state, SvgDrawable* drawable) {
    if (key == "opacity")
      tryReadFloat(drawable->opacity, value);
    else if (key == "clip-path")
      drawable->clip_path_shape = value;
    else if (key == "color")
      state.current_color = parseGradient(value);
    else if (key == "fill")
      state.fill_gradient = parseGradient(value);
    else if (key == "fill-rule")
      state.non_zero_fill = value == "nonzero";
    else if (key == "fill-opacity")
      tryReadFloat(state.fill_opacity, value);
    else if (key == "stroke")
      state.stroke_gradient = parseGradient(value);
    else if (key == "stroke-opacity")
      tryReadFloat(state.stroke_opacity, value);
    else if (key == "stroke-width")
      tryReadFloat(state.stroke_width, value);
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
    else if (key == "vector-effect")
      state.non_scaling_stroke = value == "non-scaling-stroke";
    else if (key == "stroke-miterlimit")
      tryReadFloat(state.stroke_miter_limit, value);
    else if (key == "visibility")
      state.visible = value != "hidden";
    else if (key == "display")
      state.visible = value != "none";
    else if (key == "marker-start") {
      auto id = urlId(value);
      drawable->marker_start = markers_.count(id) ? markers_.at(id).get() : nullptr;
    }
    else if (key == "marker-mid") {
      auto id = urlId(value);
      drawable->marker_mid = markers_.count(id) ? markers_.at(id).get() : nullptr;
    }
    else if (key == "marker-end") {
      auto id = urlId(value);
      drawable->marker_end = markers_.count(id) ? markers_.at(id).get() : nullptr;
    }
    else if (key == "marker") {
      auto id = urlId(value);
      drawable->marker_start = markers_.count(id) ? markers_.at(id).get() : nullptr;
      drawable->marker_mid = drawable->marker_start;
      drawable->marker_end = drawable->marker_start;
    }
  }

  void SvgParser::parseStyleAttribute(const std::string& style, DrawableState& state, SvgDrawable* drawable) {
    std::stringstream stream(style);
    std::string line;

    while (std::getline(stream, line, ';')) {
      auto pos = line.find(':');
      if (pos != std::string::npos) {
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        parseStyleDefinition(removeWhitespace(key), trimWhitespace(value), state, drawable);
      }
    }
  }

  void SvgParser::loadDrawableTransform(const Tag& tag, SvgDrawable* drawable) {
    if (tag.data.attributes.count("transform"))
      drawable->local_transform = parseTransform(tag.data.attributes.at("transform")) *
                                  drawable->local_transform;
    if (tag.data.attributes.count("transform-origin")) {
      auto args = splitArguments(tag.data.attributes.at("transform-origin"));
      if (!args.empty()) {
        for (auto& arg : args) {
          if (arg == "center")
            arg = "50%";
        }

        drawable->transform_origin_x = parseNumber(args[0], 1.0f);
        if (args.size() > 1)
          drawable->transform_origin_y = parseNumber(args[1], 1.0f);

        drawable->transform_origin_x_ratio = args[0].find('%') != std::string::npos;
        drawable->transform_origin_y_ratio = args.size() > 1 && args[1].find('%') != std::string::npos;
      }
    }
  }

  void SvgParser::loadDrawableStyle(const Tag& tag, std::vector<DrawableState>& state_stack,
                                    SvgDrawable* drawable) {
    drawable->is_clip_path = tag.data.name == "clipPath";
    drawable->is_clip_bounding_box = tag.data.attributes.count("clipPathUnits") &&
                                     tag.data.attributes.at("clipPathUnits") == "objectBoundingBox";
    auto& state = state_stack.back();

    for (auto& attribute : tag.data.attributes) {
      if (attribute.first == "style")
        parseStyleAttribute(attribute.second, state, drawable);
      else
        parseStyleDefinition(attribute.first, attribute.second, state, drawable);
    }
  }

  std::unique_ptr<SvgDrawable> SvgParser::computeDrawables(Tag& tag, std::vector<DrawableState>& state_stack) {
    if (tag.data.ignored || tag.data.name == "marker" || tag.data.name == "mask")
      return nullptr;

    state_stack.push_back(state_stack.back());
    auto drawable = std::make_unique<SvgDrawable>();
    drawable->is_defines = tag.data.name == "defs";

    for (const auto& style : style_lookup_) {
      if (style.first.matches(tag))
        parseStyleAttribute(style.second, state_stack.back(), drawable.get());
    }

    loadDrawableStyle(tag, state_stack, drawable.get());
    drawable->state = state_stack.back();

    if (loadDrawable(tag, drawable.get())) {
      loadDrawableTransform(tag, drawable.get());
      state_stack.pop_back();
      return drawable;
    }

    if (tag.data.attributes.count("id"))
      drawable->id = tag.data.attributes.at("id");

    loadOffset(tag, drawable.get());
    loadDrawableTransform(tag, drawable.get());
    for (auto& child_tag : tag.children) {
      auto child = computeDrawables(child_tag, state_stack);
      if (child)
        drawable->children.push_back(std::move(child));
    }

    state_stack.pop_back();
    return drawable;
  }

  void SvgParser::parseData(const unsigned char* data, int data_size) {
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
      if (tag.data.name == "svg")
        view_ = loadSvgViewSettings(tag);
    }

    collectDefs(tags);
    resolveUses(tags);
    loadStyleTags(tags);
    collectGradients(tags);
    collectMarkers(tags);

    std::vector<DrawableState> state_stack;
    state_stack.push_back(state);
    drawable_ = std::make_unique<SvgDrawable>();
    for (auto& tag : tags) {
      auto child = computeDrawables(tag, state_stack);
      if (child)
        drawable_->children.push_back(std::move(child));
    }

    drawable_->setSize(view_);
  }
}