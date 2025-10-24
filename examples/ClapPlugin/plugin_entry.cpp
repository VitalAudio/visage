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

#include "clap_plugin.h"

#include <cstring>

namespace plugin_entry {
  uint32_t getPluginCount(const clap_plugin_factory* f) {
    return 1;
  }

  const clap_plugin_descriptor* getPluginDescriptor(const clap_plugin_factory* f, uint32_t w) {
    return &ClapPlugin::descriptor;
  }

  static const clap_plugin* createPlugin(const clap_plugin_factory* factory, const clap_host* host,
                                         const char* plugin_id) {
    if (strcmp(plugin_id, ClapPlugin::descriptor.id) != 0)
      return nullptr;

    auto p = new ClapPlugin(host);
    return p->clapPlugin();
  }

  static const void* get_factory(const char* factory_id) {
    static constexpr clap_plugin_factory va_clap_plugin_factory = {
      plugin_entry::getPluginCount,
      plugin_entry::getPluginDescriptor,
      plugin_entry::createPlugin,
    };
    return !strcmp(factory_id, CLAP_PLUGIN_FACTORY_ID) ? &va_clap_plugin_factory : nullptr;
  }

  bool clap_init(const char* p) {
    return true;
  }
  void clap_deinit() { }
}

extern "C" {
extern const CLAP_EXPORT clap_plugin_entry clap_entry = { CLAP_VERSION, plugin_entry::clap_init,
                                                          plugin_entry::clap_deinit,
                                                          plugin_entry::get_factory };
}