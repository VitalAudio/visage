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

#include <catch2/catch_session.hpp>
#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>
#include <cstdlib>

#if defined(__SANITIZE_ADDRESS__)
#define VISAGE_TEST_ASAN 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define VISAGE_TEST_ASAN 1
#endif
#endif

#if VISAGE_TEST_ASAN
#include <sanitizer/asan_interface.h>
#endif

namespace {
  Catch::Totals run_totals;

  class TotalsListener : public Catch::EventListenerBase {
  public:
    using EventListenerBase::EventListenerBase;

    void testRunEnded(const Catch::TestRunStats& stats) override { run_totals = stats.totals; }
  };
}

CATCH_REGISTER_LISTENER(TotalsListener)

int main(int argc, char* argv[]) {
#if VISAGE_TEST_ASAN
  std::atexit([] {
    if (__asan_report_present())
      std::_Exit(1);
  });
#endif

  int result = Catch::Session().run(argc, argv);
  if (run_totals.assertions.failed > 0 || run_totals.testCases.failed > 0)
    return 1;
  return result;
}
