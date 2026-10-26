# Visage Manual Tests

An interactive app for checking behavior that the automated tests can't see: input from real
devices, windows and DPI changes, rendering quality, effects and platform integration. Each page
exercises one area and lists checks to mark as Pass, Fail or Skip.

It builds as `VisageManualTests` whenever `VISAGE_BUILD_TESTS` is on, including for Emscripten. It's
an app, not a ctest test.

```sh
cmake -S . -B build
cmake --build build --target VisageManualTests
```

The app is at `build/manual_tests/VisageManualTests.app` on macOS and
`build/manual_tests/VisageManualTests` (or `VisageManualTests.exe`) elsewhere.

### In a browser

```sh
emcmake cmake -S . -B build-web -DCMAKE_BUILD_TYPE=Release
cmake --build build-web --target VisageManualTests
python3 -m http.server 8000 --directory build-web/manual_tests/web
```

Then open `http://localhost:8000`. The browser build leaves out the pages that need extra windows
or file drag and drop (Secondary Windows, Palette Editing and File Drag and Drop). Pages that only
partly apply say so under their setup text. Results are kept in the site's IndexedDB storage, so
they survive reloads, and Export Report copies them to the clipboard. The automation environment
variables below don't apply.

## Using it

Pick a page in the sidebar, follow its setup text, and click each check to cycle it through Pass,
Fail, Skip and Untested. Right-click a check to pick a status directly. Each page has a notes box.

Results save as you go, to `VisageManualTests/results.txt` in the app data directory
(`~/Library/VisageManualTests` on macOS). Export Report writes `report.md` next to it, with
failures first, and copies it to the clipboard.

The Lifecycle page reports whether the previous session exited cleanly, using `session.txt` in the
same directory, so you can check quitting from the Dock, closing the main window, crashes and so on.

Some checks only apply to Release builds because Debug builds assert on purpose there. Their
toggles are disabled in Debug.

## Automation

These environment variables are read at startup. Run the binary directly from a terminal to pass
them and to see the output.

| Variable | Effect |
|---|---|
| `VISAGE_MANUAL_TESTS_SCREENSHOTS=<dir>` | Visits every page, saves `<page id>.png` to `<dir>` after 1.5 seconds on each, then quits. |
| `VISAGE_MANUAL_TESTS_MONKEY=<passes>` | Triggers every button and toggle and sweeps every slider on every page, `<passes>` times per page, printing each action to stdout, then quits. Skips quit, crash, message box, cursor warp, cursor hide and browser export. |
| `VISAGE_MANUAL_TESTS_FROM=<page id>` | Starts the screenshot or monkey run at that page. |
| `VISAGE_MANUAL_TESTS_AUTORUN=1` | Runs the System page's checks as soon as that page opens and prints PASS or FAIL for each. |

Page ids are listed in the `tests.push_back` calls in each `*_tests.cpp` file, for example
`mouse`, `windows`, `scrolling`, `shapes`, `svg_paint`, `stress` and `system`.

The monkey and screenshot runs update `last_page.txt` and `session.txt` like any other session.

## Adding checks

Each `add*Tests` function in a `*_tests.cpp` file registers pages with an id, a category, a title,
setup text, a list of checks and a function that creates the page's frame. A check is stored by the
page id and its index, so add new checks at the end of a list to keep existing results lined up.

The app uses the fonts, icons, images and shaders from `examples/`, embedded under the same names
the examples use.
