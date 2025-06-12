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

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef UNICODE
#define UNICODE
#endif
#ifndef _UNICODE
#define _UNICODE
#endif

#include "windowing_win32.h"

#include "visage_utils/events.h"
#include "visage_utils/file_system.h"
#include "visage_utils/string_utils.h"
#include "visage_utils/thread_utils.h"

#include <algorithm>
#include <dxgi1_4.h>
#include <map>
#include <ShlObj.h>
#include <string>
#include <windowsx.h>

#define WM_VBLANK (WM_USER + 1)

typedef DPI_AWARENESS_CONTEXT(WINAPI* GetWindowDpiAwarenessContext_t)(HWND);
typedef DPI_AWARENESS_CONTEXT(WINAPI* GetThreadDpiAwarenessContext_t)();
typedef DPI_AWARENESS_CONTEXT(WINAPI* SetThreadDpiAwarenessContext_t)(DPI_AWARENESS_CONTEXT);
typedef UINT(WINAPI* GetDpiForWindow_t)(HWND);
typedef UINT(WINAPI* GetDpiForSystem_t)();
typedef INT(WINAPI* GetSystemMetricsForDpi_t)(INT, UINT);

namespace visage {
  template<typename T>
  static T procedure(HMODULE module, LPCSTR proc_name) {
    return reinterpret_cast<T>(GetProcAddress(module, proc_name));
  }

  class NativeWindowLookup {
  public:
    static NativeWindowLookup& instance() {
      static NativeWindowLookup instance;
      return instance;
    }

    void addWindow(WindowWin32* window) {
      native_window_lookup_[window->nativeHandle()] = window;
      if (window->parentHandle())
        parent_window_lookup_[window->parentHandle()] = window;
    }

    void removeWindow(WindowWin32* window) {
      if (parent_window_lookup_.count(window->parentHandle()))
        parent_window_lookup_.erase(window->parentHandle());
      if (native_window_lookup_.count(window->nativeHandle()))
        native_window_lookup_.erase(window->nativeHandle());
    }

    bool anyWindowOpen() const {
      for (auto& window : native_window_lookup_) {
        if (window.second->isShowing())
          return true;
      }
      return false;
    }

    WindowWin32* findByNativeHandle(HWND hwnd) {
      auto it = native_window_lookup_.find(hwnd);
      return it != native_window_lookup_.end() ? it->second : nullptr;
    }

    WindowWin32* findByNativeParentHandle(HWND hwnd) {
      auto it = parent_window_lookup_.find(hwnd);
      return it != parent_window_lookup_.end() ? it->second : nullptr;
    }

    WindowWin32* findWindow(HWND hwnd) {
      WindowWin32* window = findByNativeHandle(hwnd);
      if (window)
        return window;

      return findByNativeParentHandle(hwnd);
    }

    void closeAll() {
      auto windows = native_window_lookup_;
      for (auto& window : windows)
        window.second->close();
    }

  private:
    NativeWindowLookup() = default;
    ~NativeWindowLookup() = default;

    std::map<void*, WindowWin32*> parent_window_lookup_;
    std::map<void*, WindowWin32*> native_window_lookup_;
  };

  std::string readClipboardText() {
    if (!OpenClipboard(nullptr))
      return "";

    HANDLE h_data = GetClipboardData(CF_UNICODETEXT);
    if (h_data == nullptr) {
      CloseClipboard();
      return "";
    }

    std::wstring result;
    wchar_t* text = static_cast<wchar_t*>(GlobalLock(h_data));
    if (text != nullptr) {
      result = text;
      GlobalUnlock(h_data);
    }

    CloseClipboard();
    return String::convertToUtf8(result);
  }

  void setClipboardText(const std::string& text) {
    if (!OpenClipboard(nullptr))
      return;

    std::wstring w_text = String::convertToWide(text);
    EmptyClipboard();
    size_t size = (w_text.size() + 1) * sizeof(wchar_t);
    HGLOBAL h_data = GlobalAlloc(GMEM_MOVEABLE, size);
    if (h_data == nullptr) {
      CloseClipboard();
      return;
    }

    void* destination = GlobalLock(h_data);
    if (destination != nullptr) {
      memcpy(destination, w_text.c_str(), size);
      GlobalUnlock(h_data);
    }

    SetClipboardData(CF_UNICODETEXT, h_data);
    CloseClipboard();
  }

  void setCursorStyle(MouseCursor style) {
    static const HCURSOR arrow_cursor = LoadCursor(nullptr, IDC_ARROW);
    static const HCURSOR ibeam_cursor = LoadCursor(nullptr, IDC_IBEAM);
    static const HCURSOR crosshair_cursor = LoadCursor(nullptr, IDC_CROSS);
    static const HCURSOR pointing_cursor = LoadCursor(nullptr, IDC_HAND);
    static const HCURSOR horizontal_resize_cursor = LoadCursor(nullptr, IDC_SIZEWE);
    static const HCURSOR vertical_resize_cursor = LoadCursor(nullptr, IDC_SIZENS);
    static const HCURSOR multi_directional_resize_cursor = LoadCursor(nullptr, IDC_SIZEALL);

    HCURSOR cursor;
    switch (style) {
    case MouseCursor::Arrow: cursor = arrow_cursor; break;
    case MouseCursor::IBeam: cursor = ibeam_cursor; break;
    case MouseCursor::Crosshair: cursor = crosshair_cursor; break;
    case MouseCursor::Pointing: cursor = pointing_cursor; break;
    case MouseCursor::HorizontalResize: cursor = horizontal_resize_cursor; break;
    case MouseCursor::VerticalResize: cursor = vertical_resize_cursor; break;
    case MouseCursor::MultiDirectionalResize: cursor = multi_directional_resize_cursor; break;
    default: cursor = arrow_cursor; break;
    }

    WindowWin32::setCursor(cursor);
  }

  void setCursorVisible(bool visible) {
    ShowCursor(visible);
  }

  static WindowWin32* activeWindow() {
    HWND hwnd = GetActiveWindow();
    return hwnd ? NativeWindowLookup::instance().findByNativeHandle(hwnd) : nullptr;
  }

  Point cursorPosition() {
    POINT cursor_position;
    GetCursorPos(&cursor_position);

    WindowWin32* window = activeWindow();
    if (window == nullptr)
      return { cursor_position.x * 1.0f, cursor_position.y * 1.0f };

    ScreenToClient(window->windowHandle(), &cursor_position);
    return window->convertToLogical(IPoint(cursor_position.x, cursor_position.y));
  }

  void setCursorPosition(Point window_position) {
    WindowWin32* window = activeWindow();
    if (window == nullptr)
      return;

    POINT client_position = { 0, 0 };
    ClientToScreen(window->windowHandle(), &client_position);
    IPoint position = window->convertToNative(window_position);
    SetCursorPos(client_position.x + position.x, client_position.y + position.y);
  }

  void setCursorScreenPosition(Point screen_position) {
    WindowWin32* window = activeWindow();
    if (window == nullptr)
      return;

    IPoint position = window->convertToNative(screen_position);
    SetCursorPos(position.x, position.y);
  }

  class VBlankThread : public Thread {
  public:
    explicit VBlankThread(WindowWin32* window) : window_(window) { }

    ~VBlankThread() override {
      stop();
      clear();
    }

    void clear() {
      if (factory_) {
        factory_->Release();
        factory_ = nullptr;
      }

      for (auto& output : monitor_outputs_)
        output.second->Release();
      for (auto& adapter : adapters_)
        adapter->Release();

      monitor_outputs_.clear();
      adapters_.clear();
    }

    void updateMonitors() {
      clear();
      if (FAILED(CreateDXGIFactory(__uuidof(IDXGIFactory), reinterpret_cast<void**>(&factory_)))) {
        factory_ = nullptr;
        return;
      }

      IDXGIAdapter* adapter = nullptr;
      for (int i = 0; factory_->EnumAdapters(i, &adapter) != DXGI_ERROR_NOT_FOUND; ++i) {
        adapters_.push_back(adapter);
        IDXGIOutput* output = nullptr;
        for (int j = 0; adapter->EnumOutputs(j, &output) != DXGI_ERROR_NOT_FOUND; ++j) {
          DXGI_OUTPUT_DESC desc;
          if (SUCCEEDED(output->GetDesc(&desc)))
            monitor_outputs_[desc.Monitor] = output;
        }
      }
    }

    void run() override {
      updateMonitors();
      start_us_ = time::microseconds();

      while (shouldRun()) {
        HMONITOR monitor = window_->monitor();
        if (monitor_outputs_.count(monitor) == 0) {
          updateMonitors();
          sleep(1);
        }
        else if (SUCCEEDED(monitor_outputs_[monitor]->WaitForVBlank())) {
          long long us = time::microseconds() - start_us_;
          time_ = us * (1.0 / 1000000.0);
          SendMessage(window_->windowHandle(), WM_VBLANK, 0, 0);
        }
        else
          sleep(1);
      }
    }

    double vBlankTime() const { return time_.load(); }

  private:
    WindowWin32* window_ = nullptr;

    IDXGIFactory* factory_ = nullptr;
    std::vector<IDXGIAdapter*> adapters_;
    std::map<HMONITOR, IDXGIOutput*> monitor_outputs_;

    std::atomic<double> time_ = 0.0;
    long long start_us_ = 0;
  };

  class DpiAwareness {
  public:
    DpiAwareness() {
      HMODULE user32 = LoadLibraryA("user32.dll");
      if (user32 == nullptr)
        return;

      threadDpiAwarenessContext_ =
          procedure<GetThreadDpiAwarenessContext_t>(user32, "GetThreadDpiAwarenessContext");
      setThreadDpiAwarenessContext_ =
          procedure<SetThreadDpiAwarenessContext_t>(user32, "SetThreadDpiAwarenessContext");
      dpiForWindow_ = procedure<GetDpiForWindow_t>(user32, "GetDpiForWindow");
      dpiForSystem_ = procedure<GetDpiForSystem_t>(user32, "GetDpiForSystem");
      if (threadDpiAwarenessContext_ == nullptr || setThreadDpiAwarenessContext_ == nullptr ||
          dpiForWindow_ == nullptr || dpiForSystem_ == nullptr) {
        return;
      }

      previous_dpi_awareness_ = threadDpiAwarenessContext_();
      dpi_awareness_ = DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2;
      if (!setThreadDpiAwarenessContext_(dpi_awareness_)) {
        dpi_awareness_ = DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE;
        setThreadDpiAwarenessContext_(dpi_awareness_);
      }
    }

    ~DpiAwareness() {
      if (setThreadDpiAwarenessContext_)
        setThreadDpiAwarenessContext_(previous_dpi_awareness_);
    }

    float dpiScale() const {
      if (dpi_awareness_ == nullptr)
        return 1.0f;

      return dpiForSystem_() / Window::kDefaultDpi;
    }

    float dpiScale(HWND hwnd) const {
      if (dpi_awareness_ == nullptr)
        return 1.0f;

      return dpiForWindow_(hwnd) / Window::kDefaultDpi;
    }

  private:
    DPI_AWARENESS_CONTEXT dpi_awareness_ = nullptr;
    DPI_AWARENESS_CONTEXT previous_dpi_awareness_ = nullptr;
    GetThreadDpiAwarenessContext_t threadDpiAwarenessContext_ = nullptr;
    SetThreadDpiAwarenessContext_t setThreadDpiAwarenessContext_ = nullptr;
    GetDpiForWindow_t dpiForWindow_ = nullptr;
    GetDpiForSystem_t dpiForSystem_ = nullptr;
  };

  class DragDropSource : public IDropSource {
  public:
    DragDropSource() = default;
    virtual ~DragDropSource() = default;

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDropSource) {
        *ppv_object = static_cast<IDropSource*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG STDMETHODCALLTYPE Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    HRESULT STDMETHODCALLTYPE QueryContinueDrag(BOOL escape_pressed, DWORD key_state) override {
      if (escape_pressed)
        return DRAGDROP_S_CANCEL;

      if ((key_state & (MK_LBUTTON | MK_RBUTTON)) == 0)
        return DRAGDROP_S_DROP;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE GiveFeedback(DWORD effect) override {
      return DRAGDROP_S_USEDEFAULTCURSORS;
    }

  private:
    LONG ref_count_ = 1;
  };

  class DragDropEnumFormatEtc : public IEnumFORMATETC {
  public:
    DragDropEnumFormatEtc() = default;
    virtual ~DragDropEnumFormatEtc() = default;

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDataObject) {
        *ppv_object = static_cast<IEnumFORMATETC*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG STDMETHODCALLTYPE Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    HRESULT STDMETHODCALLTYPE Clone(IEnumFORMATETC** result) override {
      if (result == nullptr)
        return E_POINTER;

      auto newOne = new DragDropEnumFormatEtc();
      newOne->index_ = index_;
      *result = newOne;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE Next(ULONG celt, LPFORMATETC lpFormatEtc, ULONG* pceltFetched) override {
      if (pceltFetched != nullptr)
        *pceltFetched = 0;
      else if (celt != 1)
        return S_FALSE;

      if (index_ == 0 && celt > 0 && lpFormatEtc != nullptr) {
        lpFormatEtc[0].cfFormat = CF_HDROP;
        lpFormatEtc[0].ptd = nullptr;
        lpFormatEtc[0].dwAspect = DVASPECT_CONTENT;
        lpFormatEtc[0].lindex = -1;
        lpFormatEtc[0].tymed = TYMED_HGLOBAL;
        ++index_;

        if (pceltFetched != nullptr)
          *pceltFetched = 1;

        return S_OK;
      }

      return S_FALSE;
    }

    HRESULT STDMETHODCALLTYPE Skip(ULONG celt) override {
      if (index_ + static_cast<int>(celt) >= 1)
        return S_FALSE;

      index_ += static_cast<int>(celt);
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE Reset() override {
      index_ = 0;
      return S_OK;
    }

  private:
    int index_ = 0;
    LONG ref_count_ = 1;
  };

  class DragDropSourceObject : public IDataObject {
  public:
    static HDROP createHDrop(const visage::File& file) {
      std::wstring file_path = file.wstring();
      size_t file_bytes = (file_path.size() + 1) * sizeof(WCHAR);
      HDROP drop = static_cast<HDROP>(GlobalAlloc(GMEM_MOVEABLE | GMEM_ZEROINIT,
                                                  sizeof(DROPFILES) + file_bytes + 4));

      if (drop == nullptr)
        return nullptr;

      auto drop_files = static_cast<LPDROPFILES>(GlobalLock(drop));

      if (drop_files == nullptr) {
        GlobalFree(drop);
        return nullptr;
      }

      drop_files->pFiles = sizeof(DROPFILES);
      drop_files->fWide = true;

      WCHAR* name_location = reinterpret_cast<WCHAR*>(reinterpret_cast<char*>(drop_files) +
                                                      sizeof(DROPFILES));
      memcpy(name_location, file_path.data(), file_bytes);

      GlobalUnlock(drop);
      return drop;
    }

    explicit DragDropSourceObject(const File& file) :
        drop_(DragDropSourceObject::createHDrop(file)) { }

    virtual ~DragDropSourceObject() = default;

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDataObject) {
        *ppv_object = static_cast<IDataObject*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG STDMETHODCALLTYPE Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    static bool acceptsFormat(const FORMATETC* format_etc) {
      return (format_etc->dwAspect & DVASPECT_CONTENT) && format_etc->cfFormat == CF_HDROP &&
             (format_etc->tymed & TYMED_HGLOBAL);
    }

    HRESULT STDMETHODCALLTYPE GetData(FORMATETC* format_etc, STGMEDIUM* medium) override {
      if (!acceptsFormat(format_etc))
        return DV_E_FORMATETC;

      medium->tymed = format_etc->tymed;
      medium->pUnkForRelease = nullptr;

      if (format_etc->tymed != TYMED_HGLOBAL)
        return DV_E_FORMATETC;

      auto length = GlobalSize(drop_);
      void* const source = GlobalLock(drop_);
      void* const dest = GlobalAlloc(GMEM_FIXED, length);

      if (source != nullptr && dest != nullptr)
        memcpy(dest, source, length);

      GlobalUnlock(drop_);

      medium->hGlobal = dest;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE QueryGetData(FORMATETC* format_etc) override {
      if (acceptsFormat(format_etc))
        return S_OK;
      return DV_E_FORMATETC;
    }

    HRESULT STDMETHODCALLTYPE GetCanonicalFormatEtc(FORMATETC*, FORMATETC* format_etc_out) override {
      format_etc_out->ptd = nullptr;
      return E_NOTIMPL;
    }

    HRESULT STDMETHODCALLTYPE EnumFormatEtc(DWORD direction, IEnumFORMATETC** result) override {
      if (result == nullptr)
        return E_POINTER;

      if (direction == DATADIR_GET) {
        *result = new DragDropEnumFormatEtc();
        return S_OK;
      }

      *result = nullptr;
      return E_NOTIMPL;
    }

    HRESULT STDMETHODCALLTYPE GetDataHere(FORMATETC*, STGMEDIUM*) override { return E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE SetData(FORMATETC*, STGMEDIUM*, BOOL) override { return E_NOTIMPL; }

    HRESULT STDMETHODCALLTYPE DAdvise(FORMATETC*, DWORD, IAdviseSink*, DWORD*) override {
      return E_NOTIMPL;
    }

    HRESULT STDMETHODCALLTYPE DUnadvise(DWORD) override { return E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE EnumDAdvise(IEnumSTATDATA**) override { return E_NOTIMPL; }

  private:
    HDROP drop_;
    LONG ref_count_ = 1;
  };

  class DragDropTarget : public IDropTarget {
  public:
    explicit DragDropTarget(WindowWin32* window) : window_(window) { }
    virtual ~DragDropTarget() = default;

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDropTarget) {
        *ppv_object = this;
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG STDMETHODCALLTYPE Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    IPoint dragPosition(POINTL point) const {
      POINT position = { point.x, point.y };
      ScreenToClient(static_cast<HWND>(window_->nativeHandle()), &position);
      return { static_cast<int>(position.x), static_cast<int>(position.y) };
    }

    HRESULT STDMETHODCALLTYPE DragEnter(IDataObject* data_object, DWORD key_state, POINTL point,
                                        DWORD* effect) override {
      IPoint position = dragPosition(point);
      files_ = dropFileList(data_object);
      if (window_->handleFileDrag(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE DragOver(DWORD key_state, POINTL point, DWORD* effect) override {
      IPoint position = dragPosition(point);
      if (window_->handleFileDrag(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE DragLeave() override {
      window_->handleFileDragLeave();
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE Drop(IDataObject* data_object, DWORD key_state, POINTL point,
                                   DWORD* effect) override {
      IPoint position = dragPosition(point);
      files_ = dropFileList(data_object);
      if (window_->handleFileDrop(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    static std::vector<std::string> dropFileList(IDataObject* data_object) {
      std::vector<std::string> files;
      FORMATETC format = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_HGLOBAL };
      STGMEDIUM storage = { TYMED_HGLOBAL };

      if (SUCCEEDED(data_object->GetData(&format, &storage))) {
        HDROP h_drop = static_cast<HDROP>(GlobalLock(storage.hGlobal));

        if (h_drop != nullptr) {
          UINT file_count = DragQueryFile(h_drop, 0xFFFFFFFF, nullptr, 0);
          WCHAR file_path[MAX_PATH];

          for (UINT i = 0; i < file_count; ++i) {
            if (DragQueryFile(h_drop, i, file_path, MAX_PATH)) {
              std::string file = String::convertToUtf8(file_path);
              files.emplace_back(file);
            }
          }

          GlobalUnlock(storage.hGlobal);
        }
        ReleaseStgMedium(&storage);
      }
      return files;
    }

  private:
    std::vector<std::string> files_;
    ULONG ref_count_ = 1;
    WindowWin32* window_ = nullptr;
  };

  bool isMobileDevice() {
    return false;
  }

  void showMessageBox(std::string title, std::string message) {
    std::wstring w_title = String::convertToWide(title);
    std::wstring w_message = String::convertToWide(message);
    MessageBox(nullptr, w_message.c_str(), w_title.c_str(), MB_OK);
  }

  static KeyCode keyCodeFromScanCode(WPARAM w_param, LPARAM l_param) {
    static constexpr int kCodeTableSize = 128;

    static constexpr KeyCode kWin32KeyCodeTable[kCodeTableSize] = {
      KeyCode::Unknown,
      KeyCode::Escape,
      KeyCode::Number1,
      KeyCode::Number2,
      KeyCode::Number3,
      KeyCode::Number4,
      KeyCode::Number5,
      KeyCode::Number6,
      KeyCode::Number7,
      KeyCode::Number8,
      KeyCode::Number9,
      KeyCode::Number0,
      KeyCode::Minus,
      KeyCode::Equals,
      KeyCode::Backspace,
      KeyCode::Tab,
      KeyCode::Q,
      KeyCode::W,
      KeyCode::E,
      KeyCode::R,
      KeyCode::T,
      KeyCode::Y,
      KeyCode::U,
      KeyCode::I,
      KeyCode::O,
      KeyCode::P,
      KeyCode::LeftBracket,
      KeyCode::RightBracket,
      KeyCode::Return,
      KeyCode::LCtrl,
      KeyCode::A,
      KeyCode::S,
      KeyCode::D,
      KeyCode::F,
      KeyCode::G,
      KeyCode::H,
      KeyCode::J,
      KeyCode::K,
      KeyCode::L,
      KeyCode::Semicolon,
      KeyCode::Apostrophe,
      KeyCode::Grave,
      KeyCode::LShift,
      KeyCode::Backslash,
      KeyCode::Z,
      KeyCode::X,
      KeyCode::C,
      KeyCode::V,
      KeyCode::B,
      KeyCode::N,
      KeyCode::M,
      KeyCode::Comma,
      KeyCode::Period,
      KeyCode::Slash,
      KeyCode::RShift,
      KeyCode::PrintScreen,
      KeyCode::LAlt,
      KeyCode::Space,
      KeyCode::CapsLock,
      KeyCode::F1,
      KeyCode::F2,
      KeyCode::F3,
      KeyCode::F4,
      KeyCode::F5,
      KeyCode::F6,
      KeyCode::F7,
      KeyCode::F8,
      KeyCode::F9,
      KeyCode::F10,
      KeyCode::NumLock,
      KeyCode::ScrollLock,
      KeyCode::Home,
      KeyCode::Up,
      KeyCode::PageUp,
      KeyCode::KPMinus,
      KeyCode::Left,
      KeyCode::KP5,
      KeyCode::Right,
      KeyCode::KPPlus,
      KeyCode::End,
      KeyCode::Down,
      KeyCode::PageDown,
      KeyCode::Insert,
      KeyCode::Delete,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::NonUSBackslash,
      KeyCode::F11,
      KeyCode::F12,
      KeyCode::Pause,
      KeyCode::Unknown,
      KeyCode::LGui,
      KeyCode::RGui,
      KeyCode::Application,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::F13,
      KeyCode::F14,
      KeyCode::F15,
      KeyCode::F16,
      KeyCode::F17,
      KeyCode::F18,
      KeyCode::F19,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::International2,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::International1,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::International4,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::Unknown,
      KeyCode::International3,
      KeyCode::Unknown,
      KeyCode::Unknown,
    };

    switch (w_param) {
    case VK_MEDIA_NEXT_TRACK: return KeyCode::AudioNext;
    case VK_MEDIA_PREV_TRACK: return KeyCode::AudioPrev;
    case VK_MEDIA_STOP: return KeyCode::AudioStop;
    case VK_MEDIA_PLAY_PAUSE: return KeyCode::AudioPlay;
    case VK_UP: return KeyCode::Up;
    case VK_DOWN: return KeyCode::Down;
    case VK_LEFT: return KeyCode::Left;
    case VK_RIGHT: return KeyCode::Right;
    default: break;
    }

    int scan_code = (l_param >> 16) & 0xFF;
    if (scan_code >= kCodeTableSize)
      return KeyCode::Unknown;

    if (GetKeyState(VK_NUMLOCK) & 0x01) {
      switch (scan_code) {
      case 0x47: return KeyCode::KP7;
      case 0x48: return KeyCode::KP8;
      case 0x49: return KeyCode::KP9;
      case 0x4B: return KeyCode::KP4;
      case 0x4C: return KeyCode::KP5;
      case 0x4D: return KeyCode::KP6;
      case 0x4F: return KeyCode::KP1;
      case 0x50: return KeyCode::KP2;
      case 0x51: return KeyCode::KP3;
      case 0x52: return KeyCode::KP0;
      case 0x53: return KeyCode::KPPeriod;
      default: break;
      }
    }

    return kWin32KeyCodeTable[scan_code];
  }

  static HMODULE WINAPI loadModuleHandle() {
    HMODULE module_handle;
    int flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
    BOOL success = GetModuleHandleEx(flags, reinterpret_cast<LPCWSTR>(&loadModuleHandle), &module_handle);
    if (success && module_handle)
      return module_handle;

    return GetModuleHandle(nullptr);
  }

  static int keyboardModifiers() {
    int modifiers = 0;
    if (GetKeyState(VK_SHIFT) & 0x8000)
      modifiers |= kModifierShift;
    if (GetKeyState(VK_CONTROL) & 0x8000)
      modifiers |= kModifierRegCtrl;
    if (GetKeyState(VK_MENU) & 0x8000)
      modifiers |= kModifierAlt;
    if (GetKeyState(VK_LWIN) & 0x8000 || GetKeyState(VK_RWIN) & 0x8000)
      modifiers |= kModifierMeta;
    return modifiers;
  }

  static int mouseButtonState() {
    int state = 0;
    if (GetKeyState(VK_LBUTTON) & 0x8000)
      state |= kMouseButtonLeft;
    if (GetKeyState(VK_RBUTTON) & 0x8000)
      state |= kMouseButtonRight;
    if (GetKeyState(VK_MBUTTON) & 0x8000)
      state |= kMouseButtonMiddle;
    return state;
  }

  static bool isTouchEvent() {  // TODO check if mouse down is touch
    return (GetMessageExtraInfo() & 0xFFFFFF00) == 0xFF515700;
  }

  static int mouseButtonState(WPARAM w_param) {
    int state = 0;
    if (w_param & MK_LBUTTON)
      state |= kMouseButtonLeft;
    if (w_param & MK_RBUTTON)
      state |= kMouseButtonRight;
    if (w_param & MK_MBUTTON)
      state |= kMouseButtonMiddle;
    return state;
  }

  static void WINAPI postMessageToParent(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    HWND parent = GetParent(hwnd);
    if (parent)
      PostMessage(parent, msg, w_param, l_param);
    else
      DefWindowProc(hwnd, msg, w_param, l_param);
  }

  static bool isWindowOccluded(HWND hwnd) {
    RECT rect;
    if (!GetWindowRect(hwnd, &rect))
      return false;

    auto next_x = [](HWND hit_hwnd, LONG x, LONG right) -> LONG {
      if (hit_hwnd == nullptr)
        return right;

      RECT window_rect;
      if (!GetWindowRect(hit_hwnd, &window_rect))
        return right;

      return std::min(right, std::max(x, window_rect.right + 1));
    };

    auto next_y = [](HWND hit_hwnd, LONG y, LONG bottom) -> LONG {
      if (hit_hwnd == nullptr)
        return bottom;

      RECT window_rect;
      if (!GetWindowRect(hit_hwnd, &window_rect))
        return bottom;

      return std::min(bottom, std::max(y, window_rect.bottom + 1));
    };

    LONG x = rect.left;
    while (x < rect.right) {
      HWND hit_hwnd = WindowFromPoint(POINT { x, rect.top });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      x = next_x(hit_hwnd, x, rect.right);
    }

    x = rect.left;
    while (x < rect.right) {
      HWND hit_hwnd = WindowFromPoint(POINT { x, rect.bottom });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      x = next_x(hit_hwnd, x, rect.right);
    }

    LONG y = rect.top;
    while (y < rect.bottom) {
      HWND hit_hwnd = WindowFromPoint(POINT { rect.left, y });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      y = next_y(hit_hwnd, y, rect.bottom);
    }

    y = rect.top;
    while (y < rect.bottom) {
      HWND hit_hwnd = WindowFromPoint(POINT { rect.right, y });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      y = next_y(hit_hwnd, y, rect.bottom);
    }

    return true;
  }

  static LRESULT WINAPI windowProcedure(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    WindowWin32* window = reinterpret_cast<WindowWin32*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (window == nullptr)
      return DefWindowProc(hwnd, msg, w_param, l_param);

    return window->handleWindowProc(hwnd, msg, w_param, l_param);
  }

  LRESULT WindowWin32::handleWindowProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    switch (msg) {
    case WM_VBLANK: {
      drawCallback(v_blank_thread_->vBlankTime());
      return 0;
    }
    case WM_SYSKEYDOWN:
    case WM_KEYDOWN: {
      KeyCode key_code = keyCodeFromScanCode(w_param, l_param);
      bool is_repeat = (l_param & (1LL << 30LL)) != 0;
      if (!handleKeyDown(key_code, keyboardModifiers(), is_repeat))
        postMessageToParent(hwnd, msg, w_param, l_param);
      return 0;
    }
    case WM_SYSKEYUP:
    case WM_KEYUP: {
      KeyCode key_code = keyCodeFromScanCode(w_param, l_param);
      if (!handleKeyUp(key_code, keyboardModifiers()))
        postMessageToParent(hwnd, msg, w_param, l_param);
      return 0;
    }
    case WM_SYSCHAR:
    case WM_CHAR: {
      handleCharacterEntry(static_cast<wchar_t>(w_param));
      SetCaretPos(-500, 200);
      ShowCaret(hwnd);
      return 0;
    }
    case WM_NCMOUSEMOVE: {
      POINT position = { GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param) };
      ScreenToClient(window_handle_, &position);
      handleMouseMove(position.x, position.y, mouseButtonState(w_param), keyboardModifiers());
      break;
    }
    case WM_MOUSEMOVE: {
      int x = GET_X_LPARAM(l_param);
      int y = GET_Y_LPARAM(l_param);

      if (!isMouseTracked()) {
        setMouseTracked(true);

        TRACKMOUSEEVENT track {};
        track.cbSize = sizeof(track);
        track.dwFlags = TME_LEAVE;
        track.hwndTrack = hwnd;
        TrackMouseEvent(&track);
      }

      handleMouseMove(x, y, mouseButtonState(w_param), keyboardModifiers());
      if (mouseRelativeMode()) {
        IPoint last_position = lastWindowMousePosition();
        POINT client_position = { last_position.x, last_position.y };
        ClientToScreen(hwnd, &client_position);
        SetCursorPos(client_position.x, client_position.y);
      }

      return 0;
    }
    case WM_NCMOUSELEAVE: {
      if (currentHitTest() != HitTestResult::Client) {
        setMouseTracked(false);
        handleMouseLeave(mouseButtonState(0), keyboardModifiers());
      }
      break;
    }
    case WM_MOUSELEAVE: {
      if (currentHitTest() == HitTestResult::Client) {
        setMouseTracked(false);
        handleMouseLeave(mouseButtonState(0), keyboardModifiers());
      }
      return 0;
    }
    case WM_LBUTTONDOWN: {
      SetFocus(static_cast<HWND>(nativeHandle()));
      SetFocus(hwnd);
      handleMouseDown(kMouseButtonLeft, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      mouseButtonState(w_param), keyboardModifiers());

      if (isDragDropSource()) {
        visage::File file = startDragDropSource();
        DragDropSource* drop_source = new DragDropSource();
        DragDropSourceObject* data_object = new DragDropSourceObject(file);

        DWORD effect;
        DoDragDrop(data_object, drop_source, DROPEFFECT_COPY, &effect);

        drop_source->Release();
        data_object->Release();
        cleanupDragDropSource();
      }
      else
        SetCapture(hwnd);

      return 0;
    }
    case WM_NCLBUTTONDOWN: {
      if (w_param == HTCLOSE || w_param == HTMAXBUTTON || w_param == HTMINBUTTON)
        return 0;
      break;
    }
    case WM_NCLBUTTONUP: {
      if (w_param == HTCLOSE) {
        PostMessage(hwnd, WM_CLOSE, 0, 0);
        return 0;
      }
      else if (w_param == HTMAXBUTTON) {
        ShowWindow(hwnd, IsZoomed(hwnd) ? SW_RESTORE : SW_MAXIMIZE);
        return 0;
      }
      else if (w_param == HTMINBUTTON) {
        ShowWindow(hwnd, SW_MINIMIZE);
        return 0;
      }
      break;
    }
    case WM_LBUTTONUP: {
      int button_state = mouseButtonState(w_param);
      handleMouseUp(kMouseButtonLeft, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    keyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_RBUTTONDOWN: {
      SetFocus(hwnd);
      SetCapture(hwnd);
      handleMouseDown(kMouseButtonRight, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      mouseButtonState(w_param), keyboardModifiers());
      return 0;
    }
    case WM_RBUTTONUP: {
      int button_state = mouseButtonState(w_param);
      handleMouseUp(kMouseButtonRight, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    keyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_MBUTTONDOWN: {
      SetFocus(hwnd);
      SetCapture(hwnd);
      handleMouseDown(kMouseButtonMiddle, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      mouseButtonState(w_param), keyboardModifiers());
      return 0;
    }
    case WM_MBUTTONUP: {
      int button_state = mouseButtonState(w_param);
      handleMouseUp(kMouseButtonMiddle, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    keyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_SETCURSOR: {
      if (LOWORD(l_param) == HTCLIENT) {
        SetCursor(WindowWin32::cursor());
        return TRUE;
      }
      break;
    }
    case WM_MOUSEWHEEL:
    case WM_MOUSEHWHEEL: {
      float delta = GET_WHEEL_DELTA_WPARAM(w_param) * 1.0f / WHEEL_DELTA;
      POINT position { GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param) };
      ScreenToClient(hwnd, &position);
      float delta_x = msg == WM_MOUSEHWHEEL ? delta : 0.0f;
      float delta_y = msg == WM_MOUSEWHEEL ? delta : 0.0f;
      handleMouseWheel(delta_x, delta_y, position.x, position.y, mouseButtonState(), keyboardModifiers());
      return 0;
    }
    case WM_KILLFOCUS: {
      handleFocusLost();
      return 0;
    }
    case WM_SETFOCUS: {
      handleFocusGained();
      return 0;
    }
    case WM_SIZING: {
      handleResizing(hwnd, l_param, w_param);
      return TRUE;
    }
    case WM_SIZE: {
      handleResizeEnd(hwnd);
      return TRUE;
    }
    case WM_EXITSIZEMOVE: {
      handleResizeEnd(hwnd);
      return 0;
    }
    case WM_DPICHANGED: {
      handleDpiChange(hwnd, l_param, w_param);
      return 0;
    }
    case WM_MOVE:
    case WM_DISPLAYCHANGE: {
      updateMonitor();
      return 0;
    }
    default: break;
    }
    return DefWindowProc(hwnd, msg, w_param, l_param);
  }

  static LRESULT WINAPI pluginParentWindowProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    WindowWin32* child_window = NativeWindowLookup::instance().findWindow(hwnd);
    if (child_window == nullptr)
      return 0;

    if (msg == WM_SIZING) {
      child_window->handleResizing(hwnd, l_param, w_param);
      return TRUE;
    }
    if (msg == WM_DPICHANGED) {
      child_window->handleDpiChange(hwnd, l_param, w_param);
      return 0;
    }
    return CallWindowProc(child_window->parentWindowProc(), hwnd, msg, w_param, l_param);
  }

  static LRESULT WINAPI standaloneWindowProcedure(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    WindowWin32* window = reinterpret_cast<WindowWin32*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (window == nullptr)
      return DefWindowProc(hwnd, msg, w_param, l_param);

    if (msg == WM_DESTROY) {
      window->hide();
      NativeWindowLookup::instance().removeWindow(window);
      if (!NativeWindowLookup::instance().anyWindowOpen())
        PostQuitMessage(0);
      return 0;
    }
    if (msg == WM_NCCALCSIZE && window->decoration() == Window::Decoration::Client) {
      NCCALCSIZE_PARAMS* params = reinterpret_cast<NCCALCSIZE_PARAMS*>(l_param);
      if (IsZoomed(hwnd)) {
        HMONITOR monitor = MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST);
        MONITORINFO monitor_info = {};
        monitor_info.cbSize = sizeof(MONITORINFO);
        GetMonitorInfo(monitor, &monitor_info);
        params->rgrc[0] = monitor_info.rcWork;
        return 0;
      }

      HMODULE user32 = LoadLibraryA("user32.dll");
      auto dpiForWindow = procedure<GetDpiForWindow_t>(user32, "GetDpiForWindow");
      auto dpi = dpiForWindow(hwnd);
      auto systemMetrics = procedure<GetSystemMetricsForDpi_t>(user32, "GetSystemMetricsForDpi");
      params->rgrc[0].top -= systemMetrics(SM_CYCAPTION, dpi) + systemMetrics(SM_CYSIZEFRAME, dpi) +
                             systemMetrics(SM_CXPADDEDBORDER, dpi);
    }
    if (msg == WM_NCHITTEST && window->decoration() == Window::Decoration::Client) {
      LRESULT result = DefWindowProc(hwnd, msg, w_param, l_param);
      if (result != HTCLIENT)
        return result;

      POINT position = { GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param) };
      ScreenToClient(hwnd, &position);
      HitTestResult hit_test = window->handleHitTest(position.x, position.y);

      switch (hit_test) {
      case HitTestResult::TitleBar: return HTCAPTION;
      case HitTestResult::CloseButton: return HTCLOSE;
      case HitTestResult::MaximizeButton: return HTMAXBUTTON;
      case HitTestResult::MinimizeButton: return HTMINBUTTON;
      default: return HTCLIENT;
      }
    }
    if (msg == WM_QUERYENDSESSION)
      return window->closeRequested();
    if (msg == WM_CLOSE) {
      if (!window->closeRequested())
        return 0;
    }
    return windowProcedure(hwnd, msg, w_param, l_param);
  }

  static IBounds boundsInMonitor(HMONITOR monitor, float dpi_scale, const Dimension& x,
                                 const Dimension& y, const Dimension& width, const Dimension& height) {
    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor, &monitor_info);

    int monitor_width = monitor_info.rcWork.right - monitor_info.rcWork.left;
    int monitor_height = monitor_info.rcWork.bottom - monitor_info.rcWork.top;
    int bounds_width = width.computeInt(dpi_scale, monitor_width, monitor_height);
    int bounds_height = height.computeInt(dpi_scale, monitor_width, monitor_height);

    int default_x = monitor_info.rcWork.left + (monitor_width - bounds_width) / 2;
    int default_y = monitor_info.rcWork.top + (monitor_height - bounds_height) / 2;
    int bounds_x = x.computeInt(dpi_scale, monitor_width, monitor_height, default_x);
    int bounds_y = y.computeInt(dpi_scale, monitor_width, monitor_height, default_y);
    return { bounds_x, bounds_y, bounds_width, bounds_height };
  }

  static IBounds windowBorderSize(HWND hwnd) {
    WINDOWINFO info {};
    info.cbSize = sizeof(info);
    if (!GetWindowInfo(hwnd, &info))
      return {};

    int x = info.rcWindow.left - info.rcClient.left;
    int y = info.rcWindow.top - info.rcClient.top;
    int width = -x + info.rcWindow.right - info.rcClient.right;
    int height = -y + info.rcWindow.bottom - info.rcClient.bottom;
    return { x, y, width, height };
  }

  static void clearMessage(MSG* message) {
    *message = {};
    message->message = WM_USER;
  }

  LRESULT CALLBACK EventHooks::eventHook(int code, WPARAM w_param, LPARAM l_param) {
    if (code == HC_ACTION && w_param == PM_REMOVE) {
      MSG* message = reinterpret_cast<MSG*>(l_param);
      WindowWin32* window = NativeWindowLookup::instance().findByNativeHandle(message->hwnd);

      if (window && window->handleHookedMessage(message)) {
        clearMessage(message);
        return 0;
      }
    }

    return CallNextHookEx(event_hook_, code, w_param, l_param);
  }

  HHOOK EventHooks::event_hook_ = nullptr;
  int EventHooks::instance_count_ = 0;

  EventHooks::EventHooks() {
    if (instance_count_++ == 0 && event_hook_ == nullptr)
      event_hook_ = SetWindowsHookEx(WH_GETMESSAGE, eventHook, loadModuleHandle(), GetCurrentThreadId());
  }

  EventHooks::~EventHooks() {
    instance_count_--;
    if (instance_count_ == 0 && event_hook_) {
      UnhookWindowsHookEx(event_hook_);
      event_hook_ = nullptr;
    }
  }

  void WindowWin32::runEventLoop() {
    MSG message = {};
    while (GetMessage(&message, nullptr, 0, 0)) {
      TranslateMessage(&message);
      DispatchMessage(&message);
    }
  }

  HCURSOR WindowWin32::cursor_ = LoadCursor(nullptr, IDC_ARROW);

  void WindowWin32::setCursor(HCURSOR cursor) {
    cursor_ = cursor;
    SetCursor(cursor);
  }

  void WindowWin32::registerWindowClass() {
    HRESULT result = OleInitialize(nullptr);
    if (result != S_OK && result != S_FALSE)
      VISAGE_LOG("Error initializing OLE");

    module_handle_ = loadModuleHandle();

    std::wstring unique_string = std::to_wstring(reinterpret_cast<uintptr_t>(this));
    std::string app_name = VISAGE_APPLICATION_NAME;
    unique_window_class_name_ = String::convertToWide(app_name) + L"_" + unique_string;

    window_class_.cbSize = sizeof(WNDCLASSEX);
    window_class_.style = CS_OWNDC;
    window_class_.cbClsExtra = 0;
    window_class_.cbWndExtra = 0;
    window_class_.hInstance = module_handle_;
    window_class_.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    window_class_.hCursor = LoadCursor(nullptr, IDC_ARROW);
    window_class_.hbrBackground = CreateSolidBrush(RGB(32, 32, 32));
    window_class_.lpszMenuName = nullptr;
    window_class_.lpszClassName = unique_window_class_name_.c_str();
#if VISAGE_WINDOWS_ICON_RESOURCE
    window_class_.hIcon = LoadIcon(module_handle_, MAKEINTRESOURCE(VA_WINDOWS_ICON_RESOURCE));
    window_class_.hIconSm = LoadIcon(module_handle_, MAKEINTRESOURCE(VA_WINDOWS_ICON_RESOURCE));
#else
    window_class_.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    window_class_.hIconSm = LoadIcon(nullptr, IDI_APPLICATION);
#endif

    drag_drop_target_ = new DragDropTarget(this);
  }

  float defaultDpiScale() {
    DpiAwareness dpi_awareness;
    return dpi_awareness.dpiScale();
  }

  IBounds computeWindowBounds(const Dimension& x, const Dimension& y, const Dimension& width,
                              const Dimension& height) {
    DpiAwareness dpi_awareness;
    POINT cursor_position;
    GetCursorPos(&cursor_position);
    float dpi_scale = dpi_awareness.dpiScale();
    int x_position = x.computeInt(dpi_scale, 0, 0, cursor_position.x);
    int y_position = y.computeInt(dpi_scale, 0, 0, cursor_position.y);

    HMONITOR monitor = MonitorFromPoint({ x_position, y_position }, MONITOR_DEFAULTTONEAREST);
    return boundsInMonitor(monitor, dpi_scale, x, y, width, height);
  }

  std::unique_ptr<Window> createWindow(const Dimension& x, const Dimension& y, const Dimension& width,
                                       const Dimension& height, Window::Decoration decoration_style) {
    IBounds bounds = computeWindowBounds(x, y, width, height);
    return std::make_unique<WindowWin32>(bounds.x(), bounds.y(), bounds.width(), bounds.height(),
                                         decoration_style);
  }

  void* headlessWindowHandle() {
    return nullptr;
  }

  void closeApplication() {
    NativeWindowLookup::instance().closeAll();
  }

  std::unique_ptr<Window> createPluginWindow(const Dimension& width, const Dimension& height,
                                             void* parent_handle) {
    IBounds bounds = computeWindowBounds(0, 0, width, height);
    return std::make_unique<WindowWin32>(bounds.width(), bounds.height(), parent_handle);
  }

  WindowWin32::WindowWin32(int x, int y, int width, int height, Window::Decoration decoration) :
      Window(width, height), decoration_(decoration) {
    static constexpr int kWindowFlags = WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX |
                                        WS_MAXIMIZEBOX;
    static constexpr int kPopupFlags = WS_POPUP;

    DpiAwareness dpi_awareness;
    setDpiScale(dpi_awareness.dpiScale());

    registerWindowClass();
    window_class_.lpfnWndProc = standaloneWindowProcedure;
    RegisterClassEx(&window_class_);

    int flags = kWindowFlags;
    if (decoration_ == Window::Decoration::Popup)
      flags = kPopupFlags;

    std::string app_name = VISAGE_APPLICATION_NAME;
    window_handle_ = CreateWindow(window_class_.lpszClassName,
                                  String::convertToWide(app_name).c_str(), flags, x, y, width,
                                  height, nullptr, nullptr, window_class_.hInstance, nullptr);

    if (window_handle_ == nullptr) {
      VISAGE_LOG("Error creating window");
      return;
    }

    IBounds borders = windowBorderSize(window_handle_);
    int window_height = height + borders.height();
    if (decoration_ == Window::Decoration::Client)
      window_height = height + borders.bottom() + 2;

    SetWindowPos(window_handle_, nullptr, x - borders.width() / 2, y, width + borders.width(),
                 window_height, SWP_NOZORDER);

    SetWindowLongPtr(window_handle_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));
    finishWindowSetup();
  }

  WindowWin32::WindowWin32(int width, int height, void* parent_handle) : Window(width, height) {
    static constexpr int kWindowFlags = WS_CHILD;

    DpiAwareness dpi_awareness;
    setDpiScale(dpi_awareness.dpiScale());

    registerWindowClass();
    window_class_.lpfnWndProc = windowProcedure;
    RegisterClassEx(&window_class_);

    parent_handle_ = static_cast<HWND>(parent_handle);
    std::string app_name = VISAGE_APPLICATION_NAME;
    window_handle_ = CreateWindow(window_class_.lpszClassName,
                                  String::convertToWide(app_name).c_str(), kWindowFlags, 0, 0, width,
                                  height, parent_handle_, nullptr, window_class_.hInstance, nullptr);
    if (window_handle_ == nullptr) {
      VISAGE_LOG("Error creating window");
      return;
    }

    SetWindowLongPtr(window_handle_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

    auto parent_proc = SetWindowLongPtr(parent_handle_, GWLP_WNDPROC,
                                        reinterpret_cast<LONG_PTR>(pluginParentWindowProc));
    parent_window_proc_ = reinterpret_cast<WNDPROC>(parent_proc);

    event_hooks_ = std::make_unique<EventHooks>();
    finishWindowSetup();
  }

  void WindowWin32::finishWindowSetup() {
    NativeWindowLookup::instance().addWindow(this);

    UpdateWindow(window_handle_);
    if (drag_drop_target_)
      RegisterDragDrop(window_handle_, drag_drop_target_);

    updateMonitor();
  }

  WindowWin32::~WindowWin32() {
    if (drag_drop_target_) {
      RevokeDragDrop(window_handle_);
      drag_drop_target_->Release();
    }

    if (parent_handle_)
      SetWindowLongPtr(parent_handle_, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(parent_window_proc_));

    NativeWindowLookup::instance().removeWindow(this);
    KillTimer(window_handle_, kTimerId);
    DestroyWindow(window_handle_);
    UnregisterClass(window_class_.lpszClassName, module_handle_);
    OleUninitialize();
  }

  void WindowWin32::windowContentsResized(int width, int height) {
    if (window_handle_ == nullptr)
      return;

    DpiAwareness dpi_awareness;
    RECT rect;
    GetWindowRect(window_handle_, &rect);
    int x = rect.left;
    int y = rect.top;
    LONG rect_width = static_cast<LONG>(width);
    LONG rect_height = static_cast<LONG>(height);
    rect.right = rect.left + rect_width;
    rect.bottom = rect.top + rect_height;

    IBounds borders = windowBorderSize(window_handle_);
    SetWindowPos(window_handle_, nullptr, x, y, rect.right - rect.left + borders.width(),
                 rect.bottom - rect.top + borders.height(), SWP_NOZORDER | SWP_NOMOVE);
  }

  void WindowWin32::show(int show_flag) {
    ShowWindow(window_handle_, show_flag);
    SetFocus(window_handle_);

    if (v_blank_thread_ == nullptr) {
      v_blank_thread_ = std::make_unique<VBlankThread>(this);
      v_blank_thread_->start();
    }
    notifyShow();
  }

  void WindowWin32::show() {
    show(SW_SHOWNORMAL);
    SetWindowPos(window_handle_, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_FRAMECHANGED);
  }

  void WindowWin32::hide() {
    ShowWindow(window_handle_, SW_HIDE);
    notifyHide();
  }

  void WindowWin32::close() {
    PostMessage(window_handle_, WM_CLOSE, 0, 0);
  }

  bool WindowWin32::isShowing() const {
    return IsWindowVisible(window_handle_);
  }

  void WindowWin32::showMaximized() {
    show(SW_MAXIMIZE);
  }

  void WindowWin32::setWindowTitle(const std::string& title) {
    std::wstring w_title = String::convertToWide(title);
    SetWindowText(window_handle_, w_title.c_str());
  }

  IPoint WindowWin32::maxWindowDimensions() const {
    IBounds borders = windowBorderSize(window_handle_);
    if (borders.width() == 0 && borders.height() == 0 && parent_handle_)
      borders = windowBorderSize(parent_handle_);

    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor_, &monitor_info);

    int display_width = monitor_info.rcWork.right - monitor_info.rcWork.left - borders.width();
    int display_height = monitor_info.rcWork.bottom - monitor_info.rcWork.top - borders.height();

    float aspect_ratio = aspectRatio();
    int width_from_height = static_cast<int>(display_height * aspect_ratio);
    int height_from_width = static_cast<int>(display_width / aspect_ratio);
    return { std::min<int>(display_width, width_from_height),
             std::min<int>(display_height, height_from_width) };
  }

  IPoint WindowWin32::minWindowDimensions() const {
    float scale = minimumWindowScale();
    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor_, &monitor_info);

    int min_width = static_cast<int>(scale * (monitor_info.rcWork.right - monitor_info.rcWork.left));
    int min_height = static_cast<int>(scale * (monitor_info.rcWork.bottom - monitor_info.rcWork.top));
    float aspect_ratio = aspectRatio();
    return { std::max<int>(min_width, static_cast<int>(min_height * aspect_ratio)),
             std::max<int>(min_height, static_cast<int>(min_width / aspect_ratio)) };
  }

  static bool is2ByteCharacter(wchar_t character) {
    return character < 0xD800 || character >= 0xDC00;
  }

  bool WindowWin32::handleCharacterEntry(wchar_t character) {
    if (!hasActiveTextEntry())
      return false;

    bool first_character = utf16_string_entry_.empty();
    utf16_string_entry_.push_back(character);
    if (!first_character || is2ByteCharacter(character)) {
      handleTextInput(String::convertToUtf8(utf16_string_entry_));
      utf16_string_entry_ = L"";
    }
    return true;
  }

  bool WindowWin32::handleHookedMessage(const MSG* message) {
    // TODO: Plugin window doesn't scale right if you resize the windows using
    // a key command and parent window was not dpi aware

    DpiAwareness dpi_awareness;

    bool character = message->message == WM_CHAR || message->message == WM_SYSCHAR;
    bool key_down = message->message == WM_KEYDOWN || message->message == WM_SYSKEYDOWN;
    bool key_up = message->message == WM_KEYUP || message->message == WM_SYSKEYUP;

    if (!character && !key_down && !key_up)
      return false;

    if (character)
      return handleCharacterEntry(static_cast<wchar_t>(message->wParam));

    bool used = false;
    if (hasActiveTextEntry()) {
      TranslateMessage(message);
      MSG peek {};
      if (PeekMessage(&peek, parentHandle(), WM_CHAR, WM_DEADCHAR, PM_REMOVE) ||
          PeekMessage(&peek, parentHandle(), WM_SYSCHAR, WM_SYSDEADCHAR, PM_REMOVE)) {
        used = true;
      }
    }

    KeyCode key_code = keyCodeFromScanCode(message->wParam, message->lParam);
    if (key_down) {
      bool is_repeat = (message->lParam & (1LL << 30LL)) != 0;
      return handleKeyDown(key_code, keyboardModifiers(), is_repeat) || used;
    }
    return handleKeyUp(key_code, keyboardModifiers()) || used;
  }

  void WindowWin32::handleResizing(HWND hwnd, LPARAM l_param, WPARAM w_param) {
    IBounds borders = windowBorderSize(hwnd);
    RECT* rect = reinterpret_cast<RECT*>(l_param);
    int width = rect->right - rect->left - borders.width();
    int height = rect->bottom - rect->top - borders.height();

    if (!isFixedAspectRatio()) {
      handleResized(width, height);
      return;
    }

    float aspect_ratio = aspectRatio();
    VISAGE_ASSERT(aspect_ratio > 0.0f);

    bool horizontal_resize = w_param == WMSZ_LEFT || w_param == WMSZ_RIGHT ||
                             w_param == WMSZ_BOTTOMLEFT || w_param == WMSZ_BOTTOMRIGHT;
    bool vertical_resize = w_param == WMSZ_TOP || w_param == WMSZ_BOTTOM ||
                           w_param == WMSZ_TOPLEFT || w_param == WMSZ_TOPRIGHT;

    Point max_dimensions = Point(maxWindowDimensions());
    Point min_dimensions = Point(minWindowDimensions());
    Point adjusted_dimensions = adjustBoundsForAspectRatio({ width * 1.0f, height * 1.0f },
                                                           min_dimensions, max_dimensions, aspect_ratio,
                                                           horizontal_resize, vertical_resize);

    switch (w_param) {
    case WMSZ_LEFT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.height();
      rect->left = rect->right - adjusted_dimensions.x - borders.width();
      break;
    case WMSZ_RIGHT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.height();
      rect->right = rect->left + adjusted_dimensions.x + borders.width();
      break;
    case WMSZ_TOP:
      rect->right = rect->left + adjusted_dimensions.x + borders.width();
      rect->top = rect->bottom - adjusted_dimensions.y - borders.height();
      break;
    case WMSZ_BOTTOM:
      rect->right = rect->left + adjusted_dimensions.x + borders.width();
      rect->bottom = rect->top + adjusted_dimensions.y + borders.height();
      break;
    case WMSZ_TOPLEFT:
      rect->top = rect->bottom - adjusted_dimensions.y - borders.height();
      rect->left = rect->right - adjusted_dimensions.x - borders.width();
      break;
    case WMSZ_TOPRIGHT:
      rect->top = rect->bottom - adjusted_dimensions.y - borders.height();
      rect->right = rect->left + adjusted_dimensions.x + borders.width();
      break;
    case WMSZ_BOTTOMLEFT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.height();
      rect->left = rect->right - adjusted_dimensions.x - borders.width();
      break;
    case WMSZ_BOTTOMRIGHT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.height();
      rect->right = rect->left + adjusted_dimensions.x + borders.width();
      break;
    default: break;
    }
  }

  void WindowWin32::handleResizeEnd(HWND hwnd) {
    float aspect_ratio = aspectRatio();
    VISAGE_ASSERT(aspect_ratio > 0.0f);

    IBounds borders = windowBorderSize(hwnd);

    RECT rect;
    GetWindowRect(hwnd, &rect);
    int width = rect.right - rect.left - borders.width();
    int height = rect.bottom - rect.top - borders.height();
    DpiAwareness dpi_awareness;
    setDpiScale(dpi_awareness.dpiScale(hwnd));
    handleResized(width, height);
  }

  void WindowWin32::handleDpiChange(HWND hwnd, LPARAM l_param, WPARAM w_param) {
    Point max_dimensions = Point(maxWindowDimensions());
    Point min_dimensions = Point(minWindowDimensions());
    IBounds borders = windowBorderSize(hwnd);
    RECT* suggested = reinterpret_cast<RECT*>(l_param);
    Point point(suggested->right - suggested->left - borders.width(),
                suggested->bottom - suggested->top - borders.height());
    Point adjusted_dimensions = adjustBoundsForAspectRatio(point, min_dimensions, max_dimensions,
                                                           aspectRatio(), true, true);

    int width = adjusted_dimensions.x;
    int height = adjusted_dimensions.y;
    SetWindowPos(hwnd, nullptr, suggested->left, suggested->top, width + borders.width(),
                 height + borders.height(), SWP_NOZORDER | SWP_NOACTIVATE);

    DpiAwareness dpi_awareness;
    setDpiScale(dpi_awareness.dpiScale(hwnd));
    handleResized(width, height);
  }

  void WindowWin32::updateMonitor() {
    monitor_ = MonitorFromWindow(window_handle_, MONITOR_DEFAULTTONEAREST);
  }
}
#endif
