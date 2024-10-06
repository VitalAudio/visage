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

#if VISAGE_WINDOWS
#define NOMINMAX

#include "windowing_win32.h"

#include "visage_utils/file_system.h"
#include "visage_utils/keycodes.h"
#include "visage_utils/string_utils.h"
#include "visage_utils/thread_utils.h"

#include <algorithm>
#include <dxgi1_4.h>
#include <map>
#include <ShlObj.h>
#include <string>
#include <windowsx.h>
#pragma comment(lib, "Shell32.lib")
#pragma comment(lib, "dxgi.lib")

#define WM_VBLANK (WM_USER + 1)

typedef DPI_AWARENESS_CONTEXT(WINAPI* GetWindowDpiAwarenessContext_t)(HWND);
typedef DPI_AWARENESS_CONTEXT(WINAPI* GetThreadDpiAwarenessContext_t)();
typedef DPI_AWARENESS_CONTEXT(WINAPI* SetThreadDpiAwarenessContext_t)(DPI_AWARENESS_CONTEXT);
typedef UINT(WINAPI* GetDpiForWindow_t)(HWND);
typedef UINT(WINAPI* GetDpiForSystem_t)();

namespace visage {
  template<class T>
  static T getProcedure(HMODULE module, LPCSTR proc_name) {
    return reinterpret_cast<T>(GetProcAddress(module, proc_name));
  }

  std::string getClipboardText() {
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

  static float getPixelScale() {
    HWND hwnd = GetActiveWindow();
    if (hwnd == nullptr)
      return 1.0f;

    HMODULE user32 = LoadLibraryA("user32.dll");
    if (user32 == nullptr)
      return 1.0f;

    auto getWindowDpiAwarenessContext =
        getProcedure<GetWindowDpiAwarenessContext_t>(user32, "GetWindowDpiAwarenessContext");
    auto setThreadDpiAwarenessContext =
        getProcedure<SetThreadDpiAwarenessContext_t>(user32, "SetWindowDpiAwarenessContext");
    auto getDpiForWindow = getProcedure<GetDpiForWindow_t>(user32, "GetDpiForWindow");

    if (getWindowDpiAwarenessContext == nullptr || setThreadDpiAwarenessContext == nullptr ||
        getDpiForWindow == nullptr) {
      return 1.0f;
    }

    DPI_AWARENESS_CONTEXT dpi_context = getWindowDpiAwarenessContext(hwnd);
    if (!setThreadDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2))
      setThreadDpiAwarenessContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE);

    int scaled_dpi = getDpiForWindow(hwnd);
    setThreadDpiAwarenessContext(dpi_context);
    return scaled_dpi * 1.0f / getDpiForWindow(hwnd);
  }

  static Point convertToSystemPosition(Point frame_position) {
    float scaling = getPixelScale();
    return { static_cast<int>(frame_position.x * scaling), static_cast<int>(frame_position.y * scaling) };
  }

  static Point convertToFramePosition(Point frame_position) {
    float scaling = getPixelScale();
    return { static_cast<int>(frame_position.x / scaling), static_cast<int>(frame_position.y / scaling) };
  }

  static Point getCursorScreenPosition() {
    POINT cursor_position;
    GetCursorPos(&cursor_position);
    return convertToFramePosition(Point(cursor_position.x, cursor_position.y));
  }

  Point getCursorPosition() {
    POINT cursor_position;
    GetCursorPos(&cursor_position);

    HWND hwnd = GetActiveWindow();
    if (hwnd == nullptr)
      return { static_cast<int>(cursor_position.x), static_cast<int>(cursor_position.y) };

    ScreenToClient(hwnd, &cursor_position);
    return convertToFramePosition(Point(cursor_position.x, cursor_position.y));
  }

  void setCursorPosition(Point window_position) {
    HWND hwnd = GetActiveWindow();
    if (hwnd == nullptr)
      return;

    POINT client_position = { 0, 0 };
    ClientToScreen(hwnd, &client_position);
    Point position = convertToSystemPosition(window_position);
    SetCursorPos(client_position.x + position.x, client_position.y + position.y);
  }

  void setCursorScreenPosition(Point screen_position) {
    Point position = convertToSystemPosition(screen_position);
    SetCursorPos(position.x, position.y);
  }

  class DxgiFactory {
  public:
    static DxgiFactory& getInstance() {
      static DxgiFactory instance;
      return instance;
    }

    static IDXGIFactory* factory() { return getInstance().dxgi_factory_; }

  private:
    DxgiFactory() {
      if (FAILED(CreateDXGIFactory(__uuidof(IDXGIFactory), reinterpret_cast<void**>(&dxgi_factory_))))
        dxgi_factory_ = nullptr;
    }

    ~DxgiFactory() {
      if (dxgi_factory_)
        dxgi_factory_->Release();
    }

    IDXGIFactory* dxgi_factory_ = nullptr;
  };

  class VBlankThread : public Thread {
  public:
    VBlankThread(WindowWin32* window) : window_(window) { }

    ~VBlankThread() override {
      if (dxgi_output_)
        dxgi_output_->Release();
      if (dxgi_adapter_)
        dxgi_adapter_->Release();
    }

    void run() override {
      // TODO get correct monitor for vblank notifications.

      if (FAILED(DxgiFactory::factory()->EnumAdapters(0, &dxgi_adapter_)))
        return;

      if (FAILED(dxgi_adapter_->EnumOutputs(0, &dxgi_output_)))
        return;

      start_us_ = time::getMicroseconds();

      while (shouldRun()) {
        if (SUCCEEDED(dxgi_output_->WaitForVBlank())) {
          long long us = time::getMicroseconds() - start_us_;
          time_ = us * (1.0 / 1000000.0);
          PostMessage(window_->getWindowHandle(), WM_VBLANK, 0, 0);
        }
        else
          sleep(1);
      }
    }

    double getVBlankTime() const { return time_.load(); }

  private:
    WindowWin32* window_ = nullptr;

    IDXGIAdapter* dxgi_adapter_ = nullptr;
    IDXGIOutput* dxgi_output_ = nullptr;

    std::atomic<double> time_ = 0.0;
    long long start_us_ = 0;
    long long us_ = 0;
  };

  class NativeWindowLookup {
  public:
    static NativeWindowLookup& getInstance() {
      static NativeWindowLookup instance;
      return instance;
    }

    void addWindow(HWND parent, WindowWin32* window) {
      parent_window_lookup_[parent] = window;
      native_window_lookup_[window->getNativeHandle()] = window;
    }

    void removeWindow(HWND parent) {
      if (parent_window_lookup_.count(parent)) {
        void* handle = parent_window_lookup_[parent]->getNativeHandle();
        if (native_window_lookup_.count(handle))
          native_window_lookup_.erase(handle);

        parent_window_lookup_.erase(parent);
      }
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

  private:
    NativeWindowLookup() = default;
    ~NativeWindowLookup() = default;

    std::map<void*, WindowWin32*> parent_window_lookup_;
    std::map<void*, WindowWin32*> native_window_lookup_;
  };

  class DpiAwareness {
  public:
    DpiAwareness() {
      HMODULE user32 = LoadLibraryA("user32.dll");
      if (user32 == nullptr)
        return;

      getThreadDpiAwarenessContext_ =
          getProcedure<GetThreadDpiAwarenessContext_t>(user32, "GetThreadDpiAwarenessContext");
      setThreadDpiAwarenessContext_ =
          getProcedure<SetThreadDpiAwarenessContext_t>(user32, "SetThreadDpiAwarenessContext");
      getDpiForWindow_ = getProcedure<GetDpiForWindow_t>(user32, "GetDpiForWindow");
      getDpiForSystem_ = getProcedure<GetDpiForSystem_t>(user32, "GetDpiForSystem");
      if (getThreadDpiAwarenessContext_ == nullptr || setThreadDpiAwarenessContext_ == nullptr ||
          getDpiForWindow_ == nullptr || getDpiForSystem_ == nullptr) {
        return;
      }

      previous_dpi_awareness_ = getThreadDpiAwarenessContext_();
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

    float getConversionFactor() const {
      if (dpi_awareness_ == nullptr)
        return 1.0f;

      setThreadDpiAwarenessContext_(previous_dpi_awareness_);
      unsigned int previous_dpi = getDpiForSystem_();
      setThreadDpiAwarenessContext_(dpi_awareness_);
      return getDpiForSystem_() * 1.0f / previous_dpi;
    }

    float getConversionFactor(HWND hwnd) const {
      if (dpi_awareness_ == nullptr)
        return 1.0f;

      setThreadDpiAwarenessContext_(previous_dpi_awareness_);
      unsigned int previous_dpi = getDpiForWindow_(hwnd);
      setThreadDpiAwarenessContext_(dpi_awareness_);
      return getDpiForWindow_(hwnd) * 1.0f / previous_dpi;
    }

  private:
    DPI_AWARENESS_CONTEXT dpi_awareness_ = nullptr;
    DPI_AWARENESS_CONTEXT previous_dpi_awareness_ = nullptr;
    GetThreadDpiAwarenessContext_t getThreadDpiAwarenessContext_ = nullptr;
    SetThreadDpiAwarenessContext_t setThreadDpiAwarenessContext_ = nullptr;
    GetDpiForWindow_t getDpiForWindow_ = nullptr;
    GetDpiForSystem_t getDpiForSystem_ = nullptr;
  };

  class DragDropSource : public IDropSource {
  public:
    DragDropSource() = default;
    virtual ~DragDropSource() = default;

    HRESULT __stdcall QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDropSource) {
        *ppv_object = static_cast<IDropSource*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG __stdcall AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG __stdcall Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    HRESULT __stdcall QueryContinueDrag(BOOL escape_pressed, DWORD key_state) override {
      if (escape_pressed)
        return DRAGDROP_S_CANCEL;

      if ((key_state & (MK_LBUTTON | MK_RBUTTON)) == 0)
        return DRAGDROP_S_DROP;
      return S_OK;
    }

    HRESULT __stdcall GiveFeedback(DWORD effect) override { return DRAGDROP_S_USEDEFAULTCURSORS; }

  private:
    LONG ref_count_ = 1;
  };

  class DragDropEnumFormatEtc : public IEnumFORMATETC {
  public:
    DragDropEnumFormatEtc() = default;
    virtual ~DragDropEnumFormatEtc() = default;

    HRESULT __stdcall QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDataObject) {
        *ppv_object = static_cast<IEnumFORMATETC*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG __stdcall AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG __stdcall Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    HRESULT Clone(IEnumFORMATETC** result) override {
      if (result == nullptr)
        return E_POINTER;

      auto newOne = new DragDropEnumFormatEtc();
      newOne->index_ = index_;
      *result = newOne;
      return S_OK;
    }

    HRESULT Next(ULONG celt, LPFORMATETC lpFormatEtc, ULONG* pceltFetched) override {
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

    HRESULT Skip(ULONG celt) override {
      if (index_ + static_cast<int>(celt) >= 1)
        return S_FALSE;

      index_ += static_cast<int>(celt);
      return S_OK;
    }

    HRESULT Reset() override {
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

    explicit DragDropSourceObject(File file) { drop_ = DragDropSourceObject::createHDrop(file); }
    virtual ~DragDropSourceObject() = default;

    HRESULT __stdcall QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDataObject) {
        *ppv_object = static_cast<IDataObject*>(this);
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG __stdcall AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG __stdcall Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    static bool acceptsFormat(const FORMATETC* format_etc) {
      return (format_etc->dwAspect & DVASPECT_CONTENT) && format_etc->cfFormat == CF_HDROP &&
             (format_etc->tymed & TYMED_HGLOBAL);
    }

    HRESULT __stdcall GetData(FORMATETC* format_etc, STGMEDIUM* medium) override {
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

    HRESULT __stdcall QueryGetData(FORMATETC* format_etc) override {
      if (acceptsFormat(format_etc))
        return S_OK;
      return DV_E_FORMATETC;
    }

    HRESULT __stdcall GetCanonicalFormatEtc(FORMATETC*, FORMATETC* format_etc_out) override {
      format_etc_out->ptd = nullptr;
      return E_NOTIMPL;
    }

    HRESULT __stdcall EnumFormatEtc(DWORD direction, IEnumFORMATETC** result) override {
      if (result == nullptr)
        return E_POINTER;

      if (direction == DATADIR_GET) {
        *result = new DragDropEnumFormatEtc();
        return S_OK;
      }

      *result = nullptr;
      return E_NOTIMPL;
    }

    HRESULT __stdcall GetDataHere(FORMATETC*, STGMEDIUM*) override { return E_NOTIMPL; }
    HRESULT __stdcall SetData(FORMATETC*, STGMEDIUM*, BOOL) override { return E_NOTIMPL; }
    HRESULT __stdcall DAdvise(FORMATETC*, DWORD, IAdviseSink*, DWORD*) override {
      return E_NOTIMPL;
    }
    HRESULT __stdcall DUnadvise(DWORD) override { return E_NOTIMPL; }
    HRESULT __stdcall EnumDAdvise(IEnumSTATDATA**) override { return E_NOTIMPL; }

  private:
    HDROP drop_;
    LONG ref_count_ = 1;
  };

  class DragDropTarget : public IDropTarget {
  public:
    explicit DragDropTarget(WindowWin32* window) : window_(window) { }
    virtual ~DragDropTarget() = default;

    HRESULT __stdcall QueryInterface(REFIID riid, void** ppv_object) override {
      if (riid == IID_IUnknown || riid == IID_IDropTarget) {
        *ppv_object = this;
        AddRef();
        return S_OK;
      }
      *ppv_object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG __stdcall AddRef() override { return InterlockedIncrement(&ref_count_); }

    ULONG __stdcall Release() override {
      LONG count = InterlockedDecrement(&ref_count_);
      if (count == 0)
        delete this;
      return count;
    }

    static float getConversion() {
      DpiAwareness dpi_awareness;
      return dpi_awareness.getConversionFactor();
    }

    Point getDragPosition(POINTL point) const {
      float conversion = getConversion() / window_->getPixelScale();
      POINT position = { static_cast<int>(std::round(point.x / conversion)),
                         static_cast<int>(std::round(point.y / conversion)) };
      ScreenToClient(static_cast<HWND>(window_->getNativeHandle()), &position);
      return { static_cast<int>(std::round(position.x * conversion)),
               static_cast<int>(std::round(position.y * conversion)) };
    }

    HRESULT __stdcall DragEnter(IDataObject* data_object, DWORD key_state, POINTL point,
                                DWORD* effect) override {
      Point position = getDragPosition(point);
      files_ = getDropFiles(data_object);
      if (window_->handleFileDrag(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    HRESULT __stdcall DragOver(DWORD key_state, POINTL point, DWORD* effect) override {
      Point position = getDragPosition(point);
      if (window_->handleFileDrag(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    HRESULT __stdcall DragLeave() override {
      window_->handleFileDragLeave();
      return S_OK;
    }

    HRESULT __stdcall Drop(IDataObject* data_object, DWORD key_state, POINTL point, DWORD* effect) override {
      Point position = getDragPosition(point);
      files_ = getDropFiles(data_object);
      if (window_->handleFileDrop(position.x, position.y, files_))
        *effect = DROPEFFECT_COPY;
      else
        *effect = DROPEFFECT_NONE;
      return S_OK;
    }

    static std::vector<std::string> getDropFiles(IDataObject* data_object) {
      std::vector<std::string> files;
      FORMATETC format = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_HGLOBAL };
      STGMEDIUM storage = { TYMED_HGLOBAL };

      if (SUCCEEDED(data_object->GetData(&format, &storage))) {
        HDROP hDrop = static_cast<HDROP>(GlobalLock(storage.hGlobal));

        if (hDrop != nullptr) {
          UINT fileCount = DragQueryFile(hDrop, 0xFFFFFFFF, nullptr, 0);
          TCHAR filePath[MAX_PATH];

          for (UINT i = 0; i < fileCount; ++i) {
            if (DragQueryFile(hDrop, i, filePath, MAX_PATH))
              files.emplace_back(filePath);
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

  int getDisplayFps() {
    static constexpr int kDefaultFps = 60;
    DEVMODE dev_mode = {};
    dev_mode.dmSize = sizeof(dev_mode);

    if (EnumDisplaySettings(nullptr, ENUM_CURRENT_SETTINGS, &dev_mode))
      return dev_mode.dmDisplayFrequency;

    return kDefaultFps;
  }

  float getWindowPixelScale() {
    DpiAwareness dpi_awareness;
    return dpi_awareness.getConversionFactor();
  }

  void showMessageBox(std::string title, std::string message) {
    MessageBox(nullptr, message.c_str(), title.c_str(), MB_OK);
  }

  static KeyCode getKeyCodeFromScanCode(WPARAM w_param, LPARAM l_param) {
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
    BOOL success = GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                                         GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                     reinterpret_cast<LPCSTR>(&loadModuleHandle), &module_handle);
    if (success && module_handle)
      return module_handle;

    return GetModuleHandle(nullptr);
  }

  static int getKeyboardModifiers() {
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

  static int getKeyboardModifiers(WPARAM w_param) {
    int modifiers = 0;
    if (w_param & MK_SHIFT)
      modifiers |= kModifierShift;
    if (w_param & MK_CONTROL)
      modifiers |= kModifierRegCtrl;
    if (GetKeyState(VK_MENU) & 0x8000)
      modifiers |= kModifierAlt;
    if (GetKeyState(VK_LWIN) & 0x8000 || GetKeyState(VK_RWIN) & 0x8000)
      modifiers |= kModifierMeta;
    return modifiers;
  }

  static int getMouseButtonState() {
    int state = 0;
    if (GetKeyState(VK_LBUTTON) & 0x8000)
      state |= kMouseButtonLeft;
    if (GetKeyState(VK_RBUTTON) & 0x8000)
      state |= kMouseButtonRight;
    if (GetKeyState(VK_MBUTTON) & 0x8000)
      state |= kMouseButtonMiddle;
    return state;
  }

  static bool isTouchEvent() {
    return (GetMessageExtraInfo() & 0xFFFFFF00) == 0xFF515700;
  }

  static int getMouseButtonState(WPARAM w_param) {
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
  }

  static bool isWindowOccluded(HWND hwnd) {
    RECT rect;
    if (!GetWindowRect(hwnd, &rect))
      return false;

    auto getNextX = [](HWND hit_hwnd, LONG x, LONG right) -> LONG {
      if (hit_hwnd == nullptr)
        return right;

      RECT window_rect;
      if (!GetWindowRect(hit_hwnd, &window_rect))
        return right;

      return std::min(right, std::max(x, window_rect.right + 1));
    };

    auto getNextY = [](HWND hit_hwnd, LONG y, LONG bottom) -> LONG {
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

      x = getNextX(hit_hwnd, x, rect.right);
    }

    x = rect.left;
    while (x < rect.right) {
      HWND hit_hwnd = WindowFromPoint(POINT { x, rect.bottom });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      x = getNextX(hit_hwnd, x, rect.right);
    }

    LONG y = rect.top;
    while (y < rect.bottom) {
      HWND hit_hwnd = WindowFromPoint(POINT { rect.left, y });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      y = getNextY(hit_hwnd, y, rect.bottom);
    }

    y = rect.top;
    while (y < rect.bottom) {
      HWND hit_hwnd = WindowFromPoint(POINT { rect.right, y });
      if (hit_hwnd == hwnd || IsChild(hwnd, hit_hwnd))
        return false;

      y = getNextY(hit_hwnd, y, rect.bottom);
    }

    return true;
  }

  static LRESULT WINAPI windowProcedure(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    WindowWin32* window = reinterpret_cast<WindowWin32*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (window == nullptr)
      return DefWindowProc(hwnd, msg, w_param, l_param);

    return window->handleWindowProc(hwnd, msg, w_param, l_param);

    return 0;
  }

  LRESULT WindowWin32::handleWindowProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    switch (msg) {
    case WM_VBLANK: {
      drawCallback(v_blank_thread_->getVBlankTime());
      return 0;
    }
    case WM_SYSKEYDOWN:
    case WM_KEYDOWN: {
      KeyCode key_code = getKeyCodeFromScanCode(w_param, l_param);
      bool is_repeat = (l_param & (1LL << 30LL)) != 0;
      if (!handleKeyDown(key_code, getKeyboardModifiers(), is_repeat))
        postMessageToParent(hwnd, msg, w_param, l_param);
      return 0;
    }
    case WM_SYSKEYUP:
    case WM_KEYUP: {
      KeyCode key_code = getKeyCodeFromScanCode(w_param, l_param);
      if (!handleKeyUp(key_code, getKeyboardModifiers()))
        postMessageToParent(hwnd, msg, w_param, l_param);
      return 0;
    }
    case WM_SYSCHAR:
    case WM_CHAR: {
      handleCharacterEntry(static_cast<wchar_t>(w_param));
      return 0;
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

      handleMouseMove(x, y, getMouseButtonState(w_param), getKeyboardModifiers());
      if (getMouseRelativeMode()) {
        Point last_position = getLastWindowMousePosition();
        POINT client_position = { last_position.x, last_position.y };
        ClientToScreen(hwnd, &client_position);
        SetCursorPos(client_position.x, client_position.y);
      }

      return 0;
    }
    case WM_MOUSELEAVE: {
      setMouseTracked(false);
      handleMouseLeave(getMouseButtonState(0), getKeyboardModifiers());
      return 0;
    }
    case WM_LBUTTONDOWN: {
      SetFocus(static_cast<HWND>(getNativeHandle()));
      SetFocus(hwnd);
      handleMouseDown(kMouseButtonLeft, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      getMouseButtonState(w_param), getKeyboardModifiers());

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
    case WM_LBUTTONUP: {
      int button_state = getMouseButtonState(w_param);
      handleMouseUp(kMouseButtonLeft, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    getKeyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_RBUTTONDOWN: {
      SetFocus(hwnd);
      SetCapture(hwnd);
      handleMouseDown(kMouseButtonRight, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      getMouseButtonState(w_param), getKeyboardModifiers());
      return 0;
    }
    case WM_RBUTTONUP: {
      int button_state = getMouseButtonState(w_param);
      handleMouseUp(kMouseButtonRight, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    getKeyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_MBUTTONDOWN: {
      SetFocus(hwnd);
      SetCapture(hwnd);
      handleMouseDown(kMouseButtonMiddle, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param),
                      getMouseButtonState(w_param), getKeyboardModifiers());
      return 0;
    }
    case WM_MBUTTONUP: {
      int button_state = getMouseButtonState(w_param);
      handleMouseUp(kMouseButtonMiddle, GET_X_LPARAM(l_param), GET_Y_LPARAM(l_param), button_state,
                    getKeyboardModifiers());
      if (button_state == 0 && GetCapture() == hwnd)
        ReleaseCapture();
      return 0;
    }
    case WM_SETCURSOR: {
      if (LOWORD(l_param) == HTCLIENT) {
        SetCursor(WindowWin32::getCursor());
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
      handleMouseWheel(delta_x, delta_y, position.x, position.y, getMouseButtonState(),
                       getKeyboardModifiers());
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
    case WM_EXITSIZEMOVE: {
      handleResizeEnd(hwnd);
      return 0;
    }
    case WM_DPICHANGED: {
      handleDpiChange(hwnd, l_param, w_param);
      return 0;
    }
    case WM_MOVE: {
      updateMonitor();
      return 0;
    }
    case WM_DISPLAYCHANGE: {
      updateMonitor();
      return 0;
    }
    default: break;
    }
    return DefWindowProc(hwnd, msg, w_param, l_param);
  }

  static LRESULT WINAPI pluginParentWindowProc(HWND hwnd, UINT msg, WPARAM w_param, LPARAM l_param) {
    WindowWin32* child_window = NativeWindowLookup::getInstance().findWindow(hwnd);
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
    if (msg == WM_DESTROY) {
      PostQuitMessage(0);
      return 0;
    }
    return windowProcedure(hwnd, msg, w_param, l_param);
  }

  static HMONITOR getMonitorFromMousePosition() {
    POINT cursor_position;
    GetCursorPos(&cursor_position);
    return MonitorFromPoint(cursor_position, MONITOR_DEFAULTTONEAREST);
  }

  static Bounds getBoundsInMonitor(HMONITOR monitor, float aspect_ratio, float display_scale) {
    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor, &monitor_info);

    int monitor_width = monitor_info.rcWork.right - monitor_info.rcWork.left;
    int monitor_height = monitor_info.rcWork.bottom - monitor_info.rcWork.top;

    float scale_width = monitor_width * display_scale;
    int height = static_cast<int>(std::min(monitor_height * display_scale, scale_width / aspect_ratio));
    int width = static_cast<int>(std::round(height * aspect_ratio));
    int x = monitor_info.rcWork.left + (monitor_width - width) / 2;
    int y = monitor_info.rcWork.top + (monitor_height - height) / 2;
    return { x, y, width, height };
  }

  static Point getWindowBorderSize(HWND hwnd) {
    WINDOWINFO info {};
    info.cbSize = sizeof(info);
    if (GetWindowInfo(hwnd, &info)) {
      int width = (info.rcClient.left - info.rcWindow.left) + (info.rcWindow.right - info.rcClient.right);
      int height = (info.rcClient.top - info.rcWindow.top) + (info.rcWindow.bottom - info.rcClient.bottom);
      return { width, height };
    }
    return { 0, 0 };
  }

  Bounds getScaledWindowBounds(float aspect_ratio, float display_scale, int x, int y) {
    DpiAwareness dpi_awareness;
    Bounds bounds;
    if (x == Window::kNotSet || y == Window::kNotSet)
      bounds = getBoundsInMonitor(getMonitorFromMousePosition(), aspect_ratio, display_scale);
    else {
      HMONITOR monitor = MonitorFromPoint({ x, y }, MONITOR_DEFAULTTONEAREST);
      bounds = getBoundsInMonitor(monitor, aspect_ratio, display_scale);
    }

    if (x != Window::kNotSet)
      bounds.setX(x);
    if (y != Window::kNotSet)
      bounds.setY(y);

    return bounds;
  }

  static inline void clearMessage(MSG* message) {
    *message = {};
    message->message = WM_USER;
  }

  LRESULT CALLBACK EventHooks::eventHook(int code, WPARAM w_param, LPARAM l_param) {
    if (code == HC_ACTION && w_param == PM_REMOVE) {
      MSG* message = reinterpret_cast<MSG*>(l_param);
      WindowWin32* window = NativeWindowLookup::getInstance().findByNativeHandle(message->hwnd);

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
    if (instance_count_++ == 0 && event_hook_ == nullptr) {
      event_hook_ = SetWindowsHookEx(WH_GETMESSAGE, eventHook, (HINSTANCE)loadModuleHandle(),
                                     GetCurrentThreadId());
    }
  }

  EventHooks::~EventHooks() {
    instance_count_--;
    if (instance_count_ == 0 && event_hook_) {
      UnhookWindowsHookEx(event_hook_);
      event_hook_ = nullptr;
    }
  }

  void WindowWin32::runEventThread() {
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

    std::string unique_string = std::to_string(reinterpret_cast<uintptr_t>(this));
    unique_window_class_name_ = std::string(VISAGE_APPLICATION_NAME) + "_" + unique_string;

    window_class_.cbSize = sizeof(WNDCLASSEX);
    window_class_.style = CS_OWNDC;
    window_class_.cbClsExtra = 0;
    window_class_.cbWndExtra = 0;
    window_class_.hInstance = module_handle_;
    window_class_.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
    window_class_.hCursor = LoadCursor(nullptr, IDC_ARROW);
    window_class_.hbrBackground = nullptr;
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

  std::unique_ptr<Window> createWindow(int x, int y, int width, int height) {
    return std::make_unique<WindowWin32>(x, y, width, height);
  }

  std::unique_ptr<Window> createPluginWindow(int width, int height, void* parent_handle) {
    return std::make_unique<WindowWin32>(width, height, parent_handle);
  }

  WindowWin32::WindowWin32(int x, int y, int width, int height) : Window(width, height) {
    static constexpr int kWindowFlags = WS_CAPTION | WS_SYSMENU | WS_THICKFRAME | WS_MINIMIZEBOX;
    DpiAwareness dpi_awareness;

    registerWindowClass();
    window_class_.lpfnWndProc = standaloneWindowProcedure;
    RegisterClassEx(&window_class_);

    window_handle_ = CreateWindow(window_class_.lpszClassName, VISAGE_APPLICATION_NAME, kWindowFlags, x,
                                  y, width, height, nullptr, nullptr, window_class_.hInstance, nullptr);
    if (window_handle_ == nullptr) {
      VISAGE_LOG("Error creating window");
      return;
    }

    Point borders = getWindowBorderSize(window_handle_);
    SetWindowPos(window_handle_, nullptr, x, y, width + borders.x, height + borders.y,
                 SWP_NOZORDER | SWP_NOMOVE);

    SetWindowLongPtr(window_handle_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

    finishWindowSetup();
  }

  WindowWin32::WindowWin32(int width, int height, void* parent_handle) : Window(width, height) {
    static constexpr int kWindowFlags = WS_CHILD;

    registerWindowClass();
    window_class_.lpfnWndProc = windowProcedure;
    RegisterClassEx(&window_class_);

    parent_handle_ = static_cast<HWND>(parent_handle);
    window_handle_ = CreateWindow(window_class_.lpszClassName, VISAGE_APPLICATION_NAME,
                                  kWindowFlags, 0, 0, width, height, parent_handle_, nullptr,
                                  window_class_.hInstance, nullptr);
    if (window_handle_ == nullptr) {
      VISAGE_LOG("Error creating window");
      return;
    }

    SetWindowLongPtr(window_handle_, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(this));

    auto parent_proc = SetWindowLongPtr(parent_handle_, GWLP_WNDPROC,
                                        reinterpret_cast<LONG_PTR>(pluginParentWindowProc));
    parent_window_proc_ = reinterpret_cast<WNDPROC>(parent_proc);
    NativeWindowLookup::getInstance().addWindow(parent_handle_, this);

    DpiAwareness dpi_awareness;
    setPixelScale(dpi_awareness.getConversionFactor());
    event_hooks_ = std::make_unique<EventHooks>();
    finishWindowSetup();
  }

  void WindowWin32::finishWindowSetup() {
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

    if (parent_handle_) {
      SetWindowLongPtr(parent_handle_, GWLP_WNDPROC, reinterpret_cast<LONG_PTR>(parent_window_proc_));
      NativeWindowLookup::getInstance().removeWindow(parent_handle_);
    }

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
    LONG rect_width = static_cast<LONG>(std::round(width * getPixelScale()));
    LONG rect_height = static_cast<LONG>(std::round(height * getPixelScale()));
    rect.right = rect.left + rect_width;
    rect.bottom = rect.top + rect_height;

    Point borders = getWindowBorderSize(window_handle_);
    SetWindowPos(window_handle_, nullptr, x, y, rect.right - rect.left + borders.x,
                 rect.bottom - rect.top + borders.y, SWP_NOZORDER | SWP_NOMOVE);
  }

  void WindowWin32::show() {
    ShowWindow(window_handle_, 1);
    SetFocus(window_handle_);

    if (v_blank_thread_ == nullptr) {
      v_blank_thread_ = std::make_unique<VBlankThread>(this);
      v_blank_thread_->start();
    }
  }

  void WindowWin32::hide() {
    ShowWindow(window_handle_, 0);
  }

  void WindowWin32::setWindowTitle(const std::string& title) {
    SetWindowText(window_handle_, title.c_str());
  }

  Point WindowWin32::getMaxWindowDimensions() const {
    Point borders = getWindowBorderSize(window_handle_);
    if (borders.x == 0 && borders.y == 0 && parent_handle_)
      borders = getWindowBorderSize(parent_handle_);

    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor_, &monitor_info);

    int display_width = monitor_info.rcWork.right - monitor_info.rcWork.left - borders.x;
    int display_height = monitor_info.rcWork.bottom - monitor_info.rcWork.top - borders.y;

    float aspect_ratio = getAspectRatio();
    int width_from_height = static_cast<int>(display_height * aspect_ratio);
    int height_from_width = static_cast<int>(display_width / aspect_ratio);
    return { std::min<int>(display_width, width_from_height),
             std::min<int>(display_height, height_from_width) };
  }

  Point WindowWin32::getMinWindowDimensions() const {
    float scale = getMinimumWindowScale();
    MONITORINFO monitor_info {};
    monitor_info.cbSize = sizeof(MONITORINFO);
    GetMonitorInfo(monitor_, &monitor_info);

    int min_width = static_cast<int>(scale * (monitor_info.rcWork.right - monitor_info.rcWork.left));
    int min_height = static_cast<int>(scale * (monitor_info.rcWork.bottom - monitor_info.rcWork.top));
    float aspect_ratio = getAspectRatio();
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
      if (PeekMessage(&peek, getParentHandle(), WM_CHAR, WM_DEADCHAR, PM_REMOVE) ||
          PeekMessage(&peek, getParentHandle(), WM_SYSCHAR, WM_SYSDEADCHAR, PM_REMOVE)) {
        used = true;
      }
    }

    KeyCode key_code = getKeyCodeFromScanCode(message->wParam, message->lParam);
    if (key_down) {
      bool is_repeat = (message->lParam & (1LL << 30LL)) != 0;
      return handleKeyDown(key_code, getKeyboardModifiers(), is_repeat) || used;
    }
    return handleKeyUp(key_code, getKeyboardModifiers()) || used;
  }

  void WindowWin32::handleResizing(HWND hwnd, LPARAM l_param, WPARAM w_param) {
    Point borders = getWindowBorderSize(hwnd);
    RECT* rect = reinterpret_cast<RECT*>(l_param);
    int width = rect->right - rect->left - borders.x;
    int height = rect->bottom - rect->top - borders.y;

    if (!isFixedAspectRatio()) {
      handleResized(width, height);
      return;
    }

    float aspect_ratio = getAspectRatio();
    VISAGE_ASSERT(aspect_ratio > 0.0f);

    bool horizontal_resize = w_param == WMSZ_LEFT || w_param == WMSZ_RIGHT ||
                             w_param == WMSZ_BOTTOMLEFT || w_param == WMSZ_BOTTOMRIGHT;
    bool vertical_resize = w_param == WMSZ_TOP || w_param == WMSZ_BOTTOM ||
                           w_param == WMSZ_TOPLEFT || w_param == WMSZ_TOPRIGHT;

    Point max_dimensions = getMaxWindowDimensions();
    Point min_dimensions = getMinWindowDimensions();
    Point adjusted_dimensions = adjustBoundsForAspectRatio({ width, height }, min_dimensions,
                                                           max_dimensions, aspect_ratio,
                                                           horizontal_resize, vertical_resize);

    switch (w_param) {
    case WMSZ_LEFT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.y;
      rect->left = rect->right - adjusted_dimensions.x - borders.x;
      break;
    case WMSZ_RIGHT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.y;
      rect->right = rect->left + adjusted_dimensions.x + borders.x;
      break;
    case WMSZ_TOP:
      rect->right = rect->left + adjusted_dimensions.x + borders.x;
      rect->top = rect->bottom - adjusted_dimensions.y - borders.y;
      break;
    case WMSZ_BOTTOM:
      rect->right = rect->left + adjusted_dimensions.x + borders.x;
      rect->bottom = rect->top + adjusted_dimensions.y + borders.y;
      break;
    case WMSZ_TOPLEFT:
      rect->top = rect->bottom - adjusted_dimensions.y - borders.y;
      rect->left = rect->right - adjusted_dimensions.x - borders.x;
      break;
    case WMSZ_TOPRIGHT:
      rect->top = rect->bottom - adjusted_dimensions.y - borders.y;
      rect->right = rect->left + adjusted_dimensions.x + borders.x;
      break;
    case WMSZ_BOTTOMLEFT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.y;
      rect->left = rect->right - adjusted_dimensions.x - borders.x;
      break;
    case WMSZ_BOTTOMRIGHT:
      rect->bottom = rect->top + adjusted_dimensions.y + borders.y;
      rect->right = rect->left + adjusted_dimensions.x + borders.x;
      break;
    default: break;
    }
  }

  void WindowWin32::handleResizeEnd(HWND hwnd) {
    float aspect_ratio = getAspectRatio();
    VISAGE_ASSERT(aspect_ratio > 0.0f);

    Point borders = getWindowBorderSize(hwnd);

    RECT rect;
    GetWindowRect(hwnd, &rect);
    int width = rect.right - rect.left - borders.x;
    int height = rect.bottom - rect.top - borders.y;
    handleResized(width, height);
  }

  void WindowWin32::handleDpiChange(HWND hwnd, LPARAM l_param, WPARAM w_param) {
    Point max_dimensions = getMaxWindowDimensions();
    Point min_dimensions = getMinWindowDimensions();
    Point borders = getWindowBorderSize(hwnd);
    RECT* suggested = reinterpret_cast<RECT*>(l_param);
    Point point(suggested->right - suggested->left - borders.x,
                suggested->bottom - suggested->top - borders.y);
    Point adjusted_dimensions = adjustBoundsForAspectRatio(point, min_dimensions, max_dimensions,
                                                           getAspectRatio(), true, true);

    int width = adjusted_dimensions.x;
    int height = adjusted_dimensions.y;
    SetWindowPos(hwnd, nullptr, suggested->left, suggested->top, width + borders.x,
                 height + borders.y, SWP_NOZORDER | SWP_NOACTIVATE);

    handleResized(width, height);
  }

  void WindowWin32::updateMonitor() {
    monitor_ = MonitorFromWindow(window_handle_, MONITOR_DEFAULTTONEAREST);
  }
}
#endif
