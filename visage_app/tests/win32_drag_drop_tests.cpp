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

#include "visage/app.h"

#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#if VISAGE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <ole2.h>
#include <shlobj.h>
#include <windows.h>

using namespace visage;

namespace {
  // Patches this executable's import address table. visage is linked in statically, so its Win32
  // calls go through it.
  class ScopedImportHook {
  public:
    ScopedImportHook(const char* dll_name, const char* function_name, void* replacement) {
      auto base = reinterpret_cast<BYTE*>(GetModuleHandleW(nullptr));
      auto dos = reinterpret_cast<IMAGE_DOS_HEADER*>(base);
      auto nt = reinterpret_cast<IMAGE_NT_HEADERS*>(base + dos->e_lfanew);
      const IMAGE_DATA_DIRECTORY& imports =
          nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
      if (imports.VirtualAddress == 0)
        return;

      auto descriptor = reinterpret_cast<IMAGE_IMPORT_DESCRIPTOR*>(base + imports.VirtualAddress);
      for (; descriptor->Name; ++descriptor) {
        if (_stricmp(reinterpret_cast<const char*>(base + descriptor->Name), dll_name) != 0 ||
            descriptor->OriginalFirstThunk == 0)
          continue;

        auto names = reinterpret_cast<IMAGE_THUNK_DATA*>(base + descriptor->OriginalFirstThunk);
        auto slots = reinterpret_cast<IMAGE_THUNK_DATA*>(base + descriptor->FirstThunk);
        for (; names->u1.AddressOfData; ++names, ++slots) {
          if (IMAGE_SNAP_BY_ORDINAL(names->u1.Ordinal))
            continue;
          auto by_name = reinterpret_cast<IMAGE_IMPORT_BY_NAME*>(base + names->u1.AddressOfData);
          if (strcmp(reinterpret_cast<const char*>(by_name->Name), function_name) == 0)
            slots_.push_back(reinterpret_cast<void**>(&slots->u1.Function));
        }
      }

      if (!slots_.empty())
        original_ = *slots_.front();
      for (void** slot : slots_)
        write(slot, replacement);
    }

    ~ScopedImportHook() {
      for (void** slot : slots_)
        write(slot, original_);
    }

    bool installed() const { return !slots_.empty(); }

    template<typename T>
    T original() const {
      return reinterpret_cast<T>(original_);
    }

  private:
    static void write(void** slot, void* value) {
      DWORD protection = 0;
      VirtualProtect(slot, sizeof(void*), PAGE_READWRITE, &protection);
      *slot = value;
      VirtualProtect(slot, sizeof(void*), protection, &protection);
    }

    std::vector<void**> slots_;
    void* original_ = nullptr;
  };

  class NullEventHandler : public Window::EventHandler {
  public:
    HitTestResult handleHitTest(int, int) override { return HitTestResult::Client; }
    HitTestResult currentHitTest() const override { return HitTestResult::Client; }
    void handleMouseMove(int, int, int, int) override { }
    void handleMouseDown(MouseButton, int, int, int, int, int) override { }
    void handleMouseUp(MouseButton, int, int, int, int, int) override { }
    void handleMouseEnter(int, int) override { }
    void handleMouseLeave(int, int, int, int) override { }
    void handleMouseWheel(float, float, float, float, int, int, int, int, bool) override { }

    bool handleKeyDown(KeyCode, int, bool) override { return false; }
    bool handleKeyUp(KeyCode, int) override { return false; }

    bool handleTextInput(const std::string&) override { return false; }
    bool hasActiveTextEntry() override { return false; }

    void handleFocusLost() override { }
    void handleFocusGained() override { }
    void handleResized(int, int) override { }
    void handleWindowShown() override { }
    void handleWindowHidden() override { }
    bool handleCloseRequested() override { return true; }

    bool handleFileDrag(int, int, const std::vector<std::string>&) override { return false; }
    void handleFileDragLeave() override { }
    bool handleFileDrop(int, int, const std::vector<std::string>&) override { return false; }

    bool isDragDropSource() override { return false; }
    std::string startDragDropSource() override { return {}; }
    void cleanupDragDropSource() override { }
  };

  class DragSourceHandler : public NullEventHandler {
  public:
    bool isDragDropSource() override { return true; }
    std::string startDragDropSource() override {
      starts++;
      return path;
    }
    void cleanupDragDropSource() override { cleanups++; }

    std::string path;
    int starts = 0;
    int cleanups = 0;
  };

  class FileDropHandler : public NullEventHandler {
  public:
    bool handleFileDrag(int, int, const std::vector<std::string>& files) override {
      dragged = files;
      return true;
    }

    bool handleFileDrop(int, int, const std::vector<std::string>& files) override {
      dropped = files;
      return true;
    }

    std::vector<std::string> dragged;
    std::vector<std::string> dropped;
  };

  std::function<void(IDataObject*)>* drag_inspector = nullptr;
  int drags_started = 0;

  HRESULT WINAPI inspectingDoDragDrop(IDataObject* data_object, IDropSource*, DWORD,
                                      DWORD* effect) {
    drags_started++;
    if (drag_inspector && data_object)
      (*drag_inspector)(data_object);
    if (effect)
      *effect = DROPEFFECT_NONE;
    return DRAGDROP_S_CANCEL;
  }

  bool dragFileOutOfWindow(const std::string& path, std::function<void(IDataObject*)> inspect) {
    DragSourceHandler handler;
    handler.path = path;
    std::unique_ptr<Window> window = createWindow(Dimension::nativePixels(200),
                                                  Dimension::nativePixels(200));
    if (window == nullptr || window->nativeHandle() == nullptr)
      return false;

    window->setEventHandler(&handler);
    {
      ScopedImportHook hook("ole32.dll", "DoDragDrop",
                            reinterpret_cast<void*>(&inspectingDoDragDrop));
      if (!hook.installed())
        return false;

      drags_started = 0;
      drag_inspector = &inspect;
      SendMessageW(static_cast<HWND>(window->nativeHandle()), WM_LBUTTONDOWN, MK_LBUTTON,
                   MAKELPARAM(20, 20));
      drag_inspector = nullptr;
    }
    window->clearEventHandler();
    return drags_started == 1 && handler.starts == 1 && handler.cleanups == 1;
  }

  std::vector<std::wstring> hdropPaths(HGLOBAL global) {
    std::vector<std::wstring> paths;
    auto drop = static_cast<HDROP>(GlobalLock(global));
    if (drop == nullptr)
      return paths;

    UINT count = DragQueryFileW(drop, 0xFFFFFFFF, nullptr, 0);
    for (UINT i = 0; i < count; ++i) {
      UINT length = DragQueryFileW(drop, i, nullptr, 0);
      std::wstring path(length + 1, L'\0');
      DragQueryFileW(drop, i, path.data(), length + 1);
      path.resize(length);
      paths.push_back(path);
    }
    GlobalUnlock(global);
    return paths;
  }

  using GlobalAlloc_t = HGLOBAL(WINAPI*)(UINT, SIZE_T);
  using GlobalLock_t = LPVOID(WINAPI*)(HGLOBAL);
  GlobalAlloc_t real_global_alloc = nullptr;
  GlobalLock_t real_global_lock = nullptr;

  DWORD recording_thread = 0;
  std::vector<HGLOBAL> moveable_allocations;

  HGLOBAL WINAPI recordingGlobalAlloc(UINT flags, SIZE_T bytes) {
    HGLOBAL result = real_global_alloc(flags, bytes);
    if (result && (flags & GMEM_MOVEABLE) && GetCurrentThreadId() == recording_thread)
      moveable_allocations.push_back(result);
    return result;
  }

  bool fail_fixed_allocations = false;
  bool fail_locks = false;
  int failed_allocations = 0;
  int failed_locks = 0;

  HGLOBAL WINAPI failingGlobalAlloc(UINT flags, SIZE_T bytes) {
    if (fail_fixed_allocations && (flags & GMEM_MOVEABLE) == 0) {
      failed_allocations++;
      SetLastError(ERROR_NOT_ENOUGH_MEMORY);
      return nullptr;
    }
    return real_global_alloc(flags, bytes);
  }

  LPVOID WINAPI failingGlobalLock(HGLOBAL global) {
    if (fail_locks) {
      failed_locks++;
      SetLastError(ERROR_NOT_ENOUGH_MEMORY);
      return nullptr;
    }
    return real_global_lock(global);
  }

  class FileListDataObject : public IDataObject {
  public:
    explicit FileListDataObject(std::vector<std::wstring> paths) : paths_(std::move(paths)) { }

    HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** object) override {
      if (riid == IID_IUnknown || riid == IID_IDataObject) {
        *object = static_cast<IDataObject*>(this);
        AddRef();
        return S_OK;
      }
      *object = nullptr;
      return E_NOINTERFACE;
    }

    ULONG STDMETHODCALLTYPE AddRef() override { return ++references_; }
    ULONG STDMETHODCALLTYPE Release() override { return --references_; }

    HRESULT STDMETHODCALLTYPE GetData(FORMATETC* format, STGMEDIUM* medium) override {
      if (format == nullptr || medium == nullptr)
        return E_INVALIDARG;
      if (QueryGetData(format) != S_OK)
        return DV_E_FORMATETC;

      size_t characters = 1;
      for (const std::wstring& path : paths_)
        characters += path.size() + 1;

      HGLOBAL global = GlobalAlloc(GHND, sizeof(DROPFILES) + characters * sizeof(wchar_t));
      if (global == nullptr)
        return E_OUTOFMEMORY;

      auto drop_files = static_cast<DROPFILES*>(GlobalLock(global));
      drop_files->pFiles = sizeof(DROPFILES);
      drop_files->fWide = TRUE;
      auto destination = reinterpret_cast<wchar_t*>(reinterpret_cast<char*>(drop_files) +
                                                    sizeof(DROPFILES));
      for (const std::wstring& path : paths_) {
        memcpy(destination, path.c_str(), path.size() * sizeof(wchar_t));
        destination += path.size() + 1;
      }
      GlobalUnlock(global);

      medium->tymed = TYMED_HGLOBAL;
      medium->hGlobal = global;
      medium->pUnkForRelease = nullptr;
      return S_OK;
    }

    HRESULT STDMETHODCALLTYPE GetDataHere(FORMATETC*, STGMEDIUM*) override { return E_NOTIMPL; }

    HRESULT STDMETHODCALLTYPE QueryGetData(FORMATETC* format) override {
      if (format->cfFormat == CF_HDROP && (format->tymed & TYMED_HGLOBAL))
        return S_OK;
      return DV_E_FORMATETC;
    }

    HRESULT STDMETHODCALLTYPE GetCanonicalFormatEtc(FORMATETC*, FORMATETC* out) override {
      out->ptd = nullptr;
      return E_NOTIMPL;
    }

    HRESULT STDMETHODCALLTYPE SetData(FORMATETC*, STGMEDIUM*, BOOL) override { return E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE EnumFormatEtc(DWORD, IEnumFORMATETC**) override { return E_NOTIMPL; }
    HRESULT STDMETHODCALLTYPE DAdvise(FORMATETC*, DWORD, IAdviseSink*, DWORD*) override {
      return OLE_E_ADVISENOTSUPPORTED;
    }
    HRESULT STDMETHODCALLTYPE DUnadvise(DWORD) override { return OLE_E_ADVISENOTSUPPORTED; }
    HRESULT STDMETHODCALLTYPE EnumDAdvise(IEnumSTATDATA**) override {
      return OLE_E_ADVISENOTSUPPORTED;
    }

  private:
    std::vector<std::wstring> paths_;
    ULONG references_ = 1;
  };

  std::string utf8(const std::wstring& wide) {
    int size = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), static_cast<int>(wide.size()),
                                   nullptr, 0, nullptr, nullptr);
    std::string result(size, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), static_cast<int>(wide.size()), result.data(),
                        size, nullptr, nullptr);
    return result;
  }

  using RegisterDragDrop_t = HRESULT(WINAPI*)(HWND, IDropTarget*);
  RegisterDragDrop_t real_register_drag_drop = nullptr;
  IDropTarget* registered_target = nullptr;
  HWND registered_window = nullptr;

  HRESULT WINAPI capturingRegisterDragDrop(HWND hwnd, IDropTarget* target) {
    registered_window = hwnd;
    registered_target = target;
    return real_register_drag_drop(hwnd, target);
  }
}

TEST_CASE("Win32 dragging a file out of a window frees the HDROP it built", "[integration]") {
  const std::string path = "C:\\visage_test\\dragged file.wav";
  const std::vector<std::wstring> expected = { L"C:\\visage_test\\dragged file.wav" };

  ScopedImportHook alloc_hook("KERNEL32.dll", "GlobalAlloc",
                              reinterpret_cast<void*>(&recordingGlobalAlloc));
  REQUIRE(alloc_hook.installed());
  real_global_alloc = alloc_hook.original<GlobalAlloc_t>();

  for (int drag = 0; drag < 3; ++drag) {
    moveable_allocations.clear();
    recording_thread = GetCurrentThreadId();
    HRESULT get_result = E_FAIL;
    STGMEDIUM receiver_copy = {};
    bool dragged = dragFileOutOfWindow(path, [&](IDataObject* data) {
      FORMATETC format = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_HGLOBAL };
      get_result = data->GetData(&format, &receiver_copy);
    });
    recording_thread = 0;

    REQUIRE(moveable_allocations.size() == 1);
    HGLOBAL drop = moveable_allocations.front();
    bool freed = GlobalFlags(drop) == GMEM_INVALID_HANDLE;
    if (!freed)
      GlobalFree(drop);

    REQUIRE(dragged);
    REQUIRE(get_result == S_OK);
    REQUIRE(receiver_copy.tymed == TYMED_HGLOBAL);
    REQUIRE(receiver_copy.pUnkForRelease == nullptr);
    REQUIRE(hdropPaths(receiver_copy.hGlobal) == expected);
    ReleaseStgMedium(&receiver_copy);

    REQUIRE(freed);
  }
}

TEST_CASE("Win32 drag data accepts requests that combine HGLOBAL with other storage types",
          "[integration]") {
  const std::string path = "C:\\visage_test\\dragged file.wav";
  const std::vector<std::wstring> expected = { L"C:\\visage_test\\dragged file.wav" };

  HRESULT exact_result = E_FAIL;
  DWORD exact_tymed = TYMED_NULL;
  std::vector<std::wstring> exact_paths;
  HRESULT combined_query = E_FAIL;
  HRESULT combined_result = E_FAIL;
  DWORD combined_tymed = TYMED_NULL;
  IUnknown* combined_release = nullptr;
  std::vector<std::wstring> combined_paths;
  HRESULT stream_query = S_OK;
  HRESULT stream_result = S_OK;

  bool dragged = dragFileOutOfWindow(path, [&](IDataObject* data) {
    FORMATETC exact = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_HGLOBAL };
    STGMEDIUM medium = {};
    exact_result = data->GetData(&exact, &medium);
    if (exact_result == S_OK) {
      exact_tymed = medium.tymed;
      exact_paths = hdropPaths(medium.hGlobal);
      ReleaseStgMedium(&medium);
    }

    FORMATETC combined = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1,
                           TYMED_HGLOBAL | TYMED_ISTREAM | TYMED_FILE };
    combined_query = data->QueryGetData(&combined);
    medium = {};
    combined_result = data->GetData(&combined, &medium);
    if (combined_result == S_OK) {
      combined_tymed = medium.tymed;
      combined_release = medium.pUnkForRelease;
      if (medium.tymed == TYMED_HGLOBAL)
        combined_paths = hdropPaths(medium.hGlobal);
      ReleaseStgMedium(&medium);
    }

    FORMATETC stream_only = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_ISTREAM };
    stream_query = data->QueryGetData(&stream_only);
    medium = {};
    stream_result = data->GetData(&stream_only, &medium);
    if (stream_result == S_OK)
      ReleaseStgMedium(&medium);
  });

  REQUIRE(dragged);
  REQUIRE(exact_result == S_OK);
  REQUIRE(exact_tymed == TYMED_HGLOBAL);
  REQUIRE(exact_paths == expected);
  REQUIRE(stream_query == DV_E_FORMATETC);
  REQUIRE(stream_result == DV_E_FORMATETC);

  REQUIRE(combined_query == S_OK);
  REQUIRE(combined_result == S_OK);
  REQUIRE(combined_tymed == TYMED_HGLOBAL);
  REQUIRE(combined_release == nullptr);
  REQUIRE(combined_paths == expected);
}

TEST_CASE("Win32 drag data reports E_OUTOFMEMORY when it can't copy the file list",
          "[integration]") {
  HRESULT alloc_result = E_FAIL;
  DWORD alloc_tymed = TYMED_NULL;
  HGLOBAL alloc_global = nullptr;
  HRESULT lock_result = E_FAIL;
  DWORD lock_tymed = TYMED_NULL;

  ScopedImportHook alloc_hook("KERNEL32.dll", "GlobalAlloc",
                              reinterpret_cast<void*>(&failingGlobalAlloc));
  ScopedImportHook lock_hook("KERNEL32.dll", "GlobalLock",
                             reinterpret_cast<void*>(&failingGlobalLock));
  REQUIRE(alloc_hook.installed());
  REQUIRE(lock_hook.installed());
  real_global_alloc = alloc_hook.original<GlobalAlloc_t>();
  real_global_lock = lock_hook.original<GlobalLock_t>();
  failed_allocations = 0;
  failed_locks = 0;

  bool dragged = dragFileOutOfWindow("C:\\visage_test\\out of memory.wav", [&](IDataObject* data) {
    FORMATETC format = { CF_HDROP, nullptr, DVASPECT_CONTENT, -1, TYMED_HGLOBAL };

    STGMEDIUM medium = {};
    fail_fixed_allocations = true;
    alloc_result = data->GetData(&format, &medium);
    fail_fixed_allocations = false;
    alloc_tymed = medium.tymed;
    alloc_global = medium.hGlobal;
    if (SUCCEEDED(alloc_result))
      ReleaseStgMedium(&medium);

    medium = {};
    fail_locks = true;
    lock_result = data->GetData(&format, &medium);
    fail_locks = false;
    lock_tymed = medium.tymed;
    if (SUCCEEDED(lock_result))
      ReleaseStgMedium(&medium);
  });

  REQUIRE(dragged);
  REQUIRE(failed_allocations == 1);
  REQUIRE(failed_locks == 1);
  CHECK(alloc_result == E_OUTOFMEMORY);
  CHECK(alloc_tymed == TYMED_NULL);
  CHECK(alloc_global == nullptr);
  CHECK(lock_result == E_OUTOFMEMORY);
  CHECK(lock_tymed == TYMED_NULL);
}

TEST_CASE("Win32 drag data format enumerators answer QueryInterface for IEnumFORMATETC",
          "[integration]") {
  HRESULT com_result = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);

  HRESULT enum_result = E_FAIL;
  HRESULT next_result = E_FAIL;
  ULONG fetched = 0;
  FORMATETC format = {};
  HRESULT enum_query = E_FAIL;
  bool same_object = false;
  HRESULT data_object_query = S_OK;
  HRESULT marshal_result = E_FAIL;

  bool dragged = dragFileOutOfWindow("C:\\visage_test\\dragged.wav", [&](IDataObject* data) {
    IEnumFORMATETC* enumerator = nullptr;
    enum_result = data->EnumFormatEtc(DATADIR_GET, &enumerator);
    if (FAILED(enum_result) || enumerator == nullptr)
      return;

    next_result = enumerator->Next(1, &format, &fetched);
    enumerator->Reset();

    void* as_enumerator = nullptr;
    enum_query = enumerator->QueryInterface(IID_IEnumFORMATETC, &as_enumerator);
    same_object = as_enumerator == static_cast<void*>(enumerator);
    if (as_enumerator)
      static_cast<IUnknown*>(as_enumerator)->Release();

    void* as_data_object = nullptr;
    data_object_query = enumerator->QueryInterface(IID_IDataObject, &as_data_object);
    if (as_data_object)
      static_cast<IUnknown*>(as_data_object)->Release();

    IStream* stream = nullptr;
    if (SUCCEEDED(CreateStreamOnHGlobal(nullptr, TRUE, &stream))) {
      marshal_result = CoMarshalInterface(stream, IID_IEnumFORMATETC, enumerator, MSHCTX_LOCAL,
                                          nullptr, MSHLFLAGS_NORMAL);
      if (SUCCEEDED(marshal_result)) {
        LARGE_INTEGER start = {};
        stream->Seek(start, STREAM_SEEK_SET, nullptr);
        CoReleaseMarshalData(stream);
      }
      stream->Release();
    }
    CoDisconnectObject(enumerator, 0);
    enumerator->Release();
  });

  if (SUCCEEDED(com_result))
    CoUninitialize();

  REQUIRE(dragged);
  REQUIRE(enum_result == S_OK);
  REQUIRE(next_result == S_OK);
  REQUIRE(fetched == 1);
  REQUIRE(format.cfFormat == CF_HDROP);
  REQUIRE(format.tymed == TYMED_HGLOBAL);

  CHECK(enum_query == S_OK);
  CHECK(same_object);
  CHECK(data_object_query == E_NOINTERFACE);
  CHECK(marshal_result == S_OK);
}

TEST_CASE("Win32 file drops deliver paths longer than MAX_PATH intact", "[integration]") {
  std::wstring short_path = L"C:\\visage_test\\short.wav";
  std::wstring long_path = L"C:\\visage_test";
  for (int i = 0; long_path.size() < 2 * MAX_PATH; ++i)
    long_path += L"\\a fairly long directory name " + std::to_wstring(i);
  long_path += L"\\the file at the end.wav";
  std::wstring unicode_path = L"C:\\visage_test\\\u00e9t\u00e9 \u65e5\u672c\u8a9e "
                              L"\u0436\u0443\u043a \U0001F3B5.wav";
  REQUIRE(long_path.size() > MAX_PATH);

  registered_target = nullptr;
  registered_window = nullptr;
  std::unique_ptr<Window> window;
  {
    ScopedImportHook hook("ole32.dll", "RegisterDragDrop",
                          reinterpret_cast<void*>(&capturingRegisterDragDrop));
    REQUIRE(hook.installed());
    real_register_drag_drop = hook.original<RegisterDragDrop_t>();
    window = createWindow(Dimension::nativePixels(200), Dimension::nativePixels(200));
  }
  REQUIRE(window);
  HWND hwnd = static_cast<HWND>(window->nativeHandle());
  REQUIRE(hwnd);
  REQUIRE(registered_window == hwnd);
  REQUIRE(registered_target);

  FileDropHandler handler;
  window->setEventHandler(&handler);

  FileListDataObject data({ short_path, long_path, unicode_path });
  POINT client_point = { 20, 20 };
  ClientToScreen(hwnd, &client_point);
  POINTL point = { client_point.x, client_point.y };

  DWORD effect = DROPEFFECT_COPY;
  REQUIRE(registered_target->DragEnter(&data, MK_LBUTTON, point, &effect) == S_OK);
  REQUIRE(effect == DROPEFFECT_COPY);
  effect = DROPEFFECT_COPY;
  REQUIRE(registered_target->Drop(&data, 0, point, &effect) == S_OK);
  REQUIRE(effect == DROPEFFECT_COPY);

  window->clearEventHandler();
  window.reset();

  REQUIRE(handler.dragged.size() == 3);
  REQUIRE(handler.dropped.size() == 3);
  REQUIRE(handler.dropped[0] == utf8(short_path));
  REQUIRE(handler.dropped[2] == utf8(unicode_path));
  REQUIRE(handler.dropped[1].size() == utf8(long_path).size());
  REQUIRE(handler.dropped[1] == utf8(long_path));
  REQUIRE(handler.dragged == handler.dropped);
}

#endif
