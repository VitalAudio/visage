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

#include "emoji.h"
#include "visage_utils/string_utils.h"

#include <d2d1.h>
#include <dwrite.h>
#include <wincodec.h>
#include <wrl/client.h>

using Microsoft::WRL::ComPtr;

namespace visage {

  class EmojiRasterizerImpl {
  public:
    EmojiRasterizerImpl() {
      HRESULT hr = CoInitialize(nullptr);
      if (FAILED(hr))
        return;

      hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, __uuidof(ID2D1Factory), &options_,
                             &d2d_factory_);
      if (FAILED(hr))
        return;

      hr = DWriteCreateFactory(DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory),
                               reinterpret_cast<IUnknown**>(dwrite_factory_.GetAddressOf()));
      if (FAILED(hr))
        return;

      hr = CoCreateInstance(CLSID_WICImagingFactory, nullptr, CLSCTX_INPROC_SERVER,
                            IID_PPV_ARGS(&wic_factory_));
      if (FAILED(hr))
        return;

      initialized_ = true;
    }

    void drawIntoBuffer(char32_t emoji, int font_size, int write_width, unsigned int* dest,
                        int dest_width, int x, int y) {
      if (!initialized_)
        return;

      D2D1_SIZE_U target_size = D2D1::SizeU(write_width, write_width);
      ComPtr<IWICBitmap> wic_bitmap;
      HRESULT hr = wic_factory_->CreateBitmap(target_size.width, target_size.height,
                                              GUID_WICPixelFormat32bppPBGRA, WICBitmapCacheOnLoad,
                                              &wic_bitmap);
      if (FAILED(hr))
        return;

      ComPtr<ID2D1RenderTarget> render_target;
      D2D1_RENDER_TARGET_PROPERTIES rt_properties =
          D2D1::RenderTargetProperties(D2D1_RENDER_TARGET_TYPE_DEFAULT,
                                       D2D1::PixelFormat(DXGI_FORMAT_UNKNOWN, D2D1_ALPHA_MODE_PREMULTIPLIED));

      hr = d2d_factory_->CreateWicBitmapRenderTarget(wic_bitmap.Get(), rt_properties, &render_target);
      if (FAILED(hr))
        return;

      hr = dwrite_factory_->CreateTextFormat(L"Segoe UI Emoji", nullptr, DWRITE_FONT_WEIGHT_NORMAL,
                                             DWRITE_FONT_STYLE_NORMAL, DWRITE_FONT_STRETCH_NORMAL,
                                             font_size, L"en-us", &text_format_);
      if (FAILED(hr))
        return;

      ComPtr<IDWriteTextLayout> text_layout;
      std::wstring emoji_string = String(emoji).toWide();
      hr = dwrite_factory_->CreateTextLayout(emoji_string.c_str(), emoji_string.length(),
                                             text_format_.Get(), target_size.width,
                                             target_size.height, &text_layout);
      if (FAILED(hr))
        return;

      if (FAILED(text_layout->SetTextAlignment(DWRITE_TEXT_ALIGNMENT_CENTER)))
        return;
      if (FAILED(text_layout->SetParagraphAlignment(DWRITE_PARAGRAPH_ALIGNMENT_CENTER)))
        return;

      render_target->BeginDraw();
      render_target->Clear(D2D1::ColorF(D2D1::ColorF::White, 0.0f));

      ComPtr<ID2D1SolidColorBrush> brush = nullptr;
      hr = render_target->CreateSolidColorBrush(D2D1::ColorF(D2D1::ColorF::White), &brush);
      if (FAILED(hr) || brush == nullptr)
        return;

      render_target->DrawTextLayout(D2D1::Point2F(0, 0), text_layout.Get(), brush.Get(),
                                    (D2D1_DRAW_TEXT_OPTIONS)(D2D1_DRAW_TEXT_OPTIONS_ENABLE_COLOR_FONT |
                                                             D2D1_DRAW_TEXT_OPTIONS_CLIP));
      hr = render_target->EndDraw();
      if (FAILED(hr))
        return;

      WICRect rcLock = { 0, 0, static_cast<int>(target_size.width), static_cast<int>(target_size.height) };
      ComPtr<IWICBitmapLock> lock;
      hr = wic_bitmap->Lock(&rcLock, WICBitmapLockRead, &lock);
      if (FAILED(hr))
        return;

      UINT buffer_size = 0;
      unsigned int* buffer = nullptr;
      hr = lock->GetDataPointer(&buffer_size, (unsigned char**)&buffer);
      if (FAILED(hr) || buffer == nullptr)
        return;

      for (UINT i = 0; i < buffer_size / 4; ++i) {
        int row = i / write_width;
        int col = i % write_width;
        dest[(y + row) * dest_width + x + col] = buffer[i];
      }
    }

    ~EmojiRasterizerImpl() {
      d2d_factory_ = nullptr;
      dwrite_factory_ = nullptr;
      text_format_ = nullptr;
      wic_factory_ = nullptr;
      CoUninitialize();
    }

  private:
    bool initialized_ = false;
    ComPtr<ID2D1Factory> d2d_factory_;
    D2D1_FACTORY_OPTIONS options_ = {};
    ComPtr<IDWriteFactory> dwrite_factory_;
    ComPtr<IDWriteTextFormat> text_format_;
    ComPtr<IWICImagingFactory> wic_factory_;
  };

  EmojiRasterizer::EmojiRasterizer() {
    impl_ = std::make_unique<EmojiRasterizerImpl>();
  }

  EmojiRasterizer::~EmojiRasterizer() = default;

  void EmojiRasterizer::drawIntoBuffer(char32_t emoji, int font_size, int write_width,
                                       unsigned int* dest, int dest_width, int x, int y) {
    impl_->drawIntoBuffer(emoji, font_size, write_width, dest, dest_width, x, y);
  }
}
