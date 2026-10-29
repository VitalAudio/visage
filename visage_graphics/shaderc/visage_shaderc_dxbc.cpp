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

#include "gpu/gpu_backend.h"

#include <d3dcompiler.h>
#include <windows.h>

using namespace visage::gpu;

bool compileDxbc(const std::string& hlsl, ShaderStage stage, std::vector<uint8_t>& result, std::string& error) {
  const char* profile = "ps_5_0";
  if (stage == ShaderStage::Vertex)
    profile = "vs_5_0";
  else if (stage == ShaderStage::Compute)
    profile = "cs_5_0";

  ID3DBlob* code = nullptr;
  ID3DBlob* errors = nullptr;
  HRESULT hr = D3DCompile(hlsl.data(), hlsl.size(), nullptr, nullptr, nullptr, "main", profile,
                          D3DCOMPILE_OPTIMIZATION_LEVEL3, 0, &code, &errors);
  if (errors) {
    error.assign(static_cast<const char*>(errors->GetBufferPointer()), errors->GetBufferSize());
    errors->Release();
  }
  if (FAILED(hr) || code == nullptr) {
    error = hlsl + "\n" + error;
    return false;
  }

  auto data = static_cast<const uint8_t*>(code->GetBufferPointer());
  result.assign(data, data + code->GetBufferSize());
  code->Release();
  return true;
}
