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

#pragma once

#include <memory>

namespace visage {
  template<class T>
  class clone_ptr {
  public:
    static_assert(std::is_copy_constructible_v<T>, "T must be copy constructible for clone_ptr");

    clone_ptr() = default;
    clone_ptr(std::unique_ptr<T> p) : p_(std::move(p)) { }

    clone_ptr(const clone_ptr& other) : p_(other.p_ ? std::make_unique<T>(*other.p_) : nullptr) { }

    clone_ptr& operator=(const clone_ptr& other) {
      if (this != &other)
        p_ = other.p_ ? std::make_unique<T>(*other.p_) : nullptr;
      return *this;
    }

    clone_ptr(clone_ptr&&) noexcept = default;
    clone_ptr& operator=(clone_ptr&&) noexcept = default;

    void reset() { p_.reset(); }
    void reset(std::unique_ptr<T> p) { p_ = std::move(p); }
    void swap(clone_ptr& other) noexcept { p_.swap(other.p_); }
    T* get() const { return p_.get(); }
    T& operator*() const { return p_.operator*(); }
    T* operator->() const { return p_.operator->(); }
    explicit operator bool() const { return static_cast<bool>(p_); }

  private:
    std::unique_ptr<T> p_;
  };
}
