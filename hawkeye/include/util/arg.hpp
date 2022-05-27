// Copyright (c) 2022, Map IV, Inc.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <string>

template <class _Ty>
struct argLoader
{
  constexpr static bool ty_func_ = std::is_function<_Ty>::value;
  using func_type_ = std::conditional_t<ty_func_, _Ty*, _Ty>;

  argLoader(int argc, char** argv, _Ty&& error_exiter)
    : argc_{ argc }, argv_{ argv }, i_{ 0 }, error_exiter_{ std::forward<_Ty&&>(error_exiter) } {};

  ~argLoader()
  {
    end();
  }

  bool valid()
  {
    return i_ < argc_;
  }

  int left()
  {
    return argc_ - i_;
  }

  void drop()
  {
    if (!valid())
      err();
    ++i_;
  }

  char* top()
  {
    if (!valid())
      err();
    return argv_[i_];
  }

  char* pop()
  {
    if (!valid())
      err();
    return argv_[i_++];
  }

  void end()
  {
    if (valid())
      err();
  }

  void err()
  {
    error_exiter_("Invalid arguments. argc: " + std::to_string(argc_));
  }

private:
  int argc_;
  char** argv_;

  func_type_ error_exiter_;
  int i_;
};