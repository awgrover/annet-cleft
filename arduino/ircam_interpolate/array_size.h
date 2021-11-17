#pragma once
template <typename T,unsigned S> inline constexpr unsigned arraysize(const T (&v)[S]) { return S; };
template <typename T,unsigned S> inline constexpr unsigned array_size(const T (&v)[S]) { return S; };
