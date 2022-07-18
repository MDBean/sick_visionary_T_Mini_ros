//
// Copyright note: Redistribution and use in source, with or without modification, are permitted.
// 
// Created: December 2017
// 
// SICK AG, Waldkirch
// email: TechSupport0905@sick.de

#pragma once

#include <cstdint>
#include <cstring>

#define ENDIAN_LITTLE

namespace visionary 
{

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template<class T>
T readUnaligned(const void *ptr)
{
  T r;
  memcpy(&r, ptr, sizeof(T));
  return r;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


template <typename TAlias, typename T>
inline T byteswapAlias(T val);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline uint8_t byteswap(uint8_t val)
{
  return val;
}
inline int8_t byteswap(int8_t val)
{
  return byteswapAlias<uint8_t>(val);
}
inline char byteswap(char val)
{
  return byteswapAlias<uint8_t>(val);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline uint16_t byteswap(uint16_t val)
{
  return ((val << 8) & 0xFF00) | ((val >> 8) & 0x00FF);
}
inline int16_t byteswap(int16_t val)
{
  return byteswapAlias<uint16_t>(val);
}
inline wchar_t byteswap(wchar_t val)
{
  return byteswapAlias<uint16_t>(val);
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline uint32_t byteswap(uint32_t val)
{
  val = ((val << 8) & 0xFF00FF00) | ((val >> 8) & 0x00FF00FF);
  return ((val << 16) & 0xFFFF0000) | ((val >> 16) & 0x0000FFFF);
}
inline int32_t byteswap(int32_t val)
{
  return byteswapAlias<uint32_t>(val);
}
inline float byteswap(float val)
{
  union {
    float f32;
    uint32_t  u32;
  } v;
  v.f32 = val;
  v.u32 = byteswap(v.u32);

  return v.f32;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

inline uint64_t byteswap(uint64_t val)
{
  val = ((val << 8) & 0xFF00FF00FF00FF00) | ((val >> 8) & 0x00FF00FF00FF00FF);
  val = ((val << 16) & 0xFFFF0000FFFF0000) | ((val >> 16) & 0x0000FFFF0000FFFF);
  return ((val << 32) & 0xFFFFFFFF00000000) | ((val >> 32) & 0x00000000FFFFFFFF);
}
inline int64_t byteswap(int64_t val)
{
  return byteswapAlias<uint64_t>(val);
}
inline double byteswap(double val)
{
  union {
    double f64;
    uint64_t  u64;
  } v;
  v.f64 = val;
  v.u64 = byteswap(v.u64);

  return v.f64;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename TAlias, typename T>
inline T byteswapAlias(T val)
{
  return static_cast<T>(byteswap(static_cast<TAlias>(val)));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

#if defined ENDIAN_LITTLE
template <typename T>
inline T nativeToLittleEndian(T x)
{
  return x;
}

template <typename T>
inline T littleEndianToNative(T x)
{
  return x;
}

template <typename T>
inline T nativeToBigEndian(T x)
{
  return byteswap(x);
}

template <typename T>
inline T bigEndianToNative(T x)
{
  return byteswap(x);
}
#elif defined ENDIAN_BIG
template <typename T>
inline T nativeToLittleEndian(T x)
{
  return byteswap(x);
}

template <typename T>
inline T littleEndianToNative(T x)
{
  return byteswap(x);
}

template <typename T>
inline T nativeToBigEndian(T x)
{
  return x;
}

template <typename T>
inline T bigEndianToNative(T x)
{
  return x;
}
#else
#error Endianess is not defined, please define either LITTLE_ENDIAN or BIG_ENDIAN depending on the platform.
#endif

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

template <typename T>
inline T readUnalignBigEndian(const void *ptr)
{
  return bigEndianToNative<T>(readUnaligned<T>(ptr));
}

template <typename T>
inline T readUnalignLittleEndian(const void *ptr)
{
  return littleEndianToNative<T>(readUnaligned<T>(ptr));
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

}
