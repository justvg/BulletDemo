#pragma once

#include "GL/glew.h"
#include "GLFW/glfw3.h"
#include "btBulletDynamicsCommon.h"

#include <stdint.h>

typedef  uint8_t u8;
typedef  uint8_t b8;
typedef  uint16_t u16;
typedef  uint32_t u32;
typedef  uint64_t u64;

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef int32_t b32;

typedef float r32;
typedef double r64;

#define Kilobytes(Value) ((Value) * 1024LL)
#define Megabytes(Value) (Kilobytes(Value) * 1024LL)
#define Gigabytes(Value) (Gigabytes(Value) * 1024LL)

#define Assert(Expression) if(!(Expression)) { *(int *)0 = 0; }
#define ArrayCount(Array) (sizeof(Array)/sizeof((Array)[0]))

#include "bullet_demo_math.cpp"