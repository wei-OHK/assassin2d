// Copyright (C) 2015-2016 Wei@OHK, Hiroshima University.
// This file is part of the "assassin2d".
// For conditions of distribution and use, see copyright notice in assassin2d.h

#ifndef _ASSA2D_TYPES_H
#define _ASSA2D_TYPES_H

#include <random>

#include <bulwark/bulwark.h>

#define	RAND_LIMIT	32767

namespace assa2d {
/// define some types.
typedef bul::dynamics::Node Node;
typedef bul::dynamics::Trigger Trigger;
typedef bul::manager::Monitor Monitor;

typedef bul::dynamics::Node_Type Node_Type;

/// Backdoors for full access.
template<typename>
class Accessor;

/// Shape types.
enum class ShapeType {
	Circle,
	Polygon
};

/// Random number in range [-1,1]
inline float32 RandomFloat()
{
    float32 r = (float32)(std::rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r = 2.0f * r - 1.0f;
    return r;
}

/// Random floating point number in range [lo, hi]
inline float32 RandomFloat(float32 lo, float32 hi)
{
    float32 r = (float32)(std::rand() & (RAND_LIMIT));
    r /= RAND_LIMIT;
    r = (hi - lo) * r + lo;
    return r;
}

} /* namespace assa2d */

#endif /* _ASSA2D_TYPES_H */
