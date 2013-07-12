/*
 * Some structs, class and function
 *
 * Copyright (C) Fedorenko Maxim <varlllog@gmail.com>
 *               Vlad Kaipetsky <vkaipetsky@gmail.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <vector>

struct Index3
{
  int i, j, k;
};

struct Vector3
{
  Vector3() {} // empty default ctor for perf reasons
  Vector3( float newX, float newY, float newZ ) : x(newX), y(newY), z(newZ) {}

  Vector3& operator -= ( const Vector3& rhs )
  {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
  }

  Vector3& operator += ( const Vector3& rhs )
  {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
  }

  Vector3& operator *= ( const float rhs )
  {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
  }


  float x, y, z;
};

inline Vector3 operator + ( const Vector3& lhs, const Vector3& rhs )
{
  return Vector3( lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z );
}

inline Vector3 operator - ( const Vector3& lhs, const Vector3& rhs )
{
  return Vector3( lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z );
}

inline Vector3 operator * ( const Vector3& lhs, const float rhs )
{
  return Vector3( lhs.x * rhs, lhs.y * rhs, lhs.z * rhs );
}

struct Particle
{
  struct Vector3 pos;
  struct Vector3 vel;
  struct Vector3 color;
  struct Index3 bucket;
};

// spatial grid
class Cell
{
public:
  std::vector<Particle> particles;
};

struct AABB
{
  AABB( const Vector3& newMin, const Vector3& newMax ) : min(newMin), max(newMax) {}
  Vector3 min;
  Vector3 max;
};

void drawAABB( const AABB& aabb, unsigned int color = 0xff00ffff ); // draw the AABBs purple by default
void interpolateColor( float r0, float g0, float b0,
                       float r1, float g1, float b1, float t );

float randInRange(float min, float max);
float distanceSquared( const Vector3& a, const Vector3& b );

inline float pow2( float val ) { return val*val; };

AABB cellAABB( int i, int j, int k );
 
#endif /* __UTILS_HPP__ */

