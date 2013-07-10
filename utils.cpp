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

#include "utils.hpp"

float randInRange(float min, float max)
{
  return min + (max - min) * rand() / (float)RAND_MAX;
}

float distanceSquared( const Vector3& a, const Vector3& b )
{
  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

void addVec( Vector3& a, const Vector3& b )
{
  a.x += b.x;
  a.y += b.y;
  a.z += b.z;
}

Vector3 diffVec( const Vector3& a, const Vector3& b )
{
  return Vector3( a.x - b.x, a.y - b.y, a.z - b.z );
}

Vector3 prodVec( const Vector3& a, const float c )
{
  return Vector3( a.x * c, a.y * c, a.z * c );
}

AABB cellAABB( int i, int j, int k )
{
  const float offsetX = -2.0f;
  const float offsetZ = -2.0f;
  const float cellSize = 8.0f / 64.0f;
  AABB res( Vector3( offsetX + i * cellSize, j * cellSize, offsetZ + k * cellSize ),
            Vector3( offsetX + (i+1) * cellSize, (j+1) * cellSize, offsetZ + (k+1) * cellSize ) );
  return res;
}

