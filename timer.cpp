/* 
 * Simple timer class
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

#include "timer.hpp"

Timer::Timer() {
  restart();
}

double Timer::elapsed() const {
  struct timeval now;
  gettimeofday(&now, (struct timezone *)NULL);
  return (now.tv_sec - start.tv_sec) + (now.tv_usec - start.tv_usec)/1000000.0;
}

void Timer::restart() {
  gettimeofday(&start, (struct timezone *)NULL);
}

