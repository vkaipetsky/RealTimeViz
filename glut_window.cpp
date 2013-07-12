/*
 * GLUT window class
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

#include <GL/freeglut.h>   // Header File For The GLUT Library
#include <GL/gl.h>         // Header File For The OpenGL32 Library
#include <GL/glu.h>        // Header File For The GLu32 Library
#include <GL/glx.h>        // Header file fot the glx libraries.

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "timer.hpp"
#include "utils.hpp"
#include "glut_window.hpp"

#define NUM_PARTICLES 10000

#define NUM_CELLS_ACROSS_X 32
#define NUM_CELLS_ACROSS_Y 8
#define NUM_CELLS_ACROSS_Z 32

Cell cells[NUM_CELLS_ACROSS_X][NUM_CELLS_ACROSS_Y][NUM_CELLS_ACROSS_Z];
struct Particle particles[NUM_PARTICLES];

/* A general OpenGL initialization function.  Sets all of the initial parameters. 
 * We call this right after our OpenGL window is created.     */
void GLUTWindow::InitGL( int w, int h ) {
  
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);    // This Will Clear The Background Color To Black
  glClearDepth(1.0);                       // Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS);                    // The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST);                 // Enables Depth Testing
  glShadeModel(GL_SMOOTH);                 // Enables Smooth Color Shading
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();             // Reset The Projection Matrix
  
  // Calculate The Aspect Ratio Of The Window 
  gluPerspective(45.0f,(GLfloat)w/(GLfloat)h,0.1f,100.0f);
  
  glMatrixMode(GL_MODELVIEW);
}

/* The function called when our window is resized */
void GLUTWindow::ReSizeGLScene( int w, int h ) {
  // Prevent A Divide By Zero If The Window Is Too Small
  if( h == 0 ) {
    h = 1;
  }
  // Reset The Current Viewport And Perspective Transformation
  glViewport(0, 0, w, h);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  
  gluPerspective(45.0f,(GLfloat)w/(GLfloat)h,0.1f,100.0f);
  glMatrixMode(GL_MODELVIEW);
}


// the heightfield memory
const int HF_WIDTH = 50;//100;
const int HF_HEIGHT = 50;//100;
float heights[HF_WIDTH][HF_HEIGHT];
int generated = 0, i, j;


void GenerateHeightfield()
{
  // generate a random heightfield within the cube 1.0f across
  for ( i = 0;  i < HF_HEIGHT;  i++ )
  {
    for ( j = 0;  j < HF_WIDTH;  j++)
    {
      float distanceFromCenter = sqrtf( pow2(0.0f+(float)(i-HF_WIDTH/3)*10.0f/HF_WIDTH) + pow2(0.0f + (float)(j-HF_HEIGHT/2)*10.0f/HF_HEIGHT) );
      heights[i][j] = 0.8f * cosf( distanceFromCenter ) * cosf( distanceFromCenter ) / (1.0f + 0.1f * distanceFromCenter * distanceFromCenter );
    }
  }
}


void GenerateParticles()
{
  // generate the particle positions above the heightfield
  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
    particles[i].pos.x = randInRange( -2.0f, 2.0f );
    particles[i].pos.y = randInRange( 1.0f, 2.0f );
    particles[i].pos.z = randInRange( -2.0f, 2.0f );

    particles[i].vel.x = particles[i].vel.y = particles[i].vel.z = 0.0f;
  }

  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
    // color the particles depending on the bucket they belong to
    particles[i].bucket.i = ( particles[i].pos.x + 2.0f ) / ( 4.0f / 16.0f );
    particles[i].bucket.j = ( particles[i].pos.y - 1.0f ) / ( 1.0f / 2.0f );
    particles[i].bucket.k = ( particles[i].pos.z + 2.0f ) / ( 4.0f / 16.0f );
    particles[i].color.x = ( particles[i].pos.x + 2.0f ) / 4.0f;
    particles[i].color.y = ( particles[i].pos.y - 1.0f ) / 1.0f;
    particles[i].color.z = ( particles[i].pos.z + 2.0f ) / 4.0f;
  }
  
  // put the particles into the appropriate grid cells
  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
    int cellI = ( particles[i].pos.x + 2.0f ) * 32.0f / ( 4.0f );
    int cellJ = ( particles[i].pos.y - 1.0f ) * 8.0f / ( 1.0f );
    int cellK = ( particles[i].pos.z + 2.0f ) * 32.0f / ( 4.0f );
    // limit the indices to the sensible range!
    if ( cellI < 0 ) cellI = 0;
    if ( cellJ < 0 ) cellJ = 0;
    if ( cellK < 0 ) cellK = 0;
    if ( cellI > 31 ) cellI = 31;
    if ( cellJ > 7 )  cellJ = 7;
    if ( cellK > 31 ) cellK = 31;
    cells[cellI][cellJ][cellK].particles.push_back( particles[i] );
  }

  // TODO: assert here that the particles are in the correct cells!
}


void DrawHeightfield()
{
  glBegin(GL_TRIANGLES);        // start drawing the cube.
  
  // draw our heightfield
  for ( i = 0;  i < HF_HEIGHT-1;  i++ )
  {
    for ( j = 0;  j < HF_WIDTH-1;  j++)
    {
      // generate the offset point in the centers from the corner locations
      float centerHeight = (heights[i][j] + heights[i][j+1] + heights[i+1][j] + heights[i+1][j+1] ) * 0.25f;

      const float darkBlue = -0.05f;
      const float darkGreen = 0.1f;
      const float lushGreen = 0.2f;
      const float stony = 0.4f;
      const float white = 0.7f;

      // dark blue to dark green to lush green to stony to white snowcaps
      if ( centerHeight < darkBlue )
      {
        // dark blue
//        glColor3f( 0.0f, 0.0f, 0.3f );
        glColor3f( 0.4f, 0.4f, 1.0f );
      }
      else
      if ( centerHeight < darkGreen )
      {
        // dark blue to dark green
//        interpolateColor( 0.0f, 0.0f, 0.3f,
//                          0.1f, 0.4f, 0.1f, ( centerHeight - darkBlue ) / (darkGreen - darkBlue) );
        interpolateColor( 0.4f, 0.4f, 1.0f,
                          0.4f, 0.8f, 0.8f, ( centerHeight - darkBlue ) / (darkGreen - darkBlue) );
      }
      else
      {
        // >= 0.2f
        if ( centerHeight < lushGreen )
        {
          // dark green to lush green
//          interpolateColor( 0.1f, 0.4f, 0.1f,
//                            0.2f, 0.5f, 0.1f, ( centerHeight - darkGreen ) / (lushGreen - darkGreen) );
          interpolateColor( 0.4f, 0.8f, 0.8f,
                            0.5f, 0.9f, 0.5f, ( centerHeight - darkGreen ) / (lushGreen - darkGreen) );
        }
        else
        {
          // >= 0.4f
          if ( centerHeight < stony )
          {
            // lush green to stony
//            interpolateColor( 0.2f, 0.5f, 0.1f,
//                              0.3f, 0.25f, 0.2f, ( centerHeight - lushGreen ) / (stony - lushGreen) );
            interpolateColor( 0.5f, 0.9f, 0.5f,
                              0.7f, 0.6f, 0.5f, ( centerHeight - lushGreen ) / (stony - lushGreen) );
          }
          else
          {
            // stony to white
//            interpolateColor( 0.3f, 0.25f, 0.2f,
//                              1.0f, 1.0f, 1.0f, ( centerHeight - stony ) / (white - stony) );
            interpolateColor( 0.7f, 0.6f, 0.5f,
                              1.0f, 1.0f, 1.0f, ( centerHeight - stony ) / (white - stony) );
          }
        }
      }

      // and draw the 4 triangles
      glVertex3f( -2.0f + i * 4.0f / HF_HEIGHT, heights[i][j], -2.0f + j * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + i * 4.0f / HF_HEIGHT, heights[i][j+1], -2.0f + (j+1) * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+0.5f) * 4.0f / HF_HEIGHT, centerHeight, -2.0f + (j+0.5f) * 4.0f / HF_HEIGHT );

      glVertex3f( -2.0f + i * 4.0f / HF_HEIGHT, heights[i][j+1], -2.0f + (j+1) * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+1) * 4.0f / HF_HEIGHT, heights[i+1][j+1], -2.0f + (j+1) * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+0.5f) * 4.0f / HF_HEIGHT, centerHeight, -2.0f + (j+0.5f) * 4.0f / HF_HEIGHT );

      glVertex3f( -2.0f + (i+1) * 4.0f / HF_HEIGHT, heights[i+1][j+1], -2.0f + (j+1) * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+1) * 4.0f / HF_HEIGHT, heights[i+1][j], -2.0f + j * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+0.5f) * 4.0f / HF_HEIGHT, centerHeight, -2.0f + (j+0.5f) * 4.0f / HF_HEIGHT );

      glVertex3f( -2.0f + (i+1) * 4.0f / HF_HEIGHT, heights[i+1][j], -2.0f + j * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + i * 4.0f / HF_HEIGHT, heights[i][j], -2.0f + j * 4.0f / HF_HEIGHT );
      glVertex3f( -2.0f + (i+0.5f) * 4.0f / HF_HEIGHT, centerHeight, -2.0f + (j+0.5f) * 4.0f / HF_HEIGHT );
    }
  }

  glEnd();
}


void UpdateAndDrawParticles()
{
  glBegin(GL_POINTS);
#if 0
  for ( i = 0;  i < NUM_CELLS_ACROSS_X;  i++ )
  {
    for ( j = 0;  j < NUM_CELLS_ACROSS_Y;  j++ )
    {
      for ( int k = 0;  k < NUM_CELLS_ACROSS_Z;  k++ )
      {
        glColor3f( i / 32.0f, j / 8.0f, k / 32.0f );
//        glColor3f( 0.0f, j / 8.0f, 0.0f );
        for ( unsigned int c = 0;  c < cells[i][j][k].particles.size();  c++ )
        {
          Particle& particle = cells[i][j][k].particles[c];
          glVertex3f( particle.pos.x, particle.pos.y, particle.pos.z );
        }
      }
    }
  }

  // update particles (calc forces, velocities and new positions),
  // and then move to the appropriate cells
  std::vector<int> reindex;
  for ( i = 0;  i < NUM_CELLS_ACROSS_X;  i++ )
  {
    for ( j = 0;  j < NUM_CELLS_ACROSS_Y;  j++ )
    {
      for ( int k = 0;  k < NUM_CELLS_ACROSS_Z;  k++ )
      {
        int curIndex = 0;
        for ( unsigned int c = 0;  c < cells[i][j][k].particles.size();  c++ )
        {
          Particle& particle = cells[i][j][k].particles[c];
/*
          // find the height under this particle
          int heightFieldI = ( particle.pos.x + 2.0f ) * 100.0f / ( 4.0f );
          int heightFieldJ = ( particle.pos.z + 2.0f ) * 100.0f / ( 4.0f );
          if ( heightFieldI < 0 ) heightFieldI = 0;
          if ( heightFieldJ < 0 ) heightFieldJ = 0;
          if ( heightFieldI >= 100 ) heightFieldI = 100-1;
          if ( heightFieldJ >= 100 ) heightFieldJ = 100-1;

          // update this particle
          particle.vel.x = -particle.pos.z;
          particle.vel.y += heights[heightFieldI][heightFieldJ]+0.1f > particle.pos.y
                         ? (heights[heightFieldI][heightFieldJ]+0.1f - particle.pos.y) * 0.5f
                         : -0.2f;
          if ( particle.vel.y < -1.2f ) particle.vel.y = -1.2f;
          particle.vel.z =  particle.pos.x;

          particle.pos.x += particle.vel.x * 0.001f;
          particle.pos.y += particle.vel.y * 0.001f;
          particle.pos.z += particle.vel.z * 0.001f;
*/
          // gravity
          particle.vel.y -= 0.098f;

          // go through all neighbouring cells
          for ( int ii = -1;  ii <= 1;  ii++ )
          {
            for ( int jj = -1;  jj <= 1;  jj++ )
            {
              for ( int kk = -1;  kk <= 1;  kk++ )
              {
                if ( ( i + ii >= 0 )  &&  ( i + ii < NUM_CELLS_ACROSS_X )
                  && ( j + jj >= 0 )  &&  ( j + jj < NUM_CELLS_ACROSS_Y )
                  && ( k + kk >= 0 )  &&  ( k + kk < NUM_CELLS_ACROSS_Z ) )
                {
                  for ( unsigned int cc = 0;  cc < cells[i+ii][j+jj][k+kk].particles.size();  cc++ )
                  {
                    if ( ( ii == 0 )  &&  ( jj == 0 )  &&  ( kk == 0 )  &&  ( cc == c ) )
                    {
                      continue; // ignore self
                    }

                    Particle& otherParticle = cells[i+ii][j+jj][k+kk].particles[cc];

                    float dsq = distanceSquared( otherParticle.pos, particle.pos );
                    if ( dsq < 1.0f / (8.0f * 8.0f) )
                    {
                      // pressure
                      if ( dsq > 0.000001f )
                      {
                        particle.vel += ( particle.pos - otherParticle.pos ) * ( 0.01f / expf(sqrtf( dsq )) );
//                        addVec( particle.vel, prodVec( diffVec( particle.pos, otherParticle.pos ), 0.01f / sqrtf( dsq ) ) );
//                        addVec( particle.vel, prodVec( diffVec( particle.pos, otherParticle.pos ), 0.01f / dsq ) );
                      }

                      // viscous drag
                    }
                  }
                }
              }
            }
          }

          particle.pos.x += particle.vel.x * 0.001f;
          particle.pos.y += particle.vel.y * 0.001f;
          particle.pos.z += particle.vel.z * 0.001f;

          // find the height under this particle
          int heightFieldI = ( particle.pos.x + 2.0f ) * HF_WIDTH / ( 4.0f );
          int heightFieldJ = ( particle.pos.z + 2.0f ) * HF_HEIGHT / ( 4.0f );
          if ( heightFieldI < 0 ) heightFieldI = 0;
          if ( heightFieldJ < 0 ) heightFieldJ = 0;
          if ( heightFieldI >= HF_WIDTH ) heightFieldI = HF_WIDTH-1;
          if ( heightFieldJ >= HF_HEIGHT ) heightFieldJ = HF_HEIGHT-1;

          // collision with the ground
          if ( particle.pos.y < heights[heightFieldI][heightFieldJ] + 0.1f )
          {
            particle.pos.y = heights[heightFieldI][heightFieldJ] + 0.1f;
            particle.vel.y = 0.2f * fabsf(particle.vel.y);
          }

          // collision with the walls
          if ( particle.pos.x < -2.0f + 0.01f )
          {
            particle.pos.x = -2.0f + 0.01f;
            particle.vel.x = -particle.vel.x;
          }

          if ( particle.pos.x > 2.0f - 0.01f )
          {
            particle.pos.x = 2.0f - 0.01f;
            particle.vel.x = -particle.vel.x;
          }

          if ( particle.pos.z < -2.0f + 0.01f )
          {
            particle.pos.z = -2.0f + 0.01f;
            particle.vel.z = -particle.vel.z;
          }

          if ( particle.pos.z > 2.0f - 0.01f )
          {
            particle.pos.z = 2.0f - 0.01f;
            particle.vel.z = -particle.vel.z;
          }

          if(1)
          {
            // move the particle to another cell if needed
            int cellI = ( particle.pos.x + 2.0f ) * 32.0f / ( 4.0f );
            int cellJ = ( particle.pos.y - 1.0f ) * 8.0f / ( 1.0f );
            int cellK = ( particle.pos.z + 2.0f ) * 32.0f / ( 4.0f );
            // limit the indices to the sensible range!
            if ( cellI < 0 ) cellI = 0;
            if ( cellJ < 0 ) cellJ = 0;
            if ( cellK < 0 ) cellK = 0;
            if ( cellI > 31 ) cellI = 31;
            if ( cellJ > 7 )  cellJ = 7;
            if ( cellK > 31 ) cellK = 31;
            if ( ( cellI != i ) || ( cellJ != j ) || ( cellK != k ) )
            {
              Particle tmp = particle;
//              cells[i][j][k].particles.erase(cells[i][j][k].particles.begin()+c);
              cells[cellI][cellJ][cellK].particles.push_back( tmp );
            }
            else
            {
              reindex.resize( curIndex + 1 );
              reindex[curIndex] = c;
              ++curIndex;
            }
          }
        }

        for ( int index = 0;  index < curIndex;  index++ )
        {
          cells[i][j][k].particles[index] = cells[i][j][k].particles[reindex[index]];
        }
        cells[i][j][k].particles.resize(curIndex);

      }
    }
  }
#else
  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
    glColor3f( particles[i].bucket.i / 16.0f, particles[i].bucket.j / 2.0f, particles[i].bucket.k / 16.0f );
    glVertex3f( particles[i].pos.x, particles[i].pos.y, particles[i].pos.z );

    // find the height under this particle
    int heightFieldI = ( particles[i].pos.x + 2.0f ) * HF_WIDTH / ( 4.0f );
    int heightFieldJ = ( particles[i].pos.z + 2.0f ) * HF_HEIGHT / ( 4.0f );
    if ( heightFieldI < 0 ) heightFieldI = 0;
    if ( heightFieldJ < 0 ) heightFieldJ = 0;
    if ( heightFieldI >= HF_WIDTH ) heightFieldI = HF_WIDTH-1;
    if ( heightFieldJ >= HF_HEIGHT ) heightFieldJ = HF_HEIGHT-1;
#if 1
    particles[i].vel.x = -particles[i].pos.z;
    particles[i].vel.y += heights[heightFieldI][heightFieldJ]+0.1f>particles[i].pos.y
                       ? (heights[heightFieldI][heightFieldJ]+0.1f-particles[i].pos.y) * 0.3f
                       : -0.03f;
    if ( particles[i].vel.y < -1.2f ) particles[i].vel.y = -1.2f;
    particles[i].vel.z =  particles[i].pos.x;

    particles[i].pos.x += particles[i].vel.x * 0.001f;
    particles[i].pos.y += particles[i].vel.y * 0.001f;
    particles[i].pos.z += particles[i].vel.z * 0.001f;
#else
    particles[i].vel.x = -particles[i].pos.z;
    particles[i].vel.y += heights[heightFieldI][heightFieldJ]+0.1f>particles[i].pos.y
                       ? (heights[heightFieldI][heightFieldJ]+0.1f-particles[i].pos.y) * 0.5f
                       : -0.2f;
    if ( particles[i].vel.y < -1.2f ) particles[i].vel.y = -1.2f;
    particles[i].vel.z =  particles[i].pos.x;

    particles[i].pos.x += particles[i].vel.x * 0.001f;
    particles[i].pos.y += particles[i].vel.y * 0.001f;
    particles[i].pos.z += particles[i].vel.z * 0.001f;
#endif
  }
#endif
  glEnd();
}


#define NUM_TRAJ 30
#define TRAJ_LENGTH 2500


const int trajIndices[NUM_TRAJ] = { 10, 20, 40, 50, 60, 70, 80, 100, 200, 300,
                                    210, 220, 240, 250, 260, 270, 280, 2100, 2200, 2300,
                                    310, 320, 340, 350, 360, 370, 380, 3100, 3200, 3300 };


struct Trajectories
{
  struct Vector3 history[NUM_TRAJ][TRAJ_LENGTH];

  Trajectories()
  {
    trajStartFrame = 0;
    lengthTrajRecorded = 0;
  }

  void Track()
  {
    int curTrajFrame = ( trajStartFrame + lengthTrajRecorded ) % TRAJ_LENGTH;
    for ( i = 0;  i < NUM_TRAJ;  i++ )
    {
      history[i][curTrajFrame] = particles[trajIndices[i]].pos;
    }

    if ( lengthTrajRecorded < TRAJ_LENGTH )
    {
      ++lengthTrajRecorded;
    }
    else
    {
      trajStartFrame = ( trajStartFrame + 1 ) % TRAJ_LENGTH;
    }
  }

  void Render()
  {
    glBegin(GL_LINES);

    // draw the point trajectories
    if(1)
    for ( i = 0;  i < NUM_TRAJ;  i++ )
    {
      if ( lengthTrajRecorded > 1 )
      {
        glColor3f( particles[trajIndices[i]].bucket.i / 16.0f, particles[trajIndices[i]].bucket.j / 2.0f, particles[trajIndices[i]].bucket.k / 16.0f );
        for ( j = 0;  j < lengthTrajRecorded-1;  j++ )
        {
          int trajIndex = (trajStartFrame+j)%TRAJ_LENGTH;
          glVertex3f( history[i][trajIndex].x, history[i][trajIndex].y, history[i][trajIndex].z );
          trajIndex = (trajStartFrame+j+1)%TRAJ_LENGTH;
          glVertex3f( history[i][trajIndex].x, history[i][trajIndex].y, history[i][trajIndex].z );
        }
      }
    }

    glEnd();
  }

  int trajStartFrame;
  int lengthTrajRecorded;
};


void DrawCells()
{
  glBegin(GL_LINES);

  // draw the grid cells
  glColor3f( 0.0f, 0.5f, 0.8f );
  for ( int i = 0;  i < NUM_CELLS_ACROSS_X;  i++ )
  {
    for ( int j = 0;  j < NUM_CELLS_ACROSS_Y;  j++ )
    {
      for ( int k = 0;  k < NUM_CELLS_ACROSS_Z;  k++ )
      {
        AABB aabb = cellAABB( i, j, k );
        drawAABB( aabb );
      }
    }
  }

  glEnd();
}


void DrawParticleVelocities()
{
  glBegin(GL_LINES);

  // draw the particle velocities
  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
    glColor3f( particles[i].bucket.i / 16.0f, particles[i].bucket.j / 2.0f, particles[i].bucket.k / 16.0f );
    glVertex3f( particles[i].pos.x, particles[i].pos.y, particles[i].pos.z );
    glVertex3f( particles[i].pos.x + particles[i].vel.x * 0.03f, particles[i].pos.y + particles[i].vel.y * 0.03f, particles[i].pos.z + particles[i].vel.z * 0.03f );
  }

  glEnd();
}


/* The main drawing function. */
void GLUTWindow::DrawGLScene() {
  static Timer timer;
  static Trajectories trajectories;

  // Clear The Screen And The Depth Buffer 
  glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

  glLoadIdentity();        // make sure we're no longer rotated.
  glTranslatef(0.0f,0.0f,-5.0f);    // Move Right 3 Units, and back into the screen 7

  glRotatef(m_rotate_x,1.0f,0.0f,0.0f);
  glRotatef(m_rotate_y,0.0f,1.0f,0.0f);

  glEnable( GL_LINE_SMOOTH );
  glEnable( GL_POLYGON_SMOOTH );
  glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
  glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

  if ( !generated )
  {
    GenerateHeightfield();
    GenerateParticles();

    generated = 1;
  }

  DrawHeightfield();

  UpdateAndDrawParticles();

  trajectories.Track();
  trajectories.Render();

//  DrawCells();

  DrawParticleVelocities();

  // swap the buffers to display, since double buffering is used.
  glutSwapBuffers();
}

/* The function called whenever a key is pressed. */
void GLUTWindow::keyPressed( unsigned char key, int x, int y ) {
  /* avoid thrashing this call */
  usleep(100);
  
  /* If escape is pressed, kill everything. */
  if( key == escape_key ) { 
    /* shut down our window */
    glutDestroyWindow(m_window); 
    glutLeaveMainLoop();
  }
}

/* For callbacks */
GLUTWindow *g_current_instance;
extern "C" void drawCallback() {
    g_current_instance->DrawGLScene();
}
extern "C" void resizeCallback( int w, int h ) {
    g_current_instance->ReSizeGLScene( w, h );
}
extern "C" void keyCallback( unsigned char key, int x, int y ) {
    g_current_instance->keyPressed( key, x, y );
}

GLUTWindow::GLUTWindow( int argc, char **argv,  
                        int w_width /* = 1024 */, int w_height /* = 768 */ )
  : m_w_width(w_width),
    m_w_height(w_height)
{
  g_current_instance = this;
  
  m_rotate_x = 30.0f;
  m_rotate_y = 0.0f;
  
  /* Initialize GLUT state */  
  glutInit(&argc, argv);  
    
  /* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffered for automatic clipping */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* get a window */
  glutInitWindowSize( w_width, w_height );  

  /* the window starts at the upper left corner of the screen */
  glutInitWindowPosition(0, 0);  

  /* Open a window */  
  m_window = glutCreateWindow("Simple particle model over heightfield computed in real time on INPE MEPhI cluster");  

  /* Register the function to do all our OpenGL drawing. */
  glutDisplayFunc(::drawCallback);

  /* Go fullscreen. This is as soon as possible. */
  //  glutFullScreen();

  /* Even if there are no events, redraw our gl scene. */
  glutIdleFunc(::drawCallback);

  /* Register the function called when our window is resized. */
  glutReshapeFunc(::resizeCallback);

  /* Register the function called when the keyboard is pressed. */
  glutKeyboardFunc(::keyCallback);

  /* Initialize our window. */
  InitGL( w_width, w_height );

  /* This will make sure that GLUT doesn't kill the program 
   * when the window is closed by the OS */
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,
      GLUT_ACTION_GLUTMAINLOOP_RETURNS);
}


void GLUTWindow::GLUTMainLoop()
{
  /* Start Event Processing Engine */  
  glutMainLoop();
}
