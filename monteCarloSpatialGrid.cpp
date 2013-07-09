//
// This code was created by Jeff Molofee '99 (ported to Linux/GLUT by Richard Campbell '99)
//
// If you've found this code useful, please let me know.
//
// Visit me at www.demonews.com/hosted/nehe 
// (email Richard Campbell at ulmont@bellsouth.net)
//
#include <GL/freeglut.h>    // Header File For The GLUT Library 
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <unistd.h>     // needed to sleep
#include <math.h>

#include <vector>

/* ASCII code for the escape key. */
#define ESCAPE 27

/* The number of our GLUT window */
int window; 

/* rotation angles. */
float aroundX = 30.0f;
float aroundY = 0.0f;

/* A general OpenGL initialization function.  Sets all of the initial parameters. */
void InitGL(int Width, int Height)	        // We call this right after our OpenGL window is created.
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);		// This Will Clear The Background Color To Black
  glClearDepth(1.0);				// Enables Clearing Of The Depth Buffer
  glDepthFunc(GL_LESS);			        // The Type Of Depth Test To Do
  glEnable(GL_DEPTH_TEST);		        // Enables Depth Testing
  glShadeModel(GL_SMOOTH);			// Enables Smooth Color Shading

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();				// Reset The Projection Matrix

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

  glMatrixMode(GL_MODELVIEW);
}

/* The function called when our window is resized (which shouldn't happen, because we're fullscreen) */
void ReSizeGLScene(int Width, int Height)
{
  if (Height==0)				// Prevent A Divide By Zero If The Window Is Too Small
    Height=1;

  glViewport(0, 0, Width, Height);		// Reset The Current Viewport And Perspective Transformation

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluPerspective(45.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);
  glMatrixMode(GL_MODELVIEW);
}


float randInRange(float min, float max)
{
  return min + (max - min) * rand() / (float)RAND_MAX;
}

float sqr( float val )
{
  return val * val;
}

struct Index3
{
  int i, j, k;
};

struct Vector3
{
  Vector3() {} // empty default ctor for perf reasons
  Vector3( float newX, float newY, float newZ ) : x(newX), y(newY), z(newZ) {}
  float x, y, z;
};

struct Particle
{
  struct Vector3 pos;
  struct Vector3 vel;
  struct Vector3 color;
  struct Index3 bucket;
};


#define NUM_PARTICLES 10000

struct Particle particles[NUM_PARTICLES];

// spatial grid
class Cell
{
public:
  std::vector<Particle> particles;
};


#define NUM_CELLS_ACROSS_X 32
#define NUM_CELLS_ACROSS_Y 8
#define NUM_CELLS_ACROSS_Z 32

Cell cells[NUM_CELLS_ACROSS_X][NUM_CELLS_ACROSS_Y][NUM_CELLS_ACROSS_Z];

struct AABB
{
  AABB( const Vector3& newMin, const Vector3& newMax ) : min(newMin), max(newMax) {}
  Vector3 min;
  Vector3 max;
};

AABB cellAABB( int i, int j, int k )
{
  const float offsetX = -2.0f;
  const float offsetZ = -2.0f;
  const float cellSize = 8.0f / 64.0f;
  AABB res( Vector3( offsetX + i * cellSize, j * cellSize, offsetZ + k * cellSize ),
            Vector3( offsetX + (i+1) * cellSize, (j+1) * cellSize, offsetZ + (k+1) * cellSize ) );
  return res;
}

float distanceSquared( const Vector3& a, const Vector3& b )
{
  return (a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z);
}

void drawAABB( const AABB& aabb, unsigned int color = 0 )
{
  glBegin(GL_LINES);
  // draw the 12 lines that make up this box:
  // bottom 4 lines
  glVertex3f( aabb.min.x, aabb.min.y, aabb.min.z );
  glVertex3f( aabb.max.x, aabb.min.y, aabb.min.z );

  glVertex3f( aabb.max.x, aabb.min.y, aabb.min.z );
  glVertex3f( aabb.max.x, aabb.min.y, aabb.max.z );

  glVertex3f( aabb.max.x, aabb.min.y, aabb.max.z );
  glVertex3f( aabb.min.x, aabb.min.y, aabb.max.z );

  glVertex3f( aabb.min.x, aabb.min.y, aabb.max.z );
  glVertex3f( aabb.min.x, aabb.min.y, aabb.min.z );
  // top 4 lines
  // connecting 4 lines
  glEnd();
}

void interpolateColor( float r0, float g0, float b0,
                       float r1, float g1, float b1, float t )
{
  glColor3f( r0 * (1.0f-t) + r1 * t, g0 * (1.0f-t) + g1 * t, b0 * (1.0f-t) + b1 * t );
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


/* The main drawing function. */
void DrawGLScene()
{
  glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );
  glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);	// Clear The Screen And The Depth Buffer

  glLoadIdentity();				// make sure we're no longer rotated.
  glTranslatef(0.0f,0.0f,-5.0f);		// Move Right 3 Units, and back into the screen 7
	
  glRotatef(aroundX,1.0f,0.0f,0.0f);
  glRotatef(aroundY,0.0f,1.0f,0.0f);

  glEnable( GL_LINE_SMOOTH );
  glEnable( GL_POLYGON_SMOOTH );
  glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
  glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

  // the heightfield memory
  static const int HF_WIDTH = 50;//100;
  static const int HF_HEIGHT = 50;//100;
  static float heights[HF_WIDTH][HF_HEIGHT];
  static int generated = 0, i, j;

  if ( !generated )
  {
    // generate a random heightfield within the cube 1.0f across
    for ( i = 0;  i < HF_HEIGHT;  i++ )
    {
      for ( j = 0;  j < HF_WIDTH;  j++)
      {
        float distanceFromCenter = sqrtf( sqr(0.0f+(float)(i-HF_WIDTH/3)*10.0f/HF_WIDTH) + sqr(0.0f + (float)(j-HF_HEIGHT/2)*10.0f/HF_HEIGHT) );
        heights[i][j] = 0.8f * cosf( distanceFromCenter ) * cosf( distanceFromCenter ) / (1.0f + 0.1f * distanceFromCenter * distanceFromCenter );
      }
    }

    // generate the particle positions above the heightfield
    for ( i = 0;  i < NUM_PARTICLES;  i++ )
    {
      particles[i].pos.x = randInRange( -2.0f, 2.0f );
      particles[i].pos.y = randInRange( 1.0f, 2.0f );
      particles[i].pos.z = randInRange( -2.0f, 2.0f );

      particles[i].vel.x = particles[i].vel.y = particles[i].vel.z = 0.0f;
    }

    generated = 1;
  }

  glBegin(GL_TRIANGLES);        // start drawing the cube.
  
  // generate a random heightfield within the cube 1.0f across
  for ( i = 0;  i < HF_HEIGHT-1;  i++ )
  {
    for ( j = 0;  j < HF_WIDTH-1;  j++)
    {
      // generate the offset point in the centers from the corner locations
      float centerHeight = (heights[i][j] + heights[i][j+1] + heights[i+1][j] + heights[i+1][j+1] ) * 0.25f;

#if 1
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
#else
      float red = centerHeight + 0.2f;
      glColor3f(red,1.0f-red,0.0f);
#endif
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

  glBegin(GL_POINTS);
//  glColor3f(0.0f,0.5f,1.0f);
  static int generatedParticleColors = 0;
  if (!generatedParticleColors)
  {
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
    generatedParticleColors = 1;
    
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
                        addVec( particle.vel, prodVec( diffVec( particle.pos, otherParticle.pos ), 0.01f / expf(sqrtf( dsq )) ) );
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

#define NUM_TRAJ 30
#define TRAJ_LENGTH 2500

  static struct Vector3 history[NUM_TRAJ][TRAJ_LENGTH];
  static int trajStartFrame = 0;
  static int lengthTrajRecorded = 0;

  int trajIndices[NUM_TRAJ] = { 10, 20, 40, 50, 60, 70, 80, 100, 200, 300,
                                210, 220, 240, 250, 260, 270, 280, 2100, 2200, 2300,
                                310, 320, 340, 350, 360, 370, 380, 3100, 3200, 3300 };

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

  glBegin(GL_LINES);

  // draw the grid cells
  glColor3f( 0.0f, 0.5f, 0.8f );
  if(0)
  for ( i = 0;  i < NUM_CELLS_ACROSS_X;  i++ )
  {
    for ( j = 0;  j < NUM_CELLS_ACROSS_Y;  j++ )
    {
      for ( int k = 0;  k < NUM_CELLS_ACROSS_Z;  k++ )
      {
        drawAABB( cellAABB( i, j, k ) );
      }
    }
  }
  
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

  // draw the particle velocities
  if(0)
  for ( i = 0;  i < NUM_PARTICLES;  i++ )
  {
//    glColor3f( 0.0f, 0.3f, 0.7f );
    glColor3f( particles[i].bucket.i / 16.0f, particles[i].bucket.j / 2.0f, particles[i].bucket.k / 16.0f );
    glVertex3f( particles[i].pos.x, particles[i].pos.y, particles[i].pos.z );
    glVertex3f( particles[i].pos.x + particles[i].vel.x * 0.03f, particles[i].pos.y + particles[i].vel.y * 0.03f, particles[i].pos.z + particles[i].vel.z * 0.03f );
/*
    // find the height under this particle
    int heightFieldI = ( particles[i].pos.x + 2.0f ) * 100.0f / ( 4.0f );
    int heightFieldJ = ( particles[i].pos.z + 2.0f ) * 100.0f / ( 4.0f );
    if ( heightFieldI < 0 ) heightFieldI = 0;
    if ( heightFieldJ < 0 ) heightFieldJ = 0;
    if ( heightFieldI >= 100 ) heightFieldI = 100-1;
    if ( heightFieldJ >= 100 ) heightFieldJ = 100-1;

    glVertex3f( particles[i].pos.x, particles[i].pos.y, particles[i].pos.z );
    glVertex3f( -2.0f + heightFieldI * 4.0f / HF_HEIGHT, heights[heightFieldI][heightFieldJ], -2.0f + heightFieldJ * 4.0f / HF_HEIGHT );
*/
  }

  glEnd();

  // swap the buffers to display, since double buffering is used.
  glutSwapBuffers();
}

/* The function called whenever a key is pressed. */
void keyPressed(unsigned char key, int x, int y) 
{
    /* avoid thrashing this call */
    usleep(100);

    /* If escape is pressed, kill everything. */
    if (key == ESCAPE) 
    { 
      /* shut down our window */
      glutDestroyWindow(window); 
      
      /* exit the program...normal termination. */
      exit(0);                   
    }
}

int main(int argc, char **argv) 
{  
  /* Initialize GLUT state - glut will take any command line arguments that pertain to it or 
     X Windows - look at its documentation at http://reality.sgi.com/mjk/spec3/spec3.html */  
  glutInit(&argc, argv);  

  /* Select type of Display mode:   
     Double buffer 
     RGBA color
     Alpha components supported 
     Depth buffered for automatic clipping */  
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);  

  /* get a 640 x 480 window */
  glutInitWindowSize(1024, 768);  

  /* the window starts at the upper left corner of the screen */
  glutInitWindowPosition(0, 0);  

  /* Open a window */  
  window = glutCreateWindow("Simple particle model over heightfield computed in real time on INPE MEPhI cluster");  

  /* Register the function to do all our OpenGL drawing. */
  glutDisplayFunc(&DrawGLScene);  

  /* Go fullscreen.  This is as soon as possible. */
//  glutFullScreen();

  /* Even if there are no events, redraw our gl scene. */
  glutIdleFunc(&DrawGLScene);

  /* Register the function called when our window is resized. */
  glutReshapeFunc(&ReSizeGLScene);

  /* Register the function called when the keyboard is pressed. */
  glutKeyboardFunc(&keyPressed);

  /* Initialize our window. */
  InitGL(640, 480);
  
  /* Start Event Processing Engine */  
  glutMainLoop();  

  return 1;
}




