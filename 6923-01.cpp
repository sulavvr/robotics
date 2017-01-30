// *****************************************************************************
//
// Computer Science 6423: Robotics
//
// Program #1: Trajectory Constraints
//
// Author: Dr. Hammerand
//
// Date: January 28, 2017
//
// *****************************************************************************

/*
constraints
1a: p1, p2 for each segment: 2 constraints per segment; 3 segments = 6 constraints or 3 linear polynomials (2 c's each)
1b: 1a + segment-to-segment velocity equality; 6 + 2 v=v = 8 constraints; 2 quadratic (3 c's each) & 1 linear (2 c's)
1c: 1b + 2 terminal velocities = 0; 8 + 2 v=0 = 10 constraints; 2 quadratic (3 c's each) & 1 cubic (4 c's)
1d: 1b + segment-to-segment acceleration equality; 8 + 2 a=a = 10 constraints; 2 quadratic & 1 cubic (as in 1c)
1e: 1c & 1d: 12 constraints;  3 cubic (4 c's each)
1f: 1e + 2 terminal accelerations = 0; 12 + 2 = 14 constraints; 2 quartic (5 c's each) + 1 cubic (4 c's)
*/

#include <iostream>
   using std::cout;
   using std::endl;
#include <fstream>
   using std::ifstream;
   using std::ofstream;
#include <cmath>

const int W = 500;  // width of image
const int H = 500;  // height of image

const int X = 0;    // constants for use as subscripts
const int Y = 1;

char BLACK  [3] = {  0,   0,   0};  // colors for use in plotting pixels
char WHITE  [3] = {255, 255, 255};
char RED    [3] = {  0,   0, 255};
char GREEN  [3] = {  0, 255,   0};
char BLUE   [3] = {255,   0,   0};
char YELLOW [3] = {  0, 255, 255};
char ORANGE [3] = {  0, 102, 255};
char PURPLE [3] = {204,   0, 153};

const int MAX_DIMENSIONS = 6; // for all semester: X, Y or X, Y, Z or 6 angles
const int MAX_POLYNOMIAL_DEGREE = 5; // s(t) = c5*t^5 + c4*t^4 + c3*t^3 + c2*t^2 + c1*t + c0: 6 coefficients

void clear
        (char image[H][W][3], char color[3]);
void setPixel
        (int x, int y, char image[H][W][3], char color[3], int borderWidth = 0);
void setPixel
        (int p[2], char image[H][W][3], char color[3], int borderWidth = 0);
void calculate2dSegmentTimes
        (int p[][MAX_DIMENSIONS], int numberOfSegments, double segmentTimes[]);
void calculate2dCoefficientsPositionsOnly
        (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
         double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS]);
void calculate2dCoefficientsPositionsAndInteriorVelocities
        (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
         double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS]);
void calculate2dCoefficientsPositionsAndInteriorVelocitiesWithTerminal
        (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
         double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS]);
void clearCoefficients
        (int numberOfSegments, double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS]);
void calculateAndPlot2dTrajectory
        (int numberOfSegments, double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS],
         double segmentTimes[], char image[H][W][3], char color[3], int borderWidth = 0);
void evaluate2dParametric
        (double c[MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS], double t, int p[MAX_DIMENSIONS]);
void writeImageToFile
        (const char outFileName[], const char headerFileName[], char image[H][W][3]);

int main ()
{
   char image[H][W][3];
   clear (image, YELLOW);

   for (int x = 0; x <= W-1; x++)  // diagonal line for reference
      setPixel (x, x, image, GREEN); // small pixel

   const int numberOfPoints = 4;  // p0, p1, p2, p3

   int p[numberOfPoints][MAX_DIMENSIONS] = {{50, 50}, {100, 400}, {300, 350}, {450, 450}};

   for (int i = 0; i <= numberOfPoints-1; i++)
      setPixel (p[i], image, BLACK, 6); // very large pixel

   const int numberOfSegments = numberOfPoints - 1;

   double segmentTimes[numberOfSegments];
   calculate2dSegmentTimes (p, numberOfSegments, segmentTimes);

   double coefficients [numberOfSegments][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS];

   // calculate2dCoefficientsPositionsOnly (numberOfSegments, p, segmentTimes, coefficients);
   // calculateAndPlot2dTrajectory (numberOfSegments, coefficients, segmentTimes, image, PURPLE, 4); // large pixel

   // calculate2dCoefficientsPositionsAndInteriorVelocities (numberOfSegments, p, segmentTimes, coefficients);
   // calculateAndPlot2dTrajectory (numberOfSegments, coefficients, segmentTimes, image, ORANGE, 2); // medium pixel

   calculate2dCoefficientsPositionsAndInteriorVelocitiesWithTerminal (numberOfSegments, p, segmentTimes, coefficients);
   calculateAndPlot2dTrajectory (numberOfSegments, coefficients, segmentTimes, image, ORANGE, 2); // medium pixel

   writeImageToFile ("4413-01.bmp", "500x500.bmp", image); // 2nd bitmap MUST be of size W x H

   return 0;
}  // end function main

void clear (char image[H][W][3], char color[3])
{
   for (int x = 0; x <= W-1; x++)

      for (int y = 0; y <= H-1; y++)

         for (int c = 0; c <= 2; c++)

            image[y][x][c] = color[c];

   return;
}  // end function clear

void setPixel (int x, int y, char image[H][W][3], char color[3], int borderWidth /* = 0*/)
{
    int p[2] = {x, y};

    setPixel (p, image, color, borderWidth);

    return;
}  // end function setPixel

void setPixel (int p[2], char image[H][W][3], char color[3], int borderWidth /* = 0*/)
{
   for (int i = p[0]-borderWidth; i <= p[0]+borderWidth; i++)

      for (int j = p[1]-borderWidth; j <= p[1]+borderWidth; j++)

         if (i >= 0  &&  i <= W-1  &&  j >= 0  && j <= H-1) // guarantee inside array bounds

            for (int k = 0; k <= 2; k++)

               image[j][i][k] = color[k];

   return;
}  // end function setPixel

void calculate2dSegmentTimes (int p[][MAX_DIMENSIONS], int numberOfSegments, double segmentTimes[])
{
    for (int i = 0; i <= numberOfSegments-1; i++)

       segmentTimes[i] = sqrt ( pow(p[i+1][X] - p[i][X], 2) + pow(p[i+1][Y] - p[i][Y], 2) ); // 2D distance formula

    return;
}  // end function calculate2dSegmentTimes

void calculate2dCoefficientsPositionsOnly (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
                                           double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS])
{
   clearCoefficients (numberOfSegments, coefficients);

   for (int i = 0; i <= numberOfSegments-1; i++)
   {

      for (int d = X; d <= Y; d++) // dimension = 0, 1
      {
         coefficients[i][1][d] = (p[i+1][d] - p[i][d]) / segmentTimes[i]; // see pdf for derivation

         coefficients[i][0][d] =  p[i][d];
      }  // end for d

   }  // end for i

   return;
}  // end function calculate2dCoefficientsPositionsOnly

void calculate2dCoefficientsPositionsAndInteriorVelocities (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
                                                            double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS])
{
   clearCoefficients (numberOfSegments, coefficients);

   for (int d = X; d <= Y; d++) // dimension = 0, 1
   {                                                   // see pdf for derivations
      coefficients[0][2][d] =   ((p[2][d] - p[1][d])*segmentTimes[0] - (p[1][d] - p[0][d])*segmentTimes[1] )
                              / (segmentTimes[1]*pow(segmentTimes[0],2));

      coefficients[0][1][d] =   (2*(p[1][d] - p[0][d])*segmentTimes[1] - (p[2][d] - p[1][d])*segmentTimes[0])
                              / (segmentTimes[0]*segmentTimes[1]);

      coefficients[0][0][d] = p[0][d];



      coefficients[1][1][d] = (p[2][d] - p[1][d]) / segmentTimes[1];

      coefficients[1][0][d] = p[1][d];



      coefficients[2][2][d] =   ((p[3][d] -p[2][d])*segmentTimes[1] - (p[2][d] - p[1][d])*segmentTimes[2])
                              / (segmentTimes[1]*pow(segmentTimes[2],2));

      coefficients[2][1][d] = (p[2][d] - p[1][d]) / segmentTimes[1];

      coefficients[2][0][d] = p[2][d];
   }  // end for d

   return;
}  // end function calculate2dCoefficientsPositionsAndInteriorVelocitiesWithTerminal

void calculate2dCoefficientsPositionsAndInteriorVelocitiesWithTerminal (int numberOfSegments, int p[][MAX_DIMENSIONS], double segmentTimes[],
                                                            double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS])
{
   clearCoefficients (numberOfSegments, coefficients);

   for (int d = X; d <= Y; d++) // dimension = 0, 1
   {                                                   // see pdf for derivations

      coefficients[0][0][d] = p[0][d];

      coefficients[0][1][d] = 0;

      coefficients[0][2][d] = (
                            ((2 * pow(segmentTimes[1], 2) - 4 * segmentTimes[1]) * (p[3][d] - p[2][d]) - 2 * segmentTimes[2] * (p[1][d] - p[2][d]))
                            / (2 * segmentTimes[0] * pow(segmentTimes[1], 2) * segmentTimes[2])
                            ) - (
                            (
                                (3 * pow(segmentTimes[0], 3) * (2 * pow(segmentTimes[1], 2) - 4 * segmentTimes[1]) * (p[3][d] - p[2][d])) -
                                (6 * pow(segmentTimes[0], 3) * segmentTimes[2] * (p[1][d] - p[2][d])) +
                                (6 * pow(segmentTimes[0], 2) * pow(segmentTimes[1], 2) * segmentTimes[2] * (p[0][d] - p[1][d]))
                            ) / (2 * pow(segmentTimes[0], 4) * pow(segmentTimes[1], 2) * segmentTimes[2])
                            );



      // coefficients[1][1][d] = (p[2][d] - p[1][d]) / segmentTimes[1];

      // coefficients[1][0][d] = p[1][d];



      // coefficients[2][2][d] =   ((p[3][d] -p[2][d])*segmentTimes[1] - (p[2][d] - p[1][d])*segmentTimes[2])
      //                         / (segmentTimes[1]*pow(segmentTimes[2],2));

      // coefficients[2][1][d] = (p[2][d] - p[1][d]) / segmentTimes[1];

      // coefficients[2][0][d] = p[2][d];
   }  // end for d

   return;
}  // end function calculate2dCoefficientsPositionsAndInteriorVelocitiesWithTerminal

void clearCoefficients (int numberOfSegments, double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS])
{
   for (int segmentIndex = 0; segmentIndex <= numberOfSegments-1; segmentIndex++)

      for (int degreeIndex = 0; degreeIndex <= MAX_POLYNOMIAL_DEGREE+1-1; degreeIndex++)

         for (int dimensionIndex = 0; dimensionIndex <= MAX_DIMENSIONS-1; dimensionIndex++)

            coefficients[segmentIndex][degreeIndex][dimensionIndex] = 0;

   return;
}  // end function clearCoefficients

void calculateAndPlot2dTrajectory (int numberOfSegments, double coefficients[][MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS],
                                   double segmentTimes[], char image[H][W][3], char color[3], int borderWidth /* = 0 */)
{
   const int pixelsPerSegment = 10;  // "density" of segment representation

   for (int i = 0; i <= numberOfSegments-1; i++)
   {
      double deltaT = segmentTimes[i] / pixelsPerSegment;

      for (double t = 0; t <= segmentTimes[i]; t += deltaT)
      {
         int s[2];

         evaluate2dParametric (coefficients[i], t, s); // 2 coefficients: linear s(t) = c[1]*t + c[0]

         setPixel (s, image, color, borderWidth);
      }  // end for t

   }  // end for i
   return;
}  // end function calculateAndPlot2dTrajectory

void evaluate2dParametric (double c[MAX_POLYNOMIAL_DEGREE+1][MAX_DIMENSIONS], double t, int p[MAX_DIMENSIONS])
{
   double sums [MAX_DIMENSIONS];

   for (int d = 0; d <= MAX_DIMENSIONS-1; d++)  // all dimensions

      sums[d] = 0;

   for (int i = MAX_POLYNOMIAL_DEGREE; i >= 0; i--) // sum = ... + c4*t^4 + c3*t^3 + c2*t^2 + c1*t (t^1) + c0 (t^0)

      for (int d = 0; d <= MAX_DIMENSIONS-1; d++) // all dimensions

         sums[d] += c[i][d] * pow(t, i);

   p[X] = round(sums[X]);  // 2 dimensions
   p[Y] = round(sums[Y]);

   return;
}  // end function evaluate2dParametric

void writeImageToFile (const char outFileName[], const char headerFileName[], char image[H][W][3])
{
   ifstream example (headerFileName);  // 54 byte template

   char header [54];
   example.read (header, 54);

   example.close ();

   ofstream outfile (outFileName);

   outfile.write (header, 54);

   for (int x = 0; x <= W-1; x++)  // write image to disk

      for (int y = 0; y <= H-1; y++)

         for (int c = 0; c <= 2; c++)

            outfile.write (&image[x][y][c], 1);

   outfile.close ();

   return;
}  // end function writeImageToFile
