#include <swri_nav_util/bezier_spline.h>

#define pt2char(x) (x.toString().c_str())

namespace swri_nav_util
{

//%Reference: https://www.particleincell.com/2012/bezier-splines
bool cubicBezierSpline(
    const std::vector<Point2d>& knot_pts,
    std::vector<std::vector<Point2d> >& ctrl_pts)
{
  //row of a Tridiagonal matrix
  //e.g. a 6x6 Tridiagonal matrix is of the form:
  //[b c 0 0 0 0]
  //[a b c 0 0 0]
  //[0 a b c 0 0]
  //[0 0 a b c 0]
  //[0 0 0 a b c]
  //[0 0 0 0 a b]

  struct TridiagonalRow
  {
    TridiagonalRow() : a(0.0), b(0.0), c(0.0) {}
    ~TridiagonalRow() {}
    double a;
    double b;
    double c;
  };

  if (knot_pts.size() < 2)
  {
    ROS_WARN("cubicBezierSpline() requires at least 2 knot points (received %zu).", knot_pts.size());
    return false;
  }
  const int nk = knot_pts.size(); //number of knot points
  const int nb = nk-1; //number of Bezier curves

  //set up curvature constraint matrix equation:
  //A*P1 = D,
  //where P1 is the 2nd control point of every Bezier curve
  //A is Tridiagonal
  std::vector<TridiagonalRow> A(nb);
  std::vector<Point2d> D(nb);

  //constrain C2 continuity at knot points
  for (int i = 1; i < nb-1; i++)
  {
    A[i].a = 1.0;
    A[i].b = 4.0;
    A[i].c = 1.0;
    D[i] = 4.0*knot_pts[i] + 2.0*knot_pts[i+1];
  }

  //constrain curvature = 0 at end points
  A[0].a = std::numeric_limits<double>::quiet_NaN(); //doesn't exist
  A[0].b = 2.0;
  A[0].c = 1.0;
  D[0] = knot_pts[0] + 2.0*knot_pts[1];

  A[nb-1].a = 2.0;
  A[nb-1].b = 7.0;
  A[nb-1].c = std::numeric_limits<double>::quiet_NaN(); //doesn't exist
  D[nb-1] = 8.0*knot_pts[nk-2] + knot_pts[nk-1];

  //DEBUGGING
//  for (int i = 0; i < nk; i++) { ROS_INFO("knot_pts[%d] = %s", i, pt2char(knot_pts[i])); }
//  for (int i = 0; i < nb; i++) { ROS_INFO("A[%d] = %f %f %f", i, A[i].a, A[i].b, A[i].c); }
//  for (int i = 0; i < nb; i++) { ROS_INFO("D[%d] = %s", i, pt2char(D[i])); }

  //solve Tridiagonal matrix using Thomas algorithm
  //https://en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
  std::vector<Point2d> P1(nb);
  {
    //forward pass, compute A' and D'
    std::vector<TridiagonalRow> Ap(A); //A'
    std::vector<Point2d> Dp(D); //D'

    Ap[0].c = A[0].c/A[0].b;
    Dp[0] = D[0]/A[0].b;
    for (int i = 1; i < nb; i++)
    {
      if (i < nb-1)
      {
        Ap[i].c = A[i].c/(A[i].b - A[i].a*Ap[i-1].c);
      }
      Dp[i] = (D[i] - A[i].a*Dp[i-1])/(A[i].b - A[i].a*Ap[i-1].c);
    }

//    for (int i = 0; i < nb; i++) { ROS_INFO("A'[%d] = %f %f %f", i, Ap[i].a, Ap[i].b, Ap[i].c); }
//    for (int i = 0; i < nb; i++) { ROS_INFO("D'[%d] = %s", i, pt2char(Dp[i])); }

    //backward pass
    P1[nb-1] = Dp[nb-1]; //the 2nd control point for each Bezier curve
    for (int i = nb-2; i >= 0; i--)
    {
      P1[i] = Dp[i] - (Ap[i].c*P1[i+1]);
    }

//    for (int i = 0; i < nb; i++) { ROS_INFO("P1[%d] = %s", i, pt2char(P1[i])); }
  }

  //compute all control points for each curve
  ctrl_pts.resize(nb);

  for (int i = 0; i < nb; i++)
  {
    ctrl_pts[i].resize(4); //4 control points for 3rd order Bezier curve
    ctrl_pts[i][0] = knot_pts[i];
    ctrl_pts[i][1] = P1[i];
    if (i < nb-1)
    {
      ctrl_pts[i][2] = 2*knot_pts[i+1] - P1[i+1];
    }
    else
    {
      ctrl_pts[i][2] = 0.5*(knot_pts[i+1] + P1[i]);
    }
    ctrl_pts[i][3] = knot_pts[i+1];
//    ROS_INFO("ctrl_pts[%d] = \n%s\n%s\n%s\n%s",
//        i,
//        pt2char(ctrl_pts[i][0]),
//        pt2char(ctrl_pts[i][1]),
//        pt2char(ctrl_pts[i][2]),
//        pt2char(ctrl_pts[i][3]));
  }

  return true;
}

} //namespace
