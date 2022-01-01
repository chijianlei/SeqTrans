#include <swri_nav_util/bezier_util.h>

#include <swri_nav_util/array_util.h> //for linspace

#define pt2char(x) ((x).toString().c_str())

namespace swri_nav_util
{

const int buffer_size = MAX_BEZIER_CURVE_ORDER+1;

bool calcWeights(
    const int order,
    const double t,
    double weights[]) //must be correct size
{

  //get binomial
  double bn[buffer_size];
  if (order == 0)
  {
    bn[0] = 1.0;
  }
  else if (order == 1)
  {
    bn[0] = 1.0;
    bn[1] = 1.0;
  }
  else if (order == 2)
  {
    bn[0] = 1.0;
    bn[1] = 2.0;
    bn[2] = 1.0;
  }
  else if (order == 3)
  {
    bn[0] = 1.0;
    bn[1] = 3.0;
    bn[2] = 3.0;
    bn[3] = 1.0;
  }
  else if (order == 4)
  {
    bn[0] = 1.0;
    bn[1] = 4.0;
    bn[2] = 6.0;
    bn[3] = 4.0;
    bn[4] = 1.0;
  }
  else if (order == 5)
  {
    bn[0] = 1.0;
    bn[1] = 5.0;
    bn[2] = 10.0;
    bn[3] = 10.0;
    bn[4] = 5.0;
    bn[5] = 1.0;
  }
  else if (order == 6)
  {
    bn[0] = 1.0;
    bn[1] = 6.0;
    bn[2] = 15.0;
    bn[3] = 20.0;
    bn[4] = 15.0;
    bn[5] = 6.0;
    bn[6] = 1.0;
  }
  else if (order == 7)
  {
    bn[0] = 1.0;
    bn[1] = 7.0;
    bn[2] = 21.0;
    bn[3] = 35.0;
    bn[4] = 35.0;
    bn[5] = 21.0;
    bn[6] = 7.0;
    bn[7] = 1.0;
  }
  else
  {
    ROS_ERROR("Bezier curve order %d is invalid.", order);
    return false;
  }

  //precompute powers of t and 1-t
  double tp[buffer_size]; //powers of t
  double one_minus_tp[buffer_size]; //powers of 1-t
  tp[0] = 1.0;
  one_minus_tp[0] = 1.0;
  for (int i = 1; i <= order; i++)
  {
    tp[i] = tp[i-1]*t;
    one_minus_tp[i] = one_minus_tp[i-1]*(1-t);
  }

  //compute weights
  for (int i = 0; i <= order; i++)
  {
    weights[i] = bn[i] * one_minus_tp[order-i] * tp[i];
  }

  return true;
}

template <typename PointType>
inline void multiplyWeightsControlPoints(
    const double weights[],
    const std::vector<Point2d>& ctrl_pts,
    PointType& pt)
{
  pt.x = 0.0;
  pt.y = 0.0;
  for (size_t i = 0; i < ctrl_pts.size(); i++)
  {
    pt.x += weights[i] * ctrl_pts[i].x;
    pt.y += weights[i] * ctrl_pts[i].y;
  }
}


template <typename PointType> //must have member vars: x, y
void calcPointOnBezierCurve(
    const double t,
    const std::vector<Point2d>& ctrl_pts,
    PointType& pt) //point on curve
{
  int order = int(ctrl_pts.size())-1;

  double weights[buffer_size];
  if (!calcWeights(order, t, weights)) { return; }
  multiplyWeightsControlPoints(weights, ctrl_pts, pt);
}

void calcPointsOnBezierCurve(
    const int num_t,
    const std::vector<Point2d>& ctrl_pts,
    std::vector<Point2d>& pts)
{
  int order = int(ctrl_pts.size())-1;

  std::vector<double> t;
  linspace(0.0, 1.0, num_t, t);

  pts.resize(t.size());
  for (size_t i = 0; i < t.size(); i++)
  {
    calcPointOnBezierCurve(t[i], ctrl_pts, pts[i]);
  }
}

std::vector<Point2d> diffPoint2dVec(
    const double coeff,
    const std::vector<Point2d>& pts)
{
  std::vector<Point2d> diff_pts;
  if (pts.empty()) { return diff_pts; }
  diff_pts.resize(pts.size()-1);
  for (size_t i = 1; i < pts.size(); i++)
  {
    diff_pts[i-1] = coeff*(pts[i] - pts[i-1]);
  }
  return diff_pts;
}

void calcPathPointOnBezierCurve(
    const double t,
    const std::vector<Point2d>& ctrl_pts,
    const std::vector<Point2d>& dctrl_pts, //precompute
    const std::vector<Point2d>& ddctrl_pts, //precompute
    swri_nav_msgs::PathPoint& pt,
    bool calc_yaw = true, //if true set yaw value
    bool calc_curvature = true) //if true set curvature value (must also set yaw)
{
  calcPointOnBezierCurve(t, ctrl_pts, pt);
  if (calc_yaw)
  {
    const int order = int(ctrl_pts.size())-1;

    //calculate 1st derivative
    swri_nav_msgs::PathPoint dxy; //derivative of x,y wrt t
    double Wp[buffer_size];
    if (!calcWeights(order-1, t, Wp)) { return; }
    multiplyWeightsControlPoints(Wp, dctrl_pts, dxy);

    //normalize
    swri_nav_msgs::PathPoint dxyn = dxy;
    double dx = dxy.x; //abbreviate
    double dy = dxy.y; //abbreviate
    double ds = sqrt(dx*dx + dy*dy);
    dxyn.x = dx/ds;
    dxyn.y = dy/ds;

    //calculate yaw
    pt.yaw = atan2(dy, dx);

    if (calc_curvature)
    {
      //calculate 2nd derivative
      swri_nav_msgs::PathPoint ddxy;
      double Wpp[buffer_size];
      if (!calcWeights(order-2, t, Wpp)) { return; }
      multiplyWeightsControlPoints(Wpp, ddctrl_pts, ddxy);

      //calculate curvature
      double ds2 = ds*ds;
      double ds3 = ds2*ds;
      double ddx = ddxy.x; //abbreviate
      double ddy = ddxy.y; //abbreviate

      pt.curvature = (dx*ddy - dy*ddx)/ds3;
    }
  }
}


void calcPathOnBezierCurve(
    const int num_t,
    const std::vector<Point2d>& ctrl_pts,
    swri_nav_msgs::PathSegment& path,
    bool calc_yaw,
    bool calc_curvature)
{

  int order = int(ctrl_pts.size())-1;

  std::vector<double> t;
  linspace(0.0, 1.0, num_t, t);

  //precompute difference of control points
  std::vector<Point2d> dctrl_pts, ddctrl_pts;
  if (calc_yaw || calc_curvature)
  {
    dctrl_pts = diffPoint2dVec(order, ctrl_pts);
    if (calc_curvature)
    {
      ddctrl_pts = diffPoint2dVec(order-1, dctrl_pts);
    }
  }

  path.points.resize(t.size());
  for (size_t i = 0; i < t.size(); i++)
  {
    calcPathPointOnBezierCurve(
        t[i],
        ctrl_pts, dctrl_pts, ddctrl_pts,
        path.points[i], //output
        calc_yaw, calc_curvature); //optional
  }
}

void elevateBezierCurveOrder(
    std::vector<Point2d>& ctrl_pts)
{
  if (ctrl_pts.size() < 3) { return; }

  int k = ctrl_pts.size(); //new order
  std::vector<Point2d> new_ctrl_pts(k+1);

  new_ctrl_pts[0] = ctrl_pts[0];
  for (int i = 1; i < k; i++)
  {
    new_ctrl_pts[i] = (double(k-i)*ctrl_pts[i] + double(i)*ctrl_pts[i-1])/double(k);
  }
  new_ctrl_pts[k] = ctrl_pts[k-1];
  ctrl_pts = new_ctrl_pts;
}

//http://web.mit.edu/hyperbook/Patrikalakis-Maekawa-Cho/node13.html
//TODO, precompute Fibonacci sequence for faster calculation if order is high?
bool subdivideBezierCurve(
    const std::vector<Point2d>& ctrl_pts,
    const double t, //range (0,1)
    std::vector<Point2d>& left_ctrl_pts, //range [0,t]
    std::vector<Point2d>& right_ctrl_pts) //range [t,1]
{
  int n = ctrl_pts.size();
  if (n < 2) { return false; }
  if (t <= 0.0 || t >= 1.0) { return false; }

  std::vector<Point2d> b = ctrl_pts; //mutable copy
  left_ctrl_pts.resize(n);
  right_ctrl_pts.resize(n);
  for (int k = 0; k < n; k++)
  {
    left_ctrl_pts[k] = b[k];
    right_ctrl_pts[n-1-k] = b[n-1];
    for (int i = n-1; i >= k; i--) //loop backwards so b[i-1] is for k-1
    {
      b[i] = (1-t)*b[i-1] + t*b[i];
    }
  }
  return true;
}

//http://steve.hollasch.net/cgindex/curves/cbezarclen.html
void calcBezierCurveLength(
    const std::vector<Point2d>& ctrl_pts,
    double& length,
    double& error)
{
  int n = ctrl_pts.size();
  if (n < 2)
  {
    length = 0.0;
    error = 0.0;
    return;
  }

  double L0 = ctrl_pts[0].distance(ctrl_pts[n-1]);
  double L1 = 0.0;
  for (int i = 1; i < n; i++)
  {
    L1 += ctrl_pts[i-1].distance(ctrl_pts[i]);
  }
  int order = n-1;
  length = (2.0*L0 + double(order-1)*L1)/double(order+1); //approximate
  error = L1 - L0;
}

void calcPointsOnBezierCurveDeCasteljau(
    const double err_tol,
    const std::vector<Point2d>& ctrl_pts,
    std::vector<Point2d>& pts,
    int level)
{
  if (level == 0) { pts.clear(); }

  double length, error;
  calcBezierCurveLength(ctrl_pts, length, error);

//  ROS_INFO("%slength = %f, error = %f", std::string(level*2, ' ').c_str(), length, error); //DEBUGGING

  if (error < err_tol || level > 10) //check level to avoid infinite recursion in case of bug
  {
    int offset = pts.empty() ? 0 : 1; //to avoid duplicate points
    pts.insert(pts.end(), //append to end
        ctrl_pts.begin() + offset,
        ctrl_pts.end());
  }
  else
  {
    //subdivide and recurse
    std::vector<Point2d> left_ctrl_pts, right_ctrl_pts;
    subdivideBezierCurve(ctrl_pts, 0.5, left_ctrl_pts, right_ctrl_pts);
    calcPointsOnBezierCurveDeCasteljau(err_tol, left_ctrl_pts, pts, level+1); //must recurse on left first
    calcPointsOnBezierCurveDeCasteljau(err_tol, right_ctrl_pts, pts, level+1);
  }
}


void calcPathOnBezierCurveDeCasteljau(
    const double err_tol,
    const std::vector<Point2d>& ctrl_pts,
    swri_nav_msgs::PathSegment& path,
    bool calc_yaw,
    bool calc_curvature,
    int level)
{
  if (level == 0) { path.points.clear(); }

  double length, error;
  calcBezierCurveLength(ctrl_pts, length, error);

//  ROS_INFO("%slength = %f, error = %f", std::string(level*2, ' ').c_str(), length, error); //DEBUGGING

  if (error < err_tol || level > 10) //check level to avoid infinite recursion in case of bug
  {
    swri_nav_msgs::PathSegment subpath;
    calcPathOnBezierCurve(ctrl_pts.size(), ctrl_pts, subpath, calc_yaw, calc_curvature);
    int offset = path.points.empty() ? 0 : 1; //to avoid duplicate points
    path.points.insert(path.points.end(), //append to end
        subpath.points.begin() + offset,
        subpath.points.end());
  }
  else
  {
    //subdivide and recurse
    std::vector<Point2d> left_ctrl_pts, right_ctrl_pts;
    subdivideBezierCurve(ctrl_pts, 0.5, left_ctrl_pts, right_ctrl_pts);
    calcPathOnBezierCurveDeCasteljau(err_tol, left_ctrl_pts, path, calc_yaw, calc_curvature, level+1); //must recurse on left first
    calcPathOnBezierCurveDeCasteljau(err_tol, right_ctrl_pts, path, calc_yaw, calc_curvature, level+1);
  }
}

} //namespace
