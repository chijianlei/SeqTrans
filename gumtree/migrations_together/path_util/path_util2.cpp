
#include <swri_nav_util/path_util.h>
#include <swri_nav_util/transform_2d_util.h> //for modPiToPi

#include <tf2/transform_datatypes.h> //for tf2::Vector3
#include <tf2/utils.h>

#include <swri_roscpp/logging.h>

namespace swri_nav_util
{

void reduceSegments(
  swri_nav_msgs::msg::Path& path)
{
  const swri_nav_msgs::msg::Path path_cpy = path; //copies
  path.segments.clear();
  path.segments.reserve(path_cpy.segments.size());

  for (const swri_nav_msgs::msg::PathSegment& segment_to_append : path_cpy.segments)
  {
    if (segment_to_append.points.empty())
      continue;

    if (path.segments.empty() ||
        path.segments.back().in_reverse != segment_to_append.in_reverse)
    {
      //in_reverse changed, new segment required
      path.segments.push_back(segment_to_append);
    }
    else
    {
      //in_reverse same, append points to existing segment
      //may result in coincident points!

      //abbreviate
      typedef std::vector<swri_nav_msgs::msg::PathPoint> PointVec;
      PointVec& points = path.segments.back().points;
      const PointVec& points_to_append = segment_to_append.points;

      points.insert(
          points.end(),
          points_to_append.begin(),
          points_to_append.end());
    }
  }
}

double updateDistance(
    const double start_distance,
    swri_nav_msgs::msg::PathSegment& segment)
{
  double last_dist = start_distance;
  if (!segment.points.empty())
  {
    segment.points[0].distance = last_dist;
    for (size_t ptno=1; ptno < segment.points.size(); ptno++)
    {
      segment.points[ptno].distance = last_dist + distanceBetweenPoints(
          segment.points[ptno-1],
          segment.points[ptno]);
      last_dist = segment.points[ptno].distance;
    }
  }
  return last_dist;
}

double updateDistance(
    const double start_distance,
    swri_nav_msgs::msg::Path& path)
{
  double last_dist = start_distance; //mutable copy
  for (swri_nav_msgs::msg::PathSegment& segment : path.segments)
  {
    updateDistance(last_dist, segment);
    if (segment.points.size() > 0)
      last_dist = segment.points.back().distance;
  }
  return last_dist;
}

void removeCoincidentPoints(
    swri_nav_msgs::msg::PathSegment& segment,
    const double tol)
{
  swri_nav_msgs::msg::PathSegment new_segment;
  new_segment.in_reverse = segment.in_reverse;
  new_segment.points.reserve(segment.points.size());

  for (size_t ptno=0; ptno < segment.points.size(); ptno++)
  {
    //for each sequence of coincident points, keep only the first point
    if (ptno == 0 ||
        segment.points[ptno].distance - segment.points[ptno-1].distance > tol)
    {
      new_segment.points.push_back(segment.points[ptno]);
    }
  }
  segment = new_segment;
}

void removeCoincidentPoints(
    swri_nav_msgs::msg::Path& path,
    const double tol)
{
  for (swri_nav_msgs::msg::PathSegment& segment : path.segments)
  {
    removeCoincidentPoints(segment, tol);
  }
}

bool getBackPoint(
    const swri_nav_msgs::msg::Path& path,
    swri_nav_msgs::msg::PathPoint& back_pt) //output
{
  //loop backwards over all segments using reverse iterator
  for (auto segment_it = path.segments.rbegin();
      segment_it != path.segments.rend();
      ++segment_it)
  {
    if (!segment_it->points.empty())
    {
      back_pt = segment_it->points.back();
      return true;
    }
  }
  return false; //no points in path
}

void updateYaw(swri_nav_msgs::msg::PathSegment& segment)
{
  double yaw;
  for (size_t ptno=0; ptno+1 < segment.points.size(); ptno++)
  {
    //abbreviate
    swri_nav_msgs::msg::PathPoint& pt = segment.points[ptno];
    swri_nav_msgs::msg::PathPoint& next_pt = segment.points[ptno+1];
    //set yaw based on orientation of vector to next point
    yaw = atan2(
        next_pt.y - pt.y,
        next_pt.x - pt.x);
    if (segment.in_reverse) { yaw += M_PI; } //flip orientation if in reverse gear
    pt.yaw = yaw;
  }
  //set back point yaw equal to yaw of preceding point
  if (segment.points.size() > 1)
    segment.points.back().yaw = yaw;
}

void updateYaw(swri_nav_msgs::msg::Path& path)
{
  for (size_t segno=0; segno < path.segments.size(); segno++)
  {
    updateYaw(path.segments[segno]);
  }
}

//unwrap yaw for smooth interpolation
void unwrapYaw(
    swri_nav_msgs::msg::PathSegment& segment)
{
  for (size_t ptno=1; ptno < segment.points.size(); ptno++)
  {
    double dyaw = modPiToPi(
        segment.points[ptno].yaw - segment.points[ptno-1].yaw);

    segment.points[ptno].yaw = segment.points[ptno-1].yaw + dyaw;
  }
}

void unwrapYaw(
    swri_nav_msgs::msg::Path& path)
{
  for (swri_nav_msgs::msg::PathSegment& segment : path.segments)
  {
    unwrapYaw(segment);
  }
}

//subfunctions of findNearestLocalDistance

//TODO, are clamp inputs necessary? always true?
void separationFromLineSegment(
  const swri_nav_msgs::msg::PathPoint point0,
  const swri_nav_msgs::msg::PathPoint point1,
  const double x, const double y,
  bool clamp0, //nearest point on line segment can't exceed point0
  bool clamp1, //can't exceed point1
  double &separation, //separation between x,y and line segment
  double &nearest_distance) // .distance value for nearest point on line segment
{
  tf2::Vector3 x0(point0.x, point0.y, 0.0);
  tf2::Vector3 x1(point1.x, point1.y, 0.0);
  tf2::Vector3 p(x, y, 0.0);

  tf2::Vector3 v = x1 - x0; //vector from point0 to point1
  double v_len2 = v.length2(); //norm^2

  //s is normalized distance of nearest point on line segment
  //from point0 to point1
  double s = 0.0;
  if (v_len2 > 0)
  {
    s = v.dot(p - x0) / v_len2;
    if (clamp0 && s < 0.0)
      s = 0.0;
    if (clamp1 && s > 1.0)
      s = 1.0;
  }
  tf2::Vector3 xn = x0 + s*v; //nearest

  separation = xn.distance(p);
  nearest_distance = point0.distance + s*(point1.distance - point0.distance);
}

inline bool interpolatePathPointPair(
  const swri_nav_msgs::msg::PathPoint p0,
  const swri_nav_msgs::msg::PathPoint p1,
  const double distance,
  swri_nav_msgs::msg::PathPoint &out_point)
{
  if (p1.distance == p0.distance)
  {
    ROS_ERROR("Cannot interpolate between points with equal distance = %f", p0.distance);
    return false;
  }

  double s = (distance - p0.distance) / (p1.distance - p0.distance);

  out_point.x = p0.x + s*(p1.x - p0.x);
  out_point.y = p0.y + s*(p1.y - p0.y);
  out_point.yaw = p0.yaw + s*modPiToPi(p1.yaw - p0.yaw); //accounts for yaw wrapping
  out_point.curvature = p0.curvature + s*(p1.curvature - p0.curvature);
  out_point.distance = distance;

  return true;
}

bool findLocalNearestDistanceForward(
    const swri_nav_msgs::msg::PathSegment& segment,
    const double x, const double y,
    const double start_distance,
    double& nearest_distance, //>= start_distance
    double& nearest_separation)
{

  //special cases
  size_t num_points = segment.points.size();
  if (num_points == 0)
    return false;
  if (num_points == 1)
  {
    nearest_distance = std::max(segment.points[0].distance, start_distance);
    nearest_separation = distanceBetweenPoints(segment.points[0], makePathPoint(x,y));
    return true;
  }

  //try line segments in the path
  double min_separation = std::numeric_limits<double>::infinity();
  for (size_t ptno = 0; ptno+1 < num_points; ptno++)
  {
    if (segment.points[ptno+1].distance < start_distance)
    {
      continue;
    }
    //segment.points[ptno].distance may be < start_distance

    double separation;
    double this_distance;

    separationFromLineSegment(
        segment.points[ptno], segment.points[ptno+1], x, y, true, true, //input
        separation, this_distance);

    //in case segment.points[ptno].distance < start_distance
    if (this_distance < start_distance)
    {
      this_distance = start_distance;
      //update separation to match
      swri_nav_msgs::msg::PathPoint this_pt;
      interpolatePathPointPair(segment.points[ptno], segment.points[ptno+1], this_distance, this_pt);
      separation = distanceBetweenPoints(this_pt, makePathPoint(x,y));
    }

    if (separation <= min_separation)
    {
      min_separation = separation;
      nearest_distance = this_distance;
    }

    //if separation is increasing, local minima was found. stop the search
    if (separation > 2.0*min_separation)
    {
      break;
    }
  }

  nearest_separation = min_separation;

  return true;
}

bool findLocalNearestDistanceBackward(
    const swri_nav_msgs::msg::PathSegment& segment,
    const double x, const double y,
    const double start_distance,
    double& nearest_distance, //<= start_distance
    double& nearest_separation)
{
  //reverse the order of points
  swri_nav_msgs::msg::PathSegment segment_rev = segment;
  std::reverse(segment_rev.points.begin(), segment_rev.points.end());

  //reversing point order makes distance values decreasing
  //negate them to fix.
  for (size_t ptno=0; ptno < segment_rev.points.size(); ptno++)
    segment_rev.points[ptno].distance *= -1.0;

  if (findLocalNearestDistanceForward(segment_rev, x, y, -start_distance,
      nearest_distance, nearest_separation))
  {
    nearest_distance *= -1.0; //undo negate
    return true;
  }
  return false;
}

bool findLocalNearestDistanceBidirect(
    const swri_nav_msgs::msg::PathSegment& segment,
    const double x, const double y,
    const double start_distance,
    double& nearest_distance, //output
    double& nearest_separation)
{
  if (segment.points.empty())
    return false;

  //else findLocalNearestDistanceForward and *Backward should return true

  //search forward
  double near_dist_f, near_sep_f;
  findLocalNearestDistanceForward(
      segment, x, y, start_distance, near_dist_f, near_sep_f);

  //search backward
  double near_dist_b, near_sep_b;
  findLocalNearestDistanceBackward(
      segment, x, y, start_distance, near_dist_b, near_sep_b);

  //determine which direction is closer
  if (near_sep_f <= near_sep_b)
  {
    nearest_distance = near_dist_f;
    nearest_separation = near_sep_f;
  }
  else
  {
    nearest_distance = near_dist_b;
    nearest_separation = near_sep_b;
  }
  return true;
}

int distanceToIndex(
    const swri_nav_msgs::msg::PathSegment& segment,
    const double distance,
    const int init_idx)
{
  int idx = -1;
  for (int ptno = init_idx; ptno < segment.points.size(); ptno++)
  {
    if (segment.points[ptno].distance <= distance)
      idx = ptno;
    else
      break;
  }
  return idx;
}


bool interpolatePathSegment(
    const swri_nav_msgs::msg::PathSegment& segment,
    const double distance,
    swri_nav_msgs::msg::PathPoint& out_point) //output
{
  //special cases
  size_t num_points = segment.points.size();
  if (num_points == 0)
    return false;
  if (num_points == 1) //TODO, this isn't necessary, special cases below should handle it
  {
    out_point = segment.points[0];
    return true;
  }
  if (distance <= segment.points.front().distance)
  {
    out_point = segment.points.front();
    return true;
  }
  if (distance >= segment.points.back().distance)
  {
    out_point = segment.points.back();
    return true;
  }

  int idx = distanceToIndex(segment, distance); //index of last point with .distance value <= input

  return interpolatePathPointPair(
      segment.points[idx],
      segment.points[idx+1], //idx can't be index of back point, that would have been handled as special case
      distance,
      out_point);
}

void updateSpacing(
    swri_nav_msgs::msg::PathSegment& segment,
    const double spacing)
{
  swri_nav_msgs::msg::PathSegment out = segment;
  if (segment.points.size() <= 1)
  {
    return; //return segment with 0 or 1 points unchanged
  }
  out.points.clear();
  out.points.push_back(segment.points[0]); //always include first point

  double distance = out.points[0].distance;
  int idx = 0;

  for (size_t iter = 0; iter < 999999; iter++) //to prevent infinite loop
  {
    distance += spacing;
    idx = distanceToIndex(segment, distance, idx);
    if (idx+1 < segment.points.size())
    {
      swri_nav_msgs::msg::PathPoint out_point;
      interpolatePathPointPair(
          segment.points[idx],
          segment.points[idx+1],
          distance,
          out_point);
      out.points.push_back(out_point);
    }
    else if (distanceBetweenPoints(out.points.back(),segment.points[idx]) > 0) //always include last point, unless coincident with previous
    {
      out.points.push_back(segment.points[idx]);
      break;
    }
  }
  segment = out;
}

//functions to transform path to a different coordinate system
void transformPathPoint(
    const swri_transform_util::Transform& transform,
    swri_nav_msgs::msg::PathPoint& point)
{

  //transform x,y
  tf2::Vector3 p0(point.x, point.y, 0.0);
  tf2::Vector3 p1 = transform * p0;

  point.x = p1.x();
  point.y = p1.y();

  //transform yaw
  tf2::Quaternion q0;
  q0.setRPY(0, 0, point.yaw);// = tf2::createQuaternionFromYaw(point.yaw);
  tf2::Quaternion q1 = transform * q0;

  point.yaw = tf2::getYaw(q1);
}

void transformPathSegment(
    const swri_transform_util::Transform& transform,
    swri_nav_msgs::msg::PathSegment& segment)
{
  for (size_t ptno = 0; ptno < segment.points.size(); ptno++)
    transformPathPoint(transform, segment.points[ptno]);
}

void transformPath(
    const swri_transform_util::Transform& transform,
    swri_nav_msgs::msg::Path& path)
{
  for (size_t segno = 0; segno < path.segments.size(); segno++)
    transformPathSegment(transform, path.segments[segno]);
}

//subfunction of makeArcPathSegment,
//call if curvature is near 0
void makeLinePathSegment(
    const double len,
    const double dist_step,
    swri_nav_msgs::msg::PathSegment& segment)
{
  double sign_len = (len < 0) ? -1.0 : 1.0;
  double abs_len = fabs(len);

  segment.in_reverse = len < 0;
  segment.points.clear();
  size_t n = ceil(abs_len/dist_step)+1;
  for (size_t i=0; i<n; i++)
  {
    double dist = static_cast<double>(i)*dist_step; //dist >= 0
    dist = std::min(dist, abs_len);

    segment.points.push_back(
        makePathPoint(dist * sign_len, 0, 0, 0));
  }
}

void makeArcPathSegment(
    const double curvature,
    double len,
    const double dist_step,
    swri_nav_msgs::msg::PathSegment& segment)
{
  // If a NaN len is passed in, it will propagate all the way
  // to numpoints, and the reserve statement below will segfault
  if (std::isnan(len))
  {
    len = 0.0;
  }
  //curvature == 0 will cause singularity,
  //curvature near zero will cause numerical issues due to finite precision
  if (fabs(curvature) < 1e-9)
  {
    makeLinePathSegment(len, dist_step, segment);
    return;
  }

  double sign_len = (len < 0) ? -1.0 : 1.0;
  double sign_curv = (curvature < 0) ? -1.0 : 1.0;
  double abs_len = fabs(len);
  double abs_curv = fabs(curvature);

  int num_points = ceil(abs_len/dist_step)+1;
  num_points = std::min(num_points,999999); //prevent bad_alloc

  segment.in_reverse = len < 0;
  segment.points.clear();
  segment.points.reserve(num_points);

  for (size_t i=0; i < num_points; i++)
  {
    double theta = static_cast<double>(i)*dist_step*abs_curv; //theta >= 0
    theta = std::min(theta, abs_len*abs_curv);
    theta *= sign_len;

    swri_nav_msgs::msg::PathPoint pt;
    pt.x = sin(theta)/abs_curv;
    pt.y = (1.0-cos(theta))/curvature;
    pt.yaw = theta*sign_curv;
    pt.curvature = 0;

    segment.points.push_back(pt);
  }
}

void makeArcPathSegment(
    const double curvature,
    const double len,
    const double dist_step,
    const swri_nav_msgs::msg::PathPoint& align_pt,
    swri_nav_msgs::msg::PathSegment& segment)
{
  makeArcPathSegment(curvature, len, dist_step, segment);

  //transform points to world coordinates
  double R[4];
  RotationMatrix2d(align_pt.yaw, R);
  for (swri_nav_msgs::msg::PathPoint& pt : segment.points) //mutable reference
  {
    RotatePoint2d(R, pt.x, pt.y);
    pt.x += align_pt.x;
    pt.y += align_pt.y;
    pt.yaw += align_pt.yaw;
  }
}

//transforms new arc to align with back point of input path
void appendArcPathSegment(
    const double curvature,
    const double len,
    const double dist_step,
    swri_nav_msgs::msg::Path& path)
{
  swri_nav_msgs::msg::PathSegment new_segment;
  swri_nav_msgs::msg::PathPoint back_pt;
  getBackPoint(path, back_pt);

  makeArcPathSegment(curvature, len, dist_step, back_pt, new_segment);
  path.segments.push_back(new_segment);
}

} //namespace
