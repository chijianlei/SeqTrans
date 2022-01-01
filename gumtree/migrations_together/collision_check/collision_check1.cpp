
#include <math.h>

#include <swri_roscpp/parameters.h>

#include <swri_nav_util/collision_check.h>
#include <swri_nav_util/transform_2d_util.h>
//#include <swri_nav_util/opencv_util.h>


namespace swri_nav_util
{

bool VehicleDimensions::getParam(ros::NodeHandle& nh,
    std::string ns)
{
  bool exist = true;
  exist &= swri::getParam(nh, ns + "/length", length);
  exist &= swri::getParam(nh, ns + "/width", width);
  exist &= swri::getParam(nh, ns + "/rear_overhang", rear_overhang);
  exist &= swri::getParam(nh, ns + "/wheelbase", wheelbase);
  exist &= swri::getParam(nh, ns + "/track", track);

  return exist;
}

VehicleDimensions VehicleDimensions::addMargin(const double margin) const
{
  VehicleDimensions out(*this); //copy
  if (margin != 0.0)
  {
    out.length += 2.0*margin;
    out.width += 2.0*margin;
    out.rear_overhang += margin;
  }
  return out;
}

VehicleDimensions VehicleDimensions::addMargins(
    const double margin_fb,
    const double margin_lr)
const
{
  VehicleDimensions out(*this); //copy
  out.length += 2.0*margin_fb;
  out.width += 2.0*margin_lr;
  out.rear_overhang += margin_fb;
  return out;
}

//http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
//https://en.wikipedia.org/wiki/Xiaolin_Wu%27s_line_algorithm

//subfunction of BresenhamLine
inline int sign(const int val)
{
  return (val > 0) - (val < 0);
}
inline Point2i swapPoint2i(
    const bool swap, const int x, const int y)
{
  return swap ? Point2i(y,x) : Point2i(x,y);
}

void BresenhamLine(
    int x0, int y0, //mutable, pass by value to allow swap
    int x1, int y1,
    std::vector<Point2i>& pts, //output
    bool pad_diagonal) //optional input
{
  pts.clear();

  bool steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep)
  {
    std::swap(x0,y0);
    std::swap(x1,y1);
  }

  int dx = x1-x0;
  int dy = y1-y0;

  int abs_dx = abs(dx);
  int abs_dy = abs(dy);
  int sign_dx = sign(dx);
  int sign_dy = sign(dy);

  int D = -abs_dx;
  int y = y0;

  for (int x = x0; true; x+=sign_dx) //x != x1+sign_dx
  {
    pts.push_back(swapPoint2i(steep,x,y));
    if (x == x1)
      break; //in case sign_dx == 0

    D += 2*abs_dy;
    if (D > 0)
    { //take a diagonal step
      if (pad_diagonal)
      {
        //pad diagonal step with additional points
        pts.push_back(swapPoint2i(steep,x+sign_dx,y));
        pts.push_back(swapPoint2i(steep,x,y+sign_dy));
      }
      y += sign_dy;
      D -= 2*abs_dx;
    }
  }

}


//area calculation using integral image:
//http://docs.opencv.org/2.4/modules/imgproc/doc/miscellaneous_transformations.html#integral
int calcArea(const cv::Mat& integral_image, //CV_32S type
    const int& i0, const int& j0,
    const int& i1, const int& j1) //non-inclusive!
{
  //doesn't check bounds!
  int val_11 = integral_image.at<int32_t>(i1,j1);
  int val_01 = integral_image.at<int32_t>(i0,j1);
  int val_10 = integral_image.at<int32_t>(i1,j0);
  int val_00 = integral_image.at<int32_t>(i0,j0);
  int area = val_11 - val_01 - val_10 + val_00;

  return area;
}

//rasterization method based on:
//http://www.angelfire.com/linux/myp/ConvexPolRas/ConvexPolRas.html
//polygon must be closed and convex!

bool collisionCheckPolygon(
    const Costmap& costmap,
    const std::vector<Point2i>& verts, //in image coords
    cv::Mat* draw_checked)
{
  int num_rows = costmap.is_obst.rows;
  int num_cols = costmap.is_obst.cols;

  //TODO, std::numeric_limits<int>::infinity() treated as zero, don't use!
  const int int_max = std::numeric_limits<int>::max();

  std::vector<int> x_min(num_rows,int_max);
  std::vector<int> x_max(num_rows,-int_max);
  int y_min = int_max;
  int y_max = -int_max;

  //get x limits for each row
  std::vector<Point2i> pts;
  for (size_t lineno = 0; lineno < verts.size()-1; lineno++)
  {
    BresenhamLine(
        verts[lineno].x, verts[lineno].y,
        verts[lineno+1].x, verts[lineno+1].y,
        pts);

    for (size_t ptno = 0; ptno < pts.size(); ptno++)
    {
      //abbreviate
      int x = pts[ptno].x;
      int y = pts[ptno].y;

      if (y >= 0 && y < num_rows)
      {
        if (x < x_min[y]) x_min[y] = x;
        if (x > x_max[y]) x_max[y] = x;
        if (y < y_min) y_min = y;
        if (y > y_max) y_max = y;
      }
    }
  }

  //scan rows, check for collision
  for (int i = y_min; i <= y_max; i++) //row subscript
  {
    if (x_min[i] < 0) x_min[i] = 0;
    if (x_max[i] >= num_cols) x_max[i] = num_cols-1;

    //DEBUGGING
    if (draw_checked != nullptr)
    {
      for (int j = x_min[i]; j <= x_max[i]; j++) //column subscript
      {
        draw_checked->at<uint8_t>(i,j) = 255;
      }
    }

    for (int j = x_min[i]; j <= x_max[i]; j++) //column subscript
    {
      if (costmap.is_obst.at<uint8_t>(i,j))
        return true;
    }
  }
  return false;
}

//get limits of x,y values in vector of points
//max values are inclusive:
// x_min <= x <= x_max
// y_min <= y <= y_max
void getLimitsPoint2i(std::vector<Point2i>& pts,
    int& x_min, int& y_min,
    int& x_max, int& y_max)
{
  //initialize
  const double int_max = std::numeric_limits<int>::max();
  x_min = int_max;
  y_min = int_max;
  x_max = -int_max;
  y_max = -int_max;

  for (size_t i = 0; i < pts.size(); i++)
  {
    x_min = std::min(x_min, pts[i].x);
    y_min = std::min(y_min, pts[i].y);
    x_max = std::max(x_max, pts[i].x);
    y_max = std::max(y_max, pts[i].y);
  }
}

bool collisionCheckPose(
    const VehiclePose& pose,
    const VehicleDimensions& dim,
    const Costmap& costmap,
    cv::Mat* draw_checked)
{
  //get footprint vertices in image coordinates
  std::vector<Point2i> verts_img(5);
  {
    double xf = dim.length - dim.rear_overhang; //front
    double xr = -dim.rear_overhang; //rear
    double w = 0.5*dim.width;
    double X[4] = {xr, xf, xf, xr};
    double Y[4] = {-w, -w, w, w};
    double R[4]; RotationMatrix2d(pose.yaw, R);

    for (int idx = 0; idx < 4; idx++)
    {
      costmap.worldToSub(
          pose.x + R[0]*X[idx] + R[2]*Y[idx],
          pose.y + R[1]*X[idx] + R[3]*Y[idx],
          &verts_img[idx].y, //row subscript
          &verts_img[idx].x); //column subscript
    }
    verts_img.back() = verts_img.front(); //closed polygon required
  }


  //preliminary check if any obstacle pixels within bounding box of vehicle footprint
  //use integral image
  if (!costmap.is_obst_integral.empty())
  {
    int i0, j0, i1, j1;
    getLimitsPoint2i(verts_img, j0, i0, j1, i1);

    //abbreviate
    int rows = costmap.is_obst.rows;
    int cols = costmap.is_obst.cols;

    if (i0 >= rows || j0 >= cols || i1 < 0 || j1 < 0)
    {
      return false;
    }

    //crop the bounding box to be within costmap
    i0 = std::max(i0,0);
    j0 = std::max(j0,0);
    i1 = std::min(i1,rows-1);
    j1 = std::min(j1,cols-1);

    //+1 because area calculation is non-inclusive of i1,j1
    //okay of i1 == rows or j1 == cols because
    //is_obst_integral has one more row and column than is_obst
    int area = calcArea(costmap.is_obst_integral, i0,j0,i1+1,j1+1);
    if (area == 0) { return false; }
  }

  return collisionCheckPolygon(costmap, verts_img, draw_checked);
}

//return true if collides
bool collisionCheckPath(
    const swri_nav_msgs::Path& path,
    const VehicleDimensions& dim,
    const Costmap& costmap,
    double* collision_dist,
    cv::Mat* draw_checked)
{
  if (collision_dist != nullptr)
  {
    *collision_dist = std::numeric_limits<double>::infinity();
  }

  //check polygon of vehicle footprint for every pose
  for (size_t segno=0; segno < path.segments.size(); segno++)
  {
    swri_nav_msgs::PathSegment segment = path.segments[segno];
    for (size_t ptno=0; ptno < segment.points.size(); ptno++)
    {
      const VehiclePose& pose = segment.points[ptno];
      if (collisionCheckPose(pose, dim, costmap, draw_checked))
      {
        if (collision_dist != nullptr) { *collision_dist = pose.distance; }
        return true;
      }
    }
  }
  return false;
}


//TODO, check this
bool collisionCheckPath(
    const swri_nav_msgs::Path& path,
    const VehicleDimensions& dim,
    const CostmapTiling& costmap_tiling,
    double* collision_dist)
{
//  ROS_INFO("called collisionCheckPath()");
  if (collision_dist != nullptr)
  {
    *collision_dist = std::numeric_limits<double>::infinity();
  }

  //pointer to current costmap
  const Costmap* costmap_ptr = nullptr; //the pointer is mutable, but the contents pointed to aren't
  const double NaN = std::numeric_limits<double>::quiet_NaN();
  TileIndex last_tile_idx;
  bool last_tile_idx_set = false;

  //check polygon of vehicle footprint for every pose
  for (size_t segno=0; segno < path.segments.size(); segno++)
  {
    swri_nav_msgs::PathSegment segment = path.segments[segno];
    for (size_t ptno=0; ptno < segment.points.size(); ptno++)
    {
      const VehiclePose& pose = segment.points[ptno];

      //update pointer to current speedmap
      Point2d center = vehicleCenter(pose,dim);
      TileIndex tile_idx = costmap_tiling.getContainingTile(center.x, center.y);
      if (!last_tile_idx_set || !(tile_idx == last_tile_idx))
      {
//        ROS_INFO("  tile indices changed to %d %d (vehicle center at %f %f)",
//            tile_idx.x, tile_idx.y, center.x, center.y);
        costmap_ptr = costmap_tiling.getPtr(tile_idx); //returns nullptr if doesn't exist
        last_tile_idx = tile_idx;
        last_tile_idx_set = true;
      }

      if (costmap_ptr != nullptr)
      {
        if (collisionCheckPose(pose, dim, *costmap_ptr))
        {
          if (collision_dist != nullptr) { *collision_dist = pose.distance; }
          return true;
        }
      } //else out of bounds, no collision
    }
  }
  return false;
}


inline void setPoseToIdentity(
    geometry_msgs::Pose& pose)
{
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.w = 1.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
//  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
}

void pathFootprintToMarker(
    const swri_nav_msgs::Path& path,
    const VehicleDimensions& dim,
    const std_msgs::ColorRGBA& color,
    visualization_msgs::Marker& marker)
{
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  setPoseToIdentity(marker.pose);

  double thk = 1.0; //TODO, make this an input?
  geometry_msgs::Vector3 scale;
  scale.x = thk;
  marker.scale = scale;

  marker.color = color;
  marker.points.clear();

//  marker.id = 0; //TODO, make this an input?

  for (size_t segno = 0; segno < path.segments.size(); segno++)
  {
    const swri_nav_msgs::PathSegment& segment = path.segments[segno];
    for (size_t ptno = 0; ptno < segment.points.size(); ptno++)
    {
      //vertices of the vehicle footprint at pose
      //TODO, eliminate duplication with collisionCheckPose
      std::vector<geometry_msgs::Point> verts(5);
      {
        const VehiclePose& pose = segment.points[ptno];
        double xf = dim.length - dim.rear_overhang; //front
        double xr = -dim.rear_overhang; //rear
        double w = 0.5*dim.width;
        double X[4] = {xr, xf, xf, xr};
        double Y[4] = {-w, -w, w, w};
        double R[4]; RotationMatrix2d(pose.yaw, R);

        for (int idx = 0; idx < 4; idx++)
        {
          verts[idx].x = pose.x + R[0]*X[idx] + R[2]*Y[idx];
          verts[idx].y = pose.y + R[1]*X[idx] + R[3]*Y[idx];
          verts[idx].z = 0.0;
        }
        verts.back() = verts.front(); //closed polygon required
      }

      //append to marker points in pairs to draw line segments
      for (size_t idx=1; idx < verts.size(); idx++)
      {
        marker.points.push_back(verts[idx-1]);
        marker.points.push_back(verts[idx]);
      }
    }
  }
}

double maxValAtWheelLocs(
    const VehiclePose& pose,
    const VehicleDimensions& dim,
    const Costmap& costmap,
    const double out_of_bounds_val,
    const bool negate_costs)
{
  double sgn = negate_costs ? -1.0 : 1.0;

  //wheel coordinates in vehicle frame
  double t = 0.5*dim.track;
  double X[4] = {0.0, dim.wheelbase, dim.wheelbase, 0.0};
  double Y[4] = {-t, -t, t, t};

  double R[4]; RotationMatrix2d(pose.yaw, R);

  double cost = -std::numeric_limits<double>::infinity();
  for (int idx = 0; idx < 4; idx++)
  {
    int i,j; //row and column subscripts
    costmap.worldToSub(
        pose.x + R[0]*X[idx] + R[2]*Y[idx],
        pose.y + R[1]*X[idx] + R[3]*Y[idx],
        &i, &j);
    cost = std::max(sgn*costmap.getValue(i,j,out_of_bounds_val), cost);
  }
  return cost;
}

} //namespace
