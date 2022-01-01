#include <swri_nav_util/no_go_zone_util.h>
#include <swri_nav_util/string_format.h>

#include <swri_roscpp/logging.h>

namespace swri_nav_util
{

void ZoneData::updateBoundingBox()
{
  bb.setNegInf();
  for (size_t i=0; i<points.size(); i++)
  {
    bb.increaseToContainPoint(points[i].x(), points[i].y());
  }
}

ZoneData::ZoneData(
    const sumet_world_model_msgs::msg::ControlZone& msg)
{
  initialize();

  frame_id = msg.header.frame_id;
  //parse the polygon points
  points.resize(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); i++)
  {
    points[i].setX(msg.points[i].x);
    points[i].setY(msg.points[i].y);
    points[i].setZ(msg.points[i].z);
    //tf2::fromMsg(msg.points[i], points[i] );
  }
  updateBoundingBox();

  //parse the configuration key value pairs
  for (size_t i = 0; i < msg.configuration.size(); i++)
  {
    const std::string& key = msg.configuration[i].key;
    const std::string& value = msg.configuration[i].value;
    if (key.compare("NO_GO") == 0 && value.compare("TRUE") == 0)
    {
      no_go_zone = true;
    }
    if (key.compare("SLOW_GO") == 0)
    {
      slow_go_zone = true;
      max_speed = std::atof(value.c_str());
    }
    if (key.compare("STEALTH") == 0 && value.compare("TRUE") == 0)
    {
      stealth_zone = true;
    }
  }
}

bool ZoneData::compare(ZoneData zone) const
{
  //compare frame_id
  if (!sameFrameId(zone.frame_id, frame_id))
  {
    return false;
  }
  //compare points
  if (zone.points.size() != points.size())
  {
    return false;
  }
  for (size_t i=0; i<points.size(); i++)
  {
    if (points[i] != zone.points[i])
    {
      return false;
    }
  }
  //compare configuration
  if (zone.no_go_zone != no_go_zone ||
      zone.slow_go_zone != slow_go_zone ||
      zone.stealth_zone != stealth_zone ||
      zone.max_speed != max_speed)
  {
    return false;
  }
  return true;
}

//return true if success
bool ZoneData::transform(
    const swri_transform_util::TransformManager& tf_mgr,
    const std::string& target_frame)
{
  if (sameFrameId(target_frame, frame_id))
  {
    return true;
  }
  swri_transform_util::Transform transform(rclcpp::Time(0));
  if (!tf_mgr.GetTransform(target_frame, frame_id, rclcpp::Time(0), transform))
  {
    ROS_ERROR("Failed to transform no go zone from frame '%s' to '%s'",
        frame_id.c_str(), target_frame.c_str()); //DEBUGGING
    return false;
  }
  for (size_t i=0; i<points.size(); i++)
  {
    points[i] = transform * points[i];
  }
  frame_id = target_frame;
  updateBoundingBox();

  return true;
}

//assume zone points already transformed into correct frame
//assume zone bounding box is correct
//return true if costmap is modified
bool renderZoneOnCostmap(
    const ZoneData& zone,
    const float value,
    Costmap& costmap)
{
  if (zone.points.empty()) { return false; }

  //check if bounding boxes intersect
  if (!zone.bb.intersects(costmap.getBoundingBox()))
  {
    return false;
  }

  std::vector<cv::Point> polygon_pts;
  polygon_pts.reserve(zone.points.size());
  for (size_t i = 0; i < zone.points.size(); ++i)
  {
    int row, col;
    costmap.worldToSub(zone.points[i].x(), zone.points[i].y(), &row, &col);
    polygon_pts.push_back(cv::Point(col, row));
  }

  if (zone.convex)
  {
    cv::Point* pts = &polygon_pts.front();
    int npts = polygon_pts.size();
    cv::fillConvexPoly(costmap.map, pts, npts, cv::Scalar(value), 8);
  }
  else
  {
    const cv::Point* pts[1] = { &polygon_pts.front() };
    int npts[1] = { static_cast<int>(zone.points.size()) };
    cv::fillPoly(costmap.map, pts, npts, 1, cv::Scalar(value), 8);
  }

  return true;
}

bool renderZonesOnCostmap(
    const std::vector<ZoneData>& zones,
    const float value,
    Costmap& costmap)
{
  bool rendered_a_zone = false;
  for (size_t i=0; i < zones.size(); i++)
  {
    rendered_a_zone |= renderZoneOnCostmap(zones[i], value, costmap);
  }
  return rendered_a_zone;
}

} //namespace
