
#include <swri_nav_util/costmap.h>
#include <swri_nav_util/opencv_util.h>

#include <geometry_msgs/Point.h> //TODO, necessary?

#include<ros/ros.h> // TODO: Remove later

namespace swri_nav_util
{

bool Costmap::fromMsg(
    const sumet_nav_msgs::Costmap& msg)
{
  *this = Costmap();

  //copy map data
  if (msg.data.size() >= msg.rows*msg.columns)
  {
    cv::Mat src = cv::Mat(msg.rows, msg.columns, CV_32FC1,
        const_cast<float*> (&msg.data[0]));
    src.copyTo(map);
  }

  if (map.empty())
    return false;

  resolution = msg.resolution;

  x_min = msg.rect.left;
  y_min = msg.rect.bottom;

  return true;
}

bool Costmap::fromMsg(
    const persistent_map_msgs::LocalGridMap& msg)
{
  ROS_ASSERT(msg.rows >= 0);
  ROS_ASSERT(msg.columns >= 0);
  *this = Costmap();

  for (const persistent_map_msgs::MapLayer& layer : msg.layers)
  {
//    ROS_INFO("layer name = %s, values size = %zu",
//        layer.name.c_str(), layer.values.size()); //DEBUGGING

    if (layer.name.compare("costmap") == 0 &&
        layer.values.size() >= msg.rows*msg.columns) //necessary!
    {
      cv::Mat src = cv::Mat(msg.rows, msg.columns, CV_32FC1,
          const_cast<float*> (&layer.values[0]));
      src.copyTo(map);
      break;
    }
  }

  if (map.empty())
    return false;

  resolution = getMapResolutionMm(msg);

  x_min = msg.position.bottom_left.x;
  y_min = msg.position.bottom_left.y;

  return true;

}

void Costmap::toMsg(
    sumet_nav_msgs::Costmap& msg) const
{
  //minimum x, y values at bottom left pixel
  msg.rect.left = x_min;
  msg.rect.right = x_min + map.cols * resolution;
  msg.rect.bottom = y_min;
  msg.rect.top = y_min + map.rows * resolution;

  msg.resolution = resolution;

  msg.rows = map.rows;
  msg.columns = map.cols;

  //copy map data
  msg.data.resize(map.total());
  cv::Mat dest = cv::Mat(msg.rows, msg.columns, CV_32FC1,
      const_cast<float*> (&msg.data[0]));

  map.copyTo(dest);

}

void Costmap::toMsg(
    persistent_map_msgs::LocalGridMap& msg) const
{
  //set position
  double x_max = x_min + map.cols * resolution;
  double y_max = y_min + map.rows * resolution;

  geometry_msgs::Point point;
  point.z = 0;

  point.x = x_min, point.y = y_min;
  msg.position.bottom_left = point;

  point.x = x_max, point.y = y_min;
  msg.position.bottom_right = point;

  point.x = x_min; point.y = y_max;
  msg.position.top_left = point;

  point.x = x_max; point.y = y_max;
  msg.position.top_right = point;

  //set num rows and columns
  msg.rows = map.rows;
  msg.columns = map.cols;

  //copy map data into costmap layer
  persistent_map_msgs::MapLayer layer;
  layer.name = "costmap";

  layer.values.resize(map.total());
  cv::Mat dest = cv::Mat(msg.rows, msg.columns, CV_32FC1,
      const_cast<float*> (&layer.values[0]));

  map.copyTo(dest);

  msg.layers.push_back(layer);
}

//TODO, necessary?
void Costmap::copyTo(Costmap& dst) const
{
  dst = *this;

  //must use .copyTo on cv::Mat in dst
  //or they will point to same image as source

  dst.map = cv::Mat();
  if (!map.empty())
    map.copyTo(dst.map);

  dst.is_obst = cv::Mat();
  if (!is_obst.empty())
    is_obst.copyTo(dst.is_obst);

  dst.is_obst_integral = cv::Mat();
  if (!is_obst_integral.empty())
    is_obst_integral.copyTo(dst.is_obst_integral);
}

} //namespace

