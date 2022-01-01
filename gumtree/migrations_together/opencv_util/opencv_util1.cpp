#include <swri_nav_util/opencv_util.h>
#include <ros/ros.h>

namespace swri_nav_util
{

void hysteresisThreshHi(
    const cv::Mat& src,
    cv::Mat& dst,
    const float hi_thresh,
    const float hi_thresh_connected,
    const uint8_t connectivity)
{
  cv::Mat mask(src.rows+2, src.cols+2, CV_8UC1, cv::Scalar(0)); //must be 2 pixels larger

  //floodFill inputs
  int flags = connectivity | ( 255 << 8 ) | cv::FLOODFILL_FIXED_RANGE | cv::FLOODFILL_MASK_ONLY;
  float upDiff = std::numeric_limits<float>::max();

  for (int i=0; i<src.rows; i++)
  {
    for (int j=0; j<src.cols; j++)
    {
      float intensity = src.at<float>(i,j);
      if (intensity >= hi_thresh && !mask.at<uint8_t>(i+1,j+1))
      {
        float loDiff = intensity - hi_thresh_connected;
        cv::floodFill(src, mask, cv::Point(j,i), 0, 0, loDiff, upDiff, flags);
      }
    }
  }

  //copy block of mask
  //cv::Range interval is inclusive left, exclusive right
  mask(cv::Range(1,mask.rows-1),cv::Range(1,mask.cols-1)).copyTo(dst); //?
}

void hysteresisThreshLo(
    const cv::Mat& src,
    cv::Mat& dst,
    const float lo_thresh,
    const float lo_thresh_connected,
    const uint8_t connectivity)
{
  //negate the image and thresholds
  cv::Mat inverse = cv::Scalar(0) - src;
  hysteresisThreshHi(inverse, dst, -lo_thresh, -lo_thresh_connected, connectivity);
}

void despeckleThreshHi(
    const cv::Mat& src,
    cv::Mat& dst,
    const float hi_thresh,
    const cv::Mat& struct_el)
{
  //basic threshold
  cv::Mat above = src >= hi_thresh;

  //erode the basic threshold mask.
  //points that survived erosion are seed points for floodfill; they are marked as 'above' regardless.
  //eroded points may still be marked 'above' if they belong to connected components of seed points.
  cv::Mat above_erode(src.rows, src.cols, CV_8UC1); //temporary
  erode(above, above_erode, struct_el);

  //make a trilevel image from threshold mask
  cv::Mat trilevel = above/2; //mask points have only partial value if eroded
  trilevel.setTo(255, above_erode);

//  showInWindow("trilevel image", trilevel); //DEBUGGING

  trilevel.convertTo(trilevel, CV_32FC1); //floodFill requires float image input
  hysteresisThreshHi(trilevel, dst, 255, (255/2), 4);
}

void despeckleThreshLo(
    const cv::Mat& src,
    cv::Mat& dst,
    const float lo_thresh,
    const cv::Mat& struct_el)
{
  //negate the image and threshold
  cv::Mat inverse = cv::Scalar(0) - src;
  despeckleThreshHi(inverse, dst, -lo_thresh, struct_el);
}

//http://nghiaho.com/?p=1102
//if OpenCV 3.0 is available use cv::connectedComponentsWithStats instead
void findBlobs(
    const cv::Mat &binary,
    std::vector < std::vector<cv::Point2i> > &blobs)
{

//  showInWindow("find blobs", binary);

  blobs.clear();

  // Fill the label_image with the blobs
  // 0  - background
  // 1  - unlabelled foreground
  // 2+ - labelled foreground

//  cv::Mat label_image;
//  binary.convertTo(label_image, CV_32SC1); //doesn't work, sets to 255 instead of 1?

  cv::Mat label_image(binary.size(), CV_32SC1);
  label_image.setTo(1, binary);

  int label_count = 2; // starts at 2 because 0,1 are used already

  for(int y=0; y < label_image.rows; y++)
  {
    int *row = (int*)label_image.ptr(y);
    for(int x=0; x < label_image.cols; x++)
    {
      if(row[x] != 1)
      {
        continue;
      }

      cv::Rect rect;
      int flags = 8; //TODO, check this.
      cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, flags);

      std::vector <cv::Point2i> blob;
      for(int i=rect.y; i < (rect.y+rect.height); i++)
      {
        int *row2 = (int*)label_image.ptr(i);
        for(int j=rect.x; j < (rect.x+rect.width); j++)
        {
          if(row2[j] != label_count)
          {
            continue;
          }

          blob.push_back(cv::Point2i(j,i));
        }
      }
      blobs.push_back(blob);

      label_count++;
    }
  }
}

cv::Point2f calcCentroid(const std::vector<cv::Point>& pts)
{
  cv::Point2f centroid(0.0, 0.0);
  if (!pts.empty())
  {
    for (size_t i = 0; i < pts.size(); i++)
    {
      centroid.x += pts[i].x;
      centroid.y += pts[i].y;
    }
    double n = pts.size();
    centroid.x /= n;
    centroid.y /= n;
  }
  return centroid;
}

} //namespace
