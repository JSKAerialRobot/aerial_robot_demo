#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// if graspable -> (angle, 1) else -> (angle, 0) if reached -> (angle, 1, pointx1, pointy1, pointx2, pointy2)
std::vector<float> space_detector(cv::Mat src, cv::Mat depth_org){
  std::vector<float> answer;
  int graspable = 0;
  cv::imwrite("/home/kuromiya/debug.png", src);
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bw;
  cv::threshold(gray, bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  std::vector<cv::Point>  contour;

  cv::Mat depth;
  cv::cvtColor(depth_org, depth, cv::COLOR_GRAY2BGR);

  for(int i = 0; i < src.rows; i++){
    cv::Vec3b * sr = src.ptr<cv::Vec3b>(i);
    for(int j =0; j < src.cols; j++){
      cv::Vec3b s = sr[j];
      if((s[0] == 0) && (s[1] == 255) == (s[2] == 0)){
        cv::Point point;
        point.x = j;
        point.y = i;
        contour.push_back(point);
      }
    }
  }

  cv::RotatedRect box ;
  cv::RotatedRect box1 ;

  box = cv::minAreaRect(contour);
  float angle = box.angle;
  answer.push_back(angle);

  // std::cout << "angle : " << angle << std::endl;
  cv::Mat thresh;
  int length = 0;

  if(std::abs(angle) > 5 && std::abs(90 - std::abs(angle)) > 5){
    std::cout << "angle : " << std::abs(90 - std::abs(angle)) << std::endl;
    cv::Mat rotate;
    cv::Mat rotate_org;
    cv::Mat transform_m = cv::getRotationMatrix2D(cv::Point(depth.cols/2, depth.rows/2), box.angle, 1);
    cv::warpAffine(depth, rotate, transform_m, depth.size());
    cv::warpAffine(src, rotate_org, transform_m, depth.size());
    std::vector<cv::Point> contour1;

    for(int i = 0; i < src.rows; i++){
      cv::Vec3b * sr = rotate_org.ptr<cv::Vec3b>(i);
      for(int j =0; j < src.cols; j++){
        cv::Vec3b s = sr[j];
        if((s[0] <= 100) && (s[1] >= 100) == (s[2] <= 100)){
          cv::Point point;
          point.x = j;
          point.y = i;
          contour1.push_back(point);
        }
      }
    }

    box1 = cv::minAreaRect(contour1);

    int width = box1.size.width;
    int height = box1.size.height;
    length = std::max(width, height);
    // cv::imwrite("/home/kuromiya/cut_org.png", depth);
    cv::Mat cut(rotate, cv::Range((-2*length)+box1.center.y, 2*length+box1.center.y), cv::Range((-2*length)+box1.center.x, 2*length+box1.center.x));

    // if you just use cut without saving you will get a strange image (not correct one)
    cv::imwrite("/home/kuromiya/cut.png", cut);
    cv::Mat cutting = cv::imread("/home/kuromiya/cut.png", 1);
    int c = cutting.at<cv::Vec3b>(2*length, 2*length)[0];
    // std::cout << c << std::endl;
    cv::threshold(cut, thresh, c + 12, 255, cv::THRESH_BINARY);  // 12 = 255 / 4.0 * 0.2
  }

  else{
    int width = box.size.width;
    int height = box.size.height;
    length = std::max(width, height);
    cv::imwrite("/home/kuromiya/cut_org.png", depth);
    cv::Mat cut(depth, cv::Range((-2*length)+box.center.y, 2*length+box.center.y), cv::Range((-2*length)+box.center.x, 2*length+box.center.x));
    // if you just use cut without saving you will get a strange image (not correct one)
    cv::imwrite("/home/kuromiya/cut.png", cut);
    cv::Mat cutting = cv::imread("/home/kuromiya/cut.png", 1);
    int c = cutting.at<cv::Vec3b>(2*length, 2*length)[0];
    // std::cout << c << std::endl;
    cv::threshold(cut, thresh, c + 12, 255, cv::THRESH_BINARY);  // 12 = 255 / 4.0 * 0.2
  }
  // check whether hydrus can grasp the object

  int left_upper_count = 0;
  int left_lower_count = 0;
  int right_upper_count = 0;
  int right_lower_count = 0;

  for(int i = 0; i < thresh.rows; i++){
    uchar * th = thresh.ptr<uchar>(i);
    for(int j = 0; j < thresh.cols; j++){
      uchar t = th[j];
      if(t == 255){
        // detect which direction is best(with a little bit margin)
        if(std::abs(i - 2 * length) < 0.6 * length && std::abs(j - 2 * length) < 0.6 * length){
          continue;
        }
        else if(i < 1.4 * length && j < 1.4 * length){
          left_upper_count++;
        }
        else if(i > 2.6 * length && j < 1.4 * length){
          left_lower_count++;
        }
        else if(i < 1.4 * length && j > 2.6 * length){
          right_upper_count++;
        }
        else if(i > 2.6 * length && j > 2.6 * length){
          right_lower_count++;
        }
      }
    }
  }

  if(left_upper_count < 100 && left_lower_count < 100){
    cv::line(thresh, cv::Point(2 * length , 2 * length) ,cv::Point(3.2 * length, 2 * length), cv::Scalar(255, 0, 0), 1);
    graspable = 1;
    answer.push_back(graspable);
    if(std::abs(angle) < 5){
      answer.push_back(box.center.x);
      answer.push_back(box.center.y);
      answer.push_back(box.center.x + 0.6 * length);
      answer.push_back(box.center.y);
    }

  cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
  }

  if(right_upper_count < 100 && right_lower_count < 100){
    cv::line(thresh, cv::Point(0.6 * length , 2 * length) ,cv::Point(2 * length, 2 * length), cv::Scalar(255, 0, 0), 1);
    graspable = 1;
    answer.push_back(graspable);
    if(std::abs(angle) < 5){
      answer.push_back(box.center.x - 0.6 * length);
      answer.push_back(box.center.y);
      answer.push_back(box.center.x);
      answer.push_back(box.center.y);
    }
    cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }

  if(left_upper_count < 100 && right_upper_count < 100){
    graspable = 1;
    answer.push_back(graspable);
    cv::line(thresh, cv::Point(2 * length , 2 * length) ,cv::Point(2 * length, 3.2 * length), cv::Scalar(255, 0, 0), 1);
    if(std::abs(angle) < 5){
      answer.push_back(box.center.x);
      answer.push_back(box.center.y);
      answer.push_back(box.center.x);
      answer.push_back(box.center.x + 0.6 * length);
    }
  cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
  }

  if(left_lower_count < 100 && right_lower_count < 100){
    graspable = 1;
    answer.push_back(graspable);
    cv::line(thresh, cv::Point(2 * length , 0.6 * length) ,cv::Point(2 * length, 2 * length), cv::Scalar(255, 0, 0), 1);
    if(std::abs(angle) < 5){
      answer.push_back(box.center.x);
      answer.push_back(box.center.y - 0.6 * length);
      answer.push_back(box.center.x);
      answer.push_back(box.center.y);
    }
  cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
  }


  answer.push_back(graspable);
  cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
}
