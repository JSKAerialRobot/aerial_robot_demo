#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

std::string folder_path = "/home/kuromiya/";

// if graspable -> (angle, 1) else -> (angle, 0) if reached -> (angle, 1, pointx1, pointy1, pointx2, pointy2)
std::vector<float> space_detector_ground(cv::Mat src, cv::Mat depth_org, double target_distance_z){
  std::vector<float> answer;
  int graspable = 0;
  // cv::imwrite("/home/kuromiya/debug.png", src);
  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bw;
  cv::threshold(gray, bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  std::vector<cv::Point>  contour;

  cv::Mat depth;

  cv::cvtColor(depth_org, depth, cv::COLOR_GRAY2BGR);  // depth : color

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
  int width = 0;
  int height = 0;

  try{
    box = cv::minAreaRect(contour);
  }
  catch(cv::Exception &ex){
    ROS_WARN("%s", ex.what());
    answer.push_back(0.0);
    answer.push_back(0);
    return answer;
  }

  float angle = box.angle;

  cv::Mat thresh;
  int length = 0;
  cv::Mat rotate;

  int center_x = box.center.x;
  int center_y = box.center.y;
  
  if(std::abs(angle) >= 0 && std::abs(90 - std::abs(angle)) >= 0){
    // not perpendicular to the target object
    cv::Mat rotate_org;
    cv::Mat transform_m = cv::getRotationMatrix2D(cv::Point(depth.cols/2, depth.rows/2), angle, 1);
    uint8_t data_init[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    cv::warpAffine(depth, rotate, transform_m, depth.size());
    cv::warpAffine(src, rotate_org, transform_m, depth.size());
    // cv::imwrite("/home/kuromiya/debug/rotate.png", rotate);
    // cv::imwrite("/home/kuromiya/debug/rotate_org.png", rotate_org);

    angle = M_PI * angle / 180;  // degree -> radian
    angle = -angle;  //  reverse(camera -> hydrus)
    center_x = (center_x - depth.cols/2) * std::cos(angle) - (center_y - depth.rows/2) * std::sin(angle) + depth.cols/2; 
    center_y =(center_x - depth.cols/2) * std::sin(angle) + (center_y - depth.rows/2) * std::cos(angle) + depth.rows/2;
      

    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    // cv::imwrite("/home/kuromiya/center_img.png", rotate);

    angle = angle * 180 / M_PI;
  }

  // else{  
  //   // box = cv::minAreaRect(contour);

  //   angle = box.angle;
  //   width = box.size.width;
  //   height = box.size.height;

  //   box1 = box;

  //   rotate = depth;
  // }

  box1 = box;

  width = box1.size.width;
  height = box1.size.height;

  cv::imwrite(folder_path + "rotate.png", rotate);
  cv::Mat rotating = cv::imread(folder_path + "rotate.png", 1); // have to write -> read

  std::cout << "angle : " << angle  << std::endl;
  std::cout << "height : " << height << std::endl;
  std::cout << "width : " << width << std::endl;

  length = std::max(width, height);
  // cv::imwrite("/home/kuromiya/cut_org.png", depth);


  std::cout << "x : " << box1.center.x << std::endl;
  std::cout << "y : " << box1.center.y << std::endl;
  std::cout << "length : " << length << std::endl;
  
  //  revised cut
  cv::Mat cut(cv::Size(4*length, 4*length), CV_8UC3, cv::Scalar(255, 255, 255));  // initialize with black(meaning : ungraspable area)

  for(int i = 0; i < 4*length; i++){
    int yy = (-2)*length+center_y + i;
    if(yy < 0 || yy > rotating.rows){
      continue;
    }

    cv::Vec3b * th = cut.ptr<cv::Vec3b>(i);
    cv::Vec3b * org = rotating.ptr<cv::Vec3b>(yy);

    for(int j = 0; j < 4*length; j++){
      int xx = (-2)*length+center_x + j;
      if(xx < 0 || xx > rotating.cols){
        continue;
    }
      if(org[xx][0] <=100 && org[xx][1] <= 100 && org[xx][2] <= 100){  // black(outlier) -> white
	th[j][0] = 255;
	th[j][1] = 255;
	th[j][2] = 255;
      }
      else{
	th[j][0] = org[xx][0];
	th[j][1] = org[xx][1];
	th[j][2] = org[xx][2];
      }
    }
  }


  cv::imwrite(folder_path + "correct_cut.png", cut);
  cv::Mat cutt = cv::imread(folder_path + "correct_cut.png", 1);

  // int c = cutt.at<cv::Vec3b>(2*length, 2*length)[0];

  int c = 140;  // threshold
  for(int i = 2 *length - 5; i < 2 * length + 5; i++){
    cv::Vec3b * th = cutt.ptr<cv::Vec3b>(i);
    for(int j = 2 * length - 5; j < 2 * length + 5; j++){
      cv::Vec3b t = th[j];
      if(t[0] != 0 && t[0] != 255){  // exclude outliers
        c = t[0];
	// cv::circle(cutt, cv::Point(j, i), 3, cv::Scalar(0, 0, 255), 10);
	// cv::imwrite("/home/kuromiya/fuck.png", cutt);
        break;
      }
    }
  }
  
  std::cout << "threshold : " << c << std::endl;
  int object_depth = 0.3;
  int thresh_margin = object_depth * c / target_distance_z;
  cv::threshold(cutt, thresh, c + thresh_margin, 255, cv::THRESH_BINARY);

//  deleted

  // check whether hydrus can grasp the object

  int left_upper_count = 0;
  int left_lower_count = 0;
  int right_upper_count = 0;
  int right_lower_count = 0;

  int left_count = 0;
  int right_count = 0;
  int down_count = 0;
  int up_count = 0;
  
  int black_count = 0;

  // only focus on the same height of the target object
  for(int i = 1.5 * length; i < 2.5 * length; i++){
    cv::Vec3b * th = thresh.ptr<cv::Vec3b>(i);
    for(int j = 0; j < thresh.cols; j++){
      cv::Vec3b t = th[j];

      if(int(t[0]) == 0){
	black_count++;

        if(i < 1.0 * length && j < 1.0 * length){
          left_upper_count++;
        }
        else if(i > 3.0 * length && j < 1.0 * length){
          left_lower_count++;
        }
        else if(i < 1.0 * length && j > 3.0 * length){
          right_upper_count++;
        }
        else if(i > 3.0 * length && j > 3.0 * length){
          right_lower_count++;
        }
	else if (j < 1.0 * length){
	  left_count++;
	}
	else if (j > 3.0 * length){
	  right_count++;
	}
	else if (i < 1.0 * length){
	  up_count++;
	}
	else if (i > 3.0 * length){
	  down_count++;
	}	
      }
    }
  }

  
  std::cout << "black count : " << black_count << std::endl;
  

  std::cout << "left_upper : " << left_upper_count << std::endl;
  std::cout << "left_lower : " << left_lower_count << std::endl;
  std::cout << "right_upper : " << right_upper_count << std::endl;
  std::cout << "right_lower : " << right_lower_count << std::endl;

  std::cout << "left : " << left_count << std::endl;
  std::cout << "right : " << right_count << std::endl;
  std::cout << "up : " << up_count << std::endl;
  std::cout << "down : " << down_count << std::endl;
  
  int limit = 2000;


  if(right_count < limit && left_count < limit){
    graspable = 1;
    answer.push_back(angle);
    answer.push_back(graspable);
    std::cout << "straight" << std::endl;
    cv::putText(thresh, "straight", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    // cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }

  if(right_count < limit){
    graspable = 1;
    answer.push_back(angle+90);
    answer.push_back(graspable);
    std::cout << "right" << std::endl;
    cv::putText(thresh, "right", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    // cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }

  if(left_count < limit){
    graspable = 1;
    answer.push_back(angle-90);
    answer.push_back(graspable);
    std::cout << "left" << std::endl;
    cv::putText(thresh, "left", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    // cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }


  answer.push_back(graspable);
  // cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
}
