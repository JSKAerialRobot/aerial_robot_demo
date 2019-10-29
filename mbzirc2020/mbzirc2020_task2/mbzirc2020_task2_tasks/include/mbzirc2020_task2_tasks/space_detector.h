#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <numeric>

using namespace std;
using namespace cv;


// if graspable -> (angle, 1) else -> (angle, 0) if reached -> (angle, 1, pointx1, pointy1, pointx2, pointy2)
std::vector<float> space_detector(cv::Mat src, cv::Mat depth_org){
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

  cv::circle(src, cv::Point(center_x, center_y), 10, cv::Scalar(0, 0, 255), 10);
  
  if(std::abs(angle) >= 0 && std::abs(90 - std::abs(angle)) >= 0){
    // not perpendicular to the target object
    cv::Mat rotate_org;
    cv::Mat transform_m = cv::getRotationMatrix2D(cv::Point(depth.cols/2, depth.rows/2), angle, 1);

    angle = M_PI * angle / 180;  // degree -> radian
    
    center_x = (center_x - depth.cols/2) * std::cos(angle) + (center_y - depth.rows/2) * std::sin(angle) + depth.cols/2; 
    center_y = (center_y - depth.rows/2) * std::cos(angle) + ((-1) * center_x + depth.cols/2) * std::sin(angle)  + depth.rows/2;

    cv::warpAffine(depth, rotate, transform_m, depth.size());
    cv::warpAffine(src, rotate_org, transform_m, depth.size());

    std::vector<int> red_x;
    std::vector<int> red_y;
    
    for(int i = 0; i < rotate_org.rows; i++){
      cv::Vec3b * th = rotate_org.ptr<cv::Vec3b>(i);
      for(int j = 0; j < rotate_org.cols; j++){
	cv::Vec3b t = th[j];
	if(t[0] == 0 && t[1] == 0 && t[2] == 255){
	  red_x.push_back(j);
	  red_y.push_back(i);
	}
      }
    }

    center_x = int(std::accumulate(red_x.begin(), red_x.end(), 0) / red_x.size());
    center_y = int(std::accumulate(red_y.begin(), red_y.end(), 0) / red_y.size());
    
    cv::imwrite("/home/kuromiya/debug/rotate.png", rotate);
    cv::imwrite("/home/kuromiya/debug/rotate_org.png", rotate_org);
 
    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/center_img_org.png", rotate);

    angle = -angle;  //  reverse(camera -> hydrus)
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

  cv::imwrite("/home/kuromiya/rotate.png", rotate);
  cv::Mat rotating = cv::imread("/home/kuromiya/rotate.png", 1); // have to write -> read

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


  cv::imwrite("/home/kuromiya/correct_cut.png", cut);
  cv::Mat cutt = cv::imread("/home/kuromiya/correct_cut.png", 1);

  // int c = cutt.at<cv::Vec3b>(2*length, 2*length)[0];

  int c = 140;  // threshold
  for(int i = 2 *length - 5; i < 2 * length + 5; i++){
    cv::Vec3b * th = cutt.ptr<cv::Vec3b>(i);
    for(int j = 2 * length - 5; j < 2 * length + 5; j++){
      cv::Vec3b t = th[j];
      if(t[0] != 0 && t[0] != 255){  // exclude outliers
        c = t[0];
	// cv::circle(cutt, cv::Point(j, i), 3, cv::Scalar(0, 0, 255), 10);
        break;
      }
    }
  }
  
  std::cout << "threshold : " << c << std::endl;
  cv::threshold(cutt, thresh, c + 30, 255, cv::THRESH_BINARY);  // 12 = 255 / 4.0 * 0.2 (altitude : 4.0[m], height : 0.2[m])

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
  for(int i = 0; i < thresh.rows; i++){
    cv::Vec3b * th = thresh.ptr<cv::Vec3b>(i);
    for(int j = 0; j < thresh.cols; j++){
      cv::Vec3b t = th[j];
      // if(i >= 0 && j >= 0){
      // 	std::cout << "abcdefuck : " << int(t[0]) << std::endl;
      // }
      if(int(t[0]) == 0){
	black_count++;
        // detect which direction is best(with a little bit margin)
        // if(std::abs(i - 2 * length) < 0.6 * length && std::abs(j - 2 * length) < 0.6 * length){
        //   continue;
        // }
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

  std::vector<int> possible_answer; // 0:left, 1:right, 2:up, 3:down
  possible_answer.push_back(0);
  possible_answer.push_back(1);
  possible_answer.push_back(2);
  possible_answer.push_back(3);
  
  int delete_time = 0;

  std::cout << "OK1" << std::endl;
  
  if(left_count > limit){
    std::cout << "left" << std::endl;
    if(delete_time == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);

      std::cout << possible_answer[0] << ":left" <<std::endl;
    }
    delete_time++;
  }

  std::cout << "OK2" << std::endl;
  
  if(right_count > limit){
    std::cout << "right" << std::endl;

    if(delete_time == 0){
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);
    }
    delete_time++;
  }

  std::cout << "OK3" << std::endl;
  
  if(up_count > limit){
    std::cout << "up" << std::endl;

    if(delete_time == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
    }
    delete_time++;
  }
  std::cout << "OK4" << std::endl;
  
  if(down_count > limit){
    std::cout << "down" << std::endl;
    
    if(delete_time == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 1);
    }
    delete_time++;    
  }

  std::cout << "OK5" << std::endl;
  
  std::cout << delete_time << std::endl;
  
  if(delete_time > 1){
    answer.push_back(graspable);
    cv::imwrite("/home/kuromiya/threshhh.png", thresh);
    return answer;
  }

  std::cout << "OK6" << std::endl;
  if(left_lower_count < limit && right_lower_count < limit && std::find(possible_answer.begin(), possible_answer.end(), 3) != possible_answer.end()){
    //  down
    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/center_img.png", rotate);
    graspable = 1;
    answer.push_back(angle+0);
    std::cout << "down" << std::endl;
    answer.push_back(graspable);
    cv::putText(thresh, "down", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 3);
    std::string ri = std::to_string(right_lower_count);
    // cv::putText(thresh, ri, cv::Point(100, 200), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 3);
    cv::imwrite("/home/kuromiya/thresh.png", thresh);
    
    return answer;
  }

  if(left_upper_count < limit && right_upper_count < limit && std::find(possible_answer.begin(), possible_answer.end(), 2) != possible_answer.end()){
    //  up
    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/center_img.png", rotate);
    graspable = 1;
    answer.push_back(angle+180);
    answer.push_back(graspable);
    std::cout << "up" << std::endl;
    cv::putText(thresh, "up", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }

  if(left_upper_count < limit && left_lower_count < limit && std::find(possible_answer.begin(), possible_answer.end(), 0) != possible_answer.end()){
    //  left
    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/center_img.png", rotate);
    graspable = 1;
    answer.push_back(angle-90);
    answer.push_back(graspable);
    std::cout << "left" << std::endl;
    cv::putText(thresh, "left", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }

  if(right_upper_count < limit && right_lower_count < limit &&std::find(possible_answer.begin(), possible_answer.end(), 1) != possible_answer.end()){
    //  right
    cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/center_img.png", rotate);
    graspable = 1;
    answer.push_back(angle+90);
    answer.push_back(graspable);
    std::cout << "right" << std::endl;
    cv::putText(thresh, "right", cv::Point(100, 100), FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0), 10);
    cv::imwrite("/home/kuromiya/thresh.png", thresh);
    return answer;
  }


  answer.push_back(graspable);
  cv::imwrite("/home/kuromiya/thresh.png", thresh);
  return answer;
}
