#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <numeric>
#include <jsk_recognition_utils/geo/polygon.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>

using namespace std;
using namespace cv;

// (Polygon, depth, robot_height, depth_paremeter) -> (graspable, angle)  (angle : camera_coordinate)

std::vector<float> space_detector(jsk_recognition_utils::Polygon line_img_polygon, cv::Mat depth_org, double robot_height, jsk_recognition_utils::CameraDepthSensor camdep){
  std::vector<float> answer;
  int graspable = 0; // whether possible to grasp or not
  std::string image_folder = "/home/leus";  //  images will be saved in this directory(necessary)

  cv::Mat src(depth_org.rows, depth_org.cols, CV_8UC3, cv::Scalar(255, 255, 255));          
  line_img_polygon.drawLineToImage(camdep, src, cv::Scalar(0, 255, 0), 1);

  cv::Mat gray;
  cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
  cv::Mat bw;
  cv::threshold(gray, bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  std::vector<cv::Point> contour;
  cv::Mat depth;

  cv::cvtColor(depth_org, depth, cv::COLOR_GRAY2BGR);  // depth : color


  // get contour points
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
  
  int width = 0;
  int height = 0;

  //  get rect (if impossible -> fail)
  try{
    box = cv::minAreaRect(contour);
  }
  catch(cv::Exception &ex){
    ROS_WARN("%s", ex.what());
    answer.push_back(0);
    answer.push_back(0.0);
    return answer;
  }

  float angle = box.angle;

  cv::Mat thresh;
  int length = 0;
  cv::Mat rotate;

  int center_x = box.center.x;
  int center_y = box.center.y;
  
  cv::circle(src, cv::Point(center_x, center_y), 10, cv::Scalar(0, 0, 255), 10);
  //  rotate image so that the contour is horizontal to the image 
  cv::Mat rotate_org;
  cv::Mat transform_m = cv::getRotationMatrix2D(cv::Point(depth.cols/2, depth.rows/2), angle, 1);

  angle = M_PI * angle / 180;  // degree -> radian

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

  //  contour center atfer rotation
  center_x = int(std::accumulate(red_x.begin(), red_x.end(), 0) / red_x.size());
  center_y = int(std::accumulate(red_y.begin(), red_y.end(), 0) / red_y.size());

  std::cout << "center_x :" << center_x << std::endl;
  std::cout << "center_y :" << center_y << std::endl;
  
  cv::circle(rotate, cv::Point(center_x, center_y), 10, cv::Scalar(255, 0, 0), 10);
  
  angle = -angle;  //  reverse(camera -> hydrus)
  angle = angle * 180 / M_PI;

  width = box.size.width;
  height = box.size.height;

  // have to write -> read (you will get strange image without this)

  cv::imwrite(image_folder + "/rotate.png", rotate);
  cv::Mat rotating = cv::imread(image_folder + "/rotate.png", 1);
  length = std::max(width, height);

  //  revised cut
  cv::Mat cut(cv::Size(4*length, 4*length), CV_8UC3, cv::Scalar(255, 255, 255));  // initialize with black(black area : object existing area)

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

  // have to write -> read (you will get strange image without this)
  cv::imwrite(image_folder + "/correct_cut.png", cut);
  cv::Mat cutt = cv::imread(image_folder + "/correct_cut.png", 1);


  int c = 0;  // threshold pixel value(around the center of contour)
  for(int i = 2 *length - 5; i < 2 * length + 5; i++){
    cv::Vec3b * th = cutt.ptr<cv::Vec3b>(i);
    for(int j = 2 * length - 5; j < 2 * length + 5; j++){
      cv::Vec3b t = th[j];
      if(t[0] != 0 && t[0] != 255){  // exclude outliers
        c = t[0];
        break;
      }
    }
  }

  
  std::cout << "threshold : " << c << std::endl;
  cv::threshold(cutt, thresh, c + (255 / robot_height) * 0.2, 255, cv::THRESH_BINARY);  // 12 = 255 / x * 0.2 (robot_height : x[m], height : 0.2[m])

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
      
      if(int(t[0]) == 0){
	black_count++;
        // detect which direction is best(with a little bit margin)
      
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


  int object_exist = 2000; //  number of black pixel in each region exceeds this -> exist

  std::vector<int> possible_answer; // 0:left, 1:right, 2:up, 3:down
  possible_answer.push_back(0);
  possible_answer.push_back(1);
  possible_answer.push_back(2);
  possible_answer.push_back(3);
  
  int delete_times = 0;
  
  if(left_count > object_exist){
    std::cout << "left" << std::endl;
    if(delete_times == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);
    }
    delete_times++;
  }

  
  if(right_count > object_exist){
    std::cout << "right" << std::endl;

    if(delete_times == 0){
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);
      possible_answer.erase(possible_answer.begin() + 1);
    }
    delete_times++;
  }

  
  if(up_count > object_exist){
    std::cout << "up" << std::endl;

    if(delete_times == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
    }
    delete_times++;
  }
  
  if(down_count > object_exist){
    std::cout << "down" << std::endl;
    
    if(delete_times == 0){
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 0);
      possible_answer.erase(possible_answer.begin() + 1);
    }
    delete_times++;    
  }
  
  std::cout << "delete times : "<< delete_times << std::endl;
  
  if(delete_times > 1){ // multiple object exists -> unable to grasp
    answer.push_back(graspable);
    answer.push_back(0.0);
    
    return answer;
  }

  if(left_lower_count < object_exist && right_lower_count < object_exist && std::find(possible_answer.begin(), possible_answer.end(), 3) != possible_answer.end()){
    //  down
    graspable = 1;
    std::cout << "down" << std::endl;
    answer.push_back(graspable);
    answer.push_back(angle+90);
    
    return answer;
  }

  if(left_upper_count < object_exist && right_upper_count < object_exist && std::find(possible_answer.begin(), possible_answer.end(), 2) != possible_answer.end()){
    //  up
    graspable = 1;
    answer.push_back(graspable);
    answer.push_back(angle+270);
    std::cout << "up" << std::endl;
    
    return answer;
  }

  if(left_upper_count < object_exist && left_lower_count < object_exist && std::find(possible_answer.begin(), possible_answer.end(), 0) != possible_answer.end()){
    //  left
    graspable = 1;
    answer.push_back(graspable);
    answer.push_back(angle+0);
    std::cout << "left" << std::endl;

    return answer;
  }

  if(right_upper_count < object_exist && right_lower_count < object_exist &&std::find(possible_answer.begin(), possible_answer.end(), 1) != possible_answer.end()){
    //  right
    graspable = 1;
    answer.push_back(graspable);
    answer.push_back(angle+180);
    std::cout << "right" << std::endl;

    return answer;
  }

  // unable to grasp
  answer.push_back(graspable);
  answer.push_back(0.0);
  
  return answer;
}
