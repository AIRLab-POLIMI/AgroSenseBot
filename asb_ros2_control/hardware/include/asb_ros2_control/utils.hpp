//
// Created by enrico on 12/09/23.
//

#ifndef ASB_ROS2_CONTROL_UTILS_H
#define ASB_ROS2_CONTROL_UTILS_H

#include <chrono>

//double chrono_duration_to_ms(std::chrono::steady_clock::time_point t1, std::chrono::steady_clock::time_point t2){
//
//  auto d = t2 - t1;
//  return (int)(std::chrono::duration_cast<std::chrono::microseconds>(d).count())/1e3;
//}

double chrono_duration_to_s(std::chrono::steady_clock::time_point t1, std::chrono::steady_clock::time_point t2){
  auto d = t2 - t1;
  return (int)(std::chrono::duration_cast<std::chrono::milliseconds>(d).count())/1e3;
}

#endif //ASB_ROS2_CONTROL_UTILS_H
