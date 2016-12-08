#ifndef FILTER_H
#define FILTER_H

#include<math.h>

float Kaerman_Filter(float data_filted, float radian, float radian_pt);
float Kalman_Filter(float measure_value, float measure_cur_err, float previous_best, float pre_best_err, float current_fore_err);

#endif
