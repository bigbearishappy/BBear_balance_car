#include"filter.h"

/************************************************************************************************ 
Name£ºKaerman_Filter 
Function:	
		  	the data filter
Parameters£º
		   	void
Returns£º
			void 
Description:
			it used a simple filter algorithum ---hubu
			later it will change to kalman filter
************************************************************************************************/
float Kaerman_Filter(float data_filted, float radian_filter, float radian_pt_fliter)//hubu filter
{
	data_filted = 0.01 * radian_filter + 0.99 * (radian_pt_fliter + data_filted);
	//data_filted += radian_pt_fliter;
	//data_filted = 0.05 * radian_filter + 0.95 * data_filted;
	return data_filted;
}

float Kalman_Filter(float measure_value, float measure_cur_err, float previous_best, float pre_best_err, float current_fore_err)
{
    float temp_forecast;				//temperature's forecast
    float temp_cur_fore_err;
    float covariance;

    temp_forecast = previous_best;
    temp_cur_fore_err = sqrt(pow(pre_best_err,2) + pow(current_fore_err,2));	

    covariance = pow(temp_cur_fore_err,2)/(pow(measure_cur_err,2) + pow(temp_cur_fore_err,2));

    if(measure_value <= temp_forecast)
    	temp_forecast = measure_value + covariance * (temp_forecast - measure_value);
    else
    	temp_forecast = temp_forecast + covariance * (measure_value - temp_forecast);

    return temp_forecast;

}

