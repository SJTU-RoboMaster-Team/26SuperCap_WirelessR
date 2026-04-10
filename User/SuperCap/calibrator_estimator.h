//
// Created by yixin on 2024/5/3.
//

#ifndef SUPER_CAP_CALIBRATOR_ESTIMATOR_H
#define SUPER_CAP_CALIBRATOR_ESTIMATOR_H

#include "types.h"
#include "math_tool/my_math.h"
#include "flash_data_structure.h"

class Calibrator {
public:
  void handle();

  bool judge_if_reasonable(CALIBRATE_PARAMETER_FLASH_STORAGE s);
  void load_calibrate_parameter(CALIBRATE_PARAMETER_FLASH_STORAGE s);

  CALIBRATE_PARAMETER_FLASH_STORAGE save_flash_parameter(uint16_t id) const;

  float calibrate_in_voltage(float voltage) const;
  float calibrate_in_current(float current) const;
  float calibrate_out_current(float current) const;
  float calibrate_cap_voltage(float voltage) const;
  float calibrate_cap_current(float current) const;

  AverageFilter<20> filtered_in_voltage;
  AverageFilter<20> filtered_in_current;
  AverageFilter<20> filtered_out_current;
  AverageFilter<20> filtered_cap_voltage;
  AverageFilter<20> filtered_cap_current;

  LinearRegressionModel lrm;
  LinearRegressionModel lrm2;


  enum CALIBRATE_STEP{
    NONE,
    ZERO_LOAD,
    I_STEP_LOAD,
    I_CAP_STEP_LOAD,
    STEP_V,
    STORAGE,
    DONE,
  }calibrate_step = NONE;

  void start_calibrate(){
    if (calibrate_step == NONE){
      calibrate_step = ZERO_LOAD;
    }
  }

private:

  struct {
    float in_voltage_offset = 0;
    float in_voltage_scale = 1;

    float in_current_offset = 0;
    float in_current_scale = 1;

    float out_current_offset = 0;
    float out_current_scale = 1;

    float cap_voltage_offset = 0;
    float cap_voltage_scale = 1;

    float cap_current_offset = 0;
    float cap_current_scale = 1;
  }calibrate_parameter;

  int32_t count_for_stable = 10000;

};

#endif //SUPER_CAP_CALIBRATOR_ESTIMATOR_H
