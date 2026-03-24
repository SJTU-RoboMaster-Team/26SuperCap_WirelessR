//
// Created by yixin on 2024/5/3.
//

#include "calibrator_estimator.h"
#include "music/music_player.h"
#include "flash_memory_manager.h"

extern MusicPlayer music_player;
extern FlashMemoryManager flash;

bool Calibrator::judge_if_reasonable(CALIBRATE_PARAMETER_FLASH_STORAGE s){
  using namespace my_math;

  if (s.SOF!=CALIBRATE_PARAMETER_FLASH_STORAGE_SOF)return false;
  if (!near(s.in_voltage_scale,1,0.1)||
      !near(s.in_voltage_offset,0,0.2)||
      !near(s.cap_current_scale,1,0.1)||
      !near(s.cap_current_offset,0,0.2)||
      !near(s.cap_voltage_scale,1,0.1)||
      !near(s.cap_voltage_offset,0,0.2)||
      !near(s.out_current_scale,1,0.1)||
      !near(s.out_current_offset,0,0.2)||
      !near(s.in_current_scale,1,0.1)||
      !near(s.in_current_offset,0,0.2)
  ){return false;}
  return true;
}

void Calibrator::load_calibrate_parameter(CALIBRATE_PARAMETER_FLASH_STORAGE s) {
  if (s.SOF != CALIBRATE_PARAMETER_FLASH_STORAGE_SOF) return;
  calibrate_parameter.in_voltage_offset = s.in_voltage_offset;
  calibrate_parameter.in_voltage_scale = s.in_voltage_scale;
  calibrate_parameter.in_current_offset = s.in_current_offset;
  calibrate_parameter.in_current_scale = s.in_current_scale;
  calibrate_parameter.out_current_offset = s.out_current_offset;
  calibrate_parameter.out_current_scale = s.out_current_scale;
  calibrate_parameter.cap_voltage_offset = s.cap_voltage_offset;
  calibrate_parameter.cap_voltage_scale = s.cap_voltage_scale;
  calibrate_parameter.cap_current_offset = s.cap_current_offset;
  calibrate_parameter.cap_current_scale = s.cap_current_scale;

}

CALIBRATE_PARAMETER_FLASH_STORAGE Calibrator::save_flash_parameter(uint16_t id) const {
  CALIBRATE_PARAMETER_FLASH_STORAGE calibration_flash_storage;
  calibration_flash_storage.SOF = CALIBRATE_PARAMETER_FLASH_STORAGE_SOF;
  calibration_flash_storage.id = id;
  calibration_flash_storage.in_voltage_offset = calibrate_parameter.in_voltage_offset;
  calibration_flash_storage.in_voltage_scale = calibrate_parameter.in_voltage_scale;
  calibration_flash_storage.in_current_offset = calibrate_parameter.in_current_offset;
  calibration_flash_storage.in_current_scale = calibrate_parameter.in_current_scale;
  calibration_flash_storage.out_current_offset = calibrate_parameter.out_current_offset;
  calibration_flash_storage.out_current_scale = calibrate_parameter.out_current_scale;
  calibration_flash_storage.cap_voltage_offset = calibrate_parameter.cap_voltage_offset;
  calibration_flash_storage.cap_voltage_scale = calibrate_parameter.cap_voltage_scale;
  calibration_flash_storage.cap_current_offset = calibrate_parameter.cap_current_offset;
  calibration_flash_storage.cap_current_scale = calibrate_parameter.cap_current_scale;
  return calibration_flash_storage;
}

float Calibrator::calibrate_in_voltage(float voltage) const {
  return voltage * calibrate_parameter.in_voltage_scale + calibrate_parameter.in_voltage_offset;
}

float Calibrator::calibrate_in_current(float current) const {
  return current * calibrate_parameter.in_current_scale + calibrate_parameter.in_current_offset;
}

float Calibrator::calibrate_out_current(float current) const {
  return current * calibrate_parameter.out_current_scale + calibrate_parameter.out_current_offset;
}

float Calibrator::calibrate_cap_voltage(float voltage) const {
  return voltage * calibrate_parameter.cap_voltage_scale + calibrate_parameter.cap_voltage_offset;
}

float Calibrator::calibrate_cap_current(float current) const {
  return current * calibrate_parameter.cap_current_scale + calibrate_parameter.cap_current_offset;
}

void Calibrator::handle() {
  using namespace my_math;
  switch (calibrate_step) {
  case NONE:
    break;
      // step1: calibrate input voltage with 24 volt, using digit power supply
  case ZERO_LOAD:
    if (filtered_in_voltage.get_write_times()>50 && near(filtered_in_voltage.get(),24,1)){
      calibrate_parameter.in_voltage_offset = 24 - filtered_in_voltage.get();
      calibrate_step = I_STEP_LOAD;
      music_player.play_one_note({H7,50});
    }
    break;
      // step2: calibrate input&output current from 0 to 4 using digit load, step = 1
  case I_STEP_LOAD:
    static float step_i_value = 0;

    if (step_i_value<4) {
      if (near(filtered_out_current.get(), (float) step_i_value, 0.25)
          &&near(filtered_in_current.get(), (float) step_i_value, 0.25)) {
        if (count_for_stable>0){
          count_for_stable--;
          return;
        }
      }
      if (count_for_stable <=0){
        lrm.add_point(filtered_out_current.get(), (float) step_i_value);
        lrm2.add_point(filtered_in_current.get(), (float) step_i_value+0.03f);// input current is usually 0.03+I out
        step_i_value += 1;
        count_for_stable=50000; // 25000为1s
        filtered_out_current.reset();
        filtered_in_current.reset();
      }
    } else{
      lrm.regress();
      calibrate_parameter.out_current_offset = lrm.get_b();
      calibrate_parameter.out_current_scale = lrm.get_a();

      lrm2.regress();
      calibrate_parameter.in_current_offset = lrm2.get_b();
      calibrate_parameter.in_current_scale = lrm2.get_a();
      calibrate_step = I_CAP_STEP_LOAD;
      music_player.play_one_note({H7,50});

      lrm.reset();
      lrm2.reset();
      step_i_value = 0;
    }
    break;
      // step3: calibrate cap current from 0 to 4 using digit load, step = 1; bbm start
  case I_CAP_STEP_LOAD:

    if (step_i_value<4) {
      if (near(filtered_cap_current.get(), (float) step_i_value, 0.25)) {
        if (count_for_stable>0){
          count_for_stable--;
          return;
        }
      }
      if (count_for_stable <=0){
        lrm.add_point(filtered_cap_current.get(), (float) step_i_value);
        step_i_value += 1;
        count_for_stable=50000;
        filtered_cap_current.reset();
      }
    } else{
      lrm.regress();
      calibrate_parameter.cap_current_offset = lrm.get_b();
      calibrate_parameter.cap_current_scale = lrm.get_a();

      calibrate_step = STEP_V;
      music_player.play_one_note({H7,50});

      lrm.reset();
    }
    break;

  case STEP_V:
      // calibrate input and cap voltage by connecting out and cap; bbm is stopped
    static float step_v_value = 14;
    //            static LinearRegressionModel lrm_vcap;
    if (step_v_value<27) {
      if (my_math::near(filtered_cap_voltage.get(), (float) step_v_value, 1) &&
          my_math::near(filtered_in_voltage.get(), (float) step_v_value, 1)) {
        if (count_for_stable>0){
          count_for_stable--;
          return;
        }
      }
      if (count_for_stable <=0){
        lrm.add_point(filtered_cap_voltage.get(), (float) step_v_value);// input current is usually 0.03+I out
        lrm2.add_point(filtered_in_voltage.get(), (float) step_v_value);// input current is usually 0.03+I out
        step_v_value += 4;
        count_for_stable=50000;
        filtered_cap_voltage.reset();
        filtered_in_voltage.reset();
      }
    } else{
      lrm.regress();
      calibrate_parameter.cap_voltage_offset = lrm.get_b();
      calibrate_parameter.cap_voltage_scale = lrm.get_a();

      lrm2.regress();
      calibrate_parameter.in_voltage_offset = lrm2.get_b();
      calibrate_parameter.in_voltage_scale = lrm2.get_a();

      calibrate_step = STORAGE;
      music_player.play_one_note({H7,50});
    }
    break;
  case STORAGE:
    flash.save_calibrate_parameter(save_flash_parameter(3));
    flash.flash_calibrate_parameter();
    calibrate_step = DONE;
  case DONE:
  default:
    break;
  }

}


