//
// Created by yixin on 2024/1/26.
//

#ifndef SUPER_CAP_MUSIC_PLAYER_H
#define SUPER_CAP_MUSIC_PLAYER_H


#include "stm32g4xx_hal.h"

#define L1 262
#define L1U 277
#define L2 294
#define L2U 311
#define L3 330
#define L4 349
#define L4U 370
#define L5 392
#define L5U 415
#define L6 440
#define L6U 466
#define L7 494

#define M1 523
#define M1U 554
#define M2 587
#define M2U 622
#define M3 659
#define M4 698
#define M4U 740
#define M5 784
#define M5U 831
#define M6 880
#define M6U 932
#define M7 988

#define H1 1046
#define H1U 1109
#define H2 1175
#define H2U 1245
#define H3 1318
#define H4 1397
#define H4U 1480
#define H5 1568
#define H5U 1661
#define H6 1760
#define H6U 1865
#define H7 1976

struct MusicNote {
    uint16_t note;
    uint32_t time;
};


class MusicPlayer {
public:
    MusicPlayer(TIM_HandleTypeDef &htim) : htim(htim) {};

    void handle();

    void start() {
        enable = true;
        HAL_TIM_PWM_Start_IT(&htim, TIM_CHANNEL_1);
    }

    void stop() {
        enable = false;
        HAL_TIM_PWM_Stop_IT(&htim, TIM_CHANNEL_1);
        warning_info.warning = false;
    }

    void play_one_note(MusicNote note);

    void start_warning(uint16_t note);

    void stop_warning(){
        warning_info.warning = false;
    }

private:
    TIM_HandleTypeDef &htim;

    bool enable = false;

    uint32_t tick = 0; // 100hz
    uint32_t count_down_to_next_note = 0;
    uint32_t notes_left = 0;

    struct {
        bool warning = false;
        bool last_note = false;
    } warning_info;

    MusicNote this_note;
};


#endif //SUPER_CAP_MUSIC_PLAYER_H
