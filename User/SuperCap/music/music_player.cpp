//
// Created by yixin on 2024/1/26.
//

#include "music_player.h"

void MusicPlayer::play_one_note(MusicNote note) {
    // calc the frequency
    // 170M / 170 = 1Mhz

    static uint32_t arr = 0,ccr = 0;
    if(note.note == 0){
         ccr = 0;
    } else{
        arr = 1000000 / note.note;
        ccr = arr / 10;
    }
    __HAL_TIM_SET_AUTORELOAD(&htim,arr);
    __HAL_TIM_SET_COMPARE(&htim,TIM_CHANNEL_1,ccr);
    __HAL_TIM_SET_COUNTER(&htim,0);
    if (!enable){
        start();
    }
    notes_left = 1;
    count_down_to_next_note = note.time;
    this_note = note;
}

void MusicPlayer::handle() {
    if (!enable){
        stop();
        return;
    }
    tick++;
    if(notes_left == 0){
        return;
    }
    count_down_to_next_note--;

    if(count_down_to_next_note> 0){
        return;
    }

    notes_left--;
    if(notes_left == 0){
        if(warning_info.warning){
            if(warning_info.last_note){
                warning_info.last_note = false;
                play_one_note({0,5});
            }else{
                play_one_note({H5,5});
                warning_info.last_note = true;
            }
        }else{
            stop();
        }
    }
}

void MusicPlayer::start_warning(uint16_t note) {
    if(warning_info.warning){
        return;
    }
    warning_info.warning = true;
    warning_info.last_note = true;
    play_one_note({note,100});
}
