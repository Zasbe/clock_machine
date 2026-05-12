#include <stdio.h>
#include <stdint.h>

float BPM_STEP= 1;
float init_bpm = 120;
float period_step;
float init_period;
float new_bpm;
float new_period;

int main(void){
    init_period = 60000000 / init_bpm;
    period_step = 60000000 / BPM_STEP;
    new_period = init_period + period_step;
    new_bpm = (60000000 / new_period);
    printf("this is initial period %f\n", init_period);
    printf("this is the initial bpm %f\n", init_bpm);
    printf("period of 0.5 BPM : %f\n", period_step);
    printf("this is new period %f\n", new_period);
    printf("this new bpm: %f\n", new_bpm);

  return 0;
}
