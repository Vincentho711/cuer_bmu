#ifndef BMU_H
#define BMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct ivt_state {
  int current;
  int voltage1;
  int voltage2;
  int voltage3;
  int temperature;
  int power;
  int charge;
  int energy;
} ivt_state_t;

typedef struct bmu_state {
    bool over_current;
    bool under_voltage;
    bool over_voltage;
    bool under_temperature;
    bool over_temperature;
    bool safe_to_drive;
    bool charging_state;
    bool precharge_state;
    bool discharge_state;
    bool contactor_state;
    uint8_t fan1_state;
    uint8_t fan2_state;
    uint8_t fan3_state;
    uint8_t fan4_state;
} bmu_state_t;

#endif