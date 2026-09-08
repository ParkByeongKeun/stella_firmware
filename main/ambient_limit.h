#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AMBIENT_H2S = 0,
    AMBIENT_NH3,
    AMBIENT_NO2,
    AMBIENT_O3,
    AMBIENT_CO,
    AMBIENT_CH2O_PPB,
    AMBIENT_COUNT
} ambient_id_t;

double ambient_limit(ambient_id_t id, double raw, double min_v, double max_v);

#ifdef __cplusplus
}
#endif
