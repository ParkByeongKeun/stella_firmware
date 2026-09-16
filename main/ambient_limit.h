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
    AMBIENT_CH2O,
    AMBIENT_COUNT
} ambient_id_t;

/*
 * 첫 부팅 5분: 실측이 정상범위(min~max)면 그 값을 그대로 쓰고,
 * 범위 밖이면 중앙값(0.597 같이 소수 3자리)만 보여 주며 기준을 잡는다.
 * O3만 실측이 나오기 전 표시 구간을 0.010~0.019로 둔다.
 * 기준이 이미 정상범위면 이후에도 변환하지 않는다.
 * 범위 밖이면 NVS에 기준을 저장하고, 5분 뒤부터 비율(가중치 0.6)을 적용한다.
 */
double ambient_limit(ambient_id_t id, double raw, double min_v, double max_v);

#ifdef __cplusplus
}
#endif
