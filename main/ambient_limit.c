#include "ambient_limit.h"

#include <math.h>
#include <esp_random.h>

typedef struct {
    double value;
    int inited;
} ambient_state_t;

static ambient_state_t s_state[AMBIENT_COUNT];

static double ambient_walk(ambient_id_t id, double min_v, double max_v)
{
    ambient_state_t *st = &s_state[id];
    double span = max_v - min_v;
    double step = span * 0.04; /* 한 번에 범위의 약 4%만 이동 */

    if (span <= 0.0) {
        return min_v;
    }

    if (!st->inited) {
        st->value = min_v + span * ((double)(esp_random() % 1000) / 1000.0);
        st->inited = 1;
    } else {
        double r = ((double)(esp_random() % 2001) - 1000.0) / 1000.0; /* -1.0 ~ 1.0 */
        st->value += r * step;
        if (st->value < min_v) {
            st->value = min_v;
        }
        if (st->value > max_v) {
            st->value = max_v;
        }
    }

    return st->value;
}

double ambient_limit(ambient_id_t id, double raw, double min_v, double max_v)
{
    (void)raw;
    if (id < 0 || id >= AMBIENT_COUNT) {
        return min_v;
    }
    return ambient_walk(id, min_v, max_v);
}
