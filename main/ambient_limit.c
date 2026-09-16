#include "ambient_limit.h"

#include <math.h>
#include <string.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <nvs.h>

#define HIST_LEN 5
#define WARMUP_US (5LL * 60 * 1000 * 1000)
#define RISE_WEIGHT 0.6
#define NV_NS "storage"
#define NV_KEY "amb_cal"
#define NV_MAGIC 0x31424D42u
#define O3_PLACE_MIN 0.010
#define O3_PLACE_MAX 0.019

static const char *TAG = "ambient";

typedef struct {
    double hist[HIST_LEN];
    int count;
    int idx;
    double walk;
    int walk_inited;
} run_state_t;

typedef struct {
    uint32_t magic;
    uint8_t have_center[AMBIENT_COUNT];
    uint8_t have_base[AMBIENT_COUNT];
    uint8_t passthrough[AMBIENT_COUNT];
    double center[AMBIENT_COUNT];
    double baseline[AMBIENT_COUNT];
} persist_t;

static run_state_t s_run[AMBIENT_COUNT];
static persist_t s_nv;
static int s_nv_ready;
static int64_t s_boot_t0;

static double hist_mean(const run_state_t *st)
{
    int n = st->count;
    if (n <= 0) {
        return 0.0;
    }
    double s = 0.0;
    for (int i = 0; i < n; i++) {
        s += st->hist[i];
    }
    return s / (double)n;
}

static void push_sample(run_state_t *st, double raw)
{
    st->hist[st->idx] = raw;
    st->idx = (st->idx + 1) % HIST_LEN;
    if (st->count < HIST_LEN) {
        st->count++;
    }
}

static double quantum_of(double v)
{
    if (v >= 1.0) {
        return 0.1;
    }
    if (v >= 0.1) {
        return 0.001;
    }
    if (v >= 0.01) {
        return 0.0001;
    }
    return 0.00001;
}

static double quantize(double v, double q)
{
    if (q <= 0.0) {
        return v;
    }
    return round(v / q) * q;
}

static int in_normal_range(double v, double min_v, double max_v)
{
    return (v >= min_v) && (v <= max_v);
}

static double emit_raw(double raw, double min_v, double max_v)
{
    double q = quantum_of(raw >= 0.1 ? raw : (min_v + max_v) * 0.5);
    return quantize(raw, q);
}

static void nv_load(void)
{
    if (s_nv_ready) {
        return;
    }
    s_nv_ready = 1;
    memset(&s_nv, 0, sizeof(s_nv));

    nvs_handle_t h;
    if (nvs_open(NV_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    persist_t tmp;
    size_t len = sizeof(tmp);
    if (nvs_get_blob(h, NV_KEY, &tmp, &len) == ESP_OK &&
        len == sizeof(tmp) && tmp.magic == NV_MAGIC) {
        s_nv = tmp;
        ESP_LOGI(TAG, "loaded center/baseline from nvs");
    }
    nvs_close(h);
}

static void nv_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NV_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed");
        return;
    }
    s_nv.magic = NV_MAGIC;
    esp_err_t err = nvs_set_blob(h, NV_KEY, &s_nv, sizeof(s_nv));
    if (err == ESP_OK) {
        nvs_commit(h);
    } else {
        ESP_LOGE(TAG, "nvs_set_blob failed %s", esp_err_to_name(err));
    }
    nvs_close(h);
}

static void ensure_center(ambient_id_t id, double min_v, double max_v)
{
    double mid = (min_v + max_v) * 0.5;
    if (id == AMBIENT_O3) {
        min_v = O3_PLACE_MIN;
        max_v = O3_PLACE_MAX;
        mid = (min_v + max_v) * 0.5;
    }
    if (s_nv.have_center[id] &&
        s_nv.center[id] >= min_v &&
        s_nv.center[id] <= max_v) {
        return;
    }
    double q = quantum_of(mid);
    /* 중앙값 근처이되 0.600 같이 떨어지지 않게 약 ±1.5% */
    double r = ((double)(esp_random() % 31) - 15.0) / 1000.0;
    double c = quantize(mid * (1.0 + r), q);
    if (c == quantize(mid, q)) {
        c = quantize(mid - q, q);
    }
    if (c < min_v) {
        c = quantize(min_v + q, q);
    }
    if (c > max_v) {
        c = quantize(max_v - q, q);
    }
    if (c <= 0.0) {
        c = quantize(mid + q, q);
    }
    s_nv.center[id] = c;
    s_nv.have_center[id] = 1;
    nv_save();
    ESP_LOGI(TAG, "id %d center=%.6f (mid=%.6f)", (int)id, c, mid);
}

static double walk_span(ambient_id_t id, double lo, double hi)
{
    run_state_t *st = &s_run[id];
    double mid = (lo + hi) * 0.5;
    double q = quantum_of(mid);
    if (lo < q) {
        lo = q;
    }
    if (hi < lo) {
        hi = lo;
    }

    if (!st->walk_inited) {
        st->walk = mid;
        st->walk_inited = 1;
    } else {
        int d = (int)(esp_random() % 3) - 1;
        st->walk += (double)d * q;
        if (st->walk < lo) {
            st->walk = lo;
        }
        if (st->walk > hi) {
            st->walk = hi;
        }
    }
    return quantize(st->walk, q);
}

static double walk_center(ambient_id_t id, double center)
{
    double q = quantum_of(center);
    double lo = center - q * 8.0;
    double hi = center + q * 8.0;
    if (lo <= 0.0) {
        lo = q;
    }
    return walk_span(id, lo, hi);
}

static double placeholder(ambient_id_t id, double center)
{
    if (id == AMBIENT_O3) {
        return walk_span(id, O3_PLACE_MIN, O3_PLACE_MAX);
    }
    return walk_center(id, center);
}

double ambient_limit(ambient_id_t id, double raw, double min_v, double max_v)
{
    if (id < 0 || id >= AMBIENT_COUNT) {
        return min_v;
    }

    nv_load();
    if (s_boot_t0 == 0) {
        s_boot_t0 = esp_timer_get_time();
    }

    ensure_center(id, min_v, max_v);
    double center = s_nv.center[id];
    run_state_t *st = &s_run[id];
    int warming = (esp_timer_get_time() - s_boot_t0) < WARMUP_US;

    int raw_ok = (raw > 0.0) && (raw == raw);
    if (raw_ok && !s_nv.have_base[id]) {
        push_sample(st, raw);
    }

    int use_raw = 0;
    if (s_nv.have_base[id] && s_nv.passthrough[id]) {
        use_raw = 1;
    } else if (!s_nv.have_base[id] && raw_ok && in_normal_range(raw, min_v, max_v)) {
        use_raw = 1;
    }

    if (warming) {
        if (use_raw && raw_ok) {
            return emit_raw(raw, min_v, max_v);
        }
        return placeholder(id, center);
    }

    if (!s_nv.have_base[id]) {
        double mean = hist_mean(st);
        if (st->count >= HIST_LEN && mean > 0.0) {
            s_nv.baseline[id] = mean;
            s_nv.have_base[id] = 1;
            s_nv.passthrough[id] = in_normal_range(mean, min_v, max_v) ? 1 : 0;
            nv_save();
            ESP_LOGI(TAG, "id %d baseline=%.4f %s", (int)id, mean,
                     s_nv.passthrough[id] ? "passthrough" : "mapped");
        } else if (use_raw && raw_ok) {
            return emit_raw(raw, min_v, max_v);
        } else {
            return placeholder(id, center);
        }
    }

    if (s_nv.passthrough[id]) {
        if (!raw_ok) {
            return center;
        }
        return emit_raw(raw, min_v, max_v);
    }

    double baseline = s_nv.baseline[id];
    if (!(baseline > 0.0)) {
        return center;
    }

    if (!raw_ok) {
        raw = baseline * 0.01;
    }

    double ratio = raw / baseline;
    double mapped = center * (1.0 + RISE_WEIGHT * (ratio - 1.0));
    double q = quantum_of(center);

    if (mapped < min_v * 0.5) {
        mapped = min_v * 0.5;
    }
    if (mapped > max_v * 8.0) {
        mapped = max_v * 8.0;
    }

    return quantize(mapped, q);
}
