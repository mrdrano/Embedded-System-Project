/*
 * Fan vibration anomaly detection prototype for Raspberry Pi 5 + PREEMPT_RT.
 *
 * Target:
 *   - Raspberry Pi 5
 *   - Linux PREEMPT_RT kernel
 *   - MPU6050 accelerometer on /dev/i2c-1, address 0x68
 *   - Button on GPIO line 17, wired GPIO -- button -- GND, pressed = LOW
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -pthread main.c -lm -o fan_vibration_rt
 * 
 * Run:
 *   sudo ./fan_vibration_rt > fan_log.txt
 *     
 *
 * Notes:
 *   - Run as root, or grant permissions for I2C/GPIO and realtime scheduling.
 *   - Thresholds and weights below are initial lab values. Tune them after
 *     collecting data from the real fan, mount, table, and operating speed.
 *   - GPIO pull-up is requested through Linux GPIO character device v2.
 *     If your kernel/platform does not support bias flags, replace the small
 *     button_* abstraction with libgpiod or platform-specific pinctrl setup.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

#define I2C_DEVICE_PATH             "/dev/i2c-1"
#define MPU6050_I2C_ADDR            0x68

#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_EXPECTED_WHO_AM_I   0x68
#define ACCEL_SENSITIVITY_LSB_PER_G 16384.0f

#define BUTTON_GPIO_CHIP            "/dev/gpiochip0"
#define BUTTON_GPIO_LINE            17
#define BUTTON_DEBOUNCE_MS          300
#define BUTTON_POLL_MS              20

#define SAMPLE_RATE_HZ              1000
#define SAMPLE_PERIOD_NS            1000000L
#define FFT_SIZE                    512
#define SPECTRUM_BINS               (FFT_SIZE / 2)
#define BASELINE_FRAMES             20

#define PI_F                        3.14159265358979323846f
#define EMA_ALPHA                   0.01f
#define EPSILON                     1.0e-6f

#define FFT_SEARCH_MIN_HZ           20.0f
#define FFT_SEARCH_MAX_HZ           300.0f
#define HARMONIC_SEARCH_BINS        2

#define RT_SAMPLING_PRIORITY        80
#define DSP_THREAD_PRIORITY         45
#define BUTTON_THREAD_PRIORITY      20
#define LOGGING_THREAD_PRIORITY     5

#define SCORE_WARNING_THRESHOLD     3.0f
#define SCORE_ALERT_THRESHOLD       6.0f

#define ENABLE_STD_FLOOR            1
#define ENABLE_SCORE_SMOOTHING      0
#define ENABLE_HEALTH_CONFIRMATION  1

#define RMS_STD_FLOOR               0.0040f
#define FREQ_STD_FLOOR              0.40f
#define RATIO_STD_FLOOR             0.040f
#define SCORE_EMA_ALPHA             0.20f
#define WARNING_CONFIRM_FRAMES      3
#define ALERT_CONFIRM_FRAMES        5
#define NORMAL_CONFIRM_FRAMES       3

#define ALERT_EMAIL_ENABLED         1
#define ALERT_EMAIL_COOLDOWN_FRAMES 120
#define SMTP_SERVER                 "smtp.gmail.com"
#define SMTP_PORT                   465
#define SENDER_EMAIL                "jimj.cai99@gmail.com"
#define SENDER_PASSWORD             "bljgfsmwbbgrsdsc"
#define RECIPIENT_EMAIL             "jimj.cai99@gmail.com"

typedef struct {
    float w_rms;
    float w_freq;
    float w_ratio2;
    float w_ratio3;
} anomaly_weights_t;

static const anomaly_weights_t g_anomaly_weights = {
    .w_rms = 1.5f,
    .w_freq = 1.0f,
    .w_ratio2 = 1.2f,
    .w_ratio3 = 1.0f,
};

/* -------------------------------------------------------------------------- */
/* Data model                                                                 */
/* -------------------------------------------------------------------------- */

typedef struct {
    float x[FFT_SIZE];
    float y[FFT_SIZE];
    float z[FFT_SIZE];
    uint64_t frame_id;
    struct timespec timestamp_start;
    struct timespec timestamp_end;
    float sum_sq_x;
    float sum_sq_y;
    float sum_sq_z;
} accel_frame_t;

typedef struct {
    float real;
    float imag;
} complex_f_t;

typedef struct {
    float dominant_freq_x;
    float dominant_freq_y;
    float dominant_freq_z;

    float dominant_mag_x;
    float dominant_mag_y;
    float dominant_mag_z;

    float rms_x;
    float rms_y;
    float rms_z;
    float rms_total;

    float energy_total;

    float kurtosis_x;
    float kurtosis_y;
    float kurtosis_z;

    float rpm_1x_estimate;
    float amp_1x_y;
    float amp_2x_y;
    float amp_3x_y;
    float ratio_2x_1x_y;
    float ratio_3x_1x_y;
    float amp_1x_z;
    float amp_2x_z;
    float amp_3x_z;
    float ratio_2x_1x_z;
    float ratio_3x_1x_z;

    float anomaly_score;
} vibration_features_t;

typedef struct {
    float spectrum_y_mean[SPECTRUM_BINS];
    float spectrum_y_std[SPECTRUM_BINS];
    float spectrum_z_mean[SPECTRUM_BINS];
    float spectrum_z_std[SPECTRUM_BINS];

    float rms_total_mean;
    float rms_total_std;

    float dominant_freq_y_mean;
    float dominant_freq_y_std;

    float dominant_freq_z_mean;
    float dominant_freq_z_std;

    float ratio_2x_1x_y_mean;
    float ratio_2x_1x_y_std;

    float ratio_3x_1x_y_mean;
    float ratio_3x_1x_y_std;

    float ratio_2x_1x_z_mean;
    float ratio_2x_1x_z_std;

    float ratio_3x_1x_z_mean;
    float ratio_3x_1x_z_std;

    bool valid;
} baseline_model_t;

typedef struct {
    float spectrum_y_sum[SPECTRUM_BINS];
    float spectrum_y_sum_sq[SPECTRUM_BINS];
    float spectrum_z_sum[SPECTRUM_BINS];
    float spectrum_z_sum_sq[SPECTRUM_BINS];
    double rms_total_sum;
    double rms_total_sum_sq;
    double dominant_freq_y_sum;
    double dominant_freq_y_sum_sq;
    double dominant_freq_z_sum;
    double dominant_freq_z_sum_sq;
    double ratio_2x_1x_y_sum;
    double ratio_2x_1x_y_sum_sq;
    double ratio_3x_1x_y_sum;
    double ratio_3x_1x_y_sum_sq;
    double ratio_2x_1x_z_sum;
    double ratio_2x_1x_z_sum_sq;
    double ratio_3x_1x_z_sum;
    double ratio_3x_1x_z_sum_sq;
    int frame_count;
} baseline_accumulator_t;

typedef enum {
    MODE_MONITORING = 0,
    MODE_BASELINE_COLLECTING
} system_mode_t;

typedef enum {
    HEALTH_NORMAL = 0,
    HEALTH_WARNING,
    HEALTH_ALERT,
    HEALTH_BASELINE_COLLECTING
} health_state_t;

typedef struct {
    bool initialized;
    float value;
} score_smoother_t;

typedef struct {
    health_state_t confirmed;
    health_state_t candidate;
    int candidate_count;
} health_confirmation_t;

typedef struct {
    atomic_ullong samples;
    atomic_llong min_ns;
    atomic_llong max_ns;
    atomic_llong sum_abs_ns;
    atomic_ullong over_50us;
    atomic_ullong over_100us;
    atomic_ullong over_500us;
    atomic_ullong over_1ms;
} latency_stats_t;

typedef struct {
    uint64_t samples;
    int64_t min_ns;
    int64_t max_ns;
    int64_t avg_abs_ns;
    uint64_t over_50us;
    uint64_t over_100us;
    uint64_t over_500us;
    uint64_t over_1ms;
} latency_snapshot_t;

typedef struct {
    uint64_t frame_id;
    system_mode_t mode;
    health_state_t health;
    vibration_features_t features;
    uint64_t sampling_overruns;
    uint64_t missed_frames;
    int baseline_progress;
    int baseline_total;
    bool baseline_valid;
    latency_snapshot_t wakeup_jitter;
    latency_snapshot_t i2c_duration;
    latency_snapshot_t handoff_duration;
} analysis_result_t;

typedef struct {
    int chip_fd;
    int line_fd;
} button_handle_t;

typedef struct {
    uint64_t frame_id;
    float score;
    float dominant_freq_y;
    float dominant_freq_z;
    float rms_total;
    float ratio_2x_1x_y;
    float ratio_3x_1x_y;
} alert_email_request_t;

/* -------------------------------------------------------------------------- */
/* Shared state                                                               */
/* -------------------------------------------------------------------------- */

static accel_frame_t g_frame_buffers[2];
static int g_active_buffer_index = 0;
static int g_ready_buffer_index = -1;
static int g_sample_count = 0;
static uint64_t g_next_frame_id = 1;

static pthread_mutex_t g_frame_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_frame_ready_cond = PTHREAD_COND_INITIALIZER;

static pthread_mutex_t g_log_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_log_cond = PTHREAD_COND_INITIALIZER;
static analysis_result_t g_latest_result;
static bool g_has_new_result = false;

static pthread_mutex_t g_alert_email_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_alert_email_cond = PTHREAD_COND_INITIALIZER;
static alert_email_request_t g_alert_email_request;
static bool g_alert_email_pending = false;

static atomic_bool g_running = true;
static atomic_bool g_baseline_request = false;
static atomic_ullong g_sampling_overruns = 0;
static atomic_ullong g_i2c_errors = 0;
static atomic_ullong g_missed_frames = 0;
static latency_stats_t g_wakeup_jitter_stats;
static latency_stats_t g_i2c_duration_stats;
static latency_stats_t g_handoff_duration_stats;

static int g_i2c_fd = -1;

/* -------------------------------------------------------------------------- */
/* Utility helpers                                                            */
/* -------------------------------------------------------------------------- */

/* Installs SCHED_FIFO priority for a pipeline thread. */
static int set_thread_realtime_priority(const char *name, int priority)
{
    struct sched_param param = { .sched_priority = priority };
    int ret;

    ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (ret != 0) {
        fprintf(stderr,
                "[WARN] %s: failed to set SCHED_FIFO priority %d: %s\n",
                name, priority, strerror(ret));
        return -1;
    }

    return 0;
}

static void timespec_add_ns(struct timespec *ts, long ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec++;
        ts->tv_nsec -= 1000000000L;
    }
}

static int64_t timespec_diff_ns(const struct timespec *a,
                                const struct timespec *b)
{
    return ((int64_t)a->tv_sec - (int64_t)b->tv_sec) * 1000000000LL +
           ((int64_t)a->tv_nsec - (int64_t)b->tv_nsec);
}

static void latency_stats_update(latency_stats_t *stats, int64_t value_ns)
{
    long long abs_ns;

    if (value_ns == INT64_MIN) {
        abs_ns = INT64_MAX;
    } else if (value_ns < 0) {
        abs_ns = (long long)-value_ns;
    } else {
        abs_ns = (long long)value_ns;
    }

    unsigned long long previous_samples = atomic_fetch_add(&stats->samples, 1);
    atomic_fetch_add(&stats->sum_abs_ns, abs_ns);

    if (abs_ns > 50000LL) {
        atomic_fetch_add(&stats->over_50us, 1);
    }
    if (abs_ns > 100000LL) {
        atomic_fetch_add(&stats->over_100us, 1);
    }
    if (abs_ns > 500000LL) {
        atomic_fetch_add(&stats->over_500us, 1);
    }
    if (abs_ns > 1000000LL) {
        atomic_fetch_add(&stats->over_1ms, 1);
    }

    if (previous_samples == 0) {
        atomic_store(&stats->min_ns, abs_ns);
        atomic_store(&stats->max_ns, abs_ns);
        return;
    }

    long long current_min = atomic_load(&stats->min_ns);
    while (abs_ns < current_min &&
           !atomic_compare_exchange_weak(&stats->min_ns,
                                         &current_min,
                                         abs_ns)) {
    }

    long long current_max = atomic_load(&stats->max_ns);
    while (abs_ns > current_max &&
           !atomic_compare_exchange_weak(&stats->max_ns,
                                         &current_max,
                                         abs_ns)) {
    }
}

static latency_snapshot_t latency_stats_snapshot(const latency_stats_t *stats)
{
    latency_snapshot_t snapshot;

    snapshot.samples = (uint64_t)atomic_load(&stats->samples);
    snapshot.min_ns = snapshot.samples > 0 ? (int64_t)atomic_load(&stats->min_ns) : 0;
    snapshot.max_ns = snapshot.samples > 0 ? (int64_t)atomic_load(&stats->max_ns) : 0;
    snapshot.over_50us = (uint64_t)atomic_load(&stats->over_50us);
    snapshot.over_100us = (uint64_t)atomic_load(&stats->over_100us);
    snapshot.over_500us = (uint64_t)atomic_load(&stats->over_500us);
    snapshot.over_1ms = (uint64_t)atomic_load(&stats->over_1ms);

    int64_t sum_abs_ns = (int64_t)atomic_load(&stats->sum_abs_ns);
    snapshot.avg_abs_ns = snapshot.samples > 0 ?
                          sum_abs_ns / (int64_t)snapshot.samples : 0;

    return snapshot;
}

static float clamp_positive(float value)
{
    return value > 0.0f ? value : 0.0f;
}

static float sqrf(float value)
{
    return value * value;
}

static const char *health_state_name(health_state_t state)
{
    switch (state) {
    case HEALTH_NORMAL:
        return "NORMAL";
    case HEALTH_WARNING:
        return "WARNING";
    case HEALTH_ALERT:
        return "ALERT";
    case HEALTH_BASELINE_COLLECTING:
        return "BASELINE";
    default:
        return "UNKNOWN";
    }
}

/* Handles SIGINT/SIGTERM so all threads can leave their loops cleanly. */
static void handle_signal(int signo)
{
    (void)signo;
    atomic_store(&g_running, false);
    pthread_cond_broadcast(&g_frame_ready_cond);
    pthread_cond_broadcast(&g_log_cond);
    pthread_cond_broadcast(&g_alert_email_cond);
}

/* -------------------------------------------------------------------------- */
/* MPU6050 I2C                                                                */
/* -------------------------------------------------------------------------- */

static int i2c_write_u8(int fd, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = { reg, value };
    return write(fd, data, sizeof(data)) == (ssize_t)sizeof(data) ? 0 : -1;
}

static int i2c_read_u8(int fd, uint8_t reg, uint8_t *value)
{
    if (write(fd, &reg, 1) != 1) {
        return -1;
    }
    return read(fd, value, 1) == 1 ? 0 : -1;
}

/*
 * Initializes the MPU6050 before realtime sampling starts.
 * This function runs in main, not in the RT sampling loop.
 */
static int mpu6050_init(const char *device_path)
{
    int fd = open(device_path, O_RDWR);
    if (fd < 0) {
        perror("open I2C device");
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, MPU6050_I2C_ADDR) < 0) {
        perror("ioctl I2C_SLAVE");
        close(fd);
        return -1;
    }

    uint8_t who_am_i = 0;
    if (i2c_read_u8(fd, MPU6050_REG_WHO_AM_I, &who_am_i) < 0) {
        perror("read MPU6050 WHO_AM_I");
        close(fd);
        return -1;
    }

    if (who_am_i != MPU6050_EXPECTED_WHO_AM_I) {
        fprintf(stderr, "MPU6050 WHO_AM_I mismatch: got 0x%02X, expected 0x%02X\n",
                who_am_i, MPU6050_EXPECTED_WHO_AM_I);
        close(fd);
        return -1;
    }

    if (i2c_write_u8(fd, MPU6050_REG_PWR_MGMT_1, 0x00) < 0) {
        perror("wake MPU6050");
        close(fd);
        return -1;
    }

    if (i2c_write_u8(fd, MPU6050_REG_ACCEL_CONFIG, 0x00) < 0) {
        perror("set MPU6050 accel range");
        close(fd);
        return -1;
    }

    return fd;
}

/*
 * Reads one 3-axis acceleration sample from MPU6050.
 * Called by the RT sampling thread once every 1 ms; keep it small.
 */
static int mpu6050_read_accel(int fd, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[6];

    if (write(fd, &reg, 1) != 1) {
        return -1;
    }

    if (read(fd, buffer, sizeof(buffer)) != (ssize_t)sizeof(buffer)) {
        return -1;
    }

    *raw_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *raw_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *raw_z = (int16_t)((buffer[4] << 8) | buffer[5]);
    return 0;
}

/* -------------------------------------------------------------------------- */
/* GPIO button abstraction                                                     */
/* -------------------------------------------------------------------------- */

/*
 * Opens a GPIO line as input with pull-up, using Linux GPIO character device v2.
 * The rest of the program only depends on button_init/button_is_pressed.
 */
static int button_init(button_handle_t *button, const char *chip_path, unsigned int line)
{
    memset(button, 0, sizeof(*button));
    button->chip_fd = -1;
    button->line_fd = -1;

    button->chip_fd = open(chip_path, O_RDONLY | O_CLOEXEC);
    if (button->chip_fd < 0) {
        perror("open GPIO chip");
        return -1;
    }

    struct gpio_v2_line_request request;
    memset(&request, 0, sizeof(request));
    request.offsets[0] = line;
    request.num_lines = 1;
    request.config.flags = GPIO_V2_LINE_FLAG_INPUT |
                           GPIO_V2_LINE_FLAG_BIAS_PULL_UP;
    snprintf(request.consumer, sizeof(request.consumer), "fan_vibration_rt");

    if (ioctl(button->chip_fd, GPIO_V2_GET_LINE_IOCTL, &request) < 0) {
        perror("request GPIO line with pull-up");
        close(button->chip_fd);
        button->chip_fd = -1;
        return -1;
    }

    button->line_fd = request.fd;
    return 0;
}

/*
 * Returns true when the active-low button is pressed.
 * This is intentionally tiny so it can be swapped for libgpiod if preferred.
 */
static bool button_is_pressed(const button_handle_t *button)
{
    struct gpio_v2_line_values values;
    memset(&values, 0, sizeof(values));
    values.mask = 1U;

    if (ioctl(button->line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &values) < 0) {
        return false;
    }

    return (values.bits & 1U) == 0U;
}

static void button_close(button_handle_t *button)
{
    if (button->line_fd >= 0) {
        close(button->line_fd);
    }
    if (button->chip_fd >= 0) {
        close(button->chip_fd);
    }
    button->line_fd = -1;
    button->chip_fd = -1;
}

/* -------------------------------------------------------------------------- */
/* FFT and spectrum analysis                                                   */
/* -------------------------------------------------------------------------- */

static unsigned int bit_reverse(unsigned int x, int log2n)
{
    unsigned int n = 0;
    for (int i = 0; i < log2n; i++) {
        n = (n << 1) | (x & 1U);
        x >>= 1;
    }
    return n;
}

static complex_f_t complex_add(complex_f_t a, complex_f_t b)
{
    return (complex_f_t){ .real = a.real + b.real, .imag = a.imag + b.imag };
}

static complex_f_t complex_sub(complex_f_t a, complex_f_t b)
{
    return (complex_f_t){ .real = a.real - b.real, .imag = a.imag - b.imag };
}

static complex_f_t complex_mul(complex_f_t a, complex_f_t b)
{
    return (complex_f_t){
        .real = (a.real * b.real) - (a.imag * b.imag),
        .imag = (a.real * b.imag) + (a.imag * b.real),
    };
}

/*
 * Performs a radix-2 FFT on one acceleration axis and outputs magnitude bins.
 * Per-frame mean removal and Hanning window reduce DC leakage before FFT.
 */
static void compute_axis_spectrum(const float samples[FFT_SIZE],
                                  float spectrum[SPECTRUM_BINS])
{
    complex_f_t data[FFT_SIZE];
    int log2n = 0;

    while ((1 << log2n) < FFT_SIZE) {
        log2n++;
    }

    float mean = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        mean += samples[i];
    }
    mean /= (float)FFT_SIZE;

    for (int i = 0; i < FFT_SIZE; i++) {
        float window = 0.5f * (1.0f - cosf((2.0f * PI_F * (float)i) /
                                           (float)(FFT_SIZE - 1)));
        unsigned int rev = bit_reverse((unsigned int)i, log2n);
        data[rev].real = (samples[i] - mean) * window;
        data[rev].imag = 0.0f;
    }

    for (int stage = 1; stage <= log2n; stage++) {
        int m = 1 << stage;
        int half_m = m >> 1;
        float theta = -2.0f * PI_F / (float)m;
        complex_f_t wm = { .real = cosf(theta), .imag = sinf(theta) };

        for (int k = 0; k < FFT_SIZE; k += m) {
            complex_f_t w = { .real = 1.0f, .imag = 0.0f };
            for (int j = 0; j < half_m; j++) {
                complex_f_t t = complex_mul(w, data[k + j + half_m]);
                complex_f_t u = data[k + j];
                data[k + j] = complex_add(u, t);
                data[k + j + half_m] = complex_sub(u, t);
                w = complex_mul(w, wm);
            }
        }
    }

    for (int i = 0; i < SPECTRUM_BINS; i++) {
        float mag = sqrtf(sqrf(data[i].real) + sqrf(data[i].imag));
        spectrum[i] = mag / ((float)FFT_SIZE / 2.0f);
    }
}

/*
 * Finds the dominant peak in the configured vibration band.
 * Parabolic interpolation reduces bin-to-bin frequency jitter.
 */
static void find_dominant_frequency(const float spectrum[SPECTRUM_BINS],
                                    float *dominant_freq,
                                    float *dominant_mag)
{
    const float freq_resolution = (float)SAMPLE_RATE_HZ / (float)FFT_SIZE;
    int start_bin = (int)ceilf(FFT_SEARCH_MIN_HZ / freq_resolution);
    int end_bin = (int)floorf(FFT_SEARCH_MAX_HZ / freq_resolution);

    if (start_bin < 1) {
        start_bin = 1;
    }
    if (end_bin > SPECTRUM_BINS - 2) {
        end_bin = SPECTRUM_BINS - 2;
    }

    int max_bin = start_bin;
    float max_mag = spectrum[start_bin];

    for (int i = start_bin + 1; i <= end_bin; i++) {
        if (spectrum[i] > max_mag) {
            max_mag = spectrum[i];
            max_bin = i;
        }
    }

    float interpolated_bin = (float)max_bin;
    if (max_bin > 0 && max_bin < SPECTRUM_BINS - 1) {
        float left = spectrum[max_bin - 1];
        float center = spectrum[max_bin];
        float right = spectrum[max_bin + 1];
        float denom = left - (2.0f * center) + right;
        if (fabsf(denom) > EPSILON) {
            interpolated_bin += 0.5f * (left - right) / denom;
        }
    }

    *dominant_freq = interpolated_bin * freq_resolution;
    *dominant_mag = max_mag;
}

/*
 * Returns the largest amplitude around a target frequency.
 * Harmonic lookup uses a small +/- bin search to tolerate speed drift.
 */
static float spectrum_amplitude_near(const float spectrum[SPECTRUM_BINS],
                                     float target_freq_hz,
                                     int search_bins)
{
    const float freq_resolution = (float)SAMPLE_RATE_HZ / (float)FFT_SIZE;
    int center_bin = (int)lroundf(target_freq_hz / freq_resolution);
    int start_bin = center_bin - search_bins;
    int end_bin = center_bin + search_bins;
    float best = 0.0f;

    if (start_bin < 1) {
        start_bin = 1;
    }
    if (end_bin >= SPECTRUM_BINS) {
        end_bin = SPECTRUM_BINS - 1;
    }

    for (int i = start_bin; i <= end_bin; i++) {
        if (spectrum[i] > best) {
            best = spectrum[i];
        }
    }

    return best;
}

/* Simplified kurtosis for one frame. Useful for impulsive fault signatures. */
static float compute_kurtosis(const float samples[FFT_SIZE])
{
    float mean = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        mean += samples[i];
    }
    mean /= (float)FFT_SIZE;

    double m2 = 0.0;
    double m4 = 0.0;
    for (int i = 0; i < FFT_SIZE; i++) {
        double d = (double)samples[i] - (double)mean;
        double d2 = d * d;
        m2 += d2;
        m4 += d2 * d2;
    }

    m2 /= (double)FFT_SIZE;
    m4 /= (double)FFT_SIZE;

    if (m2 < (double)EPSILON) {
        return 0.0f;
    }

    return (float)(m4 / (m2 * m2));
}

/*
 * Tracks the 1X reference used by harmonic analysis.
 * Temporary lab rule: use Y-axis dominant frequency as the 1X reference.
 * Current fixed-medium-speed fan logs show stable X/Y peaks near 50-51 Hz,
 * while Z often lands near 200-204 Hz, likely a 4X harmonic or resonance.
 *
 * Replace this with X/Y/Z consensus or a tachometer reference when available.
 */
static float estimate_1x_frequency(const vibration_features_t *features)
{
    return features->dominant_freq_y;
}

/*
 * Extracts all frame-level vibration features from three independent spectra.
 * FFT diagnostics stay per-axis; RMS/energy summarize the whole 3-axis motion.
 */
static void extract_features(const accel_frame_t *frame,
                             const float spectrum_x[SPECTRUM_BINS],
                             const float spectrum_y[SPECTRUM_BINS],
                             const float spectrum_z[SPECTRUM_BINS],
                             vibration_features_t *features)
{
    memset(features, 0, sizeof(*features));

    find_dominant_frequency(spectrum_x,
                            &features->dominant_freq_x,
                            &features->dominant_mag_x);
    find_dominant_frequency(spectrum_y,
                            &features->dominant_freq_y,
                            &features->dominant_mag_y);
    find_dominant_frequency(spectrum_z,
                            &features->dominant_freq_z,
                            &features->dominant_mag_z);

    features->rms_x = sqrtf(frame->sum_sq_x / (float)FFT_SIZE);
    features->rms_y = sqrtf(frame->sum_sq_y / (float)FFT_SIZE);
    features->rms_z = sqrtf(frame->sum_sq_z / (float)FFT_SIZE);
    features->energy_total = frame->sum_sq_x + frame->sum_sq_y + frame->sum_sq_z;
    features->rms_total = sqrtf(features->energy_total / (float)(FFT_SIZE * 3));

    features->kurtosis_x = compute_kurtosis(frame->x);
    features->kurtosis_y = compute_kurtosis(frame->y);
    features->kurtosis_z = compute_kurtosis(frame->z);

    float f_1x = estimate_1x_frequency(features);
    features->rpm_1x_estimate = f_1x * 60.0f;
    features->amp_1x_y = spectrum_amplitude_near(spectrum_y, f_1x, HARMONIC_SEARCH_BINS);
    features->amp_2x_y = spectrum_amplitude_near(spectrum_y, 2.0f * f_1x, HARMONIC_SEARCH_BINS);
    features->amp_3x_y = spectrum_amplitude_near(spectrum_y, 3.0f * f_1x, HARMONIC_SEARCH_BINS);
    features->ratio_2x_1x_y = features->amp_2x_y / (features->amp_1x_y + EPSILON);
    features->ratio_3x_1x_y = features->amp_3x_y / (features->amp_1x_y + EPSILON);
    features->amp_1x_z = spectrum_amplitude_near(spectrum_z, f_1x, HARMONIC_SEARCH_BINS);
    features->amp_2x_z = spectrum_amplitude_near(spectrum_z, 2.0f * f_1x, HARMONIC_SEARCH_BINS);
    features->amp_3x_z = spectrum_amplitude_near(spectrum_z, 3.0f * f_1x, HARMONIC_SEARCH_BINS);
    features->ratio_2x_1x_z = features->amp_2x_z / (features->amp_1x_z + EPSILON);
    features->ratio_3x_1x_z = features->amp_3x_z / (features->amp_1x_z + EPSILON);
}

/* -------------------------------------------------------------------------- */
/* Baseline and anomaly scoring                                                */
/* -------------------------------------------------------------------------- */

static void baseline_accumulator_reset(baseline_accumulator_t *acc)
{
    memset(acc, 0, sizeof(*acc));
}

/*
 * Accumulates frame features while MODE_BASELINE_COLLECTING is active.
 * Called only by the DSP thread, never by the RT sampling thread.
 */
static void baseline_accumulator_add(baseline_accumulator_t *acc,
                                     const vibration_features_t *features,
                                     const float spectrum_y[SPECTRUM_BINS],
                                     const float spectrum_z[SPECTRUM_BINS])
{
    for (int i = 0; i < SPECTRUM_BINS; i++) {
        acc->spectrum_y_sum[i] += spectrum_y[i];
        acc->spectrum_y_sum_sq[i] += spectrum_y[i] * spectrum_y[i];
        acc->spectrum_z_sum[i] += spectrum_z[i];
        acc->spectrum_z_sum_sq[i] += spectrum_z[i] * spectrum_z[i];
    }

    acc->rms_total_sum += features->rms_total;
    acc->rms_total_sum_sq += sqrf(features->rms_total);
    acc->dominant_freq_y_sum += features->dominant_freq_y;
    acc->dominant_freq_y_sum_sq += sqrf(features->dominant_freq_y);
    acc->dominant_freq_z_sum += features->dominant_freq_z;
    acc->dominant_freq_z_sum_sq += sqrf(features->dominant_freq_z);
    acc->ratio_2x_1x_y_sum += features->ratio_2x_1x_y;
    acc->ratio_2x_1x_y_sum_sq += sqrf(features->ratio_2x_1x_y);
    acc->ratio_3x_1x_y_sum += features->ratio_3x_1x_y;
    acc->ratio_3x_1x_y_sum_sq += sqrf(features->ratio_3x_1x_y);
    acc->ratio_2x_1x_z_sum += features->ratio_2x_1x_z;
    acc->ratio_2x_1x_z_sum_sq += sqrf(features->ratio_2x_1x_z);
    acc->ratio_3x_1x_z_sum += features->ratio_3x_1x_z;
    acc->ratio_3x_1x_z_sum_sq += sqrf(features->ratio_3x_1x_z);
    acc->frame_count++;
}

static float std_from_sum(double sum, double sum_sq, int count)
{
    if (count <= 1) {
        return 0.0f;
    }

    double mean = sum / (double)count;
    double variance = (sum_sq / (double)count) - (mean * mean);
    if (variance < 0.0) {
        variance = 0.0;
    }

    return (float)sqrt(variance);
}

/* Finalizes the baseline model after BASELINE_FRAMES complete frames. */
static void baseline_model_finalize(const baseline_accumulator_t *acc,
                                    baseline_model_t *model)
{
    int count = acc->frame_count;
    memset(model, 0, sizeof(*model));

    if (count <= 0) {
        model->valid = false;
        return;
    }

    for (int i = 0; i < SPECTRUM_BINS; i++) {
        double sum_y = acc->spectrum_y_sum[i];
        double sum_sq_y = acc->spectrum_y_sum_sq[i];
        double sum = acc->spectrum_z_sum[i];
        double sum_sq = acc->spectrum_z_sum_sq[i];
        model->spectrum_y_mean[i] = (float)(sum_y / (double)count);
        model->spectrum_y_std[i] = std_from_sum(sum_y, sum_sq_y, count);
        model->spectrum_z_mean[i] = (float)(sum / (double)count);
        model->spectrum_z_std[i] = std_from_sum(sum, sum_sq, count);
    }

    model->rms_total_mean = (float)(acc->rms_total_sum / (double)count);
    model->rms_total_std = std_from_sum(acc->rms_total_sum,
                                        acc->rms_total_sum_sq,
                                        count);

    model->dominant_freq_y_mean = (float)(acc->dominant_freq_y_sum / (double)count);
    model->dominant_freq_y_std = std_from_sum(acc->dominant_freq_y_sum,
                                              acc->dominant_freq_y_sum_sq,
                                              count);

    model->dominant_freq_z_mean = (float)(acc->dominant_freq_z_sum / (double)count);
    model->dominant_freq_z_std = std_from_sum(acc->dominant_freq_z_sum,
                                              acc->dominant_freq_z_sum_sq,
                                              count);

    model->ratio_2x_1x_y_mean = (float)(acc->ratio_2x_1x_y_sum / (double)count);
    model->ratio_2x_1x_y_std = std_from_sum(acc->ratio_2x_1x_y_sum,
                                            acc->ratio_2x_1x_y_sum_sq,
                                            count);

    model->ratio_3x_1x_y_mean = (float)(acc->ratio_3x_1x_y_sum / (double)count);
    model->ratio_3x_1x_y_std = std_from_sum(acc->ratio_3x_1x_y_sum,
                                            acc->ratio_3x_1x_y_sum_sq,
                                            count);

    model->ratio_2x_1x_z_mean = (float)(acc->ratio_2x_1x_z_sum / (double)count);
    model->ratio_2x_1x_z_std = std_from_sum(acc->ratio_2x_1x_z_sum,
                                            acc->ratio_2x_1x_z_sum_sq,
                                            count);

    model->ratio_3x_1x_z_mean = (float)(acc->ratio_3x_1x_z_sum / (double)count);
    model->ratio_3x_1x_z_std = std_from_sum(acc->ratio_3x_1x_z_sum,
                                            acc->ratio_3x_1x_z_sum_sq,
                                            count);

    model->valid = true;
}

/*
 * Computes an initial adaptive score from baseline z-scores.
 * Frequency and harmonic-ratio scoring use Y-axis features for the current
 * fan mount; RMS still summarizes total 3-axis motion.
 * Tune weights and thresholds with real experiment data before relying on it.
 */
static float safe_std(float std_value, float floor_value)
{
#if ENABLE_STD_FLOOR
    return std_value > floor_value ? std_value : floor_value;
#else
    (void)floor_value;
    return std_value > EPSILON ? std_value : EPSILON;
#endif
}

static float compute_anomaly_score(const vibration_features_t *current,
                                   const baseline_model_t *baseline,
                                   const anomaly_weights_t *weights)
{
    if (!baseline->valid) {
        return 0.0f;
    }

    float z_rms = (current->rms_total - baseline->rms_total_mean) /
                  safe_std(baseline->rms_total_std, RMS_STD_FLOOR);
    float z_freq = fabsf(current->dominant_freq_y - baseline->dominant_freq_y_mean) /
                   safe_std(baseline->dominant_freq_y_std, FREQ_STD_FLOOR);
    float z_ratio2 = (current->ratio_2x_1x_y - baseline->ratio_2x_1x_y_mean) /
                     safe_std(baseline->ratio_2x_1x_y_std, RATIO_STD_FLOOR);
    float z_ratio3 = (current->ratio_3x_1x_y - baseline->ratio_3x_1x_y_mean) /
                     safe_std(baseline->ratio_3x_1x_y_std, RATIO_STD_FLOOR);

    return (weights->w_rms * clamp_positive(z_rms)) +
           (weights->w_freq * z_freq) +
           (weights->w_ratio2 * clamp_positive(z_ratio2)) +
           (weights->w_ratio3 * clamp_positive(z_ratio3));
}

static void score_smoother_reset(score_smoother_t *smoother)
{
    smoother->initialized = false;
    smoother->value = 0.0f;
}

static float smooth_anomaly_score(score_smoother_t *smoother, float raw_score)
{
#if ENABLE_SCORE_SMOOTHING
    if (!smoother->initialized) {
        smoother->value = raw_score;
        smoother->initialized = true;
        return raw_score;
    }

    smoother->value = (SCORE_EMA_ALPHA * raw_score) +
                      ((1.0f - SCORE_EMA_ALPHA) * smoother->value);
    return smoother->value;
#else
    (void)smoother;
    return raw_score;
#endif
}

static health_state_t classify_health_raw(float score, bool baseline_valid)
{
    if (!baseline_valid) {
        return HEALTH_NORMAL;
    }
    if (score >= SCORE_ALERT_THRESHOLD) {
        return HEALTH_ALERT;
    }
    if (score >= SCORE_WARNING_THRESHOLD) {
        return HEALTH_WARNING;
    }
    return HEALTH_NORMAL;
}

static void health_confirmation_reset(health_confirmation_t *confirmation)
{
    confirmation->confirmed = HEALTH_NORMAL;
    confirmation->candidate = HEALTH_NORMAL;
    confirmation->candidate_count = 0;
}

#if ENABLE_HEALTH_CONFIRMATION
static int health_confirm_frames(health_state_t state)
{
    switch (state) {
    case HEALTH_ALERT:
        return ALERT_CONFIRM_FRAMES;
    case HEALTH_WARNING:
        return WARNING_CONFIRM_FRAMES;
    case HEALTH_NORMAL:
        return NORMAL_CONFIRM_FRAMES;
    default:
        return 1;
    }
}
#endif

static health_state_t update_health_confirmation(health_confirmation_t *confirmation,
                                                 health_state_t raw_health)
{
#if ENABLE_HEALTH_CONFIRMATION
    if (raw_health == confirmation->confirmed) {
        confirmation->candidate = raw_health;
        confirmation->candidate_count = 0;
        return confirmation->confirmed;
    }

    if (raw_health != confirmation->candidate) {
        confirmation->candidate = raw_health;
        confirmation->candidate_count = 1;
    } else {
        confirmation->candidate_count++;
    }

    if (confirmation->candidate_count >= health_confirm_frames(raw_health)) {
        confirmation->confirmed = raw_health;
        confirmation->candidate_count = 0;
    }

    return confirmation->confirmed;
#else
    (void)confirmation;
    return raw_health;
#endif
}

/* -------------------------------------------------------------------------- */
/* Logging                                                                     */
/* -------------------------------------------------------------------------- */

static void print_rt_latency_summary(const analysis_result_t *result)
{
    printf("[RT] wake_avg/max=%.1f/%.1fus i2c_avg/max=%.1f/%.1fus "
           "handoff_avg/max=%.1f/%.1fus j>100us=%" PRIu64 " "
           "i2c>500us=%" PRIu64 " handoff>100us=%" PRIu64 " "
           "overrun=%" PRIu64 " missed=%" PRIu64 "\n",
           (double)result->wakeup_jitter.avg_abs_ns / 1000.0,
           (double)result->wakeup_jitter.max_ns / 1000.0,
           (double)result->i2c_duration.avg_abs_ns / 1000.0,
           (double)result->i2c_duration.max_ns / 1000.0,
           (double)result->handoff_duration.avg_abs_ns / 1000.0,
           (double)result->handoff_duration.max_ns / 1000.0,
           result->wakeup_jitter.over_100us,
           result->i2c_duration.over_500us,
           result->handoff_duration.over_100us,
           result->sampling_overruns,
           result->missed_frames);
}

/*
 * Publishes a compact analysis result to the logging thread.
 * DSP remains the owner of heavy computation; logging owns printf.
 */
static void publish_result(const analysis_result_t *result)
{
    pthread_mutex_lock(&g_log_mutex);
    g_latest_result = *result;
    g_has_new_result = true;
    pthread_cond_signal(&g_log_cond);
    pthread_mutex_unlock(&g_log_mutex);
}

static void queue_alert_email(const analysis_result_t *result)
{
#if ALERT_EMAIL_ENABLED
    alert_email_request_t request = {
        .frame_id = result->frame_id,
        .score = result->features.anomaly_score,
        .dominant_freq_y = result->features.dominant_freq_y,
        .dominant_freq_z = result->features.dominant_freq_z,
        .rms_total = result->features.rms_total,
        .ratio_2x_1x_y = result->features.ratio_2x_1x_y,
        .ratio_3x_1x_y = result->features.ratio_3x_1x_y,
    };

    pthread_mutex_lock(&g_alert_email_mutex);
    g_alert_email_request = request;
    g_alert_email_pending = true;
    pthread_cond_signal(&g_alert_email_cond);
    pthread_mutex_unlock(&g_alert_email_mutex);
#else
    (void)result;
#endif
}

static const char *alert_email_password(void)
{
    const char *password = getenv("FAN_ALERT_EMAIL_PASSWORD");
    if (password != NULL && password[0] != '\0') {
        return password;
    }

    return SENDER_PASSWORD;
}

static int send_alert_email_child(const alert_email_request_t *request)
{
    char smtp_port[16];
    char frame_id[32];
    char score[32];
    char dominant_freq_y[32];
    char dominant_freq_z[32];
    char rms_total[32];
    char ratio_2x_1x_y[32];
    char ratio_3x_1x_y[32];
    char env_smtp_server[128];
    char env_smtp_port[48];
    char env_sender_email[256];
    char env_sender_password[512];
    char env_recipient_email[256];

    snprintf(smtp_port, sizeof(smtp_port), "%d", SMTP_PORT);
    snprintf(frame_id, sizeof(frame_id), "%" PRIu64, request->frame_id);
    snprintf(score, sizeof(score), "%.2f", request->score);
    snprintf(dominant_freq_y, sizeof(dominant_freq_y), "%.2f", request->dominant_freq_y);
    snprintf(dominant_freq_z, sizeof(dominant_freq_z), "%.2f", request->dominant_freq_z);
    snprintf(rms_total, sizeof(rms_total), "%.5f", request->rms_total);
    snprintf(ratio_2x_1x_y, sizeof(ratio_2x_1x_y), "%.3f", request->ratio_2x_1x_y);
    snprintf(ratio_3x_1x_y, sizeof(ratio_3x_1x_y), "%.3f", request->ratio_3x_1x_y);
    snprintf(env_smtp_server, sizeof(env_smtp_server),
             "FAN_ALERT_SMTP_SERVER=%s", SMTP_SERVER);
    snprintf(env_smtp_port, sizeof(env_smtp_port),
             "FAN_ALERT_SMTP_PORT=%s", smtp_port);
    snprintf(env_sender_email, sizeof(env_sender_email),
             "FAN_ALERT_SENDER_EMAIL=%s", SENDER_EMAIL);
    snprintf(env_sender_password, sizeof(env_sender_password),
             "FAN_ALERT_EMAIL_PASSWORD=%s", alert_email_password());
    snprintf(env_recipient_email, sizeof(env_recipient_email),
             "FAN_ALERT_RECIPIENT_EMAIL=%s", RECIPIENT_EMAIL);

    pid_t pid = fork();
    if (pid < 0) {
        return -1;
    }

    if (pid == 0) {
        const char *python_code =
            "import os, smtplib, sys\n"
            "from datetime import datetime\n"
            "from email.mime.text import MIMEText\n"
            "server = os.environ['FAN_ALERT_SMTP_SERVER']\n"
            "port = int(os.environ['FAN_ALERT_SMTP_PORT'])\n"
            "sender = os.environ['FAN_ALERT_SENDER_EMAIL']\n"
            "password = os.environ.get('FAN_ALERT_EMAIL_PASSWORD', '')\n"
            "recipient = os.environ['FAN_ALERT_RECIPIENT_EMAIL']\n"
            "if not password or password == 'YOUR_PASSWORD':\n"
            "    raise RuntimeError('Set FAN_ALERT_EMAIL_PASSWORD to a Gmail app password')\n"
            "frame, score, dom_y, dom_z, rms_total, ratio2, ratio3 = sys.argv[1:8]\n"
            "now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')\n"
            "body = (\n"
            "    f'Fan vibration ALERT detected at {now}\\n\\n'\n"
            "    f'Frame: {frame}\\n'\n"
            "    f'Score: {score}\\n'\n"
            "    f'Score axis: Y\\n'\n"
            "    f'Dominant frequency Y/Z: {dom_y} / {dom_z} Hz\\n'\n"
            "    f'RMS total: {rms_total} g\\n'\n"
            "    f'Y harmonic ratio 2X/1X: {ratio2}\\n'\n"
            "    f'Y harmonic ratio 3X/1X: {ratio3}\\n'\n"
            ")\n"
            "msg = MIMEText(body)\n"
            "msg['Subject'] = 'Fan vibration ALERT from Raspberry Pi'\n"
            "msg['From'] = sender\n"
            "msg['To'] = recipient\n"
            "with smtplib.SMTP_SSL(server, port, timeout=20) as smtp:\n"
            "    smtp.login(sender, password)\n"
            "    smtp.send_message(msg)\n";

        char *const argv[] = {
            "python3",
            "-c",
            (char *)python_code,
            frame_id,
            score,
            dominant_freq_y,
            dominant_freq_z,
            rms_total,
            ratio_2x_1x_y,
            ratio_3x_1x_y,
            NULL,
        };
        char *const envp[] = {
            env_smtp_server,
            env_smtp_port,
            env_sender_email,
            env_sender_password,
            env_recipient_email,
            "PATH=/usr/bin:/bin",
            NULL,
        };

        execvpe("python3", argv, envp);
        _exit(127);
    }

    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) {
            return -1;
        }
    }

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        return 0;
    }

    return -1;
}

/* -------------------------------------------------------------------------- */
/* Threads                                                                     */
/* -------------------------------------------------------------------------- */

/*
 * RT Sampling Thread:
 * 1 kHz absolute-time MPU6050 sampling, raw-to-g conversion, EMA high-pass,
 * and double-buffer frame handoff. No FFT, no printf, no file writes.
 */
static void *sampling_thread_main(void *arg)
{
    (void)arg;
    set_thread_realtime_priority("sampling", RT_SAMPLING_PRIORITY);

    float dc_x = 0.0f;
    float dc_y = 0.0f;
    float dc_z = 0.0f;
    struct timespec next_time;

    clock_gettime(CLOCK_MONOTONIC, &next_time);
    g_frame_buffers[g_active_buffer_index].frame_id = g_next_frame_id++;
    g_frame_buffers[g_active_buffer_index].sum_sq_x = 0.0f;
    g_frame_buffers[g_active_buffer_index].sum_sq_y = 0.0f;
    g_frame_buffers[g_active_buffer_index].sum_sq_z = 0.0f;
    g_frame_buffers[g_active_buffer_index].timestamp_start = next_time;

    while (atomic_load(&g_running)) {
        timespec_add_ns(&next_time, SAMPLE_PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);

        struct timespec actual_wakeup_time;
        clock_gettime(CLOCK_MONOTONIC, &actual_wakeup_time);
        latency_stats_update(&g_wakeup_jitter_stats,
                             timespec_diff_ns(&actual_wakeup_time, &next_time));

        int16_t raw_x = 0;
        int16_t raw_y = 0;
        int16_t raw_z = 0;
        struct timespec i2c_start;
        struct timespec i2c_end;

        clock_gettime(CLOCK_MONOTONIC, &i2c_start);
        int read_ret = mpu6050_read_accel(g_i2c_fd, &raw_x, &raw_y, &raw_z);
        clock_gettime(CLOCK_MONOTONIC, &i2c_end);
        latency_stats_update(&g_i2c_duration_stats,
                             timespec_diff_ns(&i2c_end, &i2c_start));

        if (read_ret < 0) {
            atomic_fetch_add(&g_i2c_errors, 1);
            continue;
        }

        float ax = (float)raw_x / ACCEL_SENSITIVITY_LSB_PER_G;
        float ay = (float)raw_y / ACCEL_SENSITIVITY_LSB_PER_G;
        float az = (float)raw_z / ACCEL_SENSITIVITY_LSB_PER_G;

        dc_x = (EMA_ALPHA * ax) + ((1.0f - EMA_ALPHA) * dc_x);
        dc_y = (EMA_ALPHA * ay) + ((1.0f - EMA_ALPHA) * dc_y);
        dc_z = (EMA_ALPHA * az) + ((1.0f - EMA_ALPHA) * dc_z);

        float ac_x = ax - dc_x;
        float ac_y = ay - dc_y;
        float ac_z = az - dc_z;

        accel_frame_t *active = &g_frame_buffers[g_active_buffer_index];
        int idx = g_sample_count;

        active->x[idx] = ac_x;
        active->y[idx] = ac_y;
        active->z[idx] = ac_z;
        active->sum_sq_x += ac_x * ac_x;
        active->sum_sq_y += ac_y * ac_y;
        active->sum_sq_z += ac_z * ac_z;
        g_sample_count++;

        if (g_sample_count < FFT_SIZE) {
            continue;
        }

        active->timestamp_end = next_time;

        struct timespec handoff_start;
        struct timespec handoff_end;

        clock_gettime(CLOCK_MONOTONIC, &handoff_start);
        pthread_mutex_lock(&g_frame_mutex);
        if (g_ready_buffer_index >= 0) {
            atomic_fetch_add(&g_sampling_overruns, 1);
        } else {
            g_ready_buffer_index = g_active_buffer_index;
            g_active_buffer_index = 1 - g_active_buffer_index;
            pthread_cond_signal(&g_frame_ready_cond);
        }
        pthread_mutex_unlock(&g_frame_mutex);
        clock_gettime(CLOCK_MONOTONIC, &handoff_end);
        latency_stats_update(&g_handoff_duration_stats,
                             timespec_diff_ns(&handoff_end, &handoff_start));

        g_sample_count = 0;
        accel_frame_t *next = &g_frame_buffers[g_active_buffer_index];
        next->frame_id = g_next_frame_id++;
        next->sum_sq_x = 0.0f;
        next->sum_sq_y = 0.0f;
        next->sum_sq_z = 0.0f;
        next->timestamp_start = next_time;
    }

    return NULL;
}

/*
 * DSP Processing Thread:
 * Waits for complete frames, copies them out of the double buffer, computes
 * three independent FFTs, features, baseline, and anomaly scores.
 */
static void *dsp_thread_main(void *arg)
{
    (void)arg;
    set_thread_realtime_priority("dsp", DSP_THREAD_PRIORITY);

    baseline_model_t baseline;
    baseline_accumulator_t baseline_acc;
    score_smoother_t score_smoother;
    health_confirmation_t health_confirmation;
    system_mode_t mode = MODE_MONITORING;
    uint64_t last_frame_id = 0;

    memset(&baseline, 0, sizeof(baseline));
    baseline_accumulator_reset(&baseline_acc);
    score_smoother_reset(&score_smoother);
    health_confirmation_reset(&health_confirmation);

    while (atomic_load(&g_running)) {
        accel_frame_t frame;

        pthread_mutex_lock(&g_frame_mutex);
        while (g_ready_buffer_index < 0 && atomic_load(&g_running)) {
            pthread_cond_wait(&g_frame_ready_cond, &g_frame_mutex);
        }

        if (!atomic_load(&g_running)) {
            pthread_mutex_unlock(&g_frame_mutex);
            break;
        }

        int ready_index = g_ready_buffer_index;
        frame = g_frame_buffers[ready_index];
        g_ready_buffer_index = -1;
        pthread_mutex_unlock(&g_frame_mutex);

        if (last_frame_id != 0 && frame.frame_id != last_frame_id + 1) {
            atomic_fetch_add(&g_missed_frames, frame.frame_id - last_frame_id - 1);
        }
        last_frame_id = frame.frame_id;

        float spectrum_x[SPECTRUM_BINS];
        float spectrum_y[SPECTRUM_BINS];
        float spectrum_z[SPECTRUM_BINS];
        vibration_features_t features;

        compute_axis_spectrum(frame.x, spectrum_x);
        compute_axis_spectrum(frame.y, spectrum_y);
        compute_axis_spectrum(frame.z, spectrum_z);
        extract_features(&frame, spectrum_x, spectrum_y, spectrum_z, &features);

        if (atomic_exchange(&g_baseline_request, false)) {
            mode = MODE_BASELINE_COLLECTING;
            baseline_accumulator_reset(&baseline_acc);
            score_smoother_reset(&score_smoother);
            health_confirmation_reset(&health_confirmation);
        }

        analysis_result_t result;
        memset(&result, 0, sizeof(result));
        result.frame_id = frame.frame_id;
        result.mode = mode;
        result.features = features;
        result.sampling_overruns = atomic_load(&g_sampling_overruns);
        result.missed_frames = atomic_load(&g_missed_frames);
        result.baseline_total = BASELINE_FRAMES;
        result.baseline_valid = baseline.valid;
        result.wakeup_jitter = latency_stats_snapshot(&g_wakeup_jitter_stats);
        result.i2c_duration = latency_stats_snapshot(&g_i2c_duration_stats);
        result.handoff_duration = latency_stats_snapshot(&g_handoff_duration_stats);

        if (mode == MODE_BASELINE_COLLECTING) {
            baseline_accumulator_add(&baseline_acc, &features, spectrum_y, spectrum_z);
            result.baseline_progress = baseline_acc.frame_count;
            result.health = HEALTH_BASELINE_COLLECTING;

            if (baseline_acc.frame_count >= BASELINE_FRAMES) {
                baseline_model_finalize(&baseline_acc, &baseline);
                score_smoother_reset(&score_smoother);
                health_confirmation_reset(&health_confirmation);
                mode = MODE_MONITORING;
                result.mode = MODE_MONITORING;
                result.baseline_valid = baseline.valid;
            }
        } else {
            float raw_score = compute_anomaly_score(&features,
                                                    &baseline,
                                                    &g_anomaly_weights);
            features.anomaly_score = smooth_anomaly_score(&score_smoother, raw_score);
            result.features = features;
            health_state_t raw_health = classify_health_raw(features.anomaly_score,
                                                            baseline.valid);
            result.health = update_health_confirmation(&health_confirmation,
                                                       raw_health);
        }

        publish_result(&result);
    }

    return NULL;
}

/*
 * Button Thread:
 * Polls an active-low GPIO button with debounce. A press only sets the atomic
 * baseline request flag; baseline work happens later in DSP at frame boundary.
 */
static void *button_thread_main(void *arg)
{
    (void)arg;
    set_thread_realtime_priority("button", BUTTON_THREAD_PRIORITY);

    button_handle_t button;
    if (button_init(&button, BUTTON_GPIO_CHIP, BUTTON_GPIO_LINE) < 0) {
        fprintf(stderr,
                "[WARN] Button disabled. Check %s line %d permissions/pull-up support.\n",
                BUTTON_GPIO_CHIP, BUTTON_GPIO_LINE);
        return NULL;
    }

    bool was_pressed = false;
    struct timespec last_press = { 0, 0 };

    while (atomic_load(&g_running)) {
        bool pressed = button_is_pressed(&button);
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        long elapsed_ms = (now.tv_sec - last_press.tv_sec) * 1000L +
                          (now.tv_nsec - last_press.tv_nsec) / 1000000L;

        if (pressed && !was_pressed && elapsed_ms >= BUTTON_DEBOUNCE_MS) {
            atomic_store(&g_baseline_request, true);
            last_press = now;
        }

        was_pressed = pressed;
        usleep(BUTTON_POLL_MS * 1000);
    }

    button_close(&button);
    return NULL;
}

/*
 * Alert Email Thread:
 * Owns SMTP notification work. It is intentionally outside all realtime paths;
 * SMTP login/network delays must never block sampling, DSP, or frame handoff.
 */
static void *alert_email_thread_main(void *arg)
{
    (void)arg;

    while (atomic_load(&g_running)) {
        alert_email_request_t request;

        pthread_mutex_lock(&g_alert_email_mutex);
        while (!g_alert_email_pending && atomic_load(&g_running)) {
            pthread_cond_wait(&g_alert_email_cond, &g_alert_email_mutex);
        }

        if (!atomic_load(&g_running)) {
            pthread_mutex_unlock(&g_alert_email_mutex);
            break;
        }

        request = g_alert_email_request;
        g_alert_email_pending = false;
        pthread_mutex_unlock(&g_alert_email_mutex);

        printf("[MAIL] sending ALERT email frame=%" PRIu64 " score=%.2f\n",
               request.frame_id,
               request.score);
        if (send_alert_email_child(&request) == 0) {
            printf("[MAIL] ALERT email sent frame=%" PRIu64 "\n", request.frame_id);
        } else {
            printf("[MAIL] ALERT email failed frame=%" PRIu64
                   " Set FAN_ALERT_EMAIL_PASSWORD to a Gmail app password.\n",
                   request.frame_id);
        }
    }

    return NULL;
}

/*
 * Logging Thread:
 * Owns stdout so realtime sampling never blocks on terminal I/O.
 */
static void *logging_thread_main(void *arg)
{
    (void)arg;
    set_thread_realtime_priority("logging", LOGGING_THREAD_PRIORITY);

    bool alert_email_armed = true;
    uint64_t last_alert_email_frame = 0;

    while (atomic_load(&g_running)) {
        analysis_result_t result;

        pthread_mutex_lock(&g_log_mutex);
        while (!g_has_new_result && atomic_load(&g_running)) {
            pthread_cond_wait(&g_log_cond, &g_log_mutex);
        }

        if (!atomic_load(&g_running)) {
            pthread_mutex_unlock(&g_log_mutex);
            break;
        }

        result = g_latest_result;
        g_has_new_result = false;
        pthread_mutex_unlock(&g_log_mutex);

        if (result.health == HEALTH_ALERT && alert_email_armed) {
            uint64_t frames_since_last = result.frame_id - last_alert_email_frame;
            if (last_alert_email_frame == 0 ||
                frames_since_last >= ALERT_EMAIL_COOLDOWN_FRAMES) {
                queue_alert_email(&result);
                last_alert_email_frame = result.frame_id;
            }
            alert_email_armed = false;
        } else if (result.health != HEALTH_ALERT) {
            alert_email_armed = true;
        }

        if (result.health == HEALTH_BASELINE_COLLECTING) {
            printf("[BASELINE] frame=%" PRIu64 " progress=%d/%d "
                   "Y=%.2fHz Z=%.2fHz rms=%.5fg "
                   "Yratio2=%.3f Yratio3=%.3f "
                   "Zratio2=%.3f Zratio3=%.3f\n",
                   result.frame_id,
                   result.baseline_progress,
                   result.baseline_total,
                   result.features.dominant_freq_y,
                   result.features.dominant_freq_z,
                   result.features.rms_total,
                   result.features.ratio_2x_1x_y,
                   result.features.ratio_3x_1x_y,
                   result.features.ratio_2x_1x_z,
                   result.features.ratio_3x_1x_z);
            print_rt_latency_summary(&result);
            continue;
        }

        printf("[%s] frame=%" PRIu64 " score=%.2f baseline=%s "
               "scoreAxis=Y domHz(x/y/z)=%.1f/%.1f/%.1f rpm1x=%.0f "
               "rms(x/y/z/t)=%.5f/%.5f/%.5f/%.5f "
               "Yharm(A1/A2/A3)=%.5f/%.5f/%.5f Yratio2=%.3f Yratio3=%.3f "
               "Zharm(A1/A2/A3)=%.5f/%.5f/%.5f Zratio2=%.3f Zratio3=%.3f "
               "\n",
               health_state_name(result.health),
               result.frame_id,
               result.features.anomaly_score,
               result.baseline_valid ? "yes" : "no",
               result.features.dominant_freq_x,
               result.features.dominant_freq_y,
               result.features.dominant_freq_z,
               result.features.rpm_1x_estimate,
               result.features.rms_x,
               result.features.rms_y,
               result.features.rms_z,
               result.features.rms_total,
               result.features.amp_1x_y,
               result.features.amp_2x_y,
               result.features.amp_3x_y,
               result.features.ratio_2x_1x_y,
               result.features.ratio_3x_1x_y,
               result.features.amp_1x_z,
               result.features.amp_2x_z,
               result.features.amp_3x_z,
               result.features.ratio_2x_1x_z,
               result.features.ratio_3x_1x_z);
        print_rt_latency_summary(&result);
    }

    return NULL;
}

/* -------------------------------------------------------------------------- */
/* Program entry                                                               */
/* -------------------------------------------------------------------------- */

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);

    printf("Fan vibration RT monitor starting\n");
    printf("I2C: %s addr=0x%02X | GPIO button: %s line %d active-low\n",
           I2C_DEVICE_PATH, MPU6050_I2C_ADDR, BUTTON_GPIO_CHIP, BUTTON_GPIO_LINE);
    printf("Frame: %d samples @ %d Hz = %.3f s | FFT resolution = %.6f Hz\n",
           FFT_SIZE,
           SAMPLE_RATE_HZ,
           (float)FFT_SIZE / (float)SAMPLE_RATE_HZ,
           (float)SAMPLE_RATE_HZ / (float)FFT_SIZE);
    printf("Press the button after the fan speed is stable to collect %d baseline frames.\n",
           BASELINE_FRAMES);

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    g_i2c_fd = mpu6050_init(I2C_DEVICE_PATH);
    if (g_i2c_fd < 0) {
        return EXIT_FAILURE;
    }

    pthread_t sampling_thread;
    pthread_t dsp_thread;
    pthread_t button_thread;
    pthread_t logging_thread;
    pthread_t alert_email_thread;

    if (pthread_create(&alert_email_thread, NULL, alert_email_thread_main, NULL) != 0 ||
        pthread_create(&logging_thread, NULL, logging_thread_main, NULL) != 0 ||
        pthread_create(&dsp_thread, NULL, dsp_thread_main, NULL) != 0 ||
        pthread_create(&button_thread, NULL, button_thread_main, NULL) != 0 ||
        pthread_create(&sampling_thread, NULL, sampling_thread_main, NULL) != 0) {
        perror("pthread_create");
        atomic_store(&g_running, false);
        pthread_cond_broadcast(&g_alert_email_cond);
        close(g_i2c_fd);
        return EXIT_FAILURE;
    }

    pthread_join(sampling_thread, NULL);
    pthread_join(dsp_thread, NULL);
    pthread_join(button_thread, NULL);
    pthread_join(logging_thread, NULL);
    pthread_join(alert_email_thread, NULL);

    close(g_i2c_fd);

    latency_snapshot_t wakeup = latency_stats_snapshot(&g_wakeup_jitter_stats);
    latency_snapshot_t i2c = latency_stats_snapshot(&g_i2c_duration_stats);
    latency_snapshot_t handoff = latency_stats_snapshot(&g_handoff_duration_stats);

    printf("Stopped. I2C errors=%" PRIu64 ", overruns=%" PRIu64 ", missed=%" PRIu64 "\n",
           (uint64_t)atomic_load(&g_i2c_errors),
           (uint64_t)atomic_load(&g_sampling_overruns),
           (uint64_t)atomic_load(&g_missed_frames));
    printf("RT latency avg/max: wakeup=%.1f/%.1fus i2c=%.1f/%.1fus "
           "handoff=%.1f/%.1fus\n",
           (double)wakeup.avg_abs_ns / 1000.0,
           (double)wakeup.max_ns / 1000.0,
           (double)i2c.avg_abs_ns / 1000.0,
           (double)i2c.max_ns / 1000.0,
           (double)handoff.avg_abs_ns / 1000.0,
           (double)handoff.max_ns / 1000.0);
    printf("RT latency over thresholds: "
           "wakeup>100/500/1000us=%" PRIu64 "/%" PRIu64 "/%" PRIu64 " "
           "i2c>100/500/1000us=%" PRIu64 "/%" PRIu64 "/%" PRIu64 " "
           "handoff>100/500/1000us=%" PRIu64 "/%" PRIu64 "/%" PRIu64 "\n",
           wakeup.over_100us,
           wakeup.over_500us,
           wakeup.over_1ms,
           i2c.over_100us,
           i2c.over_500us,
           i2c.over_1ms,
           handoff.over_100us,
           handoff.over_500us,
           handoff.over_1ms);

    return EXIT_SUCCESS;
}
