/* ESP32-S3 UVC camera: red ball -> left blue goal entrance. */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "tjpgd.h"
#include "teal_ball_color.h"

#define CAM_W 640
#define CAM_H 480
#define CAM_FPS 25
#define IMG_W 320
#define IMG_H 240

/* Set to 1 only after direction and recognition are verified with wheels lifted. */
#define MOTOR_OUTPUT_ENABLED 1
#define TURN_START_BOOST_PWM 10
#define START_BOOST_FRAMES 1
#define BASE_PWM 44
#define START_BOOST_PWM 16
#define TRACK_DRIVE_FRAMES 1
#define TRACK_PAUSE_FRAMES 2
#define CORNER_SEARCH_PWM 33
#define CORNER_SEARCH_TURN_FRAMES 1
#define CORNER_SEARCH_PAUSE_FRAMES 3

/* Existing motor wiring. */
#define STBY_PIN 10
#define PWMA_PIN 11
#define AIN1_PIN 12
#define AIN2_PIN 13
#define PWMB_PIN 14
#define BIN1_PIN 15
#define BIN2_PIN 16
#define PWMD_PIN 17
#define DIN1_PIN 18
#define DIN2_PIN 21

/* HC-SR04P wiring used by the verified Arduino/camera projects. */
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_DISTANCE_CM 400.0f
#define ULTRASONIC_TIMEOUT_US 30000
#define ULTRASONIC_INTERVAL_MS 50
#define CAPTURE_NEAR_CM 10.0f
#define CAPTURE_RELEASE_CM 25.0f

/* Red ball mask thresholds, applied directly to RGB888 pixels. */
#define RED_R_MIN 80
#define RED_RG_MIN_DIFF 35
#define RED_RB_MIN_DIFF 35
#define RED_DOMINANCE_PERCENT 125

/* Blue goal mask thresholds, applied directly to RGB888 pixels. */
#define BLUE_B_MIN 60
#define BLUE_BR_MIN_DIFF 35
#define BLUE_BG_MIN_DIFF 15
#define BLUE_DOMINANCE_PERCENT 120
#define BLUE_MIN_AREA 40
#define BLUE_MAX_AREA 30000

/* Ball and motion tuning. */
#define BALL_MIN_AREA 90
#define BALL_MAX_AREA 20000
#define CIRCULARITY_THRESH 0.5f
#define BALL_TARGET_X (IMG_W / 2)
#define BALL_CENTER_DEADBAND 24
#define BALL_ALIGN_CONFIRM_FRAMES 2
#define GOAL_CENTER_DEADBAND 20
#define BALL_CONFIRM_FRAMES 5
#define BALL_LOST_FRAMES 8
#define CAPTURE_CONFIRM_FRAMES 3
#define CAPTURE_TIMEOUT_MS 2500
#define CAPTURE_MONITOR_MISSES 30
#define BALL_NEAR_AREA 180

#define SEARCH_SWITCH_MS 1200
#define GOAL_SCAN_SWITCH_MS 1400
#define GOAL_CONFIRM_FRAMES 3
#define GOAL_LOST_FRAMES 8
#define GOAL_NEAR_AREA 900
#define GOAL_NEAR_Y 280
#define GOAL_PUSH_DURATION_MS 700
#define GOAL_VERIFY_HOLD_MS 350

#define FRAME_BUFFERS 3
#define FRAME_SIZE (CAM_W * CAM_H * 2)
#define VISION_PROCESS_EVERY_N_FRAMES 2
#define USB_PRIORITY 15
#define WIFI_AP_SSID "ESP32-RedBall"
#define WIFI_AP_PASSWORD "redball123"

static const char *TAG = "red_left_goal";

typedef struct { const uint8_t *data; size_t size, pos; } jpeg_src_t;
typedef struct {
    bool found;
    int area;
    int cx, cy;
    int min_x, min_y, max_x, max_y;
    float circularity;
} blob_t;
typedef struct {
    bool decoded;
    bool found_ball;
    bool found_goal;
    bool captured;
    int ball_x, ball_y, ball_area;
    int ball_error;
    float ball_circularity;
    int ball_min_x, ball_min_y, ball_max_x, ball_max_y;
    int goal_x, goal_y, goal_area;
    int goal_min_x, goal_min_y, goal_max_x, goal_max_y;
    float distance_cm;
    char state[24];
} status_t;
typedef enum {
    STATE_IDLE,
    STATE_SEARCH_RED,
    STATE_ALIGN_RED,
    STATE_APPROACH_RED,
    STATE_CAPTURE_RED,
    STATE_FIND_GOAL,
    STATE_GOAL_SCAN,
    STATE_GOAL_ALIGN,
    STATE_GOAL_APPROACH,
    STATE_GOAL_PUSH,
    STATE_GOAL_VERIFY,
    STATE_ALL_DONE,
    STATE_FAIL_SAFE
} state_t;

static QueueHandle_t s_frame_q;
static uvc_host_stream_hdl_t s_stream;
static volatile bool s_connected;
static volatile bool s_vision_task_running;
static volatile float s_distance_cm = MAX_DISTANCE_CM;
static volatile bool s_distance_valid;
static volatile uint32_t s_distance_sequence;
static uint8_t *s_rgb;
static uint8_t *s_mask;
static uint32_t *s_flood_queue;
static uint8_t *s_jpeg_buffers[2];
static size_t s_jpeg_sizes[2];
static int s_jpeg_index;
static uint32_t s_jpeg_version;
static SemaphoreHandle_t s_jpeg_lock;
static SemaphoreHandle_t s_status_lock;
static httpd_handle_t s_http_server;
static status_t s_status;
static state_t s_state = STATE_IDLE;
static uint32_t s_state_started_ms;
static bool s_ball_captured;
static int s_capture_monitor_misses;
static int s_capture_near_frames;
static int s_last_ball_error;

static const char *state_name(state_t state)
{
    switch (state) {
        case STATE_IDLE: return "IDLE";
        case STATE_SEARCH_RED: return "SEARCH_RED";
        case STATE_ALIGN_RED: return "ALIGN_RED";
        case STATE_APPROACH_RED: return "APPROACH_RED";
        case STATE_CAPTURE_RED: return "CAPTURE_RED";
        case STATE_FIND_GOAL: return "FIND_GOAL";
        case STATE_GOAL_SCAN: return "GOAL_SCAN";
        case STATE_GOAL_ALIGN: return "GOAL_ALIGN";
        case STATE_GOAL_APPROACH: return "GOAL_APPROACH";
        case STATE_GOAL_PUSH: return "GOAL_PUSH";
        case STATE_GOAL_VERIFY: return "GOAL_VERIFY_DEFAULT_SUCCESS";
        case STATE_ALL_DONE: return "ALL_DONE";
        case STATE_FAIL_SAFE: return "FAIL_SAFE";
        default: return "UNKNOWN";
    }
}

static uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static int clampi(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

static void enter_state(state_t next)
{
    if (s_state == next) return;
    s_state = next;
    s_state_started_ms = now_ms();
    ESP_LOGI(TAG, "state -> %s", state_name(next));
}

static void motor_channel_init(ledc_channel_t channel, int pin)
{
    ledc_channel_config_t config = {
        .gpio_num = pin, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = channel,
        .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&config));
}

static void motors_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << STBY_PIN) | (1ULL << AIN1_PIN) | (1ULL << AIN2_PIN) |
                        (1ULL << BIN1_PIN) | (1ULL << BIN2_PIN) |
                        (1ULL << DIN1_PIN) | (1ULL << DIN2_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(STBY_PIN, 0);
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0, .freq_hz = 10000, .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    motor_channel_init(LEDC_CHANNEL_0, PWMA_PIN);
    motor_channel_init(LEDC_CHANNEL_1, PWMB_PIN);
    motor_channel_init(LEDC_CHANNEL_2, PWMD_PIN);
    ESP_LOGW(TAG, "motor output: %s", MOTOR_OUTPUT_ENABLED ? "ENABLED" : "DRY RUN");
}

static void set_motor(ledc_channel_t channel, int in1, int in2, int pwm)
{
    pwm = clampi(pwm, -255, 255);
    gpio_set_level(in1, pwm > 0);
    gpio_set_level(in2, pwm < 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, abs(pwm)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

typedef enum {
    MOTION_NONE,
    MOTION_STRAIGHT,
    MOTION_TURN
} motion_mode_t;

static motion_mode_t s_motion_mode;
static int s_motion_direction;
static unsigned s_motion_phase;
static bool s_brake_active;
static int s_start_boost_frames;
static int s_turn_start_boost_frames;

static void drive_pair(int left, int right)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    gpio_set_level(STBY_PIN, left != 0 || right != 0);
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, left);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, right);
}

static void stop_drive(void)
{
    s_start_boost_frames = 0;
    s_turn_start_boost_frames = 0;
    s_brake_active = false;
    gpio_set_level(STBY_PIN, 0);
    set_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN, 0);
    set_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN, 0);
}

static void drive_straight_fixed(void)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    if (s_brake_active) {
        s_start_boost_frames = START_BOOST_FRAMES;
        s_brake_active = false;
    }
    if (s_start_boost_frames > 0) {
        int pwm = BASE_PWM + START_BOOST_PWM;
        s_start_boost_frames--;
        drive_pair(pwm, pwm);
    } else {
        drive_pair(BASE_PWM, BASE_PWM);
    }
}

static void drive_turn_pulse(int steering, bool pulse_start)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    int left = steering;
    int right = -steering;
    if (pulse_start) s_turn_start_boost_frames = START_BOOST_FRAMES;
    if (s_turn_start_boost_frames > 0) {
        if (left) left += left > 0 ? TURN_START_BOOST_PWM : -TURN_START_BOOST_PWM;
        if (right) right += right > 0 ? TURN_START_BOOST_PWM : -TURN_START_BOOST_PWM;
        s_turn_start_boost_frames--;
    }
    drive_pair(left, right);
}

static void brake_motor(ledc_channel_t channel, int in1, int in2)
{
    gpio_set_level(in1, 1);
    gpio_set_level(in2, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 255));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
}

static void brake_drive(void)
{
    if (!MOTOR_OUTPUT_ENABLED) return;
    s_brake_active = true;
    gpio_set_level(STBY_PIN, 1);
    brake_motor(LEDC_CHANNEL_0, AIN1_PIN, AIN2_PIN);
    brake_motor(LEDC_CHANNEL_1, BIN1_PIN, BIN2_PIN);
    brake_motor(LEDC_CHANNEL_2, DIN1_PIN, DIN2_PIN);
}

static void motion_begin(motion_mode_t mode, int direction)
{
    if (s_motion_mode != mode ||
        (mode == MOTION_TURN && s_motion_direction != direction)) {
        s_motion_mode = mode;
        s_motion_direction = direction;
        s_motion_phase = 0;
        s_turn_start_boost_frames = 0;
    }
}

static void motion_stop(void)
{
    stop_drive();
    s_motion_mode = MOTION_NONE;
    s_motion_direction = 0;
    s_motion_phase = 0;
}

typedef enum {
    TURN_LEFT = -1,
    TURN_RIGHT = 1
} turn_direction_t;

/* Public commands. PWM and pulse timing are fixed from the tested programs. */
static void command_straight(void)
{
    motion_begin(MOTION_STRAIGHT, 0);
    const unsigned cycle = TRACK_DRIVE_FRAMES + TRACK_PAUSE_FRAMES;
    const unsigned phase = s_motion_phase++ % cycle;
    if (phase < TRACK_DRIVE_FRAMES) drive_straight_fixed();
    else brake_drive();
}

static void command_turn(turn_direction_t direction)
{
    motion_begin(MOTION_TURN, direction);
    const unsigned cycle = CORNER_SEARCH_TURN_FRAMES + CORNER_SEARCH_PAUSE_FRAMES;
    const unsigned phase = s_motion_phase++ % cycle;
    if (phase < CORNER_SEARCH_TURN_FRAMES)
        drive_turn_pulse(direction * CORNER_SEARCH_PWM, phase == 0);
    else
        brake_drive();
}

static void stop_motors(void)
{
    motion_stop();
}

static size_t jpeg_input(JDEC *jd, uint8_t *buf, size_t len)
{
    jpeg_src_t *src = jd->device;
    size_t available = src->size - src->pos;
    if (len > available) len = available;
    if (buf) memcpy(buf, src->data + src->pos, len);
    src->pos += len;
    return len;
}

static int jpeg_output(JDEC *jd, void *bitmap, JRECT *rect)
{
    (void)jd;
    uint8_t *src = bitmap;
    const unsigned width = rect->right - rect->left + 1;
    for (unsigned y = rect->top; y <= rect->bottom && y < IMG_H; ++y) {
        for (unsigned x = rect->left; x <= rect->right && x < IMG_W; ++x) {
            unsigned src_index = ((y - rect->top) * width + (x - rect->left)) * 3;
            unsigned dst_index = (y * IMG_W + x) * 3;
            s_rgb[dst_index] = src[src_index];
            s_rgb[dst_index + 1] = src[src_index + 1];
            s_rgb[dst_index + 2] = src[src_index + 2];
        }
    }
    return 1;
}

static bool decode_jpeg(const uint8_t *data, size_t size)
{
    uint8_t work[4096];
    JDEC decoder;
    jpeg_src_t source = {.data = data, .size = size, .pos = 0};
    memset(s_rgb, 255, IMG_W * IMG_H * 3);
    if (jd_prepare(&decoder, jpeg_input, work, sizeof(work), &source) != JDR_OK) return false;
    if (decoder.width != CAM_W || decoder.height != CAM_H) return false;
    return jd_decomp(&decoder, jpeg_output, 1) == JDR_OK;
}

static bool is_blue_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    int blue = b;
    return blue >= BLUE_B_MIN &&
           blue - (int)r >= BLUE_BR_MIN_DIFF &&
           blue - (int)g >= BLUE_BG_MIN_DIFF &&
           blue * 100 >= (int)r * BLUE_DOMINANCE_PERCENT &&
           blue * 100 >= (int)g * BLUE_DOMINANCE_PERCENT;
}

static bool is_red_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return r >= RED_R_MIN &&
           (int)r - (int)g >= RED_RG_MIN_DIFF &&
           (int)r - (int)b >= RED_RB_MIN_DIFF &&
           (int)r * 100 >= (int)g * RED_DOMINANCE_PERCENT &&
           (int)r * 100 >= (int)b * RED_DOMINANCE_PERCENT;
}

static bool is_ball_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return is_red_rgb(r, g, b) || is_teal_ball_rgb(r, g, b);
}

static blob_t flood_blob(int sx, int sy)
{
    blob_t out = {.min_x = IMG_W, .min_y = IMG_H, .max_x = -1, .max_y = -1};
    int start = sy * IMG_W + sx;
    if (s_mask[start] != 1) return out;
    size_t head = 0, tail = 0;
    int sum_x = 0, sum_y = 0;
    s_mask[start] = 2;
    s_flood_queue[tail++] = (uint32_t)start;
    while (head < tail) {
        int p = (int)s_flood_queue[head++];
        int x = p % IMG_W, y = p / IMG_W;
        out.area++;
        sum_x += x; sum_y += y;
        if (x < out.min_x) out.min_x = x;
        if (x > out.max_x) out.max_x = x;
        if (y < out.min_y) out.min_y = y;
        if (y > out.max_y) out.max_y = y;
        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int i = 0; i < 4; ++i) {
            if (nx[i] < 0 || nx[i] >= IMG_W || ny[i] < 0 || ny[i] >= IMG_H) continue;
            int np = ny[i] * IMG_W + nx[i];
            if (s_mask[np] == 1) {
                s_mask[np] = 2;
                s_flood_queue[tail++] = (uint32_t)np;
            }
        }
    }
    if (out.area > 0) {
        out.cx = sum_x / out.area;
        out.cy = sum_y / out.area;
        int perimeter = 0;
        for (size_t i = 0; i < tail; ++i) {
            int p = (int)s_flood_queue[i];
            int x = p % IMG_W, y = p / IMG_W;
            const int nx[4] = {x - 1, x + 1, x, x};
            const int ny[4] = {y, y, y - 1, y + 1};
            for (int j = 0; j < 4; ++j) {
                if (nx[j] < 0 || nx[j] >= IMG_W || ny[j] < 0 || ny[j] >= IMG_H ||
                    s_mask[ny[j] * IMG_W + nx[j]] != 2) {
                    perimeter++;
                }
            }
        }
        out.circularity = perimeter > 0
            ? (float)(4.0 * M_PI * out.area / ((double)perimeter * perimeter))
            : 0.0f;
    }
    return out;
}

static blob_t detect_blob(bool red, bool choose_leftmost)
{
    memset(s_mask, 0, IMG_W * IMG_H);
    for (int y = 0; y < IMG_H; ++y) {
        for (int x = 0; x < IMG_W; ++x) {
            int p = (y * IMG_W + x) * 3;
            bool selected;
            if (red) {
                selected = is_ball_rgb(s_rgb[p], s_rgb[p + 1], s_rgb[p + 2]);
            } else {
                selected = is_blue_rgb(s_rgb[p], s_rgb[p + 1], s_rgb[p + 2]);
            }
            s_mask[y * IMG_W + x] = selected ? 1 : 0;
        }
    }
    blob_t best = {.min_x = IMG_W, .min_y = IMG_H, .max_x = -1, .max_y = -1};
    for (int y = 0; y < IMG_H; ++y) {
        for (int x = 0; x < IMG_W; ++x) {
            if (s_mask[y * IMG_W + x] != 1) continue;
            blob_t candidate = flood_blob(x, y);
            int min_area = red ? BALL_MIN_AREA : BLUE_MIN_AREA;
            int max_area = red ? BALL_MAX_AREA : BLUE_MAX_AREA;
            if (candidate.area < min_area || candidate.area > max_area) continue;
            if (red && candidate.circularity < CIRCULARITY_THRESH) continue;
            bool better = false;
            if (!best.found) better = true;
            else if (choose_leftmost) better = candidate.cx < best.cx;
            else better = candidate.area > best.area;
            if (better) {
                candidate.found = true;
                best = candidate;
            }
        }
    }
    return best;
}

static void publish_jpeg(const uvc_host_frame_t *frame)
{
    if (!frame || !frame->data || frame->data_len < 4 || frame->data_len > FRAME_SIZE) return;
    if (frame->data[0] != 0xff || frame->data[1] != 0xd8 ||
        frame->data[frame->data_len - 2] != 0xff || frame->data[frame->data_len - 1] != 0xd9) return;
    if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(10)) != pdPASS) return;
    int next = s_jpeg_index ^ 1;
    memcpy(s_jpeg_buffers[next], frame->data, frame->data_len);
    s_jpeg_sizes[next] = frame->data_len;
    s_jpeg_index = next;
    s_jpeg_version++;
    xSemaphoreGive(s_jpeg_lock);
}

static void update_status(bool decoded, blob_t ball, blob_t goal)
{
    if (xSemaphoreTake(s_status_lock, pdMS_TO_TICKS(10)) != pdPASS) return;
    s_status.decoded = decoded;
    s_status.found_ball = ball.found;
    s_status.found_goal = goal.found;
    s_status.captured = s_ball_captured;
    s_status.ball_x = ball.found ? ball.cx : -1;
    s_status.ball_y = ball.found ? ball.cy : -1;
    s_status.ball_area = ball.found ? ball.area : 0;
    s_status.ball_error = ball.found ? ball.cx - BALL_TARGET_X : 0;
    s_status.ball_circularity = ball.found ? ball.circularity : 0.0f;
    s_status.ball_min_x = ball.found ? ball.min_x : -1;
    s_status.ball_min_y = ball.found ? ball.min_y : -1;
    s_status.ball_max_x = ball.found ? ball.max_x : -1;
    s_status.ball_max_y = ball.found ? ball.max_y : -1;
    s_status.goal_x = goal.found ? goal.cx : -1;
    s_status.goal_y = goal.found ? goal.cy : -1;
    s_status.goal_area = goal.found ? goal.area : 0;
    s_status.goal_min_x = goal.found ? goal.min_x : -1;
    s_status.goal_min_y = goal.found ? goal.min_y : -1;
    s_status.goal_max_x = goal.found ? goal.max_x : -1;
    s_status.goal_max_y = goal.found ? goal.max_y : -1;
    s_status.distance_cm = s_distance_cm;
    snprintf(s_status.state, sizeof(s_status.state), "%s", state_name(s_state));
    xSemaphoreGive(s_status_lock);
}

static bool read_ultrasonic(float *distance_cm)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start >= ULTRASONIC_TIMEOUT_US) return false;
    }
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) != 0) {
        if (esp_timer_get_time() - echo_start >= ULTRASONIC_TIMEOUT_US) return false;
    }
    uint32_t duration = (uint32_t)(esp_timer_get_time() - echo_start);
    *distance_cm = clampi((int)(duration * 0.0343f / 2.0f), 0, (int)MAX_DISTANCE_CM);
    return true;
}

static void ultrasonic_task(void *arg)
{
    (void)arg;
    gpio_config_t io = {.pin_bit_mask = 1ULL << TRIG_PIN, .mode = GPIO_MODE_OUTPUT};
    ESP_ERROR_CHECK(gpio_config(&io));
    io.pin_bit_mask = 1ULL << ECHO_PIN;
    io.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&io));
    while (true) {
        float total = 0.0f, sample = 0.0f;
        int valid = 0;
        for (int i = 0; i < 3; ++i) {
            if (read_ultrasonic(&sample)) { total += sample; valid++; }
            vTaskDelay(pdMS_TO_TICKS(8));
        }
        s_distance_valid = valid > 0;
        s_distance_cm = valid ? total / valid : MAX_DISTANCE_CM;
        s_distance_sequence++;
        vTaskDelay(pdMS_TO_TICKS(ULTRASONIC_INTERVAL_MS));
    }
}

static void update_capture_monitor(void)
{
    if (!s_ball_captured || s_state == STATE_GOAL_PUSH || s_state == STATE_GOAL_VERIFY ||
        s_state == STATE_ALL_DONE) return;
    if (s_distance_valid && s_distance_cm > CAPTURE_RELEASE_CM) {
        s_capture_monitor_misses++;
        if (s_capture_monitor_misses == 1)
            ESP_LOGW(TAG, "captured ball distance became %.1f cm", s_distance_cm);
    } else {
        s_capture_monitor_misses = 0;
    }
    if (s_capture_monitor_misses >= CAPTURE_MONITOR_MISSES) {
        stop_motors();
        ESP_LOGE(TAG, "captured ball monitoring failed");
        enter_state(STATE_FAIL_SAFE);
    }
}

static void run_controller(bool decoded, blob_t ball, blob_t goal)
{
    const uint32_t elapsed = now_ms() - s_state_started_ms;
    const float distance = s_distance_cm;
    update_capture_monitor();
    if (s_state == STATE_FAIL_SAFE) { stop_motors(); return; }

    switch (s_state) {
        case STATE_IDLE:
            s_ball_captured = false;
            s_capture_monitor_misses = 0;
            enter_state(STATE_SEARCH_RED);
            break;

        case STATE_SEARCH_RED: {
            static int seen = 0;
            static int scan_dir = -1; /* Start searching toward the left goal side. */
            if (ball.found) {
                if (++seen >= BALL_CONFIRM_FRAMES) {
                    seen = 0;
                    stop_motors();
                    enter_state(STATE_ALIGN_RED);
                } else stop_motors();
            } else {
                seen = 0;
                if (elapsed >= SEARCH_SWITCH_MS) {
                    scan_dir = -scan_dir;
                    s_state_started_ms = now_ms();
                }
                command_turn(scan_dir < 0 ? TURN_LEFT : TURN_RIGHT);
            }
            break;
        }

        case STATE_ALIGN_RED: {
            static int aligned_frames = 0;
            if (!ball.found) {
                aligned_frames = 0;
                stop_motors();
                if (elapsed >= BALL_LOST_FRAMES * 40) enter_state(STATE_SEARCH_RED);
                break;
            }
            s_last_ball_error = ball.cx - BALL_TARGET_X;
            if (abs(s_last_ball_error) <= BALL_CENTER_DEADBAND) {
                ESP_LOGI(TAG, "ALIGN_RED: centered ball_x=%d error=%d -> stop (%d/%d)",
                         ball.cx, s_last_ball_error, aligned_frames + 1,
                         BALL_ALIGN_CONFIRM_FRAMES);
                stop_motors();
                if (++aligned_frames >= BALL_ALIGN_CONFIRM_FRAMES) {
                    aligned_frames = 0;
                    enter_state(STATE_APPROACH_RED);
                    command_straight();
                }
            } else {
                aligned_frames = 0;
                turn_direction_t direction =
                    s_last_ball_error > 0 ? TURN_RIGHT : TURN_LEFT;
                ESP_LOGI(TAG, "ALIGN_RED: ball_x=%d error=%d -> %s pulse",
                         ball.cx, s_last_ball_error,
                         direction == TURN_RIGHT ? "RIGHT" : "LEFT");
                command_turn(direction);
            }
            break;
        }

        case STATE_APPROACH_RED:
            if (!ball.found) {
                stop_motors();
                if (s_distance_valid && distance <= CAPTURE_NEAR_CM) enter_state(STATE_CAPTURE_RED);
                else if (elapsed >= BALL_LOST_FRAMES * 40) enter_state(STATE_SEARCH_RED);
                break;
            }
            s_last_ball_error = ball.cx - BALL_TARGET_X;
            if (ball.area >= BALL_NEAR_AREA) {
                enter_state(STATE_CAPTURE_RED);
                command_straight();
            } else {
                if (abs(s_last_ball_error) <= BALL_CENTER_DEADBAND)
                    command_straight();
                else
                    command_turn(s_last_ball_error > 0 ? TURN_RIGHT : TURN_LEFT);
            }
            break;

        case STATE_CAPTURE_RED:
            if (ball.found) {
                s_capture_near_frames = 0;
                command_straight();
                if (elapsed >= CAPTURE_TIMEOUT_MS) {
                    stop_motors();
                    s_capture_near_frames = 0;
                    enter_state(STATE_SEARCH_RED);
                }
            } else if (s_distance_valid && distance <= CAPTURE_NEAR_CM) {
                if (++s_capture_near_frames >= CAPTURE_CONFIRM_FRAMES) {
                    s_capture_near_frames = 0;
                    s_ball_captured = true;
                    s_capture_monitor_misses = 0;
                    stop_motors();
                    ESP_LOGI(TAG, "red ball captured: camera lost ball, distance=%.1f cm", distance);
                    enter_state(STATE_FIND_GOAL);
                } else stop_motors();
            } else {
                stop_motors();
                s_capture_near_frames = 0;
                if (elapsed >= CAPTURE_TIMEOUT_MS) enter_state(STATE_SEARCH_RED);
            }
            break;

        case STATE_FIND_GOAL: {
            static int seen = 0;
            if (goal.found) {
                if (++seen >= GOAL_CONFIRM_FRAMES) {
                    seen = 0;
                    stop_motors();
                    enter_state(STATE_GOAL_ALIGN);
                }
            } else {
                seen = 0;
                enter_state(STATE_GOAL_SCAN);
            }
            break;
        }

        case STATE_GOAL_SCAN: {
            static int scan_dir = -1;
            if (goal.found) {
                stop_motors();
                enter_state(STATE_GOAL_ALIGN);
            } else if (elapsed >= GOAL_SCAN_SWITCH_MS) {
                scan_dir = -scan_dir;
                s_state_started_ms = now_ms();
            } else {
                command_turn(scan_dir < 0 ? TURN_LEFT : TURN_RIGHT);
            }
            break;
        }

        case STATE_GOAL_ALIGN:
            if (!goal.found) {
                stop_motors();
                if (elapsed >= GOAL_LOST_FRAMES * 40) enter_state(STATE_FIND_GOAL);
            } else {
                int error = goal.cx - IMG_W / 2;
                if (abs(error) <= GOAL_CENTER_DEADBAND) {
                    enter_state(STATE_GOAL_APPROACH);
                    command_straight();
                } else {
                    command_turn(error > 0 ? TURN_RIGHT : TURN_LEFT);
                }
            }
            break;

        case STATE_GOAL_APPROACH:
            if (!goal.found) {
                stop_motors();
                if (elapsed >= GOAL_LOST_FRAMES * 40) enter_state(STATE_FIND_GOAL);
            } else {
                int error = goal.cx - IMG_W / 2;
                if (goal.area >= GOAL_NEAR_AREA || goal.cy >= GOAL_NEAR_Y) {
                    enter_state(STATE_GOAL_PUSH);
                    command_straight();
                } else {
                    if (abs(error) <= GOAL_CENTER_DEADBAND)
                        command_straight();
                    else
                        command_turn(error > 0 ? TURN_RIGHT : TURN_LEFT);
                }
            }
            break;

        case STATE_GOAL_PUSH:
            if (elapsed < GOAL_PUSH_DURATION_MS) command_straight();
            else {
                stop_motors();
                enter_state(STATE_GOAL_VERIFY);
            }
            break;

        case STATE_GOAL_VERIFY:
            /* Intentionally blank for now: default to success after a short stop. */
            stop_motors();
            if (elapsed >= GOAL_VERIFY_HOLD_MS) {
                s_ball_captured = false;
                ESP_LOGW(TAG, "GOAL_VERIFY is temporarily default-success");
                enter_state(STATE_ALL_DONE);
            }
            break;

        case STATE_ALL_DONE:
            stop_motors();
            break;

        case STATE_FAIL_SAFE:
        default:
            stop_motors();
            break;
    }
    (void)decoded;
}

static bool frame_callback(const uvc_host_frame_t *frame, void *ctx)
{
    uvc_host_frame_t *mutable_frame = (uvc_host_frame_t *)frame;
    return xQueueSend((QueueHandle_t)ctx, &mutable_frame, 0) != pdPASS;
}

static void stream_callback(const uvc_host_stream_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DEVICE_DISCONNECTED) {
        s_connected = false;
        stop_motors();
        ESP_LOGW(TAG, "camera disconnected; motors stopped");
    }
}

static esp_err_t root_handler(httpd_req_t *req)
{
    static const char page[] =
        "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Red Ball Left Goal</title><style>*{box-sizing:border-box}body{font-family:Arial;margin:20px;background:#eef2f1;color:#17231f}"
        ".layout{display:grid;grid-template-columns:minmax(0,1fr) minmax(300px,360px);gap:20px;max-width:1440px;align-items:start}"
        ".media{min-width:0}.view{position:relative;width:100%;aspect-ratio:4/3;background:#111;overflow:hidden}.view img{width:100%;height:100%;object-fit:contain;display:block}"
        ".view canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}"
        ".legend{display:flex;gap:16px;padding:8px 0;font-size:14px}.legend span{display:inline-flex;align-items:center;gap:6px}"
        ".swatch{width:12px;height:12px;display:inline-block;border:2px solid}.red{border-color:#ff3030}.blue{border-color:#1677ff}"
        ".status{min-width:0}.status h3{margin:0 0 8px;font-size:18px}pre{background:#fff;padding:12px;border:1px solid #ccd5d1;margin:0;min-height:240px;max-height:calc(100vh - 120px);overflow:auto;font-size:13px;line-height:1.35}</style>"
        "<style>@media(max-width:900px){.layout{grid-template-columns:1fr}.status pre{max-height:none}}</style></head>"
        "<body><h2>Red Ball -> Left Blue Goal</h2><main class='layout'><section class='media'><div class='view'><img id='stream' src='/stream'><canvas id='overlay' width='640' height='480'></canvas></div>"
        "<div class='legend'><span><i class='swatch red'></i>RED BALL</span><span><i class='swatch blue'></i>BLUE GOAL</span></div></section>"
        "<section class='status'><h3>Live status</h3><pre id='s'>connecting</pre></section></main>"
        "<script>const cv=document.querySelector('#overlay'),ctx=cv.getContext('2d');"
        "function mark(o,color,label){if(!o.found)return;const x=o.min_x*2,y=o.min_y*2,w=(o.max_x-o.min_x+1)*2,h=(o.max_y-o.min_y+1)*2;"
        "ctx.strokeStyle=color;ctx.lineWidth=3;ctx.strokeRect(x,y,w,h);ctx.fillStyle=color;ctx.beginPath();ctx.arc(o.x*2,o.y*2,5,0,Math.PI*2);ctx.fill();"
        "const text=label+'  A='+o.area+(o.circularity!==undefined?'  C='+o.circularity.toFixed(2):'');ctx.font='bold 16px Arial';const tw=ctx.measureText(text).width+12;"
        "ctx.fillStyle='rgba(0,0,0,.72)';ctx.fillRect(x,Math.max(0,y-24),tw,24);ctx.fillStyle='#fff';ctx.fillText(text,x+6,Math.max(17,y-6));}"
        "function draw(d){ctx.clearRect(0,0,cv.width,cv.height);mark({found:d.found_ball,min_x:d.ball_min_x,min_y:d.ball_min_y,max_x:d.ball_max_x,max_y:d.ball_max_y,x:d.ball_x,y:d.ball_y,area:d.ball_area,circularity:d.ball_circularity},'#ff3030','RED BALL');"
        "mark({found:d.found_goal,min_x:d.goal_min_x,min_y:d.goal_min_y,max_x:d.goal_max_x,max_y:d.goal_max_y,x:d.goal_x,y:d.goal_y,area:d.goal_area},'#1677ff','BLUE GOAL');}"
        "async function p(){try{let r=await fetch('/status',{cache:'no-store'});let d=await r.json();draw(d);document.querySelector('#s').textContent=JSON.stringify(d,null,2)}catch(e){ctx.clearRect(0,0,cv.width,cv.height);}}setInterval(p,200);p()</script></body></html>";
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_handler(httpd_req_t *req)
{
    status_t status;
    if (xSemaphoreTake(s_status_lock, pdMS_TO_TICKS(50)) != pdPASS) return ESP_FAIL;
    status = s_status;
    xSemaphoreGive(s_status_lock);
    char response[768];
    int length = snprintf(response, sizeof(response),
        "{\"decoded\":%s,\"found_ball\":%s,\"found_goal\":%s,\"captured\":%s,"
        "\"ball_x\":%d,\"ball_y\":%d,\"ball_area\":%d,\"ball_error\":%d,"
        "\"ball_circularity\":%.3f,"
        "\"ball_min_x\":%d,\"ball_min_y\":%d,\"ball_max_x\":%d,\"ball_max_y\":%d,"
        "\"goal_x\":%d,\"goal_y\":%d,\"goal_area\":%d,"
        "\"goal_min_x\":%d,\"goal_min_y\":%d,\"goal_max_x\":%d,\"goal_max_y\":%d,"
        "\"distance_cm\":%.1f,\"state\":\"%s\"}",
        status.decoded ? "true" : "false", status.found_ball ? "true" : "false",
        status.found_goal ? "true" : "false", status.captured ? "true" : "false",
        status.ball_x, status.ball_y, status.ball_area, status.ball_error,
        status.ball_circularity,
        status.ball_min_x, status.ball_min_y, status.ball_max_x, status.ball_max_y,
        status.goal_x, status.goal_y, status.goal_area,
        status.goal_min_x, status.goal_min_y, status.goal_max_x,
        status.goal_max_y, status.distance_cm, status.state);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, response, length);
}

static void stream_task(void *arg)
{
    httpd_req_t *req = arg;
    uint8_t *copy = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!copy) { httpd_req_async_handler_complete(req); vTaskDelete(NULL); return; }
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    uint32_t sent_version = 0;
    esp_err_t result = ESP_OK;
    while (result == ESP_OK) {
        size_t size = 0;
        if (xSemaphoreTake(s_jpeg_lock, pdMS_TO_TICKS(100)) == pdPASS) {
            if (s_jpeg_version != sent_version && s_jpeg_sizes[s_jpeg_index] > 0) {
                size = s_jpeg_sizes[s_jpeg_index];
                memcpy(copy, s_jpeg_buffers[s_jpeg_index], size);
                sent_version = s_jpeg_version;
            }
            xSemaphoreGive(s_jpeg_lock);
        }
        if (!size) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        char header[96];
        int header_len = snprintf(header, sizeof(header),
                                  "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                                  (unsigned)size);
        result = httpd_resp_send_chunk(req, header, header_len);
        if (result == ESP_OK) result = httpd_resp_send_chunk(req, (const char *)copy, size);
        if (result == ESP_OK) result = httpd_resp_send_chunk(req, "\r\n", 2);
    }
    heap_caps_free(copy);
    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    httpd_req_t *async_req = NULL;
    esp_err_t err = httpd_req_async_handler_begin(req, &async_req);
    if (err != ESP_OK) return err;
    if (xTaskCreate(stream_task, "http_stream", 8192, async_req, 4, NULL) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static void wifi_start_ap(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    wifi_config_t config = {0};
    snprintf((char *)config.ap.ssid, sizeof(config.ap.ssid), "%s", WIFI_AP_SSID);
    snprintf((char *)config.ap.password, sizeof(config.ap.password), "%s", WIFI_AP_PASSWORD);
    config.ap.channel = 1;
    config.ap.max_connection = 1;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi AP ready: %s / %s", WIFI_AP_SSID, WIFI_AP_PASSWORD);
}

static void start_web_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.max_open_sockets = 3;
    ESP_ERROR_CHECK(httpd_start(&s_http_server, &config));
    const httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
    const httpd_uri_t status = {.uri = "/status", .method = HTTP_GET, .handler = status_handler};
    const httpd_uri_t stream = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler};
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &root));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &status));
    ESP_ERROR_CHECK(httpd_register_uri_handler(s_http_server, &stream));
}

static void vision_task(void *arg)
{
    (void)arg;
    unsigned frame_no = 0;
    unsigned processed_frame_no = 0;
    bool decoded = false;
    blob_t ball = {0}, goal = {0};
    while (s_connected) {
        uvc_host_frame_t *frame = NULL;
        if (xQueueReceive(s_frame_q, &frame, pdMS_TO_TICKS(500)) != pdPASS) {
            stop_motors();
            update_status(false, (blob_t){0}, (blob_t){0});
            continue;
        }
        publish_jpeg(frame);
        bool process_frame = (frame_no++ % VISION_PROCESS_EVERY_N_FRAMES) == 0;
        if (process_frame) {
            decoded = decode_jpeg(frame->data, frame->data_len);
            ball = (blob_t){0};
            goal = (blob_t){0};
            if (decoded) {
                if (s_state <= STATE_CAPTURE_RED) ball = detect_blob(true, false);
                if (s_state >= STATE_FIND_GOAL && s_state <= STATE_GOAL_PUSH)
                    goal = detect_blob(false, true);
            }
        }
        if (decoded) run_controller(decoded, ball, goal);
        else stop_motors();
        update_status(decoded, ball, goal);
        if (process_frame && ++processed_frame_no % 10 == 0) {
            ESP_LOGI(TAG, "state=%s ball=%d(%d,%d) goal=%d(%d,%d) dist=%.1f captured=%d motor=%s",
                     state_name(s_state), ball.area, ball.cx, ball.cy, goal.area, goal.cx, goal.cy,
                     s_distance_cm, s_ball_captured, MOTOR_OUTPUT_ENABLED ? "ON" : "DRY");
        }
        uvc_host_frame_return(s_stream, frame);
        vTaskDelay(1);
    }
    stop_motors();
    s_vision_task_running = false;
    vTaskDelete(NULL);
}

static void camera_task(void *arg)
{
    (void)arg;
    while (true) {
        const uvc_host_stream_config_t config = {
            .event_cb = stream_callback, .frame_cb = frame_callback, .user_ctx = s_frame_q,
            .usb = {.vid = UVC_HOST_ANY_VID, .pid = UVC_HOST_ANY_PID, .uvc_stream_index = 0},
            .vs_format = {.h_res = CAM_W, .v_res = CAM_H, .fps = CAM_FPS, .format = UVC_VS_FORMAT_MJPEG},
            .advanced = {.number_of_frame_buffers = FRAME_BUFFERS, .frame_size = FRAME_SIZE,
                         .frame_heap_caps = MALLOC_CAP_SPIRAM, .number_of_urbs = 6, .urb_size = 16 * 1024},
        };
        ESP_LOGI(TAG, "opening UVC camera GPIO19=D- GPIO20=D+");
        if (uvc_host_stream_open(&config, pdMS_TO_TICKS(5000), &s_stream) != ESP_OK) {
            ESP_LOGW(TAG, "camera open failed; retrying");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }
        s_connected = true;
        ESP_ERROR_CHECK(uvc_host_stream_start(s_stream));
        ESP_LOGI(TAG, "camera streaming %dx%d@%d MJPEG", CAM_W, CAM_H, CAM_FPS);
        s_vision_task_running = true;
        if (xTaskCreatePinnedToCore(vision_task, "vision", 16384, NULL,
                                    USB_PRIORITY - 2, NULL, 1) != pdPASS) {
            s_vision_task_running = false;
            s_connected = false;
            stop_motors();
        }
        while (s_connected) vTaskDelay(pdMS_TO_TICKS(200));
        while (s_vision_task_running) vTaskDelay(pdMS_TO_TICKS(10));
        uvc_host_stream_close(s_stream);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void uvc_event_callback(const uvc_host_driver_event_data_t *event, void *ctx)
{
    (void)ctx;
    if (event->type == UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED)
        ESP_LOGI(TAG, "camera connected, USB address=%d", event->device_connected.dev_addr);
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (true) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== Red Ball -> Left Blue Goal ===");
    motors_init();
    s_rgb = heap_caps_malloc(IMG_W * IMG_H * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_mask = heap_caps_malloc(IMG_W * IMG_H, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_flood_queue = heap_caps_malloc(IMG_W * IMG_H * sizeof(uint32_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_buffers[0] = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_buffers[1] = heap_caps_malloc(FRAME_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_jpeg_lock = xSemaphoreCreateMutex();
    s_status_lock = xSemaphoreCreateMutex();
    assert(s_rgb && s_mask && s_flood_queue && s_jpeg_buffers[0] && s_jpeg_buffers[1] &&
           s_jpeg_lock && s_status_lock);
    s_frame_q = xQueueCreate(FRAME_BUFFERS, sizeof(uvc_host_frame_t *));
    assert(s_frame_q);
    wifi_start_ap();
    start_web_server();
    const usb_host_config_t usb_config = {.skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LOWMED};
    ESP_ERROR_CHECK(usb_host_install(&usb_config));
    xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, USB_PRIORITY, NULL);
    const uvc_host_driver_config_t uvc_config = {
        .driver_task_stack_size = 4096, .driver_task_priority = USB_PRIORITY + 1,
        .xCoreID = tskNO_AFFINITY, .create_background_task = true, .event_cb = uvc_event_callback,
    };
    ESP_ERROR_CHECK(uvc_host_install(&uvc_config));
    xTaskCreate(ultrasonic_task, "ultrasonic", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "ready: no line following; TRIG=GPIO6 ECHO=GPIO7");
    xTaskCreatePinnedToCore(camera_task, "camera", 6144, NULL, USB_PRIORITY - 1, NULL, 0);
}
