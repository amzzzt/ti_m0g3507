#include "zf_common_headfile.h"
#include "tick.h"
#include "track.h"
#include "motor.h"
#include "imu.h"
#include "course.h"

#define TURN_SPEED 3000
#define TURN_DEG   45.0f
#define FWD_MS     1500

typedef enum { CS_FOLLOW, CS_STOP, CS_TURN, CS_FORWARD } cs_t;

static cs_t     st;
static uint32_t t0;
static float    y0;
static int      turn;
static int      tl, tr;

static int lost(void) {
    for (int i = 0; i < 8; i++)
        if (track_value(i) == 0) return 0;
    return 1;
}

void course_init(void)       { st = CS_FOLLOW; turn = 1; }
int  course_state(void)       { return (int)st; }
void course_targets(int *a, int *b) { *a = tl; *b = tr; }

void course_update(void)
{
    uint32_t now = tick_get();
    float    yaw = imu_yaw();

    switch (st) {

    case CS_FOLLOW: {
        int dev = track_deviation();
        tl = 400 + dev;
        tr = 400 - dev;
        motor_control_update((int16_t)tl, (int16_t)tr);
        if (lost()) { st = CS_STOP; t0 = now; motor_stop(); }
        break;
    }

    case CS_STOP:
        if (now - t0 >= 200) {
            st = CS_TURN; t0 = now; y0 = yaw;
            if (turn) { motor_left(-TURN_SPEED); motor_right(TURN_SPEED); }
            else      { motor_left(TURN_SPEED); motor_right(-TURN_SPEED); }
        }
        break;

    case CS_TURN: {
        if (now - t0 < 100) break;
        float d = turn ? (y0 - yaw) : (yaw - y0);
        if (d < 0) d += 360.0f;
        if (d >= TURN_DEG && d <= 180.0f) {
            st = CS_FORWARD; t0 = now; turn = !turn;
        }
        break;
    }

    case CS_FORWARD: {
        tl = 400; tr = 400;
        motor_control_update(400, 400);
        if (now - t0 < FWD_MS) break;   /* 前1.5s只管走 */
        static int cnt = 0;
        if (!lost()) {
            if (++cnt >= 5) { cnt = 0; st = CS_FOLLOW; }
        } else { cnt = 0; }
        break;
    }
    }
}
