/**
 * radar.c — 逐点画弧, 不关中断
 */
#include "radar.h"
#include "zf_common_headfile.h"
#include "tick.h"

// sin(0°~90°)×256, 91项 (索引=角度)
static const int16_t st[]={
    0,4,9,13,18,22,27,31,36,40,44,49,53,58,62,66,
    71,75,79,83,88,92,96,100,104,108,112,116,120,124,128,132,
    136,139,143,147,150,154,158,161,165,168,171,175,178,181,184,187,
    190,193,196,199,202,204,207,210,212,215,217,219,222,224,226,228,
    230,232,234,236,237,239,241,242,243,245,246,247,248,249,250,251,
    252,253,254,254,255,255,255,256,256,256,256
};
static int16_t S(int d){d%=360;if(d<0)d+=360;if(d<=90)return st[d];if(d<=180)return st[180-d];if(d<=270)return -st[d-180];return -st[360-d];}
static int16_t C(int d){return S(d+90);}

void radar_draw_base(void)
{
    uint16_t dim=0x0120, mid=0x0380;
    int rr[]={14,29,43,58,72};
    int i,a,x;

    tft180_set_color(RGB565_WHITE,RGB565_BLACK);
    tft180_clear();

    // 5圈: 内4暗, 外1中
    uint16_t cr[]={dim,dim,dim,dim,mid};
    for(i=0;i<5;i++){
        int x0=RADAR_CX+(rr[i]*C(0))/256, y0=RADAR_CY-(rr[i]*S(0))/256;
        for(a=1;a<=180;a++){
            int x1=RADAR_CX+(rr[i]*C(a))/256, y1=RADAR_CY-(rr[i]*S(a))/256;
            tft180_draw_line((uint16)x0,(uint16)y0,(uint16)x1,(uint16)y1,cr[i]);
            x0=x1; y0=y1;
        }
    }

    // 底线
    for(x=RADAR_CX-RADAR_R;x<=RADAR_CX+RADAR_R;x++)
        tft180_draw_point((uint16)x,(uint16)RADAR_CY,mid);

    // 角度线 30/60/90/120/150 (暗)
    int ang[]={30,60,90,120,150};
    char *lbl[]={"30","60","90","120","150"};
    for(i=0;i<5;i++){
        int ex=RADAR_CX+(72*C(ang[i]))/256;
        int ey=RADAR_CY-(72*S(ang[i]))/256;
        tft180_draw_line(RADAR_CX,RADAR_CY,(uint16)ex,(uint16)ey,dim);
        // 标注角度数字 (半径外移 8px)
        int lx=RADAR_CX+(80*C(ang[i]))/256 - 5;
        int ly=RADAR_CY-(80*S(ang[i]))/256 - 4;
        if(lx>=0&&ly>=0&&lx<150&&ly<120){
            tft180_set_color(mid, RGB565_BLACK);
            tft180_show_string((uint16)lx, (uint16)ly, lbl[i]);
        }
    }
}

#define TRAIL_LEN 18   // 拖尾长度 (帧数), 18帧×40ms=720ms, 约36°

// 红点缓冲区 + 帧龄 (在 scanline 里老化, 超 4 帧自动消失)
static uint8_t dot_cm[181];
static uint8_t dot_age[181];

// ---- 局部刷新: 恢复弧线/底线/圆心 (角度线在拖尾之后画) ----
static void repair_angle(uint8_t deg)
{
    uint16_t dim = 0x0120, mid = 0x0380;
    int rr[] = {14, 29, 43, 58, 72};
    uint16_t cr[] = {dim, dim, dim, dim, mid};

    // 1) 恢复5圈弧线: deg-1 ~ deg+1
    for (int i = 0; i < 5; i++) {
        for (int da = -1; da <= 1; da++) {
            int a = (int)deg + da;
            if (a < 0) a = 0;
            if (a > 180) a = 180;
            int x = RADAR_CX + (rr[i] * C(a)) / 256;
            int y = RADAR_CY - (rr[i] * S(a)) / 256;
            if (x >= 0 && x < 160 && y >= 0 && y < 128)
                tft180_draw_point((uint16)x, (uint16)y, cr[i]);
        }
    }

    // 2) 恢复底线: 0~3° 补右半段, 177~180° 补左半段
    if (deg <= 3) {
        for (int x = RADAR_CX; x <= RADAR_CX + RADAR_R; x++)
            tft180_draw_point((uint16)x, (uint16)RADAR_CY, mid);
    } else if (deg >= 177) {
        for (int x = RADAR_CX - RADAR_R; x <= RADAR_CX; x++)
            tft180_draw_point((uint16)x, (uint16)RADAR_CY, mid);
    }

    // 3) 圆心
    tft180_draw_point((uint16)RADAR_CX, (uint16)RADAR_CY, mid);
}

// ---- 判断是否标记角度 ----
static inline int is_marked(uint8_t deg)
{
    return (deg == 30 || deg == 60 || deg == 90 || deg == 120 || deg == 150);
}

// 扫描线拖尾状态 (文件级, radar_scanline_reset 可访问)
static uint8_t  trail[TRAIL_LEN];
static uint8_t  trail_age[TRAIL_LEN];
static uint8_t  trail_cnt = 0;
static uint8_t  last_deg  = 255;
static uint32_t last_base = 0;

void radar_scanline_reset(void)
{
    trail_cnt = 0;
    last_deg  = 255;
}

void radar_draw_scanline(uint8_t deg)
{
    uint32_t now = tick_get();
    uint8_t  repair_ang = 255;

    // 安全阀: 每 60 秒全刷一次
    if (now - last_base > 60000) {
        last_base = now;
        radar_draw_base();
        trail_cnt = 0;
        last_deg  = 255;
        return;
    }

    // ---- 红点: 距离当前扫描线 >8° 即刻清除 ----
    for (int a = 0; a <= 180; a++) {
        if (dot_cm[a]) {
            int d = (int)a - (int)deg;
            if (d < 0) d = -d;
            if (d > 8) dot_cm[a] = 0;
        }
    }

    // ---- 拖尾老化: age+1, 超限则修复底图后踢出 ----
    {
        int w = 0;
        for (int i = 0; i < trail_cnt; i++) {
            trail_age[i]++;
            if (trail_age[i] >= TRAIL_LEN) {
                repair_angle(trail[i]);
                if (is_marked(trail[i]))
                    repair_ang = trail[i];
            } else {
                trail[w]     = trail[i];
                trail_age[w] = trail_age[i];
                w++;
            }
        }
        trail_cnt = w;
    }

    // ---- 存入新位置 ----
    if (deg != last_deg) {
        last_deg = deg;
        trail[trail_cnt]     = deg;
        trail_age[trail_cnt] = 0;
        trail_cnt++;
    }

    // ---- 绘制渐变拖尾 (新→亮, 老→暗) ----
    for (int i = 0; i < trail_cnt; i++) {
        int g = 63 - (trail_age[i] * 62) / (TRAIL_LEN - 1);
        if (g < 1) g = 1;
        uint16_t color = (uint16_t)(g << 5);

        int nx = RADAR_CX + (72 * C(trail[i])) / 256;
        int ny = RADAR_CY - (72 * S(trail[i])) / 256;
        tft180_draw_line(RADAR_CX, RADAR_CY, (uint16)nx, (uint16)ny, color);
    }

    // ---- 最后画角度线 (拖尾已画完, 不会被相邻线覆盖) ----
    if (repair_ang != 255) {
        uint16_t dim = 0x0120, mid = 0x0380;
        int ex = RADAR_CX + (72 * C(repair_ang)) / 256;
        int ey = RADAR_CY - (72 * S(repair_ang)) / 256;
        tft180_draw_line(RADAR_CX, RADAR_CY, (uint16)ex, (uint16)ey, dim);
        // 画完整条线后圆心可能被覆盖, 补回
        tft180_draw_point((uint16)RADAR_CX, (uint16)RADAR_CY, mid);
    }
}

// ==================== 红点操作 ====================

void radar_add_dot(uint8_t deg, float cm)
{
    if (deg > 180) return;
    if (cm < 2.0f || cm > 50.0f) return;
    dot_cm[deg]  = (uint8_t)(cm + 0.5f);
    dot_age[deg] = 0;
}

void radar_clear_dots(void)
{
    for (int i = 0; i <= 180; i++) { dot_cm[i] = 0; dot_age[i] = 0; }
}

void radar_draw_dots(void)
{
    for (int a = 0; a <= 180; a++) {
        if (dot_cm[a] == 0) continue;
        radar_draw_point((uint8_t)a, (float)dot_cm[a]);
    }
}

void radar_draw_point(uint8_t deg, float cm){
    if(cm<1)return; int r=(int)(cm*1.5f); if(r>72)r=72;
    int x=RADAR_CX+(r*C(deg))/256, y=RADAR_CY-(r*S(deg))/256;
    // 不画在底线或以下; 3×3 像素红点
    if(x>=0&&x<158&&y>=0&&y<126 && y < RADAR_CY){
        tft180_draw_point((uint16)x,   (uint16)y,   RGB565_RED);
        tft180_draw_point((uint16)(x+1),(uint16)y,   RGB565_RED);
        tft180_draw_point((uint16)(x+2),(uint16)y,   RGB565_RED);
        tft180_draw_point((uint16)x,   (uint16)(y+1),RGB565_RED);
        tft180_draw_point((uint16)(x+1),(uint16)(y+1),RGB565_RED);
        tft180_draw_point((uint16)(x+2),(uint16)(y+1),RGB565_RED);
        tft180_draw_point((uint16)x,   (uint16)(y+2),RGB565_RED);
        tft180_draw_point((uint16)(x+1),(uint16)(y+2),RGB565_RED);
        tft180_draw_point((uint16)(x+2),(uint16)(y+2),RGB565_RED);
    }
}
