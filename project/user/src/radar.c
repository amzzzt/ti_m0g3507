/**
 * radar.c — 逐点画弧, 不关中断
 */
#include "radar.h"
#include "zf_common_headfile.h"

// sin(0°~90°)×256, 91项 (索引=角度)
static const int16_t st[]={
    0,4,9,13,18,22,27,31,36,40,45,49,54,58,63,67,
    72,76,80,85,89,93,98,102,106,110,114,118,122,126,130,134,
    138,142,145,149,153,156,160,163,167,170,173,177,180,183,186,189,
    192,195,197,200,203,205,208,210,213,215,217,219,221,223,225,227,
    229,230,232,233,235,236,237,239,240,241,242,243,244,245,246,247,
    247,248,249,249,250,250,251,251,252,252,252,253,253,253,254,256
};
static int16_t S(int d){d%=360;if(d<0)d+=360;if(d<=90)return st[d];if(d<=180)return st[180-d];if(d<=270)return -st[d-180];return -st[360-d];}
static int16_t C(int d){return S(d+90);}

void radar_draw_base(void)
{
    uint16_t dim=0x0120, mid=0x0380;   // 内圈更暗   // 内圈暗, 外圈中暗
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
    for(i=0;i<5;i++){
        int ex=RADAR_CX+(72*C(ang[i]))/256;
        int ey=RADAR_CY-(72*S(ang[i]))/256;
        tft180_draw_line(RADAR_CX,RADAR_CY,(uint16)ex,(uint16)ey,dim);
    }
}

void radar_draw_point(uint8_t deg, float cm){
    if(cm<1)return; int r=(int)(cm*1.5f); if(r>72)r=72;
    int x=RADAR_CX+(r*C(deg))/256, y=RADAR_CY-(r*S(deg))/256;
    if(x>0&&x<159&&y>0&&y<127)tft180_draw_point(x,y,RGB565_RED);
}
