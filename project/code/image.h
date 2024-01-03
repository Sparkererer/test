#ifndef _image_h_
#define _image_h_
#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define uesr_RED     0XF800    //红色
#define uesr_GREEN   0X07E0    //绿色
#define uesr_BLUE    0X001F    //蓝色

//宏定义
#define image_h	100//图像高度
#define image_w	114//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	

#define CUT_H               (2) //图像顶上切除的高度
#define TFTSHOW_W           (80) //TFT屏幕显示的宽高 157  100
#define TFTSHOW_H           (50)
#define USED_LINE_BEGIN     (80) //用来循迹的首行
#define USED_LINE_END       (40) //用来循迹的尾行
#define FIND_STARTLINE      (image_h - 3)//爬线起始行
//逆透视
#define RESULT_H 100  //结果图行列
#define RESULT_W 114
#define USED_H   120  //用于透视图的行列
#define USED_W   188
#define PER_IMG     mt9v03x_image//SimBinImage:用于透视变换的图像
#define ImageUsed   *PerImg_ip//*PerImg_ip定义使用的图像，ImageUsed为用于巡线和识别的图像
#define STRAIGHTLINEX_L  36 //逆透视直线左边线x
#define STRAIGHTLINEX_R  80 //逆透视直线右边线x

//结构体定义
typedef struct
{
    float k;
    float b;
} line_equation; //直线方程y = kx + b

#define LINE_CREATE() \
{                     \
    .k = 0,           \
    .b = 0,           \
}

extern uint8 image_thereshold;//二值化阈值

extern uint16 data_stastics_l, data_stastics_r;
extern uint8 hightest;
extern uint8 l_border_hightest;
extern uint8 r_border_hightest;

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 *PerImg_ip[RESULT_H][RESULT_W];

extern uint8 image_0[RESULT_H][RESULT_W];
extern uint8 image_1[RESULT_H][RESULT_W];

extern uint8 l_border[image_h];//左线数组
extern uint8 r_border[image_h];//右线数组

extern uint16 cornerpoint[4][3];//两个角点 -- 左下，右下 {x,y,flag}

void Turn_Bin(uint8 threshold);
void ImagePerspective_Init(void);
void k_and_b(line_equation *temp_line, int16 startX, int16 startY, int16 endX, int16 endY);
void calculate_s_i(uint8 start, uint8 end, uint8 *border, line_equation *temp_line);
void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了
void imageparam_show(void);


#endif

