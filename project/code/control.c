#include "zf_common_headfile.h"
#include "image.h"
#include "math.h"
#include "imu963.h"

uint8 MotorBegin_Flag = 0; //启动标志位
uint8 shizi_status = 0; //十字状态
uint8 ShiZi_Flag = 0; //十字标志位
uint8 yuanhuan_status = 0; //圆环状态
uint8 YuanHuan_Flag = 0; //圆环标志位

void Mode_Switch(void)
{
    uint8 i;
    uint8 shizi_count1, shizi_count2 = 0;
    //检测十字
    if (shizi_status == 0 && YuanHuan_Flag == 0) //未检测到十字
    {
        if ((cornerpoint[0][2] == 1 && cornerpoint[1][2] == 1 && cornerpoint[2][2] == 1 && cornerpoint[3][2] == 1) &&
            (cornerpoint[0][1] >= USED_LINE_END || cornerpoint[0][1] >= USED_LINE_END))
        {
            shizi_status = 1;
            ShiZi_Flag = 1;
        }
    }
    if (shizi_status == 1) //检测到开始进十字
    {
        AngleGet_Flag = 1; //角度开始积分
        if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 && cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0)
        {
            shizi_status = 2;
            shizi_count1 = 0;
            shizi_count2 = 0;
        }
    }
    if (shizi_status == 2) //在十字内
    {
        //逆透视由于现在摄像头太靠前，只添加了前面两个角点条件，待摄像头移至中间后再加后面两个角点条件！！！
        if (cornerpoint[2][2] == 1 && cornerpoint[3][2] == 1 && fabs(angle) >= 250 && fabs(angle) <= 280)
        {
            shizi_status = 3; 
        }
    }
    if (shizi_status == 3) //开始出十字
    {
        if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 && cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0)
        {
            for (i = CUT_H; i <= image_h - 3; i++)
            {
                if (l_border[i] <= 4) shizi_count1++; //左丢线行数
                if (r_border[i] >= 110) shizi_count2++; //右丢线行数
            }
            if ((shizi_count1 + shizi_count2) <= 15) //总丢线行数少
            {
                shizi_status = 0; //出十字le
                ShiZi_Flag = 0;
                AngleGet_Flag = 0;
                shizi_count1 = 0;
                shizi_count2 = 0;
            }
        }
    }

    uint8 yuanhuan_count = 0; //圆环角点与其他行x差值计数
    uint8 yuanhuan_count2 = 0; //圆环单调行计数
    uint8 yuanhuan_count3 = 0; //圆环出环左边线丢线计数
    line_equation front_l = LINE_CREATE(); //左边线前方线段
    line_equation behind_l = LINE_CREATE(); //左边线后方线段
    line_equation front_r = LINE_CREATE(); //右边线前方线段
    line_equation behind_r = LINE_CREATE(); //右边线后方线段
    //检测圆环
    if (yuanhuan_status == 0 && ShiZi_Flag == 0) //未检测到圆环状态
    {
        if (cornerpoint[0][2] == 1 && cornerpoint[1][2] == 0 &&
            // cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0 &&
            cornerpoint[0][1] >= USED_LINE_END - 5) //左圆环-拐点超过循迹最后一行
        {
            //角点下面行与角点x的误差如果够大，判断为弯道，如果比较小，判定为圆环
            //圆环是直线突变，弯道是渐变
            for (i = cornerpoint[0][1]; i < cornerpoint[0][1] + 10; i++)
            {
                if (abs(cornerpoint[0][0] - l_border[i+1]) <= 1) yuanhuan_count++; //与角点x差距小
            }
            if (yuanhuan_count >= 8)
            {
                YuanHuan_Flag = 1; //左圆环
                yuanhuan_status = 1;
            }
        }
        else if (cornerpoint[0][2] == 0 && cornerpoint[1][2] == 1 &&
                //  cornerpoint[2][2] == 0 && cornerpoint[3][2] == 0 &&
                 cornerpoint[1][1] >= USED_LINE_END - 5) //右圆环
        {
            //角点下面行与角点x的误差如果够大，判断为弯道，如果比较小，判定为圆环
            //圆环是直线突变，弯道是渐变
            for (i = cornerpoint[1][1]; i < cornerpoint[1][1] + 10; i++)
            {
                if (abs(cornerpoint[1][0] - r_border[i+1]) <= 1) yuanhuan_count++; //与角点x差距小
            }
            if (yuanhuan_count >= 8)
            {
                YuanHuan_Flag = 2; //右圆环
                yuanhuan_status = 1;
            }
        }
        yuanhuan_count2 = 0;
    }
    if (YuanHuan_Flag == 1) //左圆环
    {
        AngleGet_Flag = 1; //角度开始积分
        if (yuanhuan_status == 1) //开始进圆环直道
        {   
            //左上角点到位开始进圆环
            if (cornerpoint[2][2] == 1 && cornerpoint[3][2] == 0 && cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 &&
                cornerpoint[2][1] >= 35) yuanhuan_status = 2; 
        }
        if (yuanhuan_status == 2) //正在进入圆环弯道内
        {   
            if (angle >= 45) yuanhuan_status = 3;
        }
        if (yuanhuan_status == 3) //正在环岛弯道内
        {   
            if (angle >= 240) yuanhuan_status = 4;
        }
        if (yuanhuan_status == 4) //开始出环岛
        {
            //角度差不多转了一圈
            if (angle >= 335) yuanhuan_status = 5;
        }
        if (yuanhuan_status == 5) //正在补直线出环岛
        {
            //左边几乎没有丢线行
            for (i = 2; i<= image_h - 3; i++)
            {
                if (l_border[i] == 2) yuanhuan_count2++; //丢线
            }
            if (yuanhuan_count2 <= 15)
            {
                YuanHuan_Flag = 0;
                AngleGet_Flag = 0;
                yuanhuan_status = 0;
            }
        }
    }
    if (YuanHuan_Flag == 2) //右圆环
    {
        AngleGet_Flag = 1; //角度开始积分
        if (yuanhuan_status == 1) //开始进圆环直道
        {   
            //右上角点到位开始进圆环
            if (cornerpoint[2][2] == 0 && cornerpoint[3][2] == 1 && cornerpoint[0][2] == 0 && cornerpoint[1][2] == 0 &&
                cornerpoint[3][2] >= 32) yuanhuan_status = 2; 
        }
        if (yuanhuan_status == 2) //正在进入圆环弯道内
        {
            if (angle <= -45) yuanhuan_status = 3;
        }
        if (yuanhuan_status == 3) //正在环岛弯道内
        {     
            if (angle <= -240) yuanhuan_status = 4;
        }
        if (yuanhuan_status == 4) //开始出环岛
        {
            //角度差不多转了一圈
            if (angle <= -335) yuanhuan_status = 5;
        }
        if (yuanhuan_status == 5) //正在补直线出环岛
        {
            //右边几乎没有丢线行
            for (i = 2; i<= image_h - 3; i++)
            {
                if (r_border[i] == image_w - 3) yuanhuan_count2++;
            }
            if (yuanhuan_count2 <= 15)
            {
                YuanHuan_Flag = 0;
                AngleGet_Flag = 0;
                yuanhuan_status = 0;
            }
        }
    }

    //异常停车
    if (data_stastics_l <= 5 || (bin_image[image_h - 2][image_w / 2] == 0 && bin_image[image_h - 3][image_w / 2] == 0 && bin_image[image_h - 4][image_w / 2] == 0))
    {
        MotorBegin_Flag = 0;
    }
    // printf("l = %d\tr = %d\n", data_stastics_l,data_stastics_r);
}