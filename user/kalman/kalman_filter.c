#include "kalman_filter.h"
#include "stdio.h"
#include "user_lib.h"
void kalman_init(kalman *kf)
{
    //Q//  w态误差
    kf->Qp = 50;//角度状态误差
    kf->Qv = 0.01;//速度状态误差
    //R
    kf->R = 0.01;//测量误差

    //X0
    kf->X[0] = 0;//角度初值
    kf->X[1] = 0;//速度初值

    //P0
    kf->P[0][0] = 10;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0.5;

    kf->dt = 1;//时间间隔
    kf->pre_t = 120;//预测时间
}

float kalman_run(kalman *kf, float gimbal_data, float vision_data)
{
	static float suibian;
    // (1) x(k|k-1) = AX(k-1|k-1)+BU(k)
    kf->X_pre[0] = kf->X[0] + kf->X[1] * kf->dt;
    kf->X_pre[1] = kf->X[1];

    // (2) p(k|k-1) = Ap(k-1|k-1)A'+Q
    float tmp_value[2][2];
    tmp_value[0][0] = 1*kf->P[0][0] + kf->dt * kf->P[1][0];
    tmp_value[0][1] = 1*kf->P[0][1] + kf->dt * kf->P[1][1];
    tmp_value[1][0] = 0*kf->P[0][0] + 1 * kf->P[1][0];
    tmp_value[1][1] = 0*kf->P[0][1] + 1 * kf->P[1][1];

    kf->P_pre[0][0] = tmp_value[0][0]*1 + tmp_value[0][1]*kf->dt + kf->Qp;
    kf->P_pre[0][1] = tmp_value[0][0]*0 + tmp_value[0][1]*1;
    kf->P_pre[1][0] = tmp_value[1][0]*1 + tmp_value[1][1]*kf->dt;
    kf->P_pre[1][1] = tmp_value[1][0]*0 + tmp_value[1][1]*1 + kf->Qv;

    // (3)kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    kf->Kg[0] = kf->P_pre[0][0] / (kf->P_pre[0][0] + kf->R);
    kf->Kg[1] = kf->P_pre[1][0] / (kf->P_pre[0][0] + kf->R);

    // (4) x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    float innovation = gimbal_data + vision_data - kf->X_pre[0];
    kf->X[0] = kf->X_pre[0] + kf->Kg[0] * innovation;
    kf->X[1] = kf->X_pre[1] + kf->Kg[1] * innovation;

    // (5) p(k|k) = (I-kg(k)H)P(k|k-1)
    tmp_value[0][0] = 1 - kf->Kg[0];
    tmp_value[0][1] = 0;
    tmp_value[1][0] = -kf->Kg[1];
    tmp_value[1][1] = 1;

    kf->P[0][0] = tmp_value[0][0] * kf->P_pre[0][0] + tmp_value[0][1] * kf->P_pre[1][0];
    kf->P[0][1] = tmp_value[0][0] * kf->P_pre[0][1] + tmp_value[0][1] * kf->P_pre[1][1];
    kf->P[1][0] = tmp_value[1][0] * kf->P_pre[0][0] + tmp_value[1][1] * kf->P_pre[1][0];
    kf->P[1][1] = tmp_value[1][0] * kf->P_pre[0][1] + tmp_value[1][1] * kf->P_pre[1][1];
		
		suibian=kf->X[1] * kf->pre_t;
		if(vision_data > 15.0f)
		{
			suibian=kf->X[1] * kf->pre_t*0.1f;
		}
		suibian=fp32_constrain(suibian,-5,5);
		 return kf->X[0] + suibian;
//    return kf->X[0] + kf->X[1] * kf->pre_t;
		
//    printf(" 1:position: %f\r\n", kf->X_pre[0]);
//    printf(" 2:P_pre :\n%f\t%f\r\n%f\t%f\r\n", kf->P_pre[0][0], kf->P_pre[0][1], kf->P_pre[1][0], kf->P_pre[1][1]);
//    printf(" 3:Kg = %f, %f\r\n", kf->Kg[0], kf->Kg[1]);
//    printf(" 4:pos=%f,v=%f\r\n", kf->X[0], kf->X[1]);
//    printf(" 2:P :\n%f\t%f\r\n%f\t%f\r\n", kf->P[0][0], kf->P[0][1], kf->P[1][0], kf->P[1][1]);
//    return (kf->X[0] - gimbal_data) + kf->X[1] * kf->pre_t;
}
