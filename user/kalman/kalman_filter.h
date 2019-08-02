#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

typedef struct{
    // Q
    float Qp;
    float Qv;

    float R;

    float dt;
    float pre_t;

    float P[2][2];
    float P_pre[2][2];

    float X[2];
    float X_pre[2];

    float Kg[2];
}kalman;

void kalman_init(kalman *kf);
float kalman_run(kalman *kf, float gimbal_data, float vision_data);


#endif // KALMAN_FILTER_H
