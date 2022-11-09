
#include "ahrs.h"

/* Kalman param 0 is pitch and 1 for roll*/
static kalman_param_t kalman_instance[2] = {{.kC_0 = 1,
                                            .q_bias = 0.0,

                                            .Q_angle = 0.0000050f,
                                            .Q_gyro = 0.00002639f,
                                            .R_angle = 0.31623f},

{.kC_0 = 1,
.q_bias = 0.0,

.Q_angle = 0.0000050f,
.Q_gyro = 0.00002639f,
.R_angle = 0.031623f}};

/* mahony instance */
static mahony_param_t mahony_instance = {
.kp = 2.0f,
.ki = 0.005f,
.sample_freq = 2000,
};

imu_raw_t kImuInstance = {
.gyro_has_calibration = 1,
.acc_has_calibration = 0,
.acc_offset = {0.0f, 0.0f, 0.0f},
.acc_scale = {1.0f, 1.0f, 1.0f},
.gyro_offset = {-1.15805221f, 0.652579784f, 0.358576238f},
.axis_rotation =
{
{
1,
0,
0,
},
{
0,
1,
0,
},
{
0,
0,
1,
},
},
};

/* ahrs algorith data */
static ahrs_t ahrs_instance = {
.mahony_handle = &mahony_instance,
.kalman_handle = kalman_instance,
.imu_raw = &kImuInstance,
.dt = 0.0005,
};

/**
 * @brief kalman滤波器
 * @brief 调参参考：Q是系统噪声方差，R是测量噪声方差
 * @brief          R_angle越小，滤波响应越快，收敛的越快，但过小会出现震荡
 * @brief          Q_angle, Q_gyro越小抑制滤除噪声的能力越强。
 * @param acc_angle 通过三角函数计算加速度计的值得到的角度值
 * @param gyro_rate 陀螺仪输出的角速度值
 * @return NULL
 */
void kalman_filter(kalman_param_t* phandle, float acc_angle, float gyro_rate, float* angle, float* gyro_rate_filter, float dt)
{
    /*
     * kalman公式一、角度的先验状态估计 x(k|k-1) = F * x(k-1|k-1) + B* u
     * x为角度和角速度的列向量 F=( [1,-dt],
     *                         [0, 1] )
     */
    gyro_rate -= phandle->q_bias;
    *angle += gyro_rate * dt;

    /*
     * kalman公式二、预测协方差矩阵 P(k|k-1) = F * P(k-1|k-1) * F' + Q(k)
     */
    phandle->P[0][0] += (phandle->Q_angle - phandle->P[0][1] - phandle->P[1][0] + phandle->P[1][1] * dt) * dt;
    phandle->P[0][1] += (-phandle->P[1][1]) * dt;
    phandle->P[1][0] += (-phandle->P[1][1]) * dt;
    phandle->P[1][1] += phandle[0].Q_gyro * dt;

    /*
     * 中间变量 角度残差 z(k) - H * x(k)
     * H = ([1,0])
     */
    phandle->angle_err = acc_angle - *angle;

    /*
     * 中间变量 E = H * P(k|k-1) H' + R
     */
    phandle->PCt_0 = phandle->kC_0 * phandle->P[0][0];
    phandle->PCt_1 = phandle->kC_0 * phandle->P[1][0];
    phandle->E = phandle[0].R_angle + phandle->kC_0 * phandle->PCt_0;

    /*
     * kalman公式三、K = P(k|k-1) * H' / (H * P(k|k-1) H' + R)
     */
    phandle->K_0 = phandle->PCt_0 / phandle->E;
    phandle->K_1 = phandle->PCt_1 / phandle->E;

    /*
     * kalman 公式四、P(k|k) = (I - K * H) * P(k|k-1)
     */
    phandle->t_0 = phandle->PCt_0;
    phandle->t_1 = phandle->kC_0 * phandle->P[0][1];
    phandle->P[0][0] -= phandle->K_0 * phandle->t_0;
    phandle->P[0][1] -= phandle->K_0 * phandle->t_1;
    phandle->P[1][0] -= phandle->K_1 * phandle->t_0;
    phandle->P[1][1] -= phandle->K_1 * phandle->t_1;

    /*
     * kalman公式五、x(k|k) = x(k|k-1) + K * (z(k) - H * x(k))
     */
    *angle += phandle->K_0 * phandle->angle_err;
    phandle->q_bias += phandle->K_1 * phandle->angle_err;

    *angle = *angle;                                 //最优角度
    *gyro_rate_filter = gyro_rate - phandle->q_bias; //最优角速度
}

/**
 * @brief  3 dimensional space verctor product(i X j)
 * @param  ix, iy, iz vercotr input
 * @param  jx, jy, jz vercotr input
 * @param  out[3] verctor product output
 * @retval None
 */
void Vector3Cross(float ix, float iy, float iz, float jx, float jy, float jz, float out[3])
{
    out[0] = iy * jz - iz * jy;
    out[1] = iz * jx - ix * jz;
    out[2] = ix * jy - iy * jx;
}
/**
 * @brief  matrix(3x3) transpose.
 * @param  m[3][3] matrix input
 * @param  m_t[3][3] matrix output
 * @retval None
 */
void Matrix3x3Transpose(float m[3][3], float m_t[3][3])
{
    m_t[0][0] = m[0][0];
    m_t[0][1] = m[1][0];
    m_t[0][2] = m[2][0];
    m_t[1][0] = m[0][1];
    m_t[1][1] = m[1][1];
    m_t[1][2] = m[2][1];
    m_t[2][0] = m[0][2];
    m_t[2][1] = m[1][2];
    m_t[2][2] = m[2][2];
}

/**
 * @brief  rotation matrix(navigation coordinates to body coordinates) conversion Quaternion  .
 * @param  tnb[3][3] rotation matrix input
 * @param  q[4] Quaternion output
 * @retval None
 */
void Tnb2Quaternion(float tnb[3][3], float q[4])
{
    float tr[4], t, f, max_diag;
    uint8_t max_num = 0;

    /*compute the maximun value*/
    tr[0] = tnb[0][0] + tnb[1][1] + tnb[2][2];
    tr[1] = tnb[0][0] - tnb[1][1] - tnb[2][2];
    tr[2] = -tnb[0][0] + tnb[1][1] - tnb[2][2];
    tr[3] = -tnb[0][0] - tnb[1][1] + tnb[2][2];

    max_diag = tr[0];
    for (uint8_t i = 0; i < 4; i++) {
        if (max_diag < tr[i]) {
            max_diag = tr[i];
            max_num = i;
        }
    }

    t = sqrtf(1.0f + tr[max_num]);
    if (t < 1e-6f) {
        t = 1e-6f;
    }
    f = 0.5f / t;

    switch (max_num) {
        case 0: {
            q[0] = 0.5f * t;
            q[1] = (tnb[1][2] - tnb[2][1]) * f;
            q[2] = (tnb[2][0] - tnb[0][2]) * f;
            q[3] = (tnb[0][1] - tnb[1][0]) * f;
            break;
        }
        case 1: {
            q[0] = (tnb[1][2] - tnb[2][1]) * f;
            q[1] = 0.5f * t;
            q[2] = (tnb[0][1] + tnb[1][0]) * f;
            q[3] = (tnb[2][0] + tnb[0][2]) * f;
            break;
        }
        case 2: {
            q[0] = (tnb[2][0] - tnb[0][2]) * f;
            q[1] = (tnb[0][1] + tnb[1][0]) * f;
            q[2] = 0.5f * t;
            q[3] = (tnb[1][2] + tnb[2][1]) * f;
            break;
        }
        case 3: {
            q[0] = (tnb[0][1] - tnb[1][0]) * f;
            q[1] = (tnb[2][0] + tnb[0][2]) * f;
            q[2] = (tnb[1][2] + tnb[2][1]) * f;
            q[3] = 0.5f * t;
            break;
        }
        default:
            break;
    }
}

/**
 * @brief  Quaternion conversion rotation matrix(body coordinates to navigation coordinates).
 * @param  q[4] Quaternion input
 * @param  tbn[3][3] rotation matrix output
 * @retval None
 */
void Quaternion2tbn(float q[4], float tbn[3][3])
{
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    q0q0 = q[0] * q[0];
    q0q1 = q[0] * q[1];
    q0q2 = q[0] * q[2];
    q0q3 = q[0] * q[3];
    q1q1 = q[1] * q[1];
    q1q2 = q[1] * q[2];
    q1q3 = q[1] * q[3];
    q2q2 = q[2] * q[2];
    q2q3 = q[2] * q[3];
    q3q3 = q[3] * q[3];

    tbn[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    tbn[1][0] = 2.0f * (q1q2 + q0q3);
    tbn[2][0] = 2.0f * (q1q3 - q0q2);

    tbn[0][1] = 2.0f * (q1q2 - q0q3);
    tbn[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    tbn[2][1] = 2.0f * (q2q3 + q0q1);

    tbn[0][2] = 2.0f * (q1q3 + q0q2);
    tbn[1][2] = 2.0f * (q2q3 + q0q1);
    tbn[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

float Saturationf(float a, float max, float min)
{
    if (a > max)
        a = max;
    else if (a < min)
        a = min;
    return a;
}
/**
 * @brief  rotation matrix(body coordinates to navigation coordinates) conversion Euler Angle(312 sequence).
 * @param  tbn[3][3] rotation matrix input.
 * @param  euler[3] roll-pitch-yaw angle output in rad.
 * @retval None
 */
void Tbn2Euler312(float tbn[3][3], float euler[3])
{
    float t = Saturationf(tbn[2][1], 1.0f, -1.0f);
    euler[0] = asinf(t);
    euler[1] = atan2f(-tbn[2][0], tbn[2][2]);
    euler[2] = atan2f(-tbn[0][1], tbn[1][1]);
}

/**
 * @brief  Init ahrs based on Mahony attitude estimation.
 * @param  ax: Accelerometer x-axis data, uints - m/s/s.
 * @param  ay: Accelerometer y-axis data, uints - m/s/s.
 * @param  az: Accelerometer z-axis data, uints - m/s/s.
 * @retval 1:Successful initialization
 */
int mahony_init(mahony_param_t* phandle, float ax, float ay, float az)
{
    float rnb[3][3], cross[3];
    const float mx = 0.0f, my = 0.0f, mz = 1.0f;

    if (0 == phandle->kp)
        phandle->kp = 2.0f;
    if (0 == phandle->ki)
        phandle->ki = 0.005f;
    if (0 == phandle->sample_freq) {
        phandle->sample_freq = 2000;
    }
    phandle->sample_t = 1.0f / phandle->sample_freq;
    phandle->half_t = 0.5f * phandle->sample_t;
    if ((ax == 0) && (ay == 0) && (az == 0)) {
        return 0;
    }
    /* normalise the measurements*/
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.001f) {
        norm = 0.001f;
    }
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
    rnb[0][2] = -ax;
    rnb[1][2] = -ay;
    rnb[2][2] = -az;
    Vector3Cross(rnb[0][2], rnb[1][2], rnb[2][2], mx, my, mz, cross);
    norm = sqrtf(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);

    if (norm < 0.001f) {
        norm = 0.001f;
    }
    rnb[0][1] = cross[0] / norm;
    rnb[1][1] = cross[1] / norm;
    rnb[2][1] = cross[2] / norm;
    Vector3Cross(rnb[0][1], rnb[1][1], rnb[2][1], rnb[0][2], rnb[1][2], rnb[2][2], cross);
    norm = sqrtf(cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]);
    if (norm < 0.001f) {
        norm = 0.001f;
    }
    rnb[0][0] = cross[0] / norm;
    rnb[1][0] = cross[1] / norm;
    rnb[2][0] = cross[2] / norm;
    Tnb2Quaternion(rnb, phandle->q);
    Quaternion2tbn(phandle->q, phandle->tbn);
    Tbn2Euler312(phandle->tbn, phandle->euler_angle);
    phandle->ex_int = 0.0f;
    phandle->ey_int = 0.0f;
    phandle->ez_int = 0.0f;
    return 1;
}

/**
 * @brief  Update ahrs based on Mahony attitude estimation.
 * @param  ax: Accelerometer x-axis data, uints - m/s/s.
 * @param  ay: Accelerometer y-axis data, uints - m/s/s.
 * @param  az: Accelerometer z-axis data, uints - m/s/s.
 * @param  gx: gyroscope x-axis data, uints - degrees/sec.
 * @param  gy: gyroscope y-axis data, uints - degrees/sec.
 * @param  gz: gyroscope z-axis data, uints - degrees/sec.
 * @retval None
 */
void mahony_update(mahony_param_t* phandle, float ax, float ay, float az, float gx, float gy, float gz)
{
    static uint8_t init = 0;
    float norm, q[4], vx, vy, vz, ex, ey, ez, qa, qb, qc, qd;

    if (0 == init) {
        init = mahony_init(phandle, ax, ay, az);
    } else {
        /* normalise the measurements*/
        norm = sqrtf(ax * ax + ay * ay + az * az);
        if (norm < 0.001f) {
            norm = 0.001f;
        }
        ax = -ax / norm;
        ay = -ay / norm;
        az = -az / norm;
        /* Gyroscope units are radians/second */
        gx = gx * Deg2Rad;
        gy = gy * Deg2Rad;
        gz = gz * Deg2Rad;
        /* Estimated direction of gravity */
        vx = phandle->tbn[2][0];
        vy = phandle->tbn[2][1];
        vz = phandle->tbn[2][2];
        /*error is sum of cross product between reference direction of fields and direction measured by sensors*/
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;
        /* Compute integral feedback if enabled*/
        if (phandle->ki > 0.0f) {
            phandle->ex_int += phandle->ki * ex * phandle->sample_t; // integral error scaled by Ki
            phandle->ey_int += phandle->ki * ey * phandle->sample_t;
            phandle->ez_int += phandle->ki * ez * phandle->sample_t;
        } else {
            phandle->ex_int = 0.0f; // prevent integral windup
            phandle->ey_int = 0.0f;
            phandle->ez_int = 0.0f;
        }
        /*adjusted gyroscope measurements based on integral feedback and proportional feedback*/
        gx = gx + phandle->kp * ex + phandle->ex_int;
        gy = gy + phandle->kp * ey + phandle->ey_int;
        gz = gz + phandle->kp * ez + phandle->ez_int;
        /*integrate quaternion rate*/
        gx = gx * phandle->half_t; // pre-multiply common factors
        gy = gy * phandle->half_t;
        gz = gz * phandle->half_t;
        q[0] = phandle->q[0];
        q[1] = phandle->q[1];
        q[2] = phandle->q[2];
        q[3] = phandle->q[3];
        qa = q[0];
        qb = q[1];
        qc = q[2];
        qd = q[3];
        q[0] += (-qb * gx - qc * gy - qd * gz);
        q[1] += (qa * gx + qc * gz - qd * gy);
        q[2] += (qa * gy - qb * gz + qd * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        /*Normalise quaternion */
        norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (norm < 0.001f) {
            norm = 0.001f;
        }
        phandle->q[0] = q[0] / norm;
        phandle->q[1] = q[1] / norm;
        phandle->q[2] = q[2] / norm;
        phandle->q[3] = q[3] / norm;
        Quaternion2tbn(phandle->q, phandle->tbn);
        Tbn2Euler312(phandle->tbn, phandle->euler_angle);
        phandle->euler_angle_deg[0] = phandle->euler_angle[0] * 57.295779f;
        phandle->euler_angle_deg[1] = phandle->euler_angle[1] * 57.295779f;
        phandle->euler_angle_deg[2] = phandle->euler_angle[2] * 57.295779f;
    }
}

/* get angle by acc */
__attribute__((unused)) static void GetAngleByAcc(ahrs_t* phandle)
{
    imu_raw_t* imu = phandle->imu_raw;
    float scale = 180 / PI;

    /* function 1 */
    phandle->euler_by_acc[0] =
    scale * atanf(imu->acc_raw[0] / (sqrt(imu->acc_raw[1] * imu->acc_raw[1] + imu->acc_raw[2] * imu->acc_raw[2]))); // pitch
    phandle->euler_by_acc[1] = scale * atan2f(imu->acc_raw[1], imu->acc_raw[2]); // roll
    // phandle->euler_by_acc[1] = scale * atan2f(imu->acc_raw[1], (sqrt(imu->acc_raw[0] * imu->acc_raw[0] + imu->acc_raw[2] * imu->acc_raw[2]))); 	//roll

    /* function 2 */
    // phandle->euler_by_acc[0] = scale * atan2f(-imu->acc_raw[0], imu->acc_raw[2]);
    // phandle->euler_by_acc[1] = scale * atanf(imu->acc_raw[1] / (sqrt(imu->acc_raw[0] * imu->acc_raw[0] + imu->acc_raw[2]) * imu->acc_raw[2]));    //pitch
}

/**
 * @brief update function
 * @param None
 * @retval None
 */
void ahrs_update(void)
{
    imu_raw_t* imu = ahrs_instance.imu_raw;

    icm20602Update(imu);
    mahony_update(ahrs_instance.mahony_handle, imu->acc_raw[0], imu->acc_raw[1], imu->acc_raw[2], imu->gyro_raw[0],
    imu->gyro_raw[1], imu->gyro_raw[2]);
    if (ahrs_instance.imu_raw->acc_raw[2] < 0) {
        if (ahrs_instance.mahony_handle->euler_angle[0] > 0) {
            ahrs_instance.euler_angle[0] = (PI - ahrs_instance.mahony_handle->euler_angle[0]);
            ahrs_instance.euler_angle_deg[0] = ahrs_instance.euler_angle[0] * 57.295779f;
        } else {
            ahrs_instance.euler_angle[0] = -(PI + ahrs_instance.mahony_handle->euler_angle[0]);
            ahrs_instance.euler_angle_deg[0] = ahrs_instance.euler_angle[0] * 57.295779f;
        }
    } else {
        ahrs_instance.euler_angle[0] = ahrs_instance.mahony_handle->euler_angle[0];
        ahrs_instance.euler_angle_deg[0] = ahrs_instance.mahony_handle->euler_angle_deg[0];
    }

    ahrs_instance.euler_angle[0] = ahrs_instance.mahony_handle->euler_angle[0];
    ahrs_instance.euler_angle[1] = ahrs_instance.mahony_handle->euler_angle[1];
    ahrs_instance.euler_angle[2] = ahrs_instance.mahony_handle->euler_angle[2];

    ahrs_instance.euler_angle_deg[0] = ahrs_instance.mahony_handle->euler_angle_deg[0];
    ahrs_instance.euler_angle_deg[1] = ahrs_instance.mahony_handle->euler_angle_deg[1];
    ahrs_instance.euler_angle_deg[2] = ahrs_instance.mahony_handle->euler_angle_deg[2];
}