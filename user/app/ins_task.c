/**
******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "INS_task.h"
#include "cmsis_os.h"
#include "calibrate_task.h"

extern cali_flag_t cali_flag;

#if EKF == 1
INS_t INS;
IMU_Param_t IMU_Param;
PID_t TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;


static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
	IMU_Param.scale[X] = 1;
	IMU_Param.scale[Y] = 1;
	IMU_Param.scale[Z] = 1;
	IMU_Param.Yaw = 0;
	IMU_Param.Pitch = 0;
	IMU_Param.Roll = 0;
	IMU_Param.flag = 1;

	IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
	// imu heat init
//	pid_init(&TempCtrl, 2000, 300, 0, 1000, 20, 0, 0, 0, 0, 0, 0, 0);
	//HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

	INS.AccelLPF = 0.0085;
}


void INS_Task(void const * argument)
{
	static uint32_t count = 0;
	const float gravity[3] = {0, 0, 9.7985f};
	INS_Init();
	while (1)
	{

	    dt = DWT_GetDeltaT(&INS_DWT_Count);
	    t += dt;

	    // ins update
	    // if ((count % 1) == 0)
	    // {
	        BMI088_Read(&BMI088);

	        INS.Accel[X] = BMI088.Accel[X];
	        INS.Accel[Y] = BMI088.Accel[Y];
	        INS.Accel[Z] = BMI088.Accel[Z];
	        INS.Gyro[X] = BMI088.Gyro[X];
	        INS.Gyro[Y] = BMI088.Gyro[Y];
	        INS.Gyro[Z] = BMI088.Gyro[Z];

	        // demo function,用于??正安装?????,??以不??,本demo暂时没用
	        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

	        // 计算重力加速度矢量和b系的XY两轴的夹??,??用作功能扩展,本demo暂时没用
	        INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
	        INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

	        // 核心函数,EKF更新四元??
	        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

	        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

	        // 机体系基向量??换到导航坐标系，??例选取??性系为???航??
	        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
	        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
	        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

	        // 将重力从导航坐标系n??换到机体系b,随后根据加速度计数??计算运动加速度
	        float gravity_b[3];
	        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
	        for (uint8_t i = 0; i < 3; i++) // 同样过一??低通滤??
	        {
	            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
	        }
	        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // ??换回导航系n

	        // 获取最终数值
	        INS.Yaw = QEKF_INS.Yaw * ANGLE_TO_RADIAN;
	        INS.Pitch = QEKF_INS.Pitch * ANGLE_TO_RADIAN;
	        INS.Roll = QEKF_INS.Roll * ANGLE_TO_RADIAN;
	        INS.YawTotalAngle = QEKF_INS.YawTotalAngle * ANGLE_TO_RADIAN;
	    // }

	    // temperature control
	    // if ((count % 2) == 0)
	    // {
	    //     // 500hz
	    //     //IMU_Temperature_Ctrl();
	    // }
	    //
	    // if ((count % 1000) == 0)
	    // {
	    //     // 200hz
	    // }

	    //count++;
		osDelay(1);
		}
}

#elif EKF ==0
INS_t INS;

struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;
float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
float ins_time;
int stop_time;

#define INS_GYRO_BIAS_INIT_SAMPLES 1000u
#define INS_GYRO_STILL_THRESHOLD 0.08f
#define INS_ACCEL_STILL_MIN 8.8f
#define INS_ACCEL_STILL_MAX 10.8f
#define INS_GYRO_BIAS_TRACK_ALPHA 0.0005f

static void INS_GyroBias_Init(void);
static void INS_GyroBias_Update(const float gyro[3], const float accel[3]);
static uint8_t INS_IsStill(const float gyro[3], const float accel[3]);

void INS_Init(void)
{
	mahony_init(&mahony,1.0f,0.02f,0.001f);
	INS.AccelLPF = 0.0089f;
	INS.GyroBias[X] = 0.0f;
	INS.GyroBias[Y] = 0.0f;
	INS.GyroBias[Z] = 0.0f;
}
void INS_Task(void const * argument)
{
	INS_Init();
	INS_GyroBias_Init();
	while(1)
	{
		ins_dt = DWT_GetDeltaT(&INS_DWT_Count);

		mahony.dt = ins_dt;

		BMI088_Read(&BMI088);

		INS.Accel[X] = BMI088.Accel[X];
		INS.Accel[Y] = BMI088.Accel[Y];
		INS.Accel[Z] = BMI088.Accel[Z];
		Accel.x=BMI088.Accel[0];
		Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
		INS.Gyro[X] = BMI088.Gyro[X];
		INS.Gyro[Y] = BMI088.Gyro[Y];
		INS.Gyro[Z] = BMI088.Gyro[Z];
		INS_GyroBias_Update(INS.Gyro, INS.Accel);
		INS.Gyro[X] -= INS.GyroBias[X];
		INS.Gyro[Y] -= INS.GyroBias[Y];
		INS.Gyro[Z] -= INS.GyroBias[Z];
		Gyro.x=INS.Gyro[0];
		Gyro.y=INS.Gyro[1];
		Gyro.z=INS.Gyro[2];

		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);

		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;

		// 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
		for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
		{
			INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt)
															  + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt);
			//				INS.MotionAccel_b[i] = (INS.Accel[i] ) * dt / (INS.AccelLPF + dt)
			//															+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

		//死区处理
		if(fabsf(INS.MotionAccel_n[0])<0.02f)
		{
			INS.MotionAccel_n[0]=0.0f;	//x轴
		}
		if(fabsf(INS.MotionAccel_n[1])<0.02f)
		{
			INS.MotionAccel_n[1]=0.0f;	//y轴
		}
		if(fabsf(INS.MotionAccel_n[2])<0.04f)
		{
			INS.MotionAccel_n[2]=0.0f;//z轴
		}

		if(ins_time>3000.0f)
		{
			INS.ins_flag=1;//四元数基本收敛，加速度也基本收敛，可以开始底盘任务
			// 获取最终数据
			INS.Pitch=mahony.roll;
			INS.Roll=mahony.pitch;
			INS.Yaw=mahony.yaw;

			//INS.YawTotalAngle=INS.YawTotalAngle+INS.Gyro[2]*0.001f;

			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
				INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
				INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
			ins_time++;
		}
		osDelay(1);
	}
}

static uint8_t INS_IsStill(const float gyro[3], const float accel[3])
{
	float gyro_norm = sqrtf(gyro[X] * gyro[X] + gyro[Y] * gyro[Y] + gyro[Z] * gyro[Z]);
	float accel_norm = sqrtf(accel[X] * accel[X] + accel[Y] * accel[Y] + accel[Z] * accel[Z]);

	return gyro_norm < INS_GYRO_STILL_THRESHOLD &&
		   accel_norm > INS_ACCEL_STILL_MIN &&
		   accel_norm < INS_ACCEL_STILL_MAX;
}

static void INS_GyroBias_Init(void)
{
	float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
	float gyro_sample[3];
	float accel_sample[3];
	uint16_t valid_count = 0;

	for (uint16_t i = 0; i < INS_GYRO_BIAS_INIT_SAMPLES; i++)
	{
		BMI088_Read(&BMI088);

		gyro_sample[X] = BMI088.Gyro[X];
		gyro_sample[Y] = BMI088.Gyro[Y];
		gyro_sample[Z] = BMI088.Gyro[Z];
		accel_sample[X] = BMI088.Accel[X];
		accel_sample[Y] = BMI088.Accel[Y];
		accel_sample[Z] = BMI088.Accel[Z];

		if (INS_IsStill(gyro_sample, accel_sample))
		{
			gyro_sum[X] += gyro_sample[X];
			gyro_sum[Y] += gyro_sample[Y];
			gyro_sum[Z] += gyro_sample[Z];
			valid_count++;
		}

		osDelay(1);
	}

	if (valid_count > (INS_GYRO_BIAS_INIT_SAMPLES / 2u))
	{
		INS.GyroBias[X] = gyro_sum[X] / valid_count;
		INS.GyroBias[Y] = gyro_sum[Y] / valid_count;
		INS.GyroBias[Z] = gyro_sum[Z] / valid_count;
	}
}

static void INS_GyroBias_Update(const float gyro[3], const float accel[3])
{
	if (INS_IsStill(gyro, accel))
	{
		INS.GyroBias[X] += (gyro[X] - INS.GyroBias[X]) * INS_GYRO_BIAS_TRACK_ALPHA;
		INS.GyroBias[Y] += (gyro[Y] - INS.GyroBias[Y]) * INS_GYRO_BIAS_TRACK_ALPHA;
		INS.GyroBias[Z] += (gyro[Z] - INS.GyroBias[Z]) * INS_GYRO_BIAS_TRACK_ALPHA;
	}
}
#endif


#if EKF == 1


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于????IMU安????????与标度因数?????,即陀螺仪轴和云台轴的安???偏??
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

/**
 * @brief 温度控制
 *
 */
// void IMU_Temperature_Ctrl(void)
// {
//     pid_calculate(&TempCtrl, BMI088.Temperature, RefTemp);
//
//     TIM_Set_PWM(&htim10, TIM_CHANNEL_1, fp32_constrain(fp32_rounding(TempCtrl.Output), 0, (fp32)UINT32_MAX));
// }

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}


const INS_t* get_INS_point(void)
{
    return &INS;
}

#elif EKF ==0

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

#endif
