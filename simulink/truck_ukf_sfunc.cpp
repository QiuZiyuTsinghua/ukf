/*
 * truck_ukf_sfunc.cpp
 * 
 * 半挂卡车UKF车速估计S-Function
 * 用于Simulink与TruckSim联合仿真
 * 
 * 输入:
 *   - 轮速信号 (m/s)
 *   - 加速度计信号 ax, ay (m/s²)
 *   - 陀螺仪航向角速度 (rad/s)
 *   - 油门开度 (0-1)
 *   - 制动压力 (0-1)
 *   - 方向盘转角 (rad)
 * 
 * 输出:
 *   - 估计车速 (m/s)
 *   - 估计位置 x, y (m)
 *   - 估计航向角 (rad)
 *   - 估计加速度 ax, ay (m/s²)
 *   - 速度估计不确定度 (m/s)
 */

#define S_FUNCTION_NAME  truck_ukf_sfunc
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "truck_ukf.h"
#include <memory>
#include <map>

// S-Function参数
#define NUM_PARAMS 7
#define WHEELBASE_PARAM         ssGetSFcnParam(S, 0)
#define MASS_PARAM             ssGetSFcnParam(S, 1)
#define FRONT_AREA_PARAM       ssGetSFcnParam(S, 2)
#define DRAG_COEFF_PARAM       ssGetSFcnParam(S, 3)
#define INITIAL_SPEED_PARAM    ssGetSFcnParam(S, 4)
#define SAMPLE_TIME_PARAM      ssGetSFcnParam(S, 5)
#define ENABLE_LOGGING_PARAM   ssGetSFcnParam(S, 6)

// 输入输出端口定义
#define INPUT_WHEEL_SPEED    0
#define INPUT_ACC_X          1
#define INPUT_ACC_Y          2
#define INPUT_YAW_RATE       3
#define INPUT_THROTTLE       4
#define INPUT_BRAKE          5
#define INPUT_STEERING       6

#define OUTPUT_EST_SPEED     0
#define OUTPUT_EST_POS_X     1
#define OUTPUT_EST_POS_Y     2
#define OUTPUT_EST_YAW       3
#define OUTPUT_EST_ACC_X     4
#define OUTPUT_EST_ACC_Y     5
#define OUTPUT_SPEED_UNCERT  6

// 全局UKF对象存储
static std::map<SimStruct*, std::unique_ptr<TruckUKF>> ukf_instances;

/*================*
 * S-function methods *
 *================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, NUM_PARAMS);  /* Number of expected parameters */
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        /* Return if number of expected != number of actual parameters */
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, 7);  /* [wheel_speed, acc_x, acc_y, yaw_rate, throttle, brake, steering] */
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 7);  /* [est_speed, est_x, est_y, est_yaw, est_ax, est_ay, speed_uncertainty] */

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 1);  /* 存储UKF对象指针 */
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    /* Specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S, 0);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    This function is used to specify the sample time(s) for your
 *    S-function. You must register the same number of sample times as
 *    specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    real_T sample_time = mxGetPr(SAMPLE_TIME_PARAM)[0];
    ssSetSampleTime(S, 0, sample_time);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
#if defined(MDL_INITIALIZE_CONDITIONS)
/* Function: mdlInitializeConditions ========================================
 * Abstract:
 *    In this function, you should initialize the continuous and discrete
 *    states for your S-function block.  The initial states are placed
 *    in the state vector, ssGetContStates(S) or ssGetRealDiscStates(S).
 *    You can also perform any other initialization activities that your
 *    S-function may require. Note, this routine will be called at the
 *    start of simulation and if it is present in an enabled subsystem
 *    configured to reset states, it will be called when the enabled
 *    subsystem restarts execution to reset the states.
 */
static void mdlInitializeConditions(SimStruct *S)
{
    // 创建UKF实例
    auto ukf = std::make_unique<TruckUKF>();
    
    // 设置车辆参数
    real_T wheelbase = mxGetPr(WHEELBASE_PARAM)[0];
    real_T mass = mxGetPr(MASS_PARAM)[0];
    real_T front_area = mxGetPr(FRONT_AREA_PARAM)[0];
    real_T drag_coeff = mxGetPr(DRAG_COEFF_PARAM)[0];
    real_T initial_speed = mxGetPr(INITIAL_SPEED_PARAM)[0];
    
    ukf->setVehicleParameters(wheelbase, mass, front_area, drag_coeff);
    ukf->initialize(initial_speed);
    
    // 存储UKF实例
    ukf_instances[S] = std::move(ukf);
    
    // 存储指针到PWork
    ssGetPWork(S)[0] = &ukf_instances[S];
}
#endif /* MDL_INITIALIZE_CONDITIONS */

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // 获取UKF实例
    auto* ukf_ptr = static_cast<std::unique_ptr<TruckUKF>*>(ssGetPWork(S)[0]);
    TruckUKF* ukf = ukf_ptr->get();
    
    if (!ukf) {
        ssSetErrorStatus(S, "UKF instance not found");
        return;
    }
    
    // 获取输入
    const real_T *u = (const real_T*) ssGetInputPortSignal(S, 0);
    real_T wheel_speed = u[0];
    real_T acc_x = u[1];
    real_T acc_y = u[2];
    real_T yaw_rate = u[3];
    real_T throttle = u[4];
    real_T brake = u[5];
    real_T steering = u[6];
    
    // 获取采样时间
    real_T sample_time = mxGetPr(SAMPLE_TIME_PARAM)[0];
    
    // UKF预测步骤
    ukf->predict(sample_time, throttle, brake, steering);
    
    // UKF更新步骤
    ukf->update(wheel_speed, acc_x, acc_y, yaw_rate);
    
    // 获取状态估计
    Eigen::VectorXd state = ukf->getState();
    Eigen::MatrixXd covariance = ukf->getCovariance();
    
    // 设置输出
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    y[0] = state(2);  // 估计速度
    y[1] = state(0);  // 估计位置 x
    y[2] = state(1);  // 估计位置 y
    y[3] = state(3);  // 估计航向角
    y[4] = state(5);  // 估计加速度 ax
    y[5] = state(6);  // 估计加速度 ay
    y[6] = sqrt(covariance(2,2));  // 速度不确定度
}

#define MDL_UPDATE  /* Change to #undef to remove function */
#if defined(MDL_UPDATE)
/* Function: mdlUpdate ======================================================
 * Abstract:
 *    This function is called once for every major integration time step.
 *    Discrete states are typically updated here, but this function is useful
 *    for performing any tasks that should only take place once per
 *    integration step.
 */
static void mdlUpdate(SimStruct *S, int_T tid)
{
    // 此函数在离散状态更新时调用，我们使用连续更新，所以这里为空
}
#endif /* MDL_UPDATE */

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    // 清理UKF实例
    auto it = ukf_instances.find(S);
    if (it != ukf_instances.end()) {
        ukf_instances.erase(it);
    }
}

/*======================================================*
 * See sfuntmpl_doc.c for the optional S-function methods *
 *======================================================*/

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
