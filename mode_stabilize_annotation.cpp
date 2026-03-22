#include "Copter.h"
// 引入无人机飞控的核心头文件，包含
// 所有必要的类（如AP_Motors、AttitudeControl）、全局对象（如copter、motors）、枚举（如SpoolState）和函数声明，是代码能运行的基础。

// 该函数是稳定模式的主控制器，必须以 100Hz 或更高频率调用（高频调用才能保证姿态控制的实时性，避免无人机抖动 / 失控）。

void ModeStabilize::run() // 实现之前ModeStabilize类中声明的虚函数run()，是稳定模式的核心执行逻辑。
{

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    //

    // 处理飞行员输入的前置转换

    update_simple_mode();
    // 处理 “简易模式”（如无头模式、角度模式）的输入转换，比如把遥控器的 “相对方向” 转换成无人机的 “绝对方向”，适配飞行员的操作习惯。

    // convert pilot input to lean angles 将飞行员的输入转换为倾斜角度
    float target_roll_rad, target_pitch_rad;
    get_pilot_desired_lean_angles_rad(target_roll_rad, target_pitch_rad, attitude_control->lean_angle_max_rad(), attitude_control->lean_angle_max_rad());
    // 把遥控器横滚（左右）、俯仰（前后）摇杆的输入值，转换成目标倾斜角度（单位：弧度），存入target_roll_rad（横滚）、target_pitch_rad（俯仰）

    // get pilot's desired yaw rate
    float target_yaw_rate_rads = get_pilot_desired_yaw_rate_rads();
    // 把遥控器偏航（左右旋转）摇杆的输入，转换成目标偏航角速度（单位：弧度 / 秒）（稳定模式下，飞行员控制的是 “旋转速度”，而非绝对偏航角度）

    //
    //
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (!motors->armed())
    { // 电机是否解锁
        // Motors should be Stopped
        // // 条件1：电机未解锁 → 强制关机
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN); //	SHUT_DOWN 关机状态 电机完全停转，无任何输出（即使解锁，只要设为该状态，电机也不转）。
    }
    else if (copter.ap.throttle_zero //	地面怠速 电机解锁后低转速怠速（比如螺旋桨缓慢转动），未起飞时的待命状态，便于快速响应油门起飞。
             || (copter.air_mode == AirMode::AIRMODE_ENABLED && motors->get_spool_state() == AP_Motors::SpoolState::SHUT_DOWN))
    {
        // 条件2：电机已解锁，但满足以下任一情况 → 设为地面怠速
        // 情况1：油门零位；情况2：空模式启用 且 电机当前是关机状态
        // 空模式下throttle_zero永远为false，但电机需要先进入地面怠速，以便完成滑跑启动流程
        // throttle_zero is never true in air mode, but the motors should be allowed to go through ground idle
        // in order to facilitate the spoolup block
        // Attempting to Land
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    }
    else
    {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    } // 条件3：其他所有情况 → 油门无限制（正常飞行）

    float pilot_desired_throttle = get_pilot_desired_throttle();
    // 获取飞行员期望的油门值
    switch (motors->get_spool_state())
    {
    case AP_Motors::SpoolState::SHUT_DOWN: // 电机关机
        // Motors Stopped
        attitude_control->reset_yaw_target_and_rate();     // 重置偏航控制目标/速率
        attitude_control->reset_rate_controller_I_terms(); // 清空速率控制器积分项
        pilot_desired_throttle = 0.0f;                     // 油门置0
        break;

    case AP_Motors::SpoolState::GROUND_IDLE: // 地面怠速
        // Landed
        attitude_control->reset_yaw_target_and_rate();              // 重置偏航控制目标/速率
        attitude_control->reset_rate_controller_I_terms_smoothly(); // 清空速率控制器积分项
        pilot_desired_throttle = 0.0f;                              // 油门置0
        break;

    case AP_Motors::SpoolState::THROTTLE_UNLIMITED: // 油门无限制

        if (!motors->limit.throttle_lower)
        {                             // 电机油门下限限制
            set_land_complete(false); // 设置 “着陆完成” false,飞控判定 “无人机已离地
        }
        break;

    case AP_Motors::SpoolState::SPOOLING_UP:
    case AP_Motors::SpoolState::SPOOLING_DOWN:
        // do nothing
        break;
    }

    // call attitude controller 把之前转换好的目标姿态参数传给姿态控制器，飞控内部通过 PID 算法计算每个电机的转速，实现 “按飞行员意图稳定姿态”
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw_rad(target_roll_rad, target_pitch_rad, target_yaw_rate_rads);

    // output pilot's throttle 将飞行员的油门输入输出给电机，true表示启用油门限制（防止超量程），g.throttle_filt启用油门滤波（让油门变化更平滑，避免电机转速突变导致无人机抖动）
    attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
    // 姿态控制：让无人机保持 / 达到期望的横滚、俯仰、偏航状态（比如左倾 10°、机头以 10°/ 秒转向）；
    // 油门控制：给所有电机一个 “基准动力值”（比如 50% 油门），再叠加姿态控制的力矩调整（比如左电机多 5%、右少 5%），最终实现 “既保持高度，又修正姿态”。
}
