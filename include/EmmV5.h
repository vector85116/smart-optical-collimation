#pragma once

#include <Arduino.h>

// 系统参数读取类型，作为 Emm_V5_Read_Sys_Params() 的第二个参数使用。
enum SysParams_t : uint8_t {
  S_VER = 0,    // 读取固件版本和硬件版本。
  S_RL = 1,     // 读取相电阻和相电感。
  S_PID = 2,    // 读取 PID 参数。
  S_VBUS = 3,   // 读取总线电压。
  S_CPHA = 5,   // 读取相电流。
  S_ENCL = 7,   // 读取线性化校准后的编码器值。
  S_TPOS = 8,   // 读取电机目标位置角度。
  S_VEL = 9,    // 读取电机实时转速。
  S_CPOS = 10,  // 读取电机实时位置角度。
  S_PERR = 11,  // 读取位置误差角度。
  S_FLAG = 13,  // 读取使能、到位、堵转等状态标志。
  S_Conf = 14,  // 读取驱动器配置参数。
  S_State = 15, // 读取系统状态参数。
  S_ORG = 16,   // 读取回零状态标志。
};

/**
 * @brief 将电机当前位置设为零点。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);

/**
 * @brief 清除驱动器的堵转保护状态。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr);

/**
 * @brief 按指定类型读取驱动器内部参数。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param s 要读取的参数类型，使用上面的 SysParams_t 枚举值。
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);

/**
 * @brief 修改驱动器控制模式。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param svF 是否保存到掉电存储区，false 为仅本次生效，true 为保存配置。
 * @param ctrl_mode 控制模式编号。
 *                  0：关闭脉冲输入引脚。
 *                  1：开环模式。
 *                  2：闭环模式。
 *                  3：复用 En/Dir 相关功能，具体以驱动器手册为准。
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);

/**
 * @brief 控制电机使能状态。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param state 使能状态，true 为使能电机，false 为关闭电机输出。
 * @param snF 同步标志，false 为立即执行，true 为等待同步触发。
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);

/**
 * @brief 速度模式控制。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param dir 旋转方向，0 为顺时针 CW，其他值为逆时针 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度参数，0 表示直接启动，其他值表示按加速度启动。
 * @param snF 同步标志，false 为立即执行，true 为等待同步触发。
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

/**
 * @brief 位置模式控制。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param dir 旋转方向，0 为顺时针 CW，其他值为逆时针 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度参数，0 表示直接启动，其他值表示按加速度启动。
 * @param clk 脉冲数，也就是目标位移量。
 * @param raF 位置模式，false 为相对运动，true 为绝对运动。
 * @param snF 同步标志，false 为立即执行，true 为等待同步触发。
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

/**
 * @brief 立即停止电机运动。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param snF 同步标志，false 为立即执行，true 为等待同步触发。
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF);

/**
 * @brief 触发所有已设置为同步模式的电机同时开始动作。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 */
void Emm_V5_Synchronous_motion(uint8_t addr);

/**
 * @brief 把当前位置设置为单圈回零的零点位置。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param svF 是否保存到掉电存储区，false 为仅本次生效，true 为保存配置。
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);

/**
 * @brief 修改回零参数。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param svF 是否保存到掉电存储区，false 为仅本次生效，true 为保存配置。
 * @param o_mode 回零模式编号，常见含义为：
 *               0 单圈就近回零，1 单圈按方向回零，
 *               2 多圈无限位碰撞回零，3 多圈限位开关回零。
 * @param o_dir 回零方向，0 为顺时针 CW，其他值为逆时针 CCW。
 * @param o_vel 回零速度，单位 RPM。
 * @param o_tm 回零超时时间，单位 ms。
 * @param sl_vel 碰撞回零检测速度，单位 RPM。
 * @param sl_ma 碰撞回零检测电流阈值，单位 mA。
 * @param sl_ms 碰撞回零检测持续时间，单位 ms。
 * @param potF 上电是否自动触发回零，false 为不自动回零，true 为上电自动回零。
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel,
                                 uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/**
 * @brief 触发执行回零动作。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 * @param o_mode 回零模式编号，需与驱动器支持的模式一致。
 * @param snF 同步标志，false 为立即执行，true 为等待同步触发。
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);

/**
 * @brief 强制中断当前回零过程。
 * @param addr 驱动器地址，需与驱动器实际地址一致。
 */
void Emm_V5_Origin_Interrupt(uint8_t addr);

/**
 * @brief 接收驱动器返回的数据帧。
 * @param rxCmd 接收缓冲区指针，用来保存收到的每一个字节，缓冲区建议至少 128 字节。
 * @param rxCount 输出参数，返回本次实际接收到的字节数。
 */
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount);
