#include "EmmV5.h"

/**
 * @brief 将驱动器记录的当前位置清零。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x0A;
  cmd[2] = 0x6D;
  cmd[3] = 0x6B;

  Serial1.write(cmd, 4);
}

/**
 * @brief 清除驱动器堵转保护状态。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 */
void Emm_V5_Reset_Clog_Pro(uint8_t addr) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x0E;
  cmd[2] = 0x52;
  cmd[3] = 0x6B;

  Serial1.write(cmd, 4);
}

/**
 * @brief 读取驱动器内部参数或实时状态。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param s 要读取的参数类型，使用 SysParams_t 枚举值，例如 S_VEL 读速度、S_CPOS 读当前位置。
 */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s) {
  uint8_t i = 0;
  uint8_t cmd[16] = {0};

  cmd[i] = addr;
  ++i;

  switch (s) {
    case S_VER:
      cmd[i] = 0x1F;
      ++i;
      break;
    case S_RL:
      cmd[i] = 0x20;
      ++i;
      break;
    case S_PID:
      cmd[i] = 0x21;
      ++i;
      break;
    case S_VBUS:
      cmd[i] = 0x24;
      ++i;
      break;
    case S_CPHA:
      cmd[i] = 0x27;
      ++i;
      break;
    case S_ENCL:
      cmd[i] = 0x31;
      ++i;
      break;
    case S_TPOS:
      cmd[i] = 0x33;
      ++i;
      break;
    case S_VEL:
      cmd[i] = 0x35;
      ++i;
      break;
    case S_CPOS:
      cmd[i] = 0x36;
      ++i;
      break;
    case S_PERR:
      cmd[i] = 0x37;
      ++i;
      break;
    case S_FLAG:
      cmd[i] = 0x3A;
      ++i;
      break;
    case S_ORG:
      cmd[i] = 0x3B;
      ++i;
      break;
    case S_Conf:
      cmd[i] = 0x42;
      ++i;
      cmd[i] = 0x6C;
      ++i;
      break;
    case S_State:
      cmd[i] = 0x43;
      ++i;
      cmd[i] = 0x7A;
      ++i;
      break;
    default:
      break;
  }

  cmd[i] = 0x6B;
  ++i;

  Serial1.write(cmd, i);
}

/**
 * @brief 修改驱动器控制模式。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param svF 是否保存到驱动器掉电存储区，false 表示只本次生效，true 表示保存配置。
 * @param ctrl_mode 控制模式编号，常见含义为：0 关闭脉冲输入，1 开环，2 闭环，3 复用 En/Dir 相关功能。
 */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x46;
  cmd[2] = 0x69;
  cmd[3] = svF;
  cmd[4] = ctrl_mode;
  cmd[5] = 0x6B;

  Serial1.write(cmd, 6);
}

/**
 * @brief 控制电机使能状态。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param state 使能状态，true 表示使能电机输出，false 表示关闭电机输出。
 * @param snF 同步标志，false 表示立即执行，true 表示等待同步触发后再执行。
 */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0xF3;
  cmd[2] = 0xAB;
  cmd[3] = static_cast<uint8_t>(state);
  cmd[4] = snF;
  cmd[5] = 0x6B;

  Serial1.write(cmd, 6);
}

/**
 * @brief 速度模式控制，让电机按指定方向和速度持续转动。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param dir 旋转方向，0 表示顺时针 CW，其他值表示逆时针 CCW。
 * @param vel 目标速度，单位 RPM。
 * @param acc 加速度参数，0 表示直接启动，其他值表示按驱动器加速度参数启动。
 * @param snF 同步标志，false 表示立即执行，true 表示等待同步触发后再执行。
 */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0xF6;
  cmd[2] = dir;
  cmd[3] = static_cast<uint8_t>(vel >> 8);
  cmd[4] = static_cast<uint8_t>(vel >> 0);
  cmd[5] = acc;
  cmd[6] = snF;
  cmd[7] = 0x6B;

  Serial1.write(cmd, 8);
}

/**
 * @brief 位置模式控制，让电机按指定脉冲数运动。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param dir 旋转方向，0 表示顺时针 CW，其他值表示逆时针 CCW。
 * @param vel 运动速度，单位 RPM。
 * @param acc 加速度参数，0 表示直接启动，其他值表示按驱动器加速度参数启动。
 * @param clk 目标位移脉冲数，也就是本次要走多少步。
 * @param raF 位置模式选择，false 表示相对位置运动，true 表示绝对位置运动。
 * @param snF 同步标志，false 表示立即执行，true 表示等待同步触发后再执行。
 */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0xFD;
  cmd[2] = dir;
  cmd[3] = static_cast<uint8_t>(vel >> 8);
  cmd[4] = static_cast<uint8_t>(vel >> 0);
  cmd[5] = acc;
  cmd[6] = static_cast<uint8_t>(clk >> 24);
  cmd[7] = static_cast<uint8_t>(clk >> 16);
  cmd[8] = static_cast<uint8_t>(clk >> 8);
  cmd[9] = static_cast<uint8_t>(clk >> 0);
  cmd[10] = raF;
  cmd[11] = snF;
  cmd[12] = 0x6B;

  Serial1.write(cmd, 13);
}

/**
 * @brief 立即停止当前运动。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param snF 同步标志，false 表示立即停止，true 表示等待同步触发后再停止。
 */
void Emm_V5_Stop_Now(uint8_t addr, bool snF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0xFE;
  cmd[2] = 0x98;
  cmd[3] = snF;
  cmd[4] = 0x6B;

  Serial1.write(cmd, 5);
}

/**
 * @brief 触发同步运动，让已经设置为同步等待的驱动器同时开始执行。
 * @param addr 触发同步运动的地址，通常按驱动器协议填写单个地址或广播地址。
 */
void Emm_V5_Synchronous_motion(uint8_t addr) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0xFF;
  cmd[2] = 0x66;
  cmd[3] = 0x6B;

  Serial1.write(cmd, 4);
}

/**
 * @brief 将当前位置设置为单圈回零的零点位置。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param svF 是否保存到驱动器掉电存储区，false 表示只本次生效，true 表示保存配置。
 */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x93;
  cmd[2] = 0x88;
  cmd[3] = svF;
  cmd[4] = 0x6B;

  Serial1.write(cmd, 5);
}

/**
 * @brief 修改回零相关参数。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param svF 是否保存到驱动器掉电存储区，false 表示只本次生效，true 表示保存配置。
 * @param o_mode 回零模式编号，常见含义为：0 单圈就近回零，1 单圈按方向回零，2 多圈无限位碰撞回零，3 多圈限位开关回零。
 * @param o_dir 回零方向，0 表示顺时针 CW，其他值表示逆时针 CCW。
 * @param o_vel 回零速度，单位 RPM。
 * @param o_tm 回零超时时间，单位 ms。
 * @param sl_vel 碰撞回零检测速度，单位 RPM。
 * @param sl_ma 碰撞回零检测电流阈值，单位 mA。
 * @param sl_ms 碰撞回零检测持续时间，单位 ms。
 * @param potF 上电后是否自动触发回零，false 表示不自动回零，true 表示上电自动回零。
 */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel,
                                 uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF) {
  uint8_t cmd[32] = {0};

  cmd[0] = addr;
  cmd[1] = 0x4C;
  cmd[2] = 0xAE;
  cmd[3] = svF;
  cmd[4] = o_mode;
  cmd[5] = o_dir;
  cmd[6] = static_cast<uint8_t>(o_vel >> 8);
  cmd[7] = static_cast<uint8_t>(o_vel >> 0);
  cmd[8] = static_cast<uint8_t>(o_tm >> 24);
  cmd[9] = static_cast<uint8_t>(o_tm >> 16);
  cmd[10] = static_cast<uint8_t>(o_tm >> 8);
  cmd[11] = static_cast<uint8_t>(o_tm >> 0);
  cmd[12] = static_cast<uint8_t>(sl_vel >> 8);
  cmd[13] = static_cast<uint8_t>(sl_vel >> 0);
  cmd[14] = static_cast<uint8_t>(sl_ma >> 8);
  cmd[15] = static_cast<uint8_t>(sl_ma >> 0);
  cmd[16] = static_cast<uint8_t>(sl_ms >> 8);
  cmd[17] = static_cast<uint8_t>(sl_ms >> 0);
  cmd[18] = potF;
  cmd[19] = 0x6B;

  Serial1.write(cmd, 20);
}

/**
 * @brief 触发执行回零动作。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 * @param o_mode 回零模式编号，需要和驱动器支持的模式一致。
 * @param snF 同步标志，false 表示立即执行，true 表示等待同步触发后再执行。
 */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x9A;
  cmd[2] = o_mode;
  cmd[3] = snF;
  cmd[4] = 0x6B;

  Serial1.write(cmd, 5);
}

/**
 * @brief 强制中断当前回零过程。
 * @param addr 电机驱动器地址，需要和驱动器实际地址一致。
 */
void Emm_V5_Origin_Interrupt(uint8_t addr) {
  uint8_t cmd[16] = {0};

  cmd[0] = addr;
  cmd[1] = 0x9C;
  cmd[2] = 0x48;
  cmd[3] = 0x6B;

  Serial1.write(cmd, 4);
}

/**
 * @brief 接收驱动器返回的数据帧。
 * @param rxCmd 接收缓冲区指针，用来保存收到的每一个字节，建议缓冲区至少 128 字节。
 * @param rxCount 输出参数，函数返回时写入本次实际接收到的字节数。
 */
void Emm_V5_Receive_Data(uint8_t *rxCmd, uint8_t *rxCount) {
  int i = 0;
  unsigned long lTime;
  unsigned long cTime;

  lTime = cTime = millis();

  while (1) {
    if (Serial1.available() > 0) {
      if (i < 128) {
        rxCmd[i++] = Serial1.read();
        lTime = millis();
      }
    } else {
      cTime = millis();

      if (static_cast<int>(cTime - lTime) > 100) {
        *rxCount = i;
        break;
      }
    }
  }
}
