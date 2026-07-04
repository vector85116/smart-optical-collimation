#include <Arduino.h>

#include "EmmV5.h"

namespace {

// ======================= 电机地址与方向定义 =======================

constexpr uint8_t kPanMotorAddr = 1;        // 1 号电机地址：控制左右轴。
constexpr uint8_t kTiltMotorAddr = 2;       // 2 号电机地址：控制上下轴。
constexpr uint8_t kSecondPanMotorAddr = 3;  // 3 号电机地址：第二组左右轴。
constexpr uint8_t kSecondTiltMotorAddr = 4; // 4 号电机地址：第二组上下轴。

constexpr uint8_t kCwDir = 0;               // 驱动器方向值：0 表示顺时针 CW。
constexpr uint8_t kCcwDir = 1;              // 驱动器方向值：1 表示逆时针 CCW。

constexpr uint8_t kPanLeftDir = kCwDir;     // 1 号电机顺时针 = 向左。
constexpr uint8_t kPanRightDir = kCcwDir;   // 1 号电机逆时针 = 向右。
constexpr uint8_t kTiltUpDir = kCwDir;      // 2 号电机顺时针 = 向上。
constexpr uint8_t kTiltDownDir = kCcwDir;   // 2 号电机逆时针 = 向下。
constexpr uint8_t kSecondPanLeftDir = kPanLeftDir;
constexpr uint8_t kSecondPanRightDir = kPanRightDir;
constexpr uint8_t kSecondTiltUpDir = kTiltDownDir;
constexpr uint8_t kSecondTiltDownDir = kTiltUpDir;

// ======================= 串口配置 =======================

constexpr int kHostUartRxPin = 38;          // 上位机外部串口 RX 引脚，-1 表示暂不启用外部上位机串口。
constexpr int kHostUartTxPin = 39;          // 上位机外部串口 TX 引脚，-1 表示暂不启用外部上位机串口。
constexpr uint32_t kHostUartBaud = 115200;  // 上位机外部串口波特率。

constexpr int kSecondHostUartRxPin = 13;         // 第二个上位机/相机串口 RX 引脚，未定时保持 -1。
constexpr int kSecondHostUartTxPin = 14;         // 第二个上位机/相机串口 TX 引脚，未定时保持 -1。
constexpr uint32_t kSecondHostUartBaud = 115200; // 第二个上位机/相机串口波特率。

constexpr int kMotorUartRxPin = 18;         // ESP32 接收电机驱动器数据的 RX 引脚，接驱动器 TX。
constexpr int kMotorUartTxPin = 17;         // ESP32 发送数据到电机驱动器的 TX 引脚，接驱动器 RX。
constexpr uint32_t kMotorUartBaud = 115200; // 电机驱动器串口波特率。

// ======================= 正常跟踪运动参数 =======================

constexpr uint16_t kMoveSpeedRpm = 1;       // 正常方向单步运动速度，单位 RPM。
constexpr uint8_t kMoveAcc = 0;             // 加速度参数，0 表示直接启动。
constexpr uint32_t kMovePulse = 50;         // 每收到一次方向命令运动的脉冲数。
constexpr uint32_t kSecondMovePulse = 3;    // 第二组单步脉冲数；如果仍然偏大/偏小，只调这个值。

// ======================= missing 搜索参数 =======================

// 收到 missing 后，使用位置模式画逐渐扩大的正方形。
// 如果画得太小，增大 kSearchStartPulse 和 kSearchPulseStep。
// 如果画得太慢，增大 kSearchSpeedRpm。
constexpr uint16_t kSearchSpeedRpm = 5;       // 搜索时位置模式速度，单位 RPM。
constexpr uint8_t kSearchAcc = 0;             // 搜索时加速度参数。
constexpr uint32_t kSearchStartPulse = 200;    // 第一圈正方形边长，单位：脉冲。
constexpr uint32_t kSearchPulseStep = 200;     // 每画完一圈，边长增加的脉冲数。
constexpr uint32_t kSearchMaxPulse = 3000;    // 最大搜索边长，避免无限变大。
constexpr uint32_t kMotorPulsesPerRev = 3200; // 每转脉冲数，按你的驱动器细分设置修改。
constexpr unsigned long kSearchExtraWaitMs = 10; // 每条边走完后额外等待时间。

// ======================= 通信与停止参数 =======================

constexpr uint8_t kStopRetryCount = 3;        // 停止时重复发送停止帧的次数。
constexpr size_t kMaxCommandLength = 64;      // 单条命令最大长度。
constexpr unsigned long kCommandFlushMs = 50; // 没有换行符时，超过该时间也处理当前缓冲命令。
constexpr unsigned long kSecondHostArmDelayMs = 3000; // 第一组连续 3 个 stop 后，延迟接收 HOST2 的时间。
constexpr int kForcePauseButtonPin = 6;       // 外部强制暂停按键 GPIO；按键另一端接 GND。
constexpr unsigned long kForcePauseDebounceMs = 30; // 强制暂停按键消抖时间。
constexpr int kRestartButtonPin = 7;          // 外部重启按键 GPIO；按键另一端接 GND。
constexpr unsigned long kRestartDebounceMs = 30; // 重启按键消抖时间。

// ======================= 状态定义 =======================

enum class PanMotion : int8_t {
  Stop = 0,
  Left = -1,
  Right = 1,
};

enum class TiltMotion : int8_t {
  Stop = 0,
  Down = -1,
  Up = 1,
};

struct MotionCommand {
  PanMotion pan = PanMotion::Stop;
  TiltMotion tilt = TiltMotion::Stop;
  bool valid = false;
  bool stopAll = false;
  bool missing = false;
};

struct CommandInput {
  CommandInput() = default;

  CommandInput(Stream *streamIn,
               const char *nameIn,
               bool echoIn,
               bool reportUnknownIn = true,
               bool showUnknownAsRxIn = false)
      : stream(streamIn),
        name(nameIn),
        echo(echoIn),
        reportUnknown(reportUnknownIn),
        showUnknownAsRx(showUnknownAsRxIn) {}

  Stream *stream = nullptr;
  String line;
  unsigned long lastByteMs = 0;
  const char *name = "";
  bool echo = false;
  bool reportUnknown = true;
  bool showUnknownAsRx = false;
};

struct MotorGroup {
  MotorGroup(const char *nameIn,
             uint8_t panAddrIn,
             uint8_t tiltAddrIn,
             uint8_t panLeftDirIn,
             uint8_t panRightDirIn,
             uint8_t tiltUpDirIn,
             uint8_t tiltDownDirIn,
             uint32_t movePulseIn)
      : name(nameIn),
        panMotorAddr(panAddrIn),
        tiltMotorAddr(tiltAddrIn),
        panLeftDir(panLeftDirIn),
        panRightDir(panRightDirIn),
        tiltUpDir(tiltUpDirIn),
        tiltDownDir(tiltDownDirIn),
        movePulse(movePulseIn) {}

  const char *name = "";
  uint8_t panMotorAddr = 0;
  uint8_t tiltMotorAddr = 0;
  uint8_t panLeftDir = kPanLeftDir;
  uint8_t panRightDir = kPanRightDir;
  uint8_t tiltUpDir = kTiltUpDir;
  uint8_t tiltDownDir = kTiltDownDir;
  uint32_t movePulse = kMovePulse;

  PanMotion currentPan = PanMotion::Stop;
  TiltMotion currentTilt = TiltMotion::Stop;

  bool searchMode = false;
  uint8_t searchSideIndex = 0;              // 0=右，1=上，2=左，3=下。
  uint32_t searchPulse = kSearchStartPulse; // 当前正方形边长。
  uint32_t searchCycleCount = 0;            // 已完成的正方形圈数。
  unsigned long nextSearchActionMs = 0;     // 下一条边开始执行的时间。
};

CommandInput gUsbInput(&Serial, "USB", true);
CommandInput gHostInput;
CommandInput gSecondHostInput;
bool gHostEnabled = false;
bool gSecondHostEnabled = false;

MotorGroup gPrimaryGroup("M1M2",
                         kPanMotorAddr,
                         kTiltMotorAddr,
                         kPanLeftDir,
                         kPanRightDir,
                         kTiltUpDir,
                         kTiltDownDir,
                         kMovePulse);
MotorGroup gSecondGroup("M3M4",
                        kSecondPanMotorAddr,
                        kSecondTiltMotorAddr,
                        kSecondPanLeftDir,
                        kSecondPanRightDir,
                        kSecondTiltUpDir,
                        kSecondTiltDownDir,
                        kSecondMovePulse);
uint8_t gPrimaryHostStopCount = 0;
bool gSecondHostInputArmed = false;
bool gExternalHostInputPaused = false;
unsigned long gSecondHostArmAtMs = 0;
bool gForcePauseLatched = false;
bool gForcePauseLastReading = false;
bool gForcePauseStablePressed = false;
unsigned long gForcePauseLastChangeMs = 0;
bool gRestartLastReading = false;
bool gRestartStablePressed = false;
unsigned long gRestartLastChangeMs = 0;

// ======================= 串口回复 =======================

void writeReply(CommandInput &input, const String &message) {
  if (input.stream == &Serial) {
    Serial.println(message);
    return;
  }

  Serial.print("[");
  Serial.print(input.name);
  Serial.print("] ");
  Serial.println(message);
}

const char *panName(PanMotion motion) {
  switch (motion) {
    case PanMotion::Left:
      return "left";
    case PanMotion::Right:
      return "right";
    case PanMotion::Stop:
    default:
      return "stop";
  }
}

const char *tiltName(TiltMotion motion) {
  switch (motion) {
    case TiltMotion::Up:
      return "up";
    case TiltMotion::Down:
      return "down";
    case TiltMotion::Stop:
    default:
      return "stop";
  }
}

void printCommandList(CommandInput &input) {
  writeReply(input, "支持的命令只有以下这些：");
  writeReply(input, "  left,up");
  writeReply(input, "  left,down");
  writeReply(input, "  right,up");
  writeReply(input, "  right,down");
  writeReply(input, "  left only");
  writeReply(input, "  right only");
  writeReply(input, "  up only");
  writeReply(input, "  down only");
  writeReply(input, "  stop");
  writeReply(input, "  missing");
  writeReply(input, "  g1 left,up  -> M1M2");
  writeReply(input, "  g2 left,up  -> M3M4");
}

// ======================= 电机底层控制 =======================

void flushMotorUart() {
  Serial1.flush();
  delay(2);
}

void stopMotor(uint8_t addr) {
  // 只发送立即停止帧，不关闭使能，停止后保持力矩。
  for (uint8_t i = 0; i < kStopRetryCount; ++i) {
    Emm_V5_Stop_Now(addr, false);
    flushMotorUart();
    delay(5);
  }
}

void enableMotor(uint8_t addr) {
  Emm_V5_En_Control(addr, true, false);
  flushMotorUart();
}

void enableMotorGroup(const MotorGroup &group) {
  enableMotor(group.panMotorAddr);
  enableMotor(group.tiltMotorAddr);
}

void enableAllMotorGroups() {
  enableMotorGroup(gPrimaryGroup);
  enableMotorGroup(gSecondGroup);
}

void moveMotorByPulse(uint8_t addr, uint8_t dir, uint32_t pulse, uint16_t speedRpm, uint8_t acc) {
  if (pulse == 0) {
    return;
  }

  enableMotor(addr);
  delay(2);

  Emm_V5_Pos_Control(
    addr,
    dir,
    speedRpm,
    acc,
    pulse,
    false,   // false = 相对位置运动
    false    // false = 立即执行
  );

  flushMotorUart();
}

void stopMotorGroup(MotorGroup &group) {
  stopMotor(group.panMotorAddr);
  stopMotor(group.tiltMotorAddr);

  group.currentPan = PanMotion::Stop;
  group.currentTilt = TiltMotion::Stop;
}

void stopAllMotorGroups() {
  stopMotorGroup(gPrimaryGroup);
  stopMotorGroup(gSecondGroup);
}

// ======================= missing 搜索控制 =======================

unsigned long estimateSearchSideTimeMs(uint32_t pulse) {
  if (kSearchSpeedRpm == 0 || kMotorPulsesPerRev == 0) {
    return 300;
  }

  const uint64_t numerator = static_cast<uint64_t>(pulse) * 60000ULL;
  const uint64_t denominator = static_cast<uint64_t>(kSearchSpeedRpm) * kMotorPulsesPerRev;

  unsigned long moveMs = static_cast<unsigned long>(numerator / denominator);

  if (moveMs < 50) {
    moveMs = 50;
  }

  return moveMs + kSearchExtraWaitMs;
}

void resetSearchState(MotorGroup &group) {
  group.searchSideIndex = 0;
  group.searchPulse = kSearchStartPulse;
  group.searchCycleCount = 0;
  group.nextSearchActionMs = 0;
}

void startSearchMode(MotorGroup &group) {
  if (group.searchMode) {
    return;
  }

  stopMotorGroup(group);

  group.searchMode = true;
  resetSearchState(group);
}

void stopSearchMode(MotorGroup &group) {
  if (!group.searchMode) {
    return;
  }

  group.searchMode = false;
  stopMotorGroup(group);
}

void sendOneSearchSide(MotorGroup &group) {
  switch (group.searchSideIndex) {
    case 0:
      // 第一条边：向右
      moveMotorByPulse(group.panMotorAddr, group.panRightDir, group.searchPulse, kSearchSpeedRpm, kSearchAcc);
      break;

    case 1:
      // 第二条边：向上
      moveMotorByPulse(group.tiltMotorAddr, group.tiltUpDir, group.searchPulse, kSearchSpeedRpm, kSearchAcc);
      break;

    case 2:
      // 第三条边：向左
      moveMotorByPulse(group.panMotorAddr, group.panLeftDir, group.searchPulse, kSearchSpeedRpm, kSearchAcc);
      break;

    case 3:
    default:
      // 第四条边：向下
      moveMotorByPulse(group.tiltMotorAddr, group.tiltDownDir, group.searchPulse, kSearchSpeedRpm, kSearchAcc);
      break;
  }

  group.nextSearchActionMs = millis() + estimateSearchSideTimeMs(group.searchPulse);

  group.searchSideIndex++;

  if (group.searchSideIndex >= 4) {
    group.searchSideIndex = 0;
    group.searchCycleCount++;

    if (group.searchPulse + kSearchPulseStep <= kSearchMaxPulse) {
      group.searchPulse += kSearchPulseStep;
    } else {
      group.searchPulse = kSearchMaxPulse;
    }
  }
}

void runSearchMode(MotorGroup &group) {
  if (!group.searchMode) {
    return;
  }

  const unsigned long now = millis();

  if (group.nextSearchActionMs != 0 && now < group.nextSearchActionMs) {
    return;
  }

  sendOneSearchSide(group);
}

// ======================= 命令解析 =======================

MotionCommand parseCommand(String text) {
  MotionCommand command;

  text.trim();
  text.toLowerCase();
  text.replace('\t', ' ');
  text.replace(String("，"), String(","));

  // 停止命令
  if (text == "stop") {
    command.valid = true;
    command.stopAll = true;
    return command;
  }

  // 激光丢失命令
  if (text == "missing") {
    command.valid = true;
    command.missing = true;
    return command;
  }

  // 双轴方向命令
  if (text == "left,up") {
    command.pan = PanMotion::Left;
    command.tilt = TiltMotion::Up;
    command.valid = true;
    return command;
  }

  if (text == "left,down") {
    command.pan = PanMotion::Left;
    command.tilt = TiltMotion::Down;
    command.valid = true;
    return command;
  }

  if (text == "right,up") {
    command.pan = PanMotion::Right;
    command.tilt = TiltMotion::Up;
    command.valid = true;
    return command;
  }

  if (text == "right,down") {
    command.pan = PanMotion::Right;
    command.tilt = TiltMotion::Down;
    command.valid = true;
    return command;
  }

  // 单轴方向命令
  if (text == "left only") {
    command.pan = PanMotion::Left;
    command.tilt = TiltMotion::Stop;
    command.valid = true;
    return command;
  }

  if (text == "right only") {
    command.pan = PanMotion::Right;
    command.tilt = TiltMotion::Stop;
    command.valid = true;
    return command;
  }

  if (text == "up only") {
    command.pan = PanMotion::Stop;
    command.tilt = TiltMotion::Up;
    command.valid = true;
    return command;
  }

  if (text == "down only") {
    command.pan = PanMotion::Stop;
    command.tilt = TiltMotion::Down;
    command.valid = true;
    return command;
  }

  return command;
}

// ======================= 正常方向运动控制 =======================

void movePanOneStep(MotorGroup &group, PanMotion motion) {
  if (motion == PanMotion::Left) {
    moveMotorByPulse(group.panMotorAddr, group.panLeftDir, group.movePulse, kMoveSpeedRpm, kMoveAcc);
  } else if (motion == PanMotion::Right) {
    moveMotorByPulse(group.panMotorAddr, group.panRightDir, group.movePulse, kMoveSpeedRpm, kMoveAcc);
  }
}

void moveTiltOneStep(MotorGroup &group, TiltMotion motion) {
  if (motion == TiltMotion::Up) {
    moveMotorByPulse(group.tiltMotorAddr, group.tiltUpDir, group.movePulse, kMoveSpeedRpm, kMoveAcc);
  } else if (motion == TiltMotion::Down) {
    moveMotorByPulse(group.tiltMotorAddr, group.tiltDownDir, group.movePulse, kMoveSpeedRpm, kMoveAcc);
  }
}

void applyMotion(MotorGroup &group, const MotionCommand &command) {
  // 收到正常方向指令时，如果正在 missing 搜索，先退出搜索模式。
  stopSearchMode(group);

  movePanOneStep(group, command.pan);
  moveTiltOneStep(group, command.tilt);

  group.currentPan = command.pan;
  group.currentTilt = command.tilt;
}

bool isExternalHostInput(const CommandInput &input) {
  return input.stream != nullptr && input.stream != &Serial;
}

bool isPrimaryGroup(const MotorGroup &group) {
  return &group == &gPrimaryGroup;
}

bool isSecondGroup(const MotorGroup &group) {
  return &group == &gSecondGroup;
}

bool isUsbInput(const CommandInput &input) {
  return input.stream == &Serial;
}

void pauseExternalHostInputIfManualG2Stop(CommandInput &input, MotorGroup &group) {
  if (!isUsbInput(input) || !isSecondGroup(group)) {
    return;
  }

  gExternalHostInputPaused = true;
  gSecondHostInputArmed = false;
  gSecondHostArmAtMs = 0;
  gPrimaryHostStopCount = 0;
  gHostInput.line = "";
  gSecondHostInput.line = "";

  writeReply(input, "OK external host input paused");
}

void resetPrimaryHostStopCountIfNeeded(CommandInput &input, MotorGroup &group) {
  if (isExternalHostInput(input) && isPrimaryGroup(group)) {
    gPrimaryHostStopCount = 0;
  }
}

void compensateAfterPrimaryHostStops(CommandInput &input, MotorGroup &group) {
  if (!isExternalHostInput(input) || !isPrimaryGroup(group)) {
    return;
  }

  gPrimaryHostStopCount++;

  if (gPrimaryHostStopCount < 3) {
    return;
  }

  gPrimaryHostStopCount = 0;
  gSecondHostInputArmed = false;
  gSecondHostArmAtMs = millis() + kSecondHostArmDelayMs;

  writeReply(input, "OK HOST2 input will arm in " +
                    String(kSecondHostArmDelayMs) + " ms");
}

// ======================= 命令处理 =======================

bool stripCommandPrefix(String &commandText, const String &lowerText, const char *prefix) {
  if (!lowerText.startsWith(prefix)) {
    return false;
  }

  commandText = commandText.substring(strlen(prefix));
  commandText.trim();
  return true;
}

MotorGroup &selectCommandGroup(String &commandText, MotorGroup &defaultGroup) {
  commandText.trim();

  String lowerText = commandText;
  lowerText.toLowerCase();

  if (stripCommandPrefix(commandText, lowerText, "g1 ") ||
      stripCommandPrefix(commandText, lowerText, "g1:") ||
      stripCommandPrefix(commandText, lowerText, "group1 ") ||
      stripCommandPrefix(commandText, lowerText, "group1:") ||
      stripCommandPrefix(commandText, lowerText, "m1m2 ") ||
      stripCommandPrefix(commandText, lowerText, "m1m2:") ||
      stripCommandPrefix(commandText, lowerText, "1 ")) {
    return gPrimaryGroup;
  }

  if (stripCommandPrefix(commandText, lowerText, "g2 ") ||
      stripCommandPrefix(commandText, lowerText, "g2:") ||
      stripCommandPrefix(commandText, lowerText, "group2 ") ||
      stripCommandPrefix(commandText, lowerText, "group2:") ||
      stripCommandPrefix(commandText, lowerText, "m3m4 ") ||
      stripCommandPrefix(commandText, lowerText, "m3m4:") ||
      stripCommandPrefix(commandText, lowerText, "2 ")) {
    return gSecondGroup;
  }

  return defaultGroup;
}

void handleCommand(CommandInput &input, MotorGroup &defaultGroup, String text) {
  text.trim();

  if (text.isEmpty()) {
    return;
  }

  String displayText = text;
  String commandText = text;
  MotorGroup &group = selectCommandGroup(commandText, defaultGroup);

  MotionCommand command = parseCommand(commandText);

  if (!command.valid) {
    if (input.showUnknownAsRx) {
      writeReply(input, "RX " + displayText);
      return;
    }

    if (input.reportUnknown) {
      if (input.echo) {
        writeReply(input, "> " + displayText);
      }
      writeReply(input, "ERR unknown command: " + displayText);
      writeReply(input, "Only support: left,up / left,down / right,up / right,down / left only / right only / up only / down only / stop / missing");
    }
    return;
  }

  if (input.echo) {
    writeReply(input, "> " + displayText);
  }

  if (gForcePauseLatched && !command.stopAll) {
    writeReply(input, "ERR force pause active");
    return;
  }

  if (command.stopAll) {
    group.searchMode = false;
    stopMotorGroup(group);
    writeReply(input, "OK " + String(group.name) + " stop");
    pauseExternalHostInputIfManualG2Stop(input, group);
    compensateAfterPrimaryHostStops(input, group);
    return;
  }

  resetPrimaryHostStopCountIfNeeded(input, group);

  if (command.missing) {
    if (isSecondGroup(group)) {
      writeReply(input, "OK " + String(group.name) + " missing ignored");
      return;
    }

    if (!group.searchMode) {
      startSearchMode(group);
      writeReply(input, "OK " + String(group.name) + " missing -> start square search");
    } else {
      writeReply(input, "OK " + String(group.name) + " missing -> keep searching, pulse=" + String(group.searchPulse));
    }
    return;
  }

  applyMotion(group, command);

  writeReply(input, "OK " + String(group.name) +
                    " move pan=" + String(panName(group.currentPan)) +
                    " tilt=" + String(tiltName(group.currentTilt)) +
                    " pulse=" + String(group.movePulse));
}

void processInputLine(CommandInput &input, MotorGroup &group) {
  handleCommand(input, group, input.line);
  input.line = "";
}

void discardPendingInput(CommandInput &input) {
  if (input.stream == nullptr) {
    return;
  }

  while (input.stream->available() > 0) {
    input.stream->read();
  }

  input.line = "";
}

void activateForcePause(const char *source) {
  if (gForcePauseLatched) {
    return;
  }

  gForcePauseLatched = true;
  gExternalHostInputPaused = true;
  gSecondHostInputArmed = false;
  gSecondHostArmAtMs = 0;
  gPrimaryHostStopCount = 0;
  gPrimaryGroup.searchMode = false;
  gSecondGroup.searchMode = false;
  gHostInput.line = "";
  gSecondHostInput.line = "";

  stopAllMotorGroups();

  Serial.print("OK force pause by ");
  Serial.println(source);
}

void triggerProgramRestart(const char *source) {
  Serial.print("OK program restart by ");
  Serial.println(source);
  Serial.flush();
  delay(50);
  ESP.restart();
}

void pollForcePauseButton() {
  if (kForcePauseButtonPin < 0) {
    return;
  }

  const bool reading = digitalRead(kForcePauseButtonPin) == LOW;
  const unsigned long now = millis();

  if (reading != gForcePauseLastReading) {
    gForcePauseLastReading = reading;
    gForcePauseLastChangeMs = now;
  }

  if (now - gForcePauseLastChangeMs < kForcePauseDebounceMs) {
    return;
  }

  if (reading == gForcePauseStablePressed) {
    return;
  }

  gForcePauseStablePressed = reading;

  if (gForcePauseStablePressed) {
    activateForcePause("GPIO6 button");
  }
}

void pollRestartButton() {
  if (kRestartButtonPin < 0) {
    return;
  }

  const bool reading = digitalRead(kRestartButtonPin) == LOW;
  const unsigned long now = millis();

  if (reading != gRestartLastReading) {
    gRestartLastReading = reading;
    gRestartLastChangeMs = now;
  }

  if (now - gRestartLastChangeMs < kRestartDebounceMs) {
    return;
  }

  if (reading == gRestartStablePressed) {
    return;
  }

  gRestartStablePressed = reading;

  if (gRestartStablePressed) {
    triggerProgramRestart("GPIO7 button");
  }
}

void updateSecondHostInputArmState() {
  if (gSecondHostInputArmed || gSecondHostArmAtMs == 0) {
    return;
  }

  if (static_cast<long>(millis() - gSecondHostArmAtMs) < 0) {
    return;
  }

  gSecondHostInputArmed = true;
  gSecondHostArmAtMs = 0;
  Serial.println("[HOST2] input armed");
}

void readCommands(CommandInput &input, MotorGroup &group) {
  if (input.stream == nullptr) {
    return;
  }

  while (input.stream->available() > 0) {
    const int value = input.stream->read();

    if (value < 0) {
      break;
    }

    const char ch = static_cast<char>(value);
    input.lastByteMs = millis();

    if (ch == '\r' || ch == '\n') {
      processInputLine(input, group);
      continue;
    }

    if (input.line.length() >= kMaxCommandLength) {
      input.line = "";
      if (input.reportUnknown) {
        writeReply(input, "ERR command too long");
      }
      continue;
    }

    input.line += ch;
  }

  // 兼容上位机不发换行符的情况。
  if (!input.line.isEmpty() && millis() - input.lastByteMs >= kCommandFlushMs) {
    processInputLine(input, group);
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  pinMode(kForcePauseButtonPin, INPUT_PULLUP);
  pinMode(kRestartButtonPin, INPUT_PULLUP);

  if (kHostUartRxPin >= 0 && kHostUartTxPin >= 0) {
    Serial2.begin(kHostUartBaud, SERIAL_8N1, kHostUartRxPin, kHostUartTxPin);
    gHostInput = CommandInput(&Serial2, "HOST1", true, false);
    gHostEnabled = true;
  }

  if (kSecondHostUartRxPin >= 0 && kSecondHostUartTxPin >= 0) {
    Serial0.begin(kSecondHostUartBaud, SERIAL_8N1, kSecondHostUartRxPin, kSecondHostUartTxPin);
    gSecondHostInput = CommandInput(&Serial0, "HOST2", true, false, true);
    gSecondHostEnabled = true;
  }

  if (kMotorUartRxPin >= 0 && kMotorUartTxPin >= 0) {
    Serial1.begin(kMotorUartBaud, SERIAL_8N1, kMotorUartRxPin, kMotorUartTxPin);
  } else {
    Serial1.begin(kMotorUartBaud);
  }

  delay(200);

  enableAllMotorGroups();
  stopAllMotorGroups();

  Serial.println("二维云台上位机/USB串口控制已就绪");
  Serial.println("第一组：1号电机 = 左右轴，2号电机 = 上下轴");
  Serial.println("第二组：3号电机 = 左右轴，4号电机 = 上下轴");
  Serial.println("方向定义：顺时针 = 左/上，逆时针 = 右/下");

  Serial.print("第一上位机外部串口引脚：RX=");
  Serial.print(kHostUartRxPin);
  Serial.print(" TX=");
  Serial.println(kHostUartTxPin);

  Serial.print("第二上位机/相机串口引脚：RX=");
  Serial.print(kSecondHostUartRxPin);
  Serial.print(" TX=");
  Serial.println(kSecondHostUartTxPin);

  Serial.print("正常单步运动速度：");
  Serial.print(kMoveSpeedRpm);
  Serial.println(" RPM");

  Serial.print("正常单步运动脉冲：");
  Serial.print(kMovePulse);
  Serial.println(" pulse");

  Serial.print("missing搜索初始边长：");
  Serial.print(kSearchStartPulse);
  Serial.println(" pulse");

  Serial.print("missing搜索边长增量：");
  Serial.print(kSearchPulseStep);
  Serial.println(" pulse/cycle");

  printCommandList(gUsbInput);
}

void loop() {
  pollForcePauseButton();
  pollRestartButton();

  // USB 串口调试
  readCommands(gUsbInput, gPrimaryGroup);

  // 第一外部上位机串口，只控制 1/2 号电机。
  if (gHostEnabled) {
    if (gExternalHostInputPaused) {
      discardPendingInput(gHostInput);
    } else {
      readCommands(gHostInput, gPrimaryGroup);
    }
  }

  // 第二外部上位机/相机串口，只控制 3/4 号电机。
  if (gSecondHostEnabled) {
    if (gExternalHostInputPaused) {
      discardPendingInput(gSecondHostInput);
    } else if (gSecondHostInputArmed) {
      readCommands(gSecondHostInput, gSecondGroup);
    } else {
      discardPendingInput(gSecondHostInput);
      updateSecondHostInputArmState();
    }
  }

  // missing 搜索模式。非阻塞运行，所以搜索过程中仍然能收到方向指令。
  runSearchMode(gPrimaryGroup);
}
