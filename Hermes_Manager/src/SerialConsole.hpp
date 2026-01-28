// SerialConsole.hpp
// Minimal serial console for a single Encoder + optional MotorController.
// Commands focus on reading, calibration, and basic motor bring-up.
#pragma once

#include <cstdint>

class Encoder;
class MotorController;

class SerialConsole {
public:
  struct Config {
    uint32_t baud = 115200;
    uint16_t poll_ms = 1;            // how often to poll Serial for input
    uint16_t watch_period_ms = 200;  // default watch period
  };

  SerialConsole();

  // NOTE: No default argument here (toolchain quirk with nested default member initializers).
  void begin(Encoder* enc, const Config& cfg);
  void begin(Encoder* enc); // uses default Config{}

  // Motor-enabled overloads.
  void begin(Encoder* enc, MotorController* motor, const Config& cfg);
  void begin(Encoder* enc, MotorController* motor); // uses default Config{}

  // Call frequently from loop(). now_ms should be millis().
  void poll(uint32_t now_ms);

private:
  static constexpr int kLineMax = 96;

  // IO
  int read_char_();
  void write_(const char* s);
  void writef_(const char* fmt, ...);

  // Parsing/dispatch
  void handle_line_(uint32_t now_ms, const char* line);
  static int tokenize_(char* buf, char* argv[], int max_argv);

  // Encoder commands
  void cmd_help_();
  void cmd_status_(uint32_t now_ms);
  void cmd_read_(uint32_t now_ms);
  void cmd_watch_(uint32_t now_ms, int argc, char* argv[]);
  void cmd_invert_(int argc, char* argv[]);
  void cmd_zero_();
  void cmd_set_(int argc, char* argv[]);
  void cmd_offset_(int argc, char* argv[]);
  void cmd_probe_();

  // Motor commands (only valid if motor_ != nullptr)
  void cmd_m_(uint32_t now_ms, int argc, char* argv[]);
  void cmd_m_help_();
  void cmd_m_status_();
  void cmd_m_enable_(int argc, char* argv[]);
  void cmd_m_out_(int argc, char* argv[]);
  void cmd_m_stop_();

  Encoder* enc_ = nullptr;
  MotorController* motor_ = nullptr;
  Config cfg_{};

  // input line buffer
  char line_[kLineMax]{};
  int line_len_ = 0;

  // watch
  bool watch_on_ = false;
  uint32_t watch_period_ms_ = 200;
  uint32_t next_watch_ms_ = 0;
  bool printing_watch_ = false; // when true, status prints with '\r'

  uint32_t next_poll_ms_ = 0;
};
