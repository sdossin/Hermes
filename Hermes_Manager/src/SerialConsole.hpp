// SerialConsole.hpp
#ifndef SERIALCONSOLE_HPP
#define SERIALCONSOLE_HPP

#include <cstdint>

class AS5600Encoder;
class MotorController;

class SerialConsole {
public:
  struct Config {
    uint32_t baud = 115200;
    uint32_t poll_ms = 5;
    uint32_t watch_period_ms = 100;
  };

  SerialConsole();

  void begin(AS5600Encoder* enc);
  void begin(AS5600Encoder* enc, const Config& cfg);
  void begin(AS5600Encoder* enc, MotorController* motor);
  void begin(AS5600Encoder* enc, MotorController* motor, const Config& cfg);

  void poll(uint32_t now_ms);

private:
  static constexpr int kLineMax = 128;

  AS5600Encoder* enc_ = nullptr;
  MotorController* motor_ = nullptr;
  Config cfg_;

  char line_[kLineMax];
  int line_len_ = 0;

  bool watch_on_ = false;
  uint32_t watch_period_ms_ = 100;
  uint32_t next_watch_ms_ = 0;
  uint32_t next_poll_ms_ = 0;
  bool printing_watch_ = false;

  int read_char_();
  void write_(const char* s);
  void writef_(const char* fmt, ...);
  int tokenize_(char* buf, char* argv[], int max_argv);
  void handle_line_(uint32_t now_ms, const char* line);

  // Command handlers
  void cmd_help_();
  void cmd_probe_();
  void cmd_status_(uint32_t now_ms);
  void cmd_read_(uint32_t now_ms);
  void cmd_watch_(uint32_t now_ms, int argc, char* argv[]);
  void cmd_invert_(int argc, char* argv[]);
  void cmd_zero_();
  void cmd_set_(int argc, char* argv[]);
  void cmd_offset_(int argc, char* argv[]);
  void cmd_motor_(uint32_t now_ms, int argc, char* argv[]);  // THIS WAS MISSING!
};

#endif // SERIALCONSOLE_HPP