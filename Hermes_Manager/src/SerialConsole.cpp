// SerialConsole.cpp
// Minimal serial console for a single Encoder + optional MotorController.

#include "SerialConsole.hpp"
#include "Encoder.hpp"
#include "MotorController.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#if defined(ARDUINO)
  #include <Arduino.h>
  #ifdef DISABLED
    #undef DISABLED
  #endif
#endif

SerialConsole::SerialConsole() = default;

void SerialConsole::begin(Encoder* enc) {
  begin(enc, nullptr, Config{});
}

void SerialConsole::begin(Encoder* enc, const Config& cfg) {
  begin(enc, nullptr, cfg);
}

void SerialConsole::begin(Encoder* enc, MotorController* motor) {
  begin(enc, motor, Config{});
}

void SerialConsole::begin(Encoder* enc, MotorController* motor, const Config& cfg) {
  enc_ = enc;
  motor_ = motor;
  cfg_ = cfg;

  line_len_ = 0;
  watch_on_ = false;
  watch_period_ms_ = cfg_.watch_period_ms;
  next_watch_ms_ = 0;
  next_poll_ms_ = 0;
  printing_watch_ = false;

#if defined(ARDUINO)
  Serial.begin(cfg_.baud);
  while (!Serial) { /* wait */ }
#endif

  write_("=-=-=-=-=-=-=-=-=-=-   Hermes   -=-=-=-=-=-=-=-=-=-=\n");
  cmd_help_();
  write_("\n> ");
}

void SerialConsole::poll(uint32_t now_ms) {
  if (!enc_) return;

  // periodic watch: overwrite same line using '\r'
  if (watch_on_ && now_ms >= next_watch_ms_) {
    next_watch_ms_ = now_ms + watch_period_ms_;
    printing_watch_ = true;
    cmd_status_(now_ms);
    printing_watch_ = false;
  }

  if (now_ms < next_poll_ms_) return;
  next_poll_ms_ = now_ms + cfg_.poll_ms;

  while (true) {
    const int ch = read_char_();
    if (ch < 0) break;

    if (ch == '\r') continue;

    if (ch == '\n') {
      line_[line_len_] = '\0';
      if (line_len_ > 0) handle_line_(now_ms, line_);
      line_len_ = 0;
      continue;
    }

    if (line_len_ + 1 < kLineMax) {
      line_[line_len_++] = static_cast<char>(ch);
    } else {
      line_len_ = 0;
    }
  }
}

int SerialConsole::read_char_() {
#if defined(ARDUINO)
  if (Serial.available() <= 0) return -1;
  return Serial.read();
#else
  return -1;
#endif
}

void SerialConsole::write_(const char* s) {
#if defined(ARDUINO)
  Serial.print(s);
#else
  (void)s;
#endif
}

void SerialConsole::writef_(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  write_(buf);
}

int SerialConsole::tokenize_(char* buf, char* argv[], int max_argv) {
  int argc = 0;
  char* p = buf;
  while (*p && argc < max_argv) {
    while (*p == ' ' || *p == '\t') ++p;
    if (!*p) break;
    argv[argc++] = p;
    while (*p && *p != ' ' && *p != '\t') ++p;
    if (*p) *p++ = '\0';
  }
  return argc;
}

void SerialConsole::handle_line_(uint32_t now_ms, const char* line) {
  // If watch is running, its output uses '\r' and can collide with user input.
  // Print a newline before echoing the command for readability.
  if (watch_on_) write_("\n");

  // Echo the command back to the user.
  writef_("> %s\n", line);

  char buf[kLineMax];
  std::snprintf(buf, sizeof(buf), "%s", line);

  char* argv[12];
  const int argc = tokenize_(buf, argv, 12);
  if (argc == 0) {
    write_("> ");
    return;
  }

  const char* c = argv[0];

  if (!std::strcmp(c, "help")) cmd_help_();
  else if (!std::strcmp(c, "status")) cmd_status_(now_ms);
  else if (!std::strcmp(c, "read")) cmd_read_(now_ms);
  else if (!std::strcmp(c, "watch")) cmd_watch_(now_ms, argc, argv);
  else if (!std::strcmp(c, "invert")) cmd_invert_(argc, argv);
  else if (!std::strcmp(c, "zero")) cmd_zero_();
  else if (!std::strcmp(c, "set")) cmd_set_(argc, argv);
  else if (!std::strcmp(c, "offset")) cmd_offset_(argc, argv);
  else if (!std::strcmp(c, "probe")) cmd_probe_();
  else if (!std::strcmp(c, "m")) cmd_motor_(now_ms, argc, argv);
  else write_("ERR: unknown command. type 'help'\n");

  write_("> ");
}

// ---------------- Commands ----------------

void SerialConsole::cmd_help_() {
  write_(
    "Commands:\n"
    "  help\n"
    "  probe                 (I2C ACK test)\n"
    "  status                (read+print)\n"
    "  read                  (same as status)\n"
    "  watch on|off\n"
    "  watch rate <ms>       (period in ms; watch updates in-place with \\r)\n"
    "  invert on|off\n"
    "  zero                  (set current angle to 0)\n"
    "  set <deg>             (set current angle to <deg>)\n"
    "  offset <deg>          (directly set offset)\n"
    "\n"
    "Motor (if present):\n"
    "  m help\n"
    "  m status\n"
    "  m enable on|off\n"
    "  m out <u1> <u2> <u3>  (each in [-1,1])\n"
    "  m stop\n"
  );
}

void SerialConsole::cmd_probe_() {
  const bool ok = enc_->probe();
  write_(ok ? "probe: ok\n" : "probe: FAIL\n");
}

void SerialConsole::cmd_status_(uint32_t now_ms) {
  enc_->read(now_ms);
  const auto& r = enc_->last();

  if (printing_watch_) {
    // One-line, in-place update for watch mode
    if (!r.ok) {
      writef_("raw12=---- raw_deg=---.--- deg=---.--- vel=---.--- inv=----- off=---.--- ERR(i2c=%u)\r",
              (unsigned)r.i2c_error);
      return;
    }

    writef_(
      "raw12=%4u raw_deg=%7.3f deg=%7.3f vel=%8.3f inv=%5s off=%7.3f\r",
      (unsigned)r.raw12,
      r.deg_raw,
      r.deg,
      r.vel_dps,
      enc_->invert() ? "true" : "false",
      enc_->offset_deg()
    );
    return;
  }

  // Multi-line for human readability
  if (!r.ok) {
    writef_("Encoder:\n  status: ERROR\n  i2c_error: %u\n", (unsigned)r.i2c_error);
    return;
  }

  write_("Encoder:\n");
  writef_("  status     : OK\n");
  writef_("  raw12      : %u\n", (unsigned)r.raw12);
  writef_("  raw_deg    : %.3f\n", r.deg_raw);
  writef_("  deg        : %.3f\n", r.deg);
  writef_("  vel_dps    : %.3f\n", r.vel_dps);
  writef_("  invert     : %s\n", enc_->invert() ? "true" : "false");
  writef_("  offset_deg : %.3f\n", enc_->offset_deg());
}

void SerialConsole::cmd_read_(uint32_t now_ms) {
  const bool prev = printing_watch_;
  printing_watch_ = false;
  cmd_status_(now_ms);
  printing_watch_ = prev;
}

void SerialConsole::cmd_watch_(uint32_t now_ms, int argc, char* argv[]) {
  (void)now_ms;
  if (argc < 2) {
    writef_("watch: %s period=%ums\n", watch_on_ ? "on" : "off", (unsigned)watch_period_ms_);
    return;
  }

  if (!std::strcmp(argv[1], "on")) {
    watch_on_ = true;
    next_watch_ms_ = 0;
    write_("watch: on\n");
    return;
  }
  if (!std::strcmp(argv[1], "off")) {
    watch_on_ = false;
    write_("\nwatch: off\n");
    return;
  }
  if (!std::strcmp(argv[1], "rate") && argc >= 3) {
    const int ms = std::atoi(argv[2]);
    watch_period_ms_ = (ms <= 1) ? 1u : (uint32_t)ms;
    writef_("watch period: %ums\n", (unsigned)watch_period_ms_);
    return;
  }

  write_("ERR: watch on|off | watch rate <ms>\n");
}

void SerialConsole::cmd_invert_(int argc, char* argv[]) {
  if (argc < 2) {
    writef_("invert: %s\n", enc_->invert() ? "true" : "false");
    return;
  }
  const bool on = (!std::strcmp(argv[1], "on") || !std::strcmp(argv[1], "1") || !std::strcmp(argv[1], "true"));
  const bool off = (!std::strcmp(argv[1], "off") || !std::strcmp(argv[1], "0") || !std::strcmp(argv[1], "false"));
  if (!on && !off) {
    write_("ERR: invert on|off\n");
    return;
  }
  enc_->set_invert(on);
  write_("ok\n");
}

void SerialConsole::cmd_zero_() {
#if defined(ARDUINO)
  enc_->read(millis());
  enc_->zero_here();
#else
  enc_->zero_here();
#endif
  write_("ok\n");
}

void SerialConsole::cmd_set_(int argc, char* argv[]) {
  if (argc < 2) {
    write_("ERR: set <deg>\n");
    return;
  }
  const float deg = std::atof(argv[1]);
#if defined(ARDUINO)
  enc_->read(millis());
  enc_->set_here(deg);
#else
  enc_->set_here(deg);
#endif
  write_("ok\n");
}

void SerialConsole::cmd_offset_(int argc, char* argv[]) {
  if (argc < 2) {
    writef_("offset: %.3f\n", enc_->offset_deg());
    return;
  }
  enc_->set_offset_deg(std::atof(argv[1]));
  write_("ok\n");
}

void SerialConsole::cmd_motor_(uint32_t now_ms, int argc, char* argv[]) {
  (void)now_ms;
  if (!motor_) {
    write_("motor: not configured\n");
    return;
  }

  if (argc < 2) {
    write_("ERR: m help|status|enable|out|stop\n");
    return;
  }

  const char* sub = argv[1];

  if (!std::strcmp(sub, "help")) {
    write_(
      "m commands:\n"
      "  m status\n"
      "  m enable on|off\n"
      "  m out <u1> <u2> <u3>   (each in [-1,1])\n"
      "  m stop\n"
    );
    return;
  }

  if (!std::strcmp(sub, "status")) {
    const auto& s = motor_->state();
    write_("Motor:\n");
    writef_("  configured: %s\n", s.configured ? "true" : "false");
    writef_("  enabled   : %s\n", s.enabled ? "true" : "false");
    writef_("  u1 u2 u3  : %.3f %.3f %.3f\n", s.u1, s.u2, s.u3);
    return;
  }

  if (!std::strcmp(sub, "enable")) {
    if (argc < 3) {
      write_("ERR: m enable on|off\n");
      return;
    }
    const bool on = (!std::strcmp(argv[2], "on") || !std::strcmp(argv[2], "1") || !std::strcmp(argv[2], "true"));
    const bool off = (!std::strcmp(argv[2], "off") || !std::strcmp(argv[2], "0") || !std::strcmp(argv[2], "false"));
    if (!on && !off) {
      write_("ERR: m enable on|off\n");
      return;
    }
    motor_->enable(on);
    write_("ok\n");
    return;
  }

  if (!std::strcmp(sub, "out")) {
    if (argc < 5) {
      write_("ERR: m out <u1> <u2> <u3>\n");
      return;
    }
    const float u1 = std::atof(argv[2]);
    const float u2 = std::atof(argv[3]);
    const float u3 = std::atof(argv[4]);
    motor_->set_outputs(u1, u2, u3);
    write_("ok\n");
    return;
  }

  if (!std::strcmp(sub, "stop")) {
    motor_->stop();
    write_("ok\n");
    return;
  }

  write_("ERR: m help|status|enable|out|stop\n");
}
