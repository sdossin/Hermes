// SerialConsole.cpp
// Minimal serial console for a single Encoder object.

#include "SerialConsole.hpp"
#include "Encoder.hpp"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <cstdlib>

#if defined(ARDUINO)
  #include <Arduino.h>
  // Avoid macro collision with enums named DISABLED, etc.
  #ifdef DISABLED
    #undef DISABLED
  #endif
#endif

SerialConsole::SerialConsole() = default;

void SerialConsole::begin(Encoder* enc) {
  begin(enc, Config{});
}

void SerialConsole::begin(Encoder* enc, const Config& cfg) {
  enc_ = enc;
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

  write_("=-=-=-=-=-=-=-=-=-=-   Hermes   -=-=-=-=-=-=-=-=-=-=\n ");
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
      // overflow -> reset
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
  // If watch is running, its output uses ' and can collide with user input.
  // Print a newline before echoing the command for readability.
  if (watch_on_) write_("");

  // Echo the command back to the user.
  writef_("%s\n\n", line);

  char buf[kLineMax];
  std::snprintf(buf, sizeof(buf), "%s", line);

  char* argv[10];
  const int argc = tokenize_(buf, argv, 10);
  if (argc == 0) return;

  const char* c = argv[0];

  if (!std::strcmp(c, "help")) { cmd_help_(); }
  else if (!std::strcmp(c, "status")) { cmd_status_(now_ms); }
  else if (!std::strcmp(c, "read")) { cmd_read_(now_ms); }
  else if (!std::strcmp(c, "watch")) { cmd_watch_(now_ms, argc, argv); }
  else if (!std::strcmp(c, "invert")) { cmd_invert_(argc, argv); }
  else if (!std::strcmp(c, "zero")) { cmd_zero_(); }
  else if (!std::strcmp(c, "set")) { cmd_set_(argc, argv); }
  else if (!std::strcmp(c, "offset")) { cmd_offset_(argc, argv); }
  else if (!std::strcmp(c, "probe")) { cmd_probe_(); }
  else write_("ERR: unknown command. type 'help'");
  write_("\n > ");
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
    "  watch rate <ms>       (period in ms; updates in-place with \\r)\n"
    "  invert on|off\n"
    "  zero                  (set current angle to 0)\n"
    "  set <deg>             (set current angle to <deg>)\n"
    "  offset <deg>          (directly set offset)\n"
  );
}

void SerialConsole::cmd_probe_() {
  const bool ok = enc_->probe();
  write_(ok ? "probe: ok\n" : "probe: FAIL\n");
}

void SerialConsole::cmd_status_(uint32_t now_ms) {
  // Trigger a fresh read for convenience
  enc_->read(now_ms);
  const auto& r = enc_->last();

  if (!r.ok) {
    writef_("Encoder:\n\tstatus: ERROR\n\ti2c_error: %u", (unsigned)r.i2c_error);
    return;
  }

//   write_("Encoder:\n");
//   writef_("  raw12      : %u\n", (unsigned)r.raw12);
//   writef_("  raw_deg    : %.3f\n", r.deg_raw);
//   writef_("  deg        : %.3f\n", r.deg);
//   writef_("  vel_dps    : %.3f\n", r.vel_dps);
//   writef_("  invert     : %s\n", enc_->invert() ? "on" : "off");
//   writef_("  offset_deg : %.3f\n", enc_->offset_deg());

  writef_("  raw12 = %4u    raw_deg = %07.3f    deg = %07.3f    vel_dps = %08.3f    invert = %s    offset_deg = %07.3f\r", 
        (unsigned)r.raw12,
        r.deg_raw,
        r.deg,
        r.vel_dps,
        enc_->invert() ? "true " : "false",
        enc_->offset_deg()
    );
}

void SerialConsole::cmd_read_(uint32_t now_ms) {
  // Ensure read prints a newline even if watch is currently on.
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
    // When watch ends, print a newline to avoid leaving prompt mid-line.
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
    writef_("invert: %s\n", enc_->invert() ? "on" : "off");
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
    writef_("offset: %.2f\n", enc_->offset_deg());
    return;
  }
  enc_->set_offset_deg(std::atof(argv[1]));
  write_("ok\n");
}
