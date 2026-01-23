#pragma once
#include "SerialConsole.h"
#include "Constants.h"

SerialConsole::SerialConsole(Manager& manager)
: _manager(manager) {
    _buffer.reserve(128);
}

// SerialConsole::SerialConsole() {
//     _buffer.reserve(128);
// }

void SerialConsole::init(uint32_t baud) {
    Serial.begin(baud);
    _printHelp();
}

void SerialConsole::execute() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if(c == '\r') continue;

        if(c == '\n') {
            String line = _buffer;
            _buffer = "";
            line.trim();
            if (line.length() > 0) {
                _last_user_cmd_ms = millis();
                _handleLine(line);
            }
        }
        else {
            _buffer += c;
            if(_buffer.length() > 200) _buffer = "";
        }
    }
}

void SerialConsole::_handleLine(const String& line) {
    constexpr int MAX_ARGS = 5;
    String tokens[MAX_ARGS];

    int n = _tokenize(line, tokens, MAX_ARGS);
    if(n <= 0) return;

    Serial.print("> ");
    Serial.println(line);
    const String cmd = tokens[0];
    _dispatch(cmd, &tokens[1], n-1);
}

void SerialConsole::_dispatch(const String& cmd, const String args[], int argc) {
    if(cmd == "help") {_cmdHelp(args, argc); return;}
    if(cmd == "status") {_cmdStatus(args, argc); return;}
    if(cmd == "mode") {_cmdMode(args, argc); return;}
    if(cmd == "arm") {_cmdArm(args, argc); return;}
    if(cmd == "c0") {_cmdC0(args, argc); return;}
    if(cmd == "c1") {_cmdC1(args, argc); return;}
    if(cmd == "c") {_cmdC(args, argc); return;}
    if(cmd == "fault") {_cmdFault(args, argc); return;}
    if(cmd == "restart") {_cmdRestart(args, argc); return;}

    Serial.println("Unknown Command. Type 'help'.");
}

int SerialConsole::_tokenize(const String& line, String outArgs[], int maxArgs){
    int count = 0;
    int start = 0;

    while (start < (int)line.length() && count < maxArgs) {
        while (start < (int)line.length() && line[start] == ' ') start++;
        if (start >= (int)line.length()) break;

        int end = start;
        while (end < (int)line.length() && line[end] != ' ') end++;

        outArgs[count++] = line.substring(start, end);
        start = end + 1;
    }

    outArgs[0].toLowerCase();
    return count;
}

void SerialConsole::_cmdHelp(const String[], int) {
    _printHelp();
}

void SerialConsole::_cmdStatus(const String[], int) {
    _printStatus();
}

void SerialConsole::_cmdMode(const String args[], int argc) {
    if(argc < 1) {
        Serial.println("Usage: mode idle|independent|calibration|fault");
        return;
    }

    String m = args[0];
    m.toLowerCase();

    if (m == "idle") {
        Serial.println("OK (TODO: mgr_.setMode(Mode::Idle))");
    } else if (m == "independent") {
        Serial.println("OK (TODO: mgr_.setMode(Mode::Independent))");
    } else if (m == "calibration") {
        Serial.println("OK (TODO: mgr_.calibrate()))");
    } else if (m == "fault") {
        Serial.println("OK (TODO: mgr_.triggerFault(...))");
    } else {
        Serial.println("Unknown mode. Use: idle|independent|calibration|fault");
    }
}

void SerialConsole::_cmdArm(const String args[], int argc) {
    if (argc < 1) {
        Serial.println("Usage: arm 0|1");
        return;
    }

    int v = args[0].toInt();
    bool on = (v != 0);

    Serial.print("OK (TODO: _manager.arm(");
    Serial.print(on ? "true" : "false");
    Serial.println("))");
}

void SerialConsole::_cmdC0(const String args[], int argc) {
    if (argc < 1) {
        Serial.println("Usage: c0 <rad>");
        return;
    }

    float v = args[0].toFloat();
    Serial.print("OK (TODO: _manager.setCommand0(");
    Serial.print(v);
    Serial.println("))");
}

void SerialConsole::_cmdC1(const String args[], int argc) {
    if (argc < 1) {
        Serial.println("Usage: c1 <rad>");
        return;
    }

    float v = args[0].toFloat();
    Serial.print("OK (TODO: _manager.setCommand1(");
    Serial.print(v);
    Serial.println("))");
}

void SerialConsole::_cmdC(const String args[], int argc) {
    if (argc < 2) {
        Serial.println("Usage: c <rad0> <rad1>");
        return;
    }

    float v0 = args[0].toFloat();
    float v1 = args[1].toFloat();

    Serial.print("OK (TODO: _manager.setCommand0(");
    Serial.print(v0);
    Serial.print("), _manager.setCommand1(");
    Serial.print(v1);
    Serial.println("))");
}

void SerialConsole::_cmdFault(const String args[], int argc) {
    if (argc < 1) {
        Serial.println("Usage: fault clear");
        return;
    }

    String sub = args[0];
    sub.toLowerCase();

    if (sub == "clear") {
        Serial.println("OK (TODO: _manager.clearFault())");
    } else {
        Serial.println("Unknown fault command. Use: fault clear");
    }
}

void SerialConsole::_cmdRestart(const String args[], int argc) {
    if(argc > 0) {
        int delay_ms = args[0].toInt();
        Serial.print("Restarting in ");
        Serial.print(delay_ms);
        Serial.println(" ms");
        delay(delay_ms);
    }
    Serial.println("=-=-=-= Restarting =-=-=-=");
    ESP.restart();
}

void SerialConsole::_printHelp() {
    Serial.println();
    Serial.println("Commands:");
    Serial.println("  help");
    Serial.println("  status");
    Serial.println("  mode idle|independent|calibration|fault");
    Serial.println("  arm 0|1");
    Serial.println("  c0 <rad>");
    Serial.println("  c1 <rad>");
    Serial.println("  c <rad0> <rad1>");
    Serial.println("  fault clear");
    Serial.println("  restart <delay_ms>");
    Serial.println();
}

const char* SerialConsole::_modeToStr(Mode m) {
    switch (m) {
        case Mode::Idle: return "Idle";
        case Mode::Independent: return "Independent";
        case Mode::Calibration: return "Calibration";
        case Mode::Fault: return "Fault";
    }
    return "?";
}

void SerialConsole::_printStatus() {
    Serial.println("Status (TODO: print _manager.status() fields)");
}