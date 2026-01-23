#pragma once
#include <Arduino.h>
#include "Types.h"
#include "Manager.h"

class SerialConsole {
public:
    explicit SerialConsole(Manager& manager);
    // explicit SerialConsole();

    void init(uint32_t baud);
    void execute();

private:
    void _handleLine(const String& line);
    void _dispatch(const String& cmd, const String args[], int argc);

    void _cmdHelp(const String args[], int argc);
    void _cmdStatus(const String args[], int argc);
    void _cmdMode(const String args[], int argc);
    void _cmdArm(const String args[], int argc);
    void _cmdC0(const String args[], int argc);
    void _cmdC1(const String args[], int argc);
    void _cmdC(const String args[], int argc);
    void _cmdFault(const String args[], int argc);
    void _cmdRestart(const String args[], int argc);

    int _tokenize(const String& line, String outArgs[], int maxArgs);
    void _printHelp();
    void _printStatus();
    static const char* _modeToStr(Mode m);

private:
    Manager& _manager;
    String _buffer;

    uint32_t _last_user_cmd_ms = 0;
};