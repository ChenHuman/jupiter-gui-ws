#ifndef CGLOBAL_H
#define CGLOBAL_H

extern enum KeyValue {
    NO_OPERATION,
    STOP,
    UP,
    DOWN,
    TURN_LEFT,
    TURN_RIGHT,
    LEFT_FORWARD,
    RIGHT_FORWARD,
    LEFT_BACK,
    RIGHT_BACK,
    LEFT,
    RIGHT,
    INCREASE_MAX_SPEEDS,
    DECREASE_MAX_SPEEDS,
    INCREASE_LINEAR_SPEED,
    DECREASE_LINEAR_SPEED,
    INCREASE_ANGULAR_SPEED,
    DECREASE_ANGULAR_SPEED,
    START_ROSCORE,
    CLOSE_ROSCORE,
    OPEN_SERIAL_PORT,
    CLOSE_SERIAL_PORT,
    RESET,
    BRAKE
} key_value;

extern enum TeleopMode {
    GUI_TELEOP,
    KEYBOARD_TELEOP,
    NO_TELEOP
} teleop_mode;

#endif // CGLOBAL_H
