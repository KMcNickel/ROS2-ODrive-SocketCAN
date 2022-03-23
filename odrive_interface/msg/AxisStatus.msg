float32 current_position
float32 current_velocity
int8 axis_status

int8 board_error
int32 axis_error
int64 motor_error
int8 controller_error
int16 encoder_error

bool watchdog_from_odrive_timeout_error
bool incoming_velocity_timeout_error
bool incoming_velocity_timeout_error_latch

int8 AXIS_STATUS_ERROR = -1
int8 AXIS_STATUS_NOT_READY = 0
int8 AXIS_STATUS_CALIBRATING = 1
int8 AXIS_STATUS_IDLE = 2
int8 AXIS_STATUS_RUNNING = 3