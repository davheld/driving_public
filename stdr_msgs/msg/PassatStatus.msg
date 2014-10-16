Header header
uint8 steering_state
uint8 dsr
uint8 brake_state
uint8 brake_221_BCU3status
uint8 brake_221_Detected_failure
string brake_flags

# meaning of brake_state and steering_state:
#   0: Off
#   1: Init
#   2: Transitioning
#   3: On
#   4: Err
