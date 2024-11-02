
//--------------------HID report gamepad--------------------
// Gamepad Report Descriptor Template
// with 32 buttons, 2 joysticks and 1 hat/dpad with following layout
// | X | Y | Z | Rz | Rx | Ry (1 byte each) | hat/DPAD (1 byte) | Button Map (4 bytes) |
#define TUD_HID_REPORT_DESC_GAMEPAD(...) \
HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
/* Report ID if any */\
__VA_ARGS__ \
/* 8 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
HID_LOGICAL_MIN    ( 0x81                                   ) ,\
HID_LOGICAL_MAX    ( 0x7f                                   ) ,\
HID_REPORT_COUNT   ( 6                                      ) ,\
HID_REPORT_SIZE    ( 8                                      ) ,\
HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
/* 8 bit DPad/Hat Button Map  */ \
HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
HID_LOGICAL_MIN    ( 1                                      ) ,\
HID_LOGICAL_MAX    ( 8                                      ) ,\
HID_PHYSICAL_MIN   ( 0                                      ) ,\
HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
HID_REPORT_COUNT   ( 1                                      ) ,\
HID_REPORT_SIZE    ( 8                                      ) ,\
HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
/* 32 bit Button Map */ \
HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
HID_USAGE_MIN      ( 1                                      ) ,\
HID_USAGE_MAX      ( 32                                     ) ,\
HID_LOGICAL_MIN    ( 0                                      ) ,\
HID_LOGICAL_MAX    ( 1                                      ) ,\
HID_REPORT_COUNT   ( 32                                     ) ,\
HID_REPORT_SIZE    ( 1                                      ) ,\
HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
HID_COLLECTION_END \
//--------------------HID report gamepad--------------------



