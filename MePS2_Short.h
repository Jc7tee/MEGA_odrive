#ifndef MePS2_Short_h
#define MePS2_Short_h

#include <MeAuriga.h>
MePS2 MePS2(PORT_5);//PORT_5 = Serial2

#define BTN_UP MePS2.ButtonPressed(MeJOYSTICK_UP)
#define BTN_DOWN MePS2.ButtonPressed(MeJOYSTICK_DOWN)
#define BTN_LEFT MePS2.ButtonPressed(MeJOYSTICK_LEFT)
#define BTN_RIGHT MePS2.ButtonPressed(MeJOYSTICK_RIGHT)
#define BTN_TRIANGLE MePS2.ButtonPressed(MeJOYSTICK_TRIANGLE)
#define BTN_SQUARE MePS2.ButtonPressed(MeJOYSTICK_SQUARE)
#define BTN_ROUND MePS2.ButtonPressed(MeJOYSTICK_ROUND)
#define BTN_X MePS2.ButtonPressed(MeJOYSTICK_XSHAPED)
#define BTN_R1 MePS2.ButtonPressed(MeJOYSTICK_R1)
#define BTN_L1 MePS2.ButtonPressed(MeJOYSTICK_L1)
#define BTN_L2 MePS2.ButtonPressed(MeJOYSTICK_L2)
#define BTN_R2 MePS2.ButtonPressed(MeJOYSTICK_R2)
#define BTN_L3 MePS2.ButtonPressed(MeJOYSTICK_BUTTON_L)
#define BTN_SELECT MePS2.ButtonPressed(MeJOYSTICK_SELECT)
#define BTN_START MePS2.ButtonPressed(MeJOYSTICK_START)
#define BTN_R3 MePS2.ButtonPressed(MeJOYSTICK_BUTTON_R)
#define JOY_RX MePS2.ButtonPressed(MeJOYSTICK_RX)
#define JOY_RY MePS2.ButtonPressed(MeJOYSTICK_RY)
#define JOY_LX MePS2.ButtonPressed(MeJOYSTICK_LX)
#define JOY_LY MePS2.ButtonPressed(MeJOYSTICK_LY)

#endif
