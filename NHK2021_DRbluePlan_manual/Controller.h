#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "define.h"

#define TRANSPARENT_MODE ( 1 )
#define API_MODE ( 2 )

#define XBEE_MODE ( TRANSPARENT_MODE )

#if XBEE_MODE == TRANSPARENT_MODE   

    #define BUTTON_SIKAKU 0x0001
    #define BUTTON_SANKAKU 0x0002
    #define BUTTON_BATU 0x0004
    #define BUTTON_MARU 0x0008

    #define BUTTON_L1 0x0010
    #define BUTTON_R1 0x0020
    #define BUTTON_L2 0x0040
    #define BUTTON_R2 0x0080

    #define BUTTON_PAD 0x0100
    #define BUTTON_PS 0x0200
    // #define BUTTON_SHARE 0x0400
    // #define BUTTON_OPTION 0x0800
    #define BUTTON_L3 0x0400
    #define BUTTON_R3 0x0800

    #define BUTTON_UP 0x1000
    #define BUTTON_RIGHT 0x2000
    #define BUTTON_DOWN 0x4000
    #define BUTTON_LEFT 0x8000

#elif XBEE_MODE == API_MODE

    #define BUTTON_JOYL 0x0001
    #define BUTTON_JOYR 0x0002
    #define BUTTON_SHARE 0x0004
    #define BUTTON_OPTION 0x0008
    #define BUTTON_PAD 0x0010
    #define BUTTON_PS 0x0020

#endif

#define PUSHE 1
#define RELEASE 0
#define PUSHED 2
#define RELEASED -1

class Controller
{
public:
    Controller(HardwareSerial *_Ser);

    void begin(int baudrate);
    void begin_api(int baudrate);

    void update(byte PinName);
    void update_api(byte PinName);
    void update_api_DR(byte PinName);

    bool readButton(unsigned int button,int status);

    unsigned int getButtonState() const;
    unsigned int getpreButtonState() const;

    /**
     * ジョイスティックの値を読む
     * @return -1.0 ~ 1.0
     * 
     *      X
     *      ^
     *      |
     * Y<---+----
     *      |
     *      |
     */
    double readJoyLX() const;
    double readJoyLY() const;
    double readJoyRX() const;
    double readJoyRY() const;

    bool getButtonChanged() const; //スイッチの状態を確認する
    
    unsigned int ButtonState, preButtonState;
    unsigned int LJoyX, LJoyY, RJoyX, RJoyY;

private:
    HardwareSerial *Ser;

    bool buttonChanged;
    

private:
    int recv_num;

    char recv_msgs[9];
    int recive_num[9];
    int recive_num_DR[5];
};

#endif