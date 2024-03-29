// DR青に関連する関数

#ifndef DRBLUE_h
#define DRBLUE_h

#include "define.h"
#include "Button_Encorder.h"
#include "lpms_me1Peach.h"
#include "phaseCounterPeach.h"
#include "RoboClaw.h"

#define _PUSHED     1
#define _RELEASED   2

struct expand_value{
  bool flag_pahse1;
  bool flag_pahse2;
};

class DRBlue{
public:
    /*********** 変数宣言 ***********/
    coords position; //自己位置(x,y,z)
    double roboAngle; //ロボットの姿勢(角度)

    /*********** 関数宣言 ***********/
    DRBlue(lpms_me1 *_lpms, phaseCounter *_enc1, phaseCounter *_enc2);
    void updateRobotPosition(void); //自己推定を行う
    void updateRoboAngle(void); //ロボットの姿勢(角度)のみ取得
    void setPosition(double x, double y, double z); //自己位置の初期化
    void BasicSetup(void); //汎用基板の基本的なセットアップを行う
    void DRsetup(void); //DRに関するセットアップを行う
    void allOutputLow(void); //全てのデジタル出力をLOWにする
    void LEDblink(byte pin, int times, int interval); //LEDを点滅させる
    void RGB_led(int period); //フルカラーLEDを奇麗に光らせる

private:
    /****自己位置推定用の変数****/
    lpms_me1 *lpms;
    phaseCounter *enc1;
    phaseCounter *enc2;
    double encX_rad , encX  ,pre_encX;
    double encY_rad , encY , pre_encY;
    double x_axis_prime, y_axis_prime;
    double angle_rad;
    double anlge_ofset;
    double setAngleNum;
};

class DRexpand{
public:
    DRexpand(byte _sw_pinName, byte _mosfet);
    void expand_func(int ConButton, int mode); //展開機構を操作する
    void init(void);
    expand_value expand;
private:
    byte sw_pinName;
    byte mosfet;
};

class DRwall{
public:
    DRwall(byte pinSW, byte pinSupport, int MDadress, RoboClaw *_roboclaw);

    void wall_time_count(double int_time); //壁越えに関する時間を計算
    void roboclawSpeedM1(double vel); //足回りの速度指定
    void roboclawSpeedAccelM1(double accel, double speed); //足回りの角速度指定
    void roboclaw_begin(int baudlate);
    void roboclawResetEncoders();
    bool send_wall_position(double refAngle, double refOmega); //壁越え機構の動作確認に使用
    bool send_wall_cmd(double robot_x_vel); //壁越えのコマンドを送信
    void init(void); // 初期化
        
    int position; //壁越えモータの回転角度[pulse]
    int omega; //壁越えモータの角速度[pulse/s]
    int accel; //壁越えモータの角加速度[pulse/ss]
    int adress; //RoboClawのアドレス(128,129,130,131)

private:
    Button sw;
    RoboClaw *roboclaw;

    byte pinSpt;
    double wall_time;
    bool wall_start;
    bool phase_1 = false;
    double seconds; //壁越えに必要な時間[s]

};

#endif