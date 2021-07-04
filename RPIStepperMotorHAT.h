#ifndef RPISTEPPERMOTORHAT_H
#define RPISTEPPERMOTORHAT_H

#include "RPIStepperMotorHAT_global.h"

#include "RPIConfig.h"

#include <vector>
#include <thread>

#include <QObject>
#include <QTimer>

/*
控制树莓派步进电机驱动板DRV8825的动态库
2个步进电机+4个限位开关
*/

enum MotorNumber
{
    MOTOR1 = 1,
    MOTOR2
};

enum MotorDir
{
    FORWARD = 0,
    BACKWARD
};

class DI
{
public:
    DI();
    ~DI();

private:

};

class Motor
{
public:
    Motor(MotorNumber);
    ~Motor();

    struct Para
    {
        MotorNumber eNum;

        double nPulesLap = 1000.0;		// 轴转动每圈脉冲值
        double dGearRatio = 1.0;      // 传动比率

        //回零参数
        bool bHomeDir;	//回零方向

        double dMinPosMM = -100;		// 最小值，mm，默认为归零点，旋转轴无此值
        double dMaxPosMM = 100;		// 最大值，mm，旋转轴无此值

        double dAcc;	//加速度, mm/s
        double dDec;	//减速度, mm/s
    }m_Para;

    struct Status
    {
        bool bAbruptStop;	// 急停
        bool bRun;	//运动
        //正负限位，与初始数字输入信号无关，true==限位触发
        bool bPosiLimit;
        bool bNegaLimit;

        long lPrfPos = 0;	// 规划位置，pluse
        double dPrfPosMM = 0;	// 规划真实位置，mm
    }m_Status;

    //暂只清除限位触发标志
    void ClrStatus();

    //点动运动
    void TrapMove(const double& pos, const double& vel, MotorDir dir, const bool& absolute = true);

    //回零，限位回零方式
    void GoHome();

    //急停
    void Stop();

    //更新状态，ignore==true初始化更新
    void UpdateStatus(bool ignore = false);

private:
    //步进
    //目标脉冲位置 pluse
    //目标脉冲速度 pluse/ms
    void Step(long steps, double vel);

    //TODO：线程管理
    //每个电机在独立的线程中运动，step（）会先销毁之前的线程再初始化一个新的线程
    std::thread* m_pThread;
    bool m_bThreadExit = false; //运动线程退出标志

    //pin
    UBYTE m_EnablePin;
    UBYTE m_DirPin;
    UBYTE m_StepPin;
    UBYTE m_M0Pin;
    UBYTE m_M1Pin;
    UBYTE m_M2Pin;
};

class RPISTEPPERMOTORHAT_EXPORT RPIMotion : public QObject
{
    Q_OBJECT

public:
    RPIMotion();
    ~RPIMotion();

    void AddMotor(MotorNumber);

    //点动运动
    void MotorTrapMove(MotorNumber, const double& pos, const double& vel, MotorDir dir, const bool& absolute = true);

    //回零，限位回零方式
    void MotorGoHome(MotorNumber);

    //急停
    void MotorStop(MotorNumber);

private:
    std::vector<Motor*> m_vMotorPtrs;

    Motor* FindMotor(MotorNumber);

    QTimer* m_pTimer = nullptr;
};

#endif // RPISTEPPERMOTORHAT_H
