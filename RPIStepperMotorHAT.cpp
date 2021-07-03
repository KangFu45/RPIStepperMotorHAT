#include "RPIStepperMotorHAT.h"

#include <math.h>

#include <QDebug>

static const uint gDelayMs = 2;

Motor::Motor(MotorNumber num)
{
    qDebug()<<"Motor("<<num<<")";
    m_Para.eNum = num;
    switch (m_Para.eNum)
    {
    case MOTOR1:
        m_EnablePin = M1_ENABLE_PIN;
        m_DirPin = M1_DIR_PIN;
        m_StepPin = M1_STEP_PIN;
        m_M0Pin = M1_M0_PIN;
        m_M1Pin = M1_M1_PIN;
        m_M2Pin = M1_M2_PIN;
        break;
    case MOTOR2:
        m_EnablePin = M2_ENABLE_PIN;
        m_DirPin = M2_DIR_PIN;
        m_StepPin = M2_STEP_PIN;
        m_M0Pin = M2_M0_PIN;
        m_M1Pin = M2_M1_PIN;
        m_M2Pin = M2_M2_PIN;
        break;
    default:
        break;
    }

    qDebug()<<"pinMode() all pin";
    pinMode(m_EnablePin, OUTPUT);
    pinMode(m_DirPin, OUTPUT);
    pinMode(m_StepPin, OUTPUT);
    pinMode(m_M0Pin, OUTPUT);
    pinMode(m_M1Pin, OUTPUT);
    pinMode(m_M2Pin, OUTPUT);

    qDebug()<<"digitalWrite() enable";
    digitalWrite(m_EnablePin, 1);
}

Motor::~Motor()
{

}

void Motor::UpdateStatus(bool ignore)
{
    //lpos
    if(m_Status.bRun || ignore){
        m_Status.dPrfPosMM = m_Status.lPrfPos / m_Para.nPulesLap * m_Para.dGearRatio;
        qDebug()<<m_Status.dPrfPosMM;
    }
}

void Motor::ClrStatus()
{

}

void Motor::TrapMove(const double& pos, const double& vel, MotorDir dir, const bool& absolute)
{    
    double tPos;
    if (absolute) {
        tPos = pos;
    }
    else {
        if (dir)
            tPos = m_Status.dPrfPosMM + pos;
        else
            tPos = m_Status.dPrfPosMM - pos;
    }
    if (tPos < m_Para.dMinPosMM)
        tPos = m_Para.dMinPosMM;
    if (tPos > m_Para.dMaxPosMM)
        tPos = m_Para.dMaxPosMM;
    qDebug()<<"tPos: "<<tPos;
    //mm->pules
    long sPos = tPos / m_Para.dGearRatio * m_Para.nPulesLap;//绝对位置
    qDebug()<<"sPos: "<<sPos;

    //mm/ms->pules/ms
    double tVel = vel / m_Para.dGearRatio * m_Para.nPulesLap;

    if(m_Status.bRun)
        Stop();
    //m_Status.bRun = true;
    m_pThread = new std::thread(&Motor::Step,this,sPos,tVel);
}

void Motor::GoHome()
{

}

void Motor::Stop()
{
    qDebug()<<"run: "<<m_Status.bRun;
    if(m_Status.bRun){
        //m_Status.bRun = false;
        qDebug()<<"Stop() delete thread";
        m_bThreadExit = true;
        m_pThread->join();
        delete m_pThread;
        qDebug()<<"Stop() delete thread down";
    }
}

//TODO:电机加减速
void Motor::Step(long steps, long vel)
{
    //1.设置方向
    bool bDir;
    if (steps > m_Status.lPrfPos){
        digitalWrite(m_DirPin, 0);
        bDir = false;
    }
    else if (steps < m_Status.lPrfPos){
        digitalWrite(m_DirPin, 1);
        bDir = true;
    }
    else
        return;

    m_Status.bRun = true;

    //2.设置速度，计算延时长度

    //3.发射相对脉冲数
    unsigned long tSteps = labs(steps - m_Status.lPrfPos);
    qDebug()<<"Step() "<<tSteps;
    for (unsigned long i = 0; i < tSteps; i++) {
        if(m_bThreadExit)
            break;

        digitalWrite(m_StepPin, 1);
        delay(gDelayMs);
        digitalWrite(m_StepPin, 0);
        delay(gDelayMs);

        if(bDir)
            ++m_Status.lPrfPos;
        else
            --m_Status.lPrfPos;
    }

    m_bThreadExit = false;
    m_Status.bRun = false;
    UpdateStatus(true);
}

//*********************************RPIMotion*********************************

RPIMotion::RPIMotion()
{
    if(wiringPiSetupGpio() < 0){
        qDebug()<<"set wiringPi lib failed!";
        return;
    }
    else {
        qDebug()<<"set wiringPi lib success!";
        m_pTimer = new QTimer;
        connect(m_pTimer, &QTimer::timeout, [=]{
            for(auto ptr = m_vMotorPtrs.begin(); ptr!=m_vMotorPtrs.end(); ++ptr){
                (*ptr)->UpdateStatus();
            }
        });
        m_pTimer->start(500);
    }
}

RPIMotion::~RPIMotion()
{
    if(m_pTimer!= nullptr){
        m_pTimer->stop();
        delete m_pTimer;
    }

    while(!m_vMotorPtrs.empty()){
        delete *(m_vMotorPtrs.begin());
        m_vMotorPtrs.erase(m_vMotorPtrs.begin());
    }
}

void RPIMotion::AddMotor(MotorNumber num)
{
    m_vMotorPtrs.emplace_back(new Motor(num));
}

//点动运动
void RPIMotion::MotorTrapMove(MotorNumber num, const double& pos, const double& vel, MotorDir dir, const bool& absolute)
{
    Motor* m = FindMotor(num);
    if(m == nullptr)
        return;

    m->TrapMove(pos, vel, dir, absolute);
}

//回零，限位回零方式
void RPIMotion::MotorGoHome(MotorNumber num)
{
    Motor* m = FindMotor(num);
    if(m == nullptr)
        return;

    m->GoHome();
}

//急停
void RPIMotion::MotorStop(MotorNumber num)
{
    Motor* m = FindMotor(num);
    if(m == nullptr)
        return;

    m->Stop();
}

Motor* RPIMotion::FindMotor(MotorNumber num)
{
    for(auto ptr = m_vMotorPtrs.begin(); ptr!=m_vMotorPtrs.end(); ++ptr){
        if((*ptr)->m_Para.eNum == num)
            return *ptr;
    }
    return nullptr;
}
