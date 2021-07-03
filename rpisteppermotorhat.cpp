#include "RPIStepperMotorHAT.h"

#include <math.h>

Motor::Motor(MotorNumber num)
{
    m_Num = num;
    switch (m_Num)
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

    //pinMode(M1_ENABLE_PIN, OUTPUT);
    //pinMode(M1_DIR_PIN, OUTPUT);
    //pinMode(M1_STEP_PIN, OUTPUT);
    //pinMode(M1_M0_PIN, OUTPUT);
    //pinMode(M1_M1_PIN, OUTPUT);
    //pinMode(M1_M2_PIN, OUTPUT);

    //DEV_Digital_Write(Motor.EnablePin, 0);
}

Motor::~Motor()
{

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
    //mm->pules
    long sPos = tPos / m_Para.dGearRatio * m_Para.nPulesLap;//绝对位置

    //mm/ms->pules/ms
    double tVel = vel / m_Para.dGearRatio * m_Para.nPulesLap;

    //errors.emplace_back(GTN_SetPos(core, m_para.sNum, sPos));
    //errors.emplace_back(GTN_SetVel(core, m_para.sNum, tVel));
    //errors.emplace_back(GTN_Update(core, 1 << (m_para.sNum - 1)));
}

void Motor::GoHome()
{

}

void Motor::Stop()
{

}

//TODO:电机加减速
void Motor::Step(long steps, long vel)
{
    //1.设置方向
    if (steps > m_Status.lPrfPos)
        ;//DEV_Digital_Write(Motor.DirPin, 0);
    else if (steps < m_Status.lPrfPos)
        ;//DEV_Digital_Write(Motor.DirPin, 1);
    else
        return;

    //2.设置速度，计算延时长度

    //3.发射相对脉冲数
    unsigned long tSteps = labs(steps - m_Status.lPrfPos);
    for (unsigned long i = 0; i < tSteps; i++) {
        //DEV_Digital_Write(Motor.StepPin, 1);
        //DEV_Delay_ms(stepdelay);
        //DEV_Digital_Write(Motor.StepPin, 0);
        //DEV_Delay_ms(stepdelay);
    }
}
