#include "RPIStepperMotorHAT.h"
#include "RPISignals.h"

#include <math.h>

#include <QDebug>

static const uint gDelayMs = 2;
static const uint gMaxDelayMs = 1000;
static const double gMinVel = 0.001;    //pluse/ms

//****************************DI*****************************

DI::DI(uint pin)
    : m_iPin(pin)
{
    pinMode(m_iPin, INPUT);
}

DI::~DI()
{

}

void DI::UpdateStatus()
{
   bool bVal = digitalRead(m_iPin);
   if(bVal != m_bValue){
       m_bValue = bVal;
       emit RPISignals::instance()->RPIDIStatusChangedSig(m_iPin, m_bValue);
   }
}

//**************************Motor*******************************

Motor::Motor(MotorNumber num)
{
    //qDebug()<<"Motor("<<num<<")";
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

    pinMode(m_EnablePin, OUTPUT);
    pinMode(m_DirPin, OUTPUT);
    pinMode(m_StepPin, OUTPUT);
    pinMode(m_M0Pin, OUTPUT);
    pinMode(m_M1Pin, OUTPUT);
    pinMode(m_M2Pin, OUTPUT);

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
        //qDebug()<<m_Status.dPrfPosMM;

        //
        if(m_pNegaLimitDI != nullptr){
            if(m_Status.bNegaLimit != m_pNegaLimitDI->m_bValue)
                m_Status.bNegaLimit = m_pNegaLimitDI->m_bValue;
        }
        if(m_pPosiLimitDI != nullptr){
            if(m_Status.bPosiLimit != m_pPosiLimitDI->m_bValue)
                m_Status.bPosiLimit = m_pPosiLimitDI->m_bValue;
        }

        emit RPISignals::instance()->RPIAxisStatusChangedSig(m_Para.eNum);
    }

    if(m_bHomeStageDown){
        MotorDir dir;
        switch (m_Status.eHomeStage) {
        case HOME_NULL:
            break;
        case HOME_FIND_LIMIT:
            m_Status.eHomeStage = HOME_ESCAPE_STEP;
            if(m_Para.sHomePrm.eDir == FORWARD)
                dir = BACKWARD;
            else {
                dir = FORWARD;
            }
            TrapMove(m_Para.sHomePrm.dEscapeStep, m_Para.sHomePrm.dVel, dir, false);
            break;
        case HOME_ESCAPE_STEP:
            m_Status.eHomeStage = HOME_END;
            m_Status.lPrfPos = 0;
            m_Status.dPrfPosMM = 0;
            break;
        case HOME_END:
            break;
        default:
            break;
        }
        m_bHomeStageDown = false;
    }

    //qDebug()<<m_Status.dPrfPosMM<<" "<<m_Status.eHomeStage;
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
        if (dir ==  FORWARD)
            tPos = m_Status.dPrfPosMM + pos;
        else
            tPos = m_Status.dPrfPosMM - pos;
    }

    if (tPos < m_Para.dMinPosMM)
        tPos = m_Para.dMinPosMM;
    if (tPos > m_Para.dMaxPosMM)
        tPos = m_Para.dMaxPosMM;

    //qDebug()<<tPos;

    //mm->pules
    long sPos = tPos / m_Para.dGearRatio * m_Para.nPulesLap;//绝对位置    

    //mm/s->pules/ms
    double tVel = vel / m_Para.dGearRatio * m_Para.nPulesLap / 1000;

    //qDebug()<<sPos;
    if(m_Status.bRun)
        Stop();
    //m_Status.bRun = true;

    m_pThread = new std::thread(&Motor::Step,this,sPos,tVel);
}

void Motor::GoHome()
{
    //
    if((m_Para.sHomePrm.eDir == FORWARD && m_pPosiLimitDI != nullptr)
            ||(m_Para.sHomePrm.eDir == BACKWARD && m_pNegaLimitDI != nullptr))
    {
        m_Status.eHomeStage = HOME_FIND_LIMIT;
        TrapMove(m_Para.dMaxPosMM - m_Para.dMinPosMM, m_Para.sHomePrm.dVel, m_Para.sHomePrm.eDir, false);
    }
}

void Motor::Stop()
{
    //qDebug()<<"Stop()";
    if(m_Status.bRun){
        m_bThreadExit = true;
        m_pThread->join();
        delete m_pThread;
    }
}

//计算速度曲线
//输入：起始脉冲距离(pluse)，加减速度(pluse/ms2)，目标速度(pluse/ms)
//输出：加减速阶段延时长(ms)，加速结束及减速开始位置(pluse)
bool CalVelocityLine(long beginPos, long endPos,
                     double vel, double acc, double dec,
                     long& accEndPos, long& decBeginPos)
{
    //1.确定匀加减速阶段距离
    //完整的加减速时间
    double accTime = vel / acc;
    double decTime = vel / dec;
    long accDistance = acc * accTime * accTime * 0.5;
    long decDistance = dec * decTime * decTime * 0.5;
    qDebug()<<vel<<" "<< accTime<<" "<<decTime<<" "<<accDistance<<" "<<decDistance;
    if((accDistance + decDistance) < labs(endPos - beginPos)){
        //达到目标速度
        accEndPos = beginPos + accDistance;
        decBeginPos = endPos - decDistance;
        return true;
    }
    else{
        //未达到目标速度
        return false;
    }
}

//TODO:电机加减速
void Motor::Step(long steps, double vel)
{
    //1.设置方向
    bool bDir;
    if (steps < m_Status.lPrfPos){
        digitalWrite(m_DirPin, 0);
        bDir = false;
    }
    else if (steps > m_Status.lPrfPos){
        digitalWrite(m_DirPin, 1);
        bDir = true;
    }
    else
        return;

    //2.设置速度，计算延时长度
    unsigned long tSteps = labs(steps - m_Status.lPrfPos);
    //tSteps = 200 * 32;

    //mm/s->pluse/ms
    //double acc = m_Para.dAcc / m_Para.dGearRatio * m_Para.nPulesLap / 1000;
    //double dec = m_Para.dDec / m_Para.dGearRatio * m_Para.nPulesLap / 1000;
    //long accEndPos, decBeginPos;
    //qDebug()<<steps;
    //if(!CalVelocityLine(0, tSteps,vel,acc,dec,accEndPos,decBeginPos))
    //    return;
    //qDebug()<<accEndPos<<" "<<decBeginPos;

    //3.发射相对脉冲数
    m_Status.bRun = true;

    //uint curVel = 160;
    uint curVel = 1 / vel * 1000;
    //qDebug()<<"vel: "<<targetVel<<" "<<"steps: "<<tSteps;

    //uint curDelayMs;
    //ulong accSumMillis = 0;
    //ulong decSumMillis = 0;
    for (unsigned long i = 0; i < tSteps; i++) {
        if(m_bThreadExit)
            break;

        if(m_Status.eHomeStage == HOME_FIND_LIMIT){
            if(m_Para.sHomePrm.eDir == FORWARD){
                if(m_Status.bPosiLimit){
                    m_bHomeStageDown = true;
                    break;
                }
            }
            else if(m_Para.sHomePrm.eDir == BACKWARD){
                if(m_Status.bNegaLimit){
                    m_bHomeStageDown = true;
                    break;
                }
            }
        }

        if(!IsHomeing()){
            if(bDir){
                if(m_Status.bPosiLimit)
                    break;
            }
            else{
                if(m_Status.bNegaLimit)
                    break;
            }
        }

        //if(i < tSteps / 2){
        //    //acc
        //    if(curVel > targetVel)
        //        --curVel;
        //}
        //else{
        //    //dec
        //}

        //if(i < accEndPos){
        //    curVel =  accSumMillis * acc + gMinVel;
        //    curDelayMs = 1 / curVel;
        //    accSumMillis += curDelayMs * 2;
        //}
        //else if(i > decBeginPos){
        //    curVel = curVel - decSumMillis * dec;
        //    curDelayMs = 1 / curVel;
        //    decSumMillis += curDelayMs * 2;
        //}
        //qDebug()<<"curVel: "<<curVel<<" curDelayMs: "<<curDelayMs;
        digitalWrite(m_StepPin, 1);
        delayMicroseconds(curVel);
        digitalWrite(m_StepPin, 0);
        delayMicroseconds(curVel);

        if(bDir)
            ++m_Status.lPrfPos;
        else
            --m_Status.lPrfPos;
    }

    if(m_Status.eHomeStage == HOME_ESCAPE_STEP){
        m_bHomeStageDown = true;
        //emit RPISignals::instance()->RPIAxisHomeStageChangedSig(m_Para.eNum, HOME_END);
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
            //
            for(auto ptr = m_vDIPtrs.begin(); ptr!=m_vDIPtrs.end(); ++ptr){
                (*ptr)->UpdateStatus();
            }
            //
            for(auto ptr = m_vMotorPtrs.begin(); ptr!=m_vMotorPtrs.end(); ++ptr){
                (*ptr)->UpdateStatus();
            }
        });
        m_pTimer->start(200);
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

    while(!m_vDIPtrs.empty()){
        delete *(m_vDIPtrs.begin());
        m_vDIPtrs.erase(m_vDIPtrs.begin());
    }
}

void RPIMotion::AddMotor(MotorNumber num)
{
    m_vMotorPtrs.emplace_back(new Motor(num));
}

void RPIMotion::AddDI(uint pin)
{
    for(auto ptr = m_vDIPtrs.begin(); ptr != m_vDIPtrs.end(); ++ptr){
        if((*ptr)->GetPin() == pin)
            return;
    }

    m_vDIPtrs.emplace_back(new DI(pin));
}

const Motor* RPIMotion::GetMotor(MotorNumber num)
{
    for(auto ptr = m_vMotorPtrs.begin(); ptr!=m_vMotorPtrs.end(); ++ptr){
        if((*ptr)->m_Para.eNum == num)
            return *ptr;
    }
    return nullptr;
}

const DI* RPIMotion::GetDI(uint pin)
{
    for(auto ptr = m_vDIPtrs.begin(); ptr != m_vDIPtrs.end(); ++ptr){
        if((*ptr)->GetPin() == pin)
            return *ptr;
    }
    return nullptr;
}

//点动运动
void RPIMotion::MotorTrapMove(MotorNumber num, const double& pos, const double& vel, MotorDir dir, const bool& absolute)
{
    //qDebug()<<num<<" "<<pos<<" "<<vel;
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

void RPIMotion::MotorAddLimit(MotorNumber num, uint pin, MotorDir dir)
{
    Motor* m = FindMotor(num);
    if(m == nullptr)
        return;

    DI* di = FindDI(pin);
    if(di == nullptr)
        return;

    if(dir == FORWARD){
        m->SetPosiLimit(di);
    }
    else {
        m->SetNegaLimit(di);
    }
}

bool RPIMotion::MotorIsRunning(MotorNumber num)
{
    Motor* m = FindMotor(num);
    if(m == nullptr)
        return false;

    return m->m_Status.bRun;
}

Motor* RPIMotion::FindMotor(MotorNumber num)
{
    for(auto ptr = m_vMotorPtrs.begin(); ptr!=m_vMotorPtrs.end(); ++ptr){
        if((*ptr)->m_Para.eNum == num)
            return *ptr;
    }
    return nullptr;
}

DI* RPIMotion::FindDI(uint pin)
{
    for(auto ptr = m_vDIPtrs.begin(); ptr != m_vDIPtrs.end(); ++ptr){
        if((*ptr)->GetPin() == pin)
            return *ptr;
    }
    return nullptr;
}
