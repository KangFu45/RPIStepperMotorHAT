#ifndef RPISIGNALS_H
#define RPISIGNALS_H

#include <QObject>

#include "RPIStepperMotorHAT.h"

class RPISignals : public QObject
{
    Q_OBJECT

public:
    static RPISignals* instance();

protected:
    explicit RPISignals(){};

signals:
    //
    void RPIAxisStatusChangedSig(MotorNumber);

    //
    void RPIDIStatusChangedSig(uint pin, bool val);

    //
    void RPIAxisHomeStageChangedSig(MotorNumber, HomeStage);

private:
    static RPISignals* _instance;
};

#endif // RPISIGNALS_H
