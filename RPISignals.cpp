#include "RPISignals.h"

RPISignals* RPISignals::_instance = 0;

RPISignals* RPISignals::instance()
{
    if(_instance == 0){
        _instance = new RPISignals;
    }
    return _instance;
}
