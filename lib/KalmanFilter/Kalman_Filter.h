#ifndef _KALMAN_FILTER_h
#define _KALMAN_FILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "arduino.h"
#else
    #include "WProgram.h"
#endif

#include <BasicLinearAlgebra.h>
using namespace BLA;

class Kalman{
    //HOLA BRO RICHIE LA CHUPA POR QUE ES JTO, O SEA NO 2TIENE NADA DE MALO SER JTO PERO RICHIE LA CHUPA equisDe
    public:
    Kalman();
    ~Kalman();

    BLA::Matrix<2,1> xk_pred(BLA::Matrix<2,2> F, BLA::Matrix<2,1> G, BLA::Matrix<2,1> xk_1, BLA::Matrix<1,1> uk_1);
    BLA::Matrix<2,2> Pk_pred(BLA::Matrix<2,2> F,BLA::Matrix<2,2> Pk_1_correc,BLA::Matrix<2,2> Qk_1);
    private:

};

#endif