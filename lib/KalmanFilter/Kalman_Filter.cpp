#include "Kalman_Filter.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;



Kalman::Kalman()
{
};
Kalman::~Kalman()
{
};

BLA::Matrix<2,1> Kalman::xk_pred(BLA::Matrix<2,2> F, BLA::Matrix<2,1> G, BLA::Matrix<2,1> xk_1, BLA::Matrix<1,1> uk_1)
{
    return (F*xk_1+G*uk_1);
}

BLA::Matrix<2,2> Kalman::Pk_pred(BLA::Matrix<2,2> F,BLA::Matrix<2,2> Pk_1_correc,BLA::Matrix<2,2> Qk_1)
{

    return (F*Pk_1_correc)*(~F)+Qk_1;
}