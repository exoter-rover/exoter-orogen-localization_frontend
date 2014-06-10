#ifndef FRONT_END_TYPES_H
#define FRONT_END_TYPES_H

#include <vector>
#include <base/time.h>
#include <base/eigen.h>

namespace localization_frontend
{
    /** Coefficient for the filter **/
    struct FilterCoefficients
    {
        bool filterOn; /** Set to true if want to use it with the following coefficients **/
        base::VectorXd feedForwardCoeff; /** b coefficients **/
        base::VectorXd feedBackCoeff; /** a coefficients **/
    };

}
#endif
