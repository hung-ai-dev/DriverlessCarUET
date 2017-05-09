#ifndef POLYFIT_H_
#define POLYFIT_H_

#ifdef __cplusplus
extern "C" {
#endif

//using namespace std;
//using namespace cv;

int polyfit(const double* const dependentValues,
            const double* const independentValues,
            unsigned int        countOfElements,
            unsigned int        order,
            double*             coefficients);


#ifdef __cplusplus
}
#endif

#endif
