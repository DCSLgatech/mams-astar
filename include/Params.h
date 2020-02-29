/*
 * Params.h
 *
 *  Created on: Feb 12, 2015
 *      Author: florian
 */

#ifndef PARAMS_H_
#define PARAMS_H_

template <int N>
struct TwoPow
{
     enum { value = 2 * TwoPow<N - 1>::value };
};

template <>
struct TwoPow<0>
{
    enum { value = 1 };
};

#ifndef RESDIR2
#define RESDIR2 ""
#endif

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define RESDIR TOSTRING(RESDIR2)

#endif /* PARAMS_H_ */
