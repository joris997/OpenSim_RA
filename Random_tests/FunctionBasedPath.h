#ifndef FBP_H
#define FBP_H

#include "GeometryPath.h"

class FunctionBasedPath : public GeometryPath {
    // Your interp implementation will be in FunctionBasedPath.cpp
    double getLength(double const& s) const override{
        return 2*s;
    }
};
#endif
