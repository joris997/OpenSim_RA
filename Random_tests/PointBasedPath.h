#ifndef PBP_H
#define PBP_H

#include "GeometryPath.h"

class PointBasedPath : public GeometryPath {
    // PointBasedPath.cpp will contain the implementation of this method
    double getLength(double const& s) const override{
        return 3*s;
    }
};
#endif
