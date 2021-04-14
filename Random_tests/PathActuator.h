#ifndef PA_H
#define PA_H

#include "GeometryPath.h"
#include <memory>

class PathActuator {
private:
    std::unique_ptr<GeometryPath> path;

public:
    double function(double arg) {
        // calls PointBasedPath::getLength or FunctionBasedPath::getLength
        return path->getLength(arg);
    }
};

#endif
