#ifndef GP_H
#define GP_H
class GeometryPath {
    // note: this prevents constructing a `GeometryPath`: the only way to "have"
    // a `GeometryPath` is to construct a derived class (e.g. `PointBasedPath`)
public:
    virtual double getLength(double const&) const = 0;
public:
    ~GeometryPath() = default;
};
#endif
