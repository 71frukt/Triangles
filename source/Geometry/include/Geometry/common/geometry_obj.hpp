#pragma once
#include "RLogSU/logger.hpp"
#include <memory>

namespace Geometry {

enum ObjType
{
    POINT3,
    LINE3,
    PLANE3,
    
    NOT_AN_OBJ,
};


class GeomObj
{
public:
    virtual ~GeomObj() = default;

    [[nodiscard]] virtual ObjType WhoAmI() const = 0;

    virtual void Dump() const = 0;

private:
};

using GeomObjUniqPtr = std::unique_ptr<GeomObj>;

class NotAnObj : public GeomObj
{
public:
    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::NOT_AN_OBJ; };

    virtual void Dump() const override final { RLSU_DUMP("NOT-AN-OBJECT [{}]", static_cast<const void*>(this)); };
};

}