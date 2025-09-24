#pragma once
#include "RLogSU/logger.hpp"
#include <memory>
#include <typeinfo>

namespace Geometry {

enum ObjType
{
    // primitives
    POINT3,
    LINE3,
    PLANE3,
    
    // shapes
    LINESECT3,
    TRIANGLE3,

    NOT_AN_OBJ,
};

static const char *const ObjTypeStr(ObjType type)
{
    #define CASE_TYPE(type_)  case(type_): return #type_

    switch (type)
    {
        CASE_TYPE(POINT3);
        CASE_TYPE(LINE3);
        CASE_TYPE(PLANE3);
        CASE_TYPE(NOT_AN_OBJ);

        default:
        {
            throw std::runtime_error("UNKNOWN TYPE in ObjTypeStr()");
            return "UNKNOWN TYPE!!!";
        }
    }
    #undef CASE_TYPE
}


class GeomObj
{
public:
    virtual ~GeomObj() = default;

    [[nodiscard]] virtual ObjType WhoAmI() const;
    
    virtual void Assert()                                   const;
    virtual void Dump(const std::string& name = "some_obj") const;

protected:
};

using GeomObjUniqPtr = std::unique_ptr<GeomObj>;

class NotAnObj : public GeomObj
{
public:
    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::NOT_AN_OBJ; };

    virtual void Assert() const override {};
    
    virtual void Dump(const std::string& name = "some_obj") const override final
    {
        RLSU_LOG("'{}' [{}]  (typeof {})\n", name, static_cast<const void*>(this), ObjTypeStr(WhoAmI()));
    };
};

}