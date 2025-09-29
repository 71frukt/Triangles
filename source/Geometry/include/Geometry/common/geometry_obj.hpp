#pragma once
#include "RLogSU/error_handler.hpp"
#include "RLogSU/logger.hpp"
#include <cstddef>
#include <memory>
#include <typeinfo>

namespace Geometry {

enum ObjType
{
    // primitives
    POINT3     = 0,
    LINE3      = 1,
    PLANE3     = 2,
    
    // shapes
    LINESECT3  = 3,
    TRIANGLE3  = 4,

    NOT_AN_OBJ = 5,
};

const size_t OBJ_TYPES_NUM = 6;


static const char *const ObjTypeStr(ObjType type)
{
    #define CASE_TYPE(type_)  case(type_): return #type_

    switch (type)
    {
        CASE_TYPE(POINT3);
        CASE_TYPE(LINE3);
        CASE_TYPE(PLANE3);

        CASE_TYPE(LINESECT3);
        CASE_TYPE(TRIANGLE3);
        
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

    [[nodiscard]] virtual ObjType WhoAmI() const = 0;
    
    virtual void Assert() const = 0;

    void Dump(const std::string& name) const
    {
        RLSU_LOG("\n");
        RLSU_LOG("'{}' [{}]  (typeof {}) {{\n", name, static_cast<const void*>(this), ERROR_HANDLE(ObjTypeStr(WhoAmI())));
        RLSU_BASETAB_INCREACE;

        DumpDetails();

        RLSU_BASETAB_DECREACE;
        RLSU_LOG("}}\n");
    }

protected:
    virtual void DumpDetails() const = 0;
};


using GeomObjUniqPtr = std::unique_ptr<GeomObj>;


class NotAnObj : public GeomObj
{
public:
    [[nodiscard]] virtual ObjType WhoAmI() const override final { return ObjType::NOT_AN_OBJ; };

    virtual void Assert() const override {};
    
    virtual void DumpDetails() const override final {};
};


}