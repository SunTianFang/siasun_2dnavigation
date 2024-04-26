#pragma once

#include "AbstractParameter.h"
#include "MagicSingleton.h"

namespace robo
{
/**
   * Set of parameters
   */
class ParameterObject : public Object
{
private:
    /**
     * Default constructor
     */
    ParameterObject()
        : Object()
    {
    }

    friend MagicSingleton<ParameterObject>;

public:
    /**
     * Destructor
     */
    virtual ~ParameterObject()
    {
    }

    bool InitializeParameters();

private:
    ParameterObject(const ParameterObject&);
    const ParameterObject& operator=(const ParameterObject&);

    bool LoadRoboLocParm();

    bool LoadSimuParm();

    bool LoadRoboAdvParm();

};  // Parameters

}  // namespace robo

using ParameterObjectSingleton = MagicSingleton<robo::ParameterObject>;
