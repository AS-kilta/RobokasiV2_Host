#include "gui/StepTypes.hpp"

#include <string.h>

int gui::getStepTypeNameIdx(const char *typeName)
{
    for (int i = 0; i < numStepTypeNames; ++i)
        if (!strcmp(typeName, stepTypeNames[i]))
            return i;
    return -1;
}
