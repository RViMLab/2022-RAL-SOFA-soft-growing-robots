#include <SofaMAMMOBOT/config.h>

namespace sofa
{
namespace component
{

extern "C" {

SOFAMAMMOBOT_API
void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

SOFAMAMMOBOT_API
const char* getModuleName()
{
    return "SofaMAMMOBOT";
}

SOFAMAMMOBOT_API
const char* getModuleVersion()
{
    return "1.0";
}

SOFAMAMMOBOT_API
const char* getModuleLicense()
{
    return "LGPL";
}

SOFAMAMMOBOT_API
const char* getModuleDescription()
{
    return "SOFA plugin containing scenes with eversion simulations";
}

SOFAMAMMOBOT_API
const char* getModuleComponentList()
{
    // string containing the names of the classes provided by the plugin
    return "Eversion scenes";
}

} // extern "C"

} // namespace component
} // namespace sofa