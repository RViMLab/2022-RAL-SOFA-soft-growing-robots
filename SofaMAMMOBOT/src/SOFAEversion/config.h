#pragma once

#include <sofa/config.h>

#ifdef SOFA_BUILD_SOFAMAMMOBOT
#  define SOFAMAMMOBOT_API SOFA_EXPORT_DYNAMIC_LIBRARY
#else
#  define SOFAMAMMOBOT_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif