#ifndef IMG_TSPT_EXPORTDECL_H
#define IMG_TSPT_EXPORTDECL_H

#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef image_transport_EXPORTS // we are building a shared lib/dll
    #define IMG_TSPT_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define IMG_TSPT_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define IMG_TSPT_DECL
#endif

#endif // IMG_TSPT_EXPORTDECL_H
