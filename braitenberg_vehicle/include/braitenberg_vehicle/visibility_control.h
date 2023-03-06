#ifndef BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_
#define BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BRAITENBERG_VEHICLE_EXPORT __attribute__ ((dllexport))
    #define BRAITENBERG_VEHICLE_IMPORT __attribute__ ((dllimport))
  #else
    #define BRAITENBERG_VEHICLE_EXPORT __declspec(dllexport)
    #define BRAITENBERG_VEHICLE_IMPORT __declspec(dllimport)
  #endif
  #ifdef BRAITENBERG_VEHICLE_BUILDING_LIBRARY
    #define BRAITENBERG_VEHICLE_PUBLIC BRAITENBERG_VEHICLE_EXPORT
  #else
    #define BRAITENBERG_VEHICLE_PUBLIC BRAITENBERG_VEHICLE_IMPORT
  #endif
  #define BRAITENBERG_VEHICLE_PUBLIC_TYPE BRAITENBERG_VEHICLE_PUBLIC
  #define BRAITENBERG_VEHICLE_LOCAL
#else
  #define BRAITENBERG_VEHICLE_EXPORT __attribute__ ((visibility("default")))
  #define BRAITENBERG_VEHICLE_IMPORT
  #if __GNUC__ >= 4
    #define BRAITENBERG_VEHICLE_PUBLIC __attribute__ ((visibility("default")))
    #define BRAITENBERG_VEHICLE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BRAITENBERG_VEHICLE_PUBLIC
    #define BRAITENBERG_VEHICLE_LOCAL
  #endif
  #define BRAITENBERG_VEHICLE_PUBLIC_TYPE
#endif

#endif  // BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_
