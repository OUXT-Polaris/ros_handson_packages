#ifndef TUTORIAL__VISIBILITY_CONTROL_H_
#define TUTORIAL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TUTORIAL_EXPORT __attribute__ ((dllexport))
    #define TUTORIAL_IMPORT __attribute__ ((dllimport))
  #else
    #define TUTORIAL_EXPORT __declspec(dllexport)
    #define TUTORIAL_IMPORT __declspec(dllimport)
  #endif
  #ifdef TUTORIAL_BUILDING_LIBRARY
    #define TUTORIAL_PUBLIC TUTORIAL_EXPORT
  #else
    #define TUTORIAL_PUBLIC TUTORIAL_IMPORT
  #endif
  #define TUTORIAL_PUBLIC_TYPE TUTORIAL_PUBLIC
  #define TUTORIAL_LOCAL
#else
  #define TUTORIAL_EXPORT __attribute__ ((visibility("default")))
  #define TUTORIAL_IMPORT
  #if __GNUC__ >= 4
    #define TUTORIAL_PUBLIC __attribute__ ((visibility("default")))
    #define TUTORIAL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TUTORIAL_PUBLIC
    #define TUTORIAL_LOCAL
  #endif
  #define TUTORIAL_PUBLIC_TYPE
#endif

#endif  // TUTORIAL__VISIBILITY_CONTROL_H_
