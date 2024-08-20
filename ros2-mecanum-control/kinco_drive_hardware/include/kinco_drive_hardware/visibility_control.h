#ifndef KINCO_DRIVE_HARDWARE__VISIBILITY_CONTROL_H_
#define KINCO_DRIVE_HARDWARE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KINCO_DRIVE_HARDWARE_EXPORT __attribute__ ((dllexport))
    #define KINCO_DRIVE_HARDWARE_IMPORT __attribute__ ((dllimport))
  #else
    #define KINCO_DRIVE_HARDWARE_EXPORT __declspec(dllexport)
    #define KINCO_DRIVE_HARDWARE_IMPORT __declspec(dllimport)
  #endif
  #ifdef KINCO_DRIVE_HARDWARE_BUILDING_LIBRARY
    #define KINCO_DRIVE_HARDWARE_PUBLIC KINCO_DRIVE_HARDWARE_EXPORT
  #else
    #define KINCO_DRIVE_HARDWARE_PUBLIC KINCO_DRIVE_HARDWARE_IMPORT
  #endif
  #define KINCO_DRIVE_HARDWARE_PUBLIC_TYPE KINCO_DRIVE_HARDWARE_PUBLIC
  #define KINCO_DRIVE_HARDWARE_LOCAL
#else
  #define KINCO_DRIVE_HARDWARE_EXPORT __attribute__ ((visibility("default")))
  #define KINCO_DRIVE_HARDWARE_IMPORT
  #if __GNUC__ >= 4
    #define KINCO_DRIVE_HARDWARE_PUBLIC __attribute__ ((visibility("default")))
    #define KINCO_DRIVE_HARDWARE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KINCO_DRIVE_HARDWARE_PUBLIC
    #define KINCO_DRIVE_HARDWARE_LOCAL
  #endif
  #define KINCO_DRIVE_HARDWARE_PUBLIC_TYPE
#endif

#endif  // KINCO_DRIVE_HARDWARE__VISIBILITY_CONTROL_H_
