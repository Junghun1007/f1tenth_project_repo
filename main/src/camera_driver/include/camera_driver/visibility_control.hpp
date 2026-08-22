#ifndef CAMERA_DRIVER__VISIBILITY_CONTROL_HPP_
#define CAMERA_DRIVER__VISIBILITY_CONTROL_HPP_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define CAMERA_DRIVER_EXPORT __attribute__((dllexport))
    #define CAMERA_DRIVER_IMPORT __attribute__((dllimport))
  #else
    #define CAMERA_DRIVER_EXPORT __declspec(dllexport)
    #define CAMERA_DRIVER_IMPORT __declspec(dllimport)
  #endif
  #ifdef CAMERA_DRIVER_BUILDING_DLL
    #define CAMERA_DRIVER_PUBLIC CAMERA_DRIVER_EXPORT
  #else
    #define CAMERA_DRIVER_PUBLIC CAMERA_DRIVER_IMPORT
  #endif
  #define CAMERA_DRIVER_LOCAL
#else
  #define CAMERA_DRIVER_EXPORT __attribute__((visibility("default")))
  #define CAMERA_DRIVER_IMPORT
  #if __GNUC__ >= 4
    #define CAMERA_DRIVER_PUBLIC __attribute__((visibility("default")))
    #define CAMERA_DRIVER_LOCAL __attribute__((visibility("hidden")))
  #else
    #define CAMERA_DRIVER_PUBLIC
    #define CAMERA_DRIVER_LOCAL
  #endif
#endif

#endif  // CAMERA_DRIVER__VISIBILITY_CONTROL_HPP_
