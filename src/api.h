#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define ObjectTrackingToss_DLLIMPORT __declspec(dllimport)
#  define ObjectTrackingToss_DLLEXPORT __declspec(dllexport)
#  define ObjectTrackingToss_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define ObjectTrackingToss_DLLIMPORT __attribute__((visibility("default")))
#    define ObjectTrackingToss_DLLEXPORT __attribute__((visibility("default")))
#    define ObjectTrackingToss_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define ObjectTrackingToss_DLLIMPORT
#    define ObjectTrackingToss_DLLEXPORT
#    define ObjectTrackingToss_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ObjectTrackingToss_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ObjectTrackingToss_DLLAPI
#  define ObjectTrackingToss_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef ObjectTrackingToss_EXPORTS
#    define ObjectTrackingToss_DLLAPI ObjectTrackingToss_DLLEXPORT
#  else
#    define ObjectTrackingToss_DLLAPI ObjectTrackingToss_DLLIMPORT
#  endif // ObjectTrackingToss_EXPORTS
#  define ObjectTrackingToss_LOCAL ObjectTrackingToss_DLLLOCAL
#endif // ObjectTrackingToss_STATIC