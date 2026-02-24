#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define UrXLearning_DLLIMPORT __declspec(dllimport)
#  define UrXLearning_DLLEXPORT __declspec(dllexport)
#  define UrXLearning_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define UrXLearning_DLLIMPORT __attribute__((visibility("default")))
#    define UrXLearning_DLLEXPORT __attribute__((visibility("default")))
#    define UrXLearning_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define UrXLearning_DLLIMPORT
#    define UrXLearning_DLLEXPORT
#    define UrXLearning_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef UrXLearning_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define UrXLearning_DLLAPI
#  define UrXLearning_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef UrXLearning_EXPORTS
#    define UrXLearning_DLLAPI UrXLearning_DLLEXPORT
#  else
#    define UrXLearning_DLLAPI UrXLearning_DLLIMPORT
#  endif // UrXLearning_EXPORTS
#  define UrXLearning_LOCAL UrXLearning_DLLLOCAL
#endif // UrXLearning_STATIC
