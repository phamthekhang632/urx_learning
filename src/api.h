#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define Ur10Learning_DLLIMPORT __declspec(dllimport)
#  define Ur10Learning_DLLEXPORT __declspec(dllexport)
#  define Ur10Learning_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define Ur10Learning_DLLIMPORT __attribute__((visibility("default")))
#    define Ur10Learning_DLLEXPORT __attribute__((visibility("default")))
#    define Ur10Learning_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define Ur10Learning_DLLIMPORT
#    define Ur10Learning_DLLEXPORT
#    define Ur10Learning_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef Ur10Learning_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define Ur10Learning_DLLAPI
#  define Ur10Learning_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef Ur10Learning_EXPORTS
#    define Ur10Learning_DLLAPI Ur10Learning_DLLEXPORT
#  else
#    define Ur10Learning_DLLAPI Ur10Learning_DLLIMPORT
#  endif // Ur10Learning_EXPORTS
#  define Ur10Learning_LOCAL Ur10Learning_DLLLOCAL
#endif // Ur10Learning_STATIC