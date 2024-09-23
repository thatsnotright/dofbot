#ifndef DOFBOT__VISIBILITY_CONTROL_H_
#define DOFBOT__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define DOFBOT_EXPORT __attribute__((dllexport))
#define DOFBOT_IMPORT __attribute__((dllimport))
#else
#define DOFBOT_EXPORT __declspec(dllexport)
#define DOFBOT_IMPORT __declspec(dllimport)
#endif
#ifdef DOFBOT_BUILDING_DLL
#define DOFBOT_PUBLIC DOFBOT_EXPORT
#else
#define DOFBOT_PUBLIC DOFBOT_IMPORT
#endif
#define DOFBOT_PUBLIC_TYPE DOFBOT_PUBLIC
#define DOFBOT_LOCAL
#else
#define DOFBOT_EXPORT __attribute__((visibility("default")))
#define DOFBOT_IMPORT
#if __GNUC__ >= 4
#define DOFBOT_PUBLIC __attribute__((visibility("default")))
#define DOFBOT_LOCAL __attribute__((visibility("hidden")))
#else
#define DOFBOT_PUBLIC
#define DOFBOT_LOCAL
#endif
#define DOFBOT_PUBLIC_TYPE
#endif

#endif  // DOFBOT__VISIBILITY_CONTROL_H_
