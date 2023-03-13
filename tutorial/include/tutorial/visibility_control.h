// Copyright (c) 2023 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TUTORIAL__VISIBILITY_CONTROL_H_
#define TUTORIAL__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TUTORIAL_EXPORT __attribute__((dllexport))
#define TUTORIAL_IMPORT __attribute__((dllimport))
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
#define TUTORIAL_EXPORT __attribute__((visibility("default")))
#define TUTORIAL_IMPORT
#if __GNUC__ >= 4
#define TUTORIAL_PUBLIC __attribute__((visibility("default")))
#define TUTORIAL_LOCAL __attribute__((visibility("hidden")))
#else
#define TUTORIAL_PUBLIC
#define TUTORIAL_LOCAL
#endif
#define TUTORIAL_PUBLIC_TYPE
#endif

#endif  // TUTORIAL__VISIBILITY_CONTROL_H_
