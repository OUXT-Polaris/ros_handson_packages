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

#ifndef BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_
#define BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define BRAITENBERG_VEHICLE_EXPORT __attribute__((dllexport))
#define BRAITENBERG_VEHICLE_IMPORT __attribute__((dllimport))
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
#define BRAITENBERG_VEHICLE_EXPORT __attribute__((visibility("default")))
#define BRAITENBERG_VEHICLE_IMPORT
#if __GNUC__ >= 4
#define BRAITENBERG_VEHICLE_PUBLIC __attribute__((visibility("default")))
#define BRAITENBERG_VEHICLE_LOCAL __attribute__((visibility("hidden")))
#else
#define BRAITENBERG_VEHICLE_PUBLIC
#define BRAITENBERG_VEHICLE_LOCAL
#endif
#define BRAITENBERG_VEHICLE_PUBLIC_TYPE
#endif

#endif  // BRAITENBERG_VEHICLE__VISIBILITY_CONTROL_H_
