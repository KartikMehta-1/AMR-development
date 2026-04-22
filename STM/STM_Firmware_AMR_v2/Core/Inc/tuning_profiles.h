// Firmware tuning profiles
//
// Purpose:
// - Allow quick A/B testing of control changes (launch guard, feed-forward, etc.)
//   without editing multiple constants across the codebase.
//
// How to use:
// - Set TUNING_PROFILE below to one of the TUNING_PROFILE_* values.
// - Rebuild + flash the STM firmware.
//
// Notes:
// - This header is included by app_config.h at the end, and overrides a small set
//   of macros via #undef/#define based on the selected profile.

#ifndef TUNING_PROFILES_H
#define TUNING_PROFILES_H

// Profile IDs
#define TUNING_PROFILE_BASELINE      0
#define TUNING_PROFILE_NO_GUARD      1
#define TUNING_PROFILE_NO_STATIC_FF  2
#define TUNING_PROFILE_NO_FF         3

// Select profile here.
// Baseline means: use the values defined in app_config.h as-is.
#ifndef TUNING_PROFILE
#define TUNING_PROFILE TUNING_PROFILE_NO_STATIC_FF
#endif

#endif  // TUNING_PROFILES_H
