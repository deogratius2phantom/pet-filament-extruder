#ifndef VERSION_H
#define VERSION_H

// FIRMWARE_VERSION is injected at build time via platformio.ini build_flags.
// Falls back to "dev" for local builds without the env var set.
#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "dev"
#endif

#endif // VERSION_H
