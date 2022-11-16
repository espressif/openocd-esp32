# We don't want cmake to run target test outputs on the build host machine.
# Otherwise, cmake configuration will fail for the cross-builds.
# Do we need to make this conditional?
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(CMAKE_SYSTEM_NAME   Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(host arm-linux-gnueabihf)

set(CMAKE_C_COMPILER    ${host}-gcc)
set(CMAKE_CXX_COMPILER  ${host}-g++)
set(AS                  ${host}-as)
set(CMAKE_AR            ${host}-gcc-ar)
set(OBJCOPY             ${host}-objcopy)
set(OBJDUMP             ${host}-objdump)
set(SIZE                ${host}-size)
