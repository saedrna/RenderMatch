file(GLOB_RECURSE sourceFiles ./*.cpp ./*.c)
file(GLOB_RECURSE headerFiles ./*.h)

add_library(glad STATIC "${sourceFiles};${headerFiles}")
