file(GLOB_RECURSE sourceFiles ./*.cpp ./*.c)
file(GLOB_RECURSE headerFiles ./*.h)

add_library(MathGeoLib STATIC "${sourceFiles};${headerFiles}")
