add_subdirectory(MathGeoLib)
add_subdirectory(siftgpu)
add_subdirectory(fastann)
add_subdirectory(acransac)
add_subdirectory(glad)

set_target_properties(glad acransac fastann siftgpu MathGeoLib PROPERTIES FOLDER "depends")
