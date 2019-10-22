add_subdirectory(MathGeoLib)
add_subdirectory(siftgpu)
add_subdirectory(fastann)
add_subdirectory(acransac)

set_target_properties(acransac fastann siftgpu MathGeoLib PROPERTIES FOLDER "depends")
