file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/compression/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
