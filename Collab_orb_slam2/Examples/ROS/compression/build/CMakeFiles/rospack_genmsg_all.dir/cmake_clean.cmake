file(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/compression/msg"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_genmsg_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
