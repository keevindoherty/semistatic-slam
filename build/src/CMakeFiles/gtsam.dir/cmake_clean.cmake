file(REMOVE_RECURSE
  "libgtsam.a"
  "libgtsam.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/gtsam.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
