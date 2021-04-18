file(REMOVE_RECURSE
  "libadv_ds.pdb"
  "libadv_ds.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/adv_ds.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
