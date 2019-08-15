add_library(
  SSE4.1
  INTERFACE
)

add_library(SIMD::SSE4.1 ALIAS SSE4.1)

target_compile_options(
  SSE4.1
  INTERFACE
  $<$<PLATFORM_ID:Linux>:-msse4.1;-march=native>
  $<$<PLATFORM_ID:Windows>:/arch:AVX2>
)
