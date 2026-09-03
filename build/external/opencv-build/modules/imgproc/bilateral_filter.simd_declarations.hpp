#define CV_CPU_SIMD_FILENAME "C:/Users/coolb/OneDrive/Desktop/ray tracing engine GPU port/external/opencv-5.0.0/modules/imgproc/src/bilateral_filter.simd.hpp"
#define CV_CPU_DISPATCH_MODE AVX2
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODE AVX512_SKX
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODES_ALL AVX512_SKX, AVX2, BASELINE

#undef CV_CPU_SIMD_FILENAME
