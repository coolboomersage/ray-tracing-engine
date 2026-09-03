#define CV_CPU_SIMD_FILENAME "C:/Users/coolb/OneDrive/Desktop/ray tracing engine GPU port/external/opencv-5.0.0/modules/core/src/norm.simd.hpp"
#define CV_CPU_DISPATCH_MODE SSE4_1
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODE AVX
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODE AVX2
#include "opencv2/core/private/cv_cpu_include_simd_declarations.hpp"

#define CV_CPU_DISPATCH_MODES_ALL AVX2, AVX, SSE4_1, BASELINE

#undef CV_CPU_SIMD_FILENAME
