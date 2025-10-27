#ifndef PPMTOPNG_H
#define PPMTOPNG_H

#include <opencv2/opencv.hpp>

#include <iostream>

void convertppm(std::string Name){
    // Path to the input PPM file
    std::string inputPath = Name + ".ppm";
    // Path to the output PNG file
    std::string outputPath = Name + ".png";

    // Load the image using OpenCV
    cv::Mat image = cv::imread(inputPath, cv::IMREAD_UNCHANGED);

    if (image.empty()) {
        std::cerr << "Failed to load image: " << inputPath << std::endl;
    }

    // Save the image as PNG
    if (!cv::imwrite(outputPath, image)) {
        std::cerr << "Failed to save image as PNG: " << outputPath << std::endl;
    }
}

#endif