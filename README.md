# Traffic Sign Detection Project
## Overview
This project focuses on the detection and recognition of traffic signs in color images using image processing techniques. It aims to accurately identify various types of traffic signs through a robust pipeline, based on their shapes, colors, and inner content, providing essential information for traffic safety systems.
## Objectives
1. Traffic Sign Detection: Developing an algorithm that can identify traffic signs in real-world images.
2. Shape and Color Analysis: Extracting the shape of the sign and analyzing its color to determine the type of sign.
3. Sign Recognition: Recognizing and classifying different types of traffic signs based on their appearance (circular, triangular, square, etc.).
4. Performance Evaluation: Testing the method on various images to ensure robustness and accuracy.
## Project Structure
1. Preprocessing:
    * Resizing: Images are resized to ensure optimal detection.
    * Noise Reduction: Gaussian filtering is applied to smooth the image and reduce noise.
    * Edge Detection: Canny edge detection is used to extract important edges.
2. Shape Recognition:
    * The shape of the sign is extracted and classified into various categories such as circle, triangle, square, etc.
    * The algorithm then proceeds to analyze the internal contents of the sign.
3. Sign Recognition:
    * Signs are categorized into different types: "Stop", "Yield", "No Entry", "Pedestrian Crossing", etc., based on their color and internal content.
## How to Run
  * The project uses OpenCV to process images and detect traffic signs.
  * Run the program, select the images for processing, and the system will automatically detect the signs and display the results.
  * The system allows you to display all images or focus on specific results.
## Dependencies
  * OpenCV for image processing
  * C++ Standard Library for basic operations
