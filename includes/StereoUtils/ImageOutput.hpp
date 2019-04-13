//
// Created by yyhu on 4/12/19.
//

#ifndef STEREO_UTILS_IMAGEOUTPUT_HPP
#define STEREO_UTILS_IMAGEOUTPUT_HPP

#include <opencv2/opencv.hpp>
#include <string>

/** Clip and normalize an image.
 *
 * Clip and normalize an image. The input image will be clipped according to
 * minValue and maxValue. Then the clipped image will be converted to CV_32F typed image.
 *
 * If the input image has multiple channels, then each channel is clipped and normalized
 * by minValue and maxValue. Only works with single channel and 3-channel input images.
 *
 * It is required that maxValue > minValue >= 0.
 *
 * @param src
 * @param dst
 * @param minValue
 * @param maxValue
 */
void clip_normalize(const cv::Mat& src, cv::Mat& dst, float minValue, float maxValue);

/** Convert an input image src into floating point image and save to file.
 *
 * Convert an input image src into a floating point image. Using minValue and maxValue
 * to clip and nomalize the input image. Then the image is saved as a CV_32F typed image.
 *
 * If the image has multiple channels, every channel will be clipped and normalized.
 *
 * It is required that maxValue > minValue >= 0.
 *
 * @param src
 * @param minValue
 * @param maxValue
 */
void write_2_float_image(const std::string& fn, const cv::Mat& src, float minValue, float maxValue);

#endif //STEREO_UTILS_IMAGEOUTPUT_HPP
