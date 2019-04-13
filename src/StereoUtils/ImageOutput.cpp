//
// Created by yyhu on 4/12/19.
//

#include <opencv2/highgui.hpp>
#include <sstream>

#include "StereoUtils/ImageOutput.hpp"

void clip_normalize(const cv::Mat& src, cv::Mat& dst, float minValue, float maxValue)
{
    if ( maxValue <= minValue || minValue < 0 )
    {
        std::stringstream ss;
        ss << "Wrong minValue maxValue setttings. minValue = " << minValue
           << ", maxValue = " << maxValue << ".";
        throw std::runtime_error(ss.str());
    }

    const int channels = src.channels();

    if ( 1 != channels and 3 != channels )
    {
        std::stringstream ss;
        ss << "Input image could only be 1- or 3-channel image. channels = " << channels << ".";
        throw std::runtime_error(ss.str());
    }

    const int rows = src.rows;
    const int cols = src.cols;

    // Convert src to CV_32F.
    cv::Mat srcCvt;

    if ( 1 == channels )
    {
        src.convertTo(srcCvt, CV_32FC1);
        dst = cv::Mat(rows, cols, CV_32FC1);
    }
    else
    {
        src.convertTo(srcCvt, CV_32FC3);
        dst = cv::Mat(rows, cols, CV_32FC3);
    }

    // Clipping.
    float* ptrS = NULL; // The staring pointer of a column in src.
    float* ptrD = NULL; // The staring pointer of a column in dst.
    float temp;
    float range = maxValue - minValue;

    for ( int i = 0; i < rows; ++i )
    {
        ptrS = srcCvt.ptr<float>(i);
        ptrD = dst.ptr<float>(i);

        for ( int j = 0; j < cols; ++j )
        {
            for ( int k = 0; k < channels; ++k )
            {
                temp = ptrS[ k ];

                if ( temp > maxValue )
                {
                    temp = maxValue;
                }
                else if ( temp < minValue )
                {
                    temp = minValue;
                }

                ptrD[ k ] = (temp - minValue) / range;
            }

            ptrS += channels;
            ptrD += channels;
        }
    }
}

void write_2_float_image(const std::string& fn, const cv::Mat& src, float minValue, float maxValue)
{
    // Get the parameters of src.
    const int rows     = src.rows;
    const int cols     = src.cols;
    const int channels = src.channels();

    // Clip and normalize src.
    cv::Mat cn;
    clip_normalize(src, cn, minValue, maxValue);

    // Save image to file.
    cv::scaleAdd( cn, 255, cv::Mat::zeros(cn.rows, cn.cols, cn.type()), cn );

    if ( 1 == cn.channels() )
    {
        cn.convertTo(cn, CV_8UC1);
    }
    else if ( 3 == cn.channels() )
    {
        cn.convertTo(cn, CV_8UC3);
    }
    else
    {
        std::stringstream ss;
        ss << "Wrong channels (" << cn.channels() << ".";
        throw std::runtime_error(ss.str());
    }

    cv::imwrite( fn, cn );
}
