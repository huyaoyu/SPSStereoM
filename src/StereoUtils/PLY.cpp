//
// Created by yyhu on 4/12/19.
//

#define TINYPLY_IMPLEMENTATION
#include "StereoUtils/PLY.hpp"

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <vector>

void write_ply_with_color(const std::string& fn, const cv::Mat& disp, const cv::Mat& color,
                          const float* Q, bool flip, bool binary)
{
    // Check the the dimensions of the input images.
    const int rows     = disp.rows;
    const int cols     = disp.cols;
    const int nPixels  = rows * cols;
    const int channels = disp.channels();

    if ( rows != color.rows || cols != color.cols )
    {
        std::stringstream ss;
        ss << "Dimensions of disp[" << rows << ", " << cols
           << "] and color[" << color.rows << "," << color.rows << "] are not compatible.";
        throw std::runtime_error(ss.str());
    }

    if ( 1 != channels || 3 != color.channels() )
    {
        std::stringstream ss;
        ss << "Channel of disp[" << channels << "] or color[" << color.channels() << "] is wrong.";
        throw std::runtime_error(ss.str());
    }

    if ( NULL == Q )
    {
        throw std::runtime_error("Q is NULL.");
    }

    // Good to go.

    struct float3_t { float x, y, z; };
    struct uchar3_t { unsigned char r, g, b; };

    std::vector<float3_t> vertexCoor;
    std::vector<uchar3_t> vertexColor;

    const float* pDisp = NULL;
    const unsigned char* pColor = NULL;

    cv::Mat colorRGB;
    cv::cvtColor(color.clone(), colorRGB, cv::COLOR_BGR2RGB);

    const auto eQ = Eigen::Map<
            const Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor >,
            Eigen::Unaligned,
            Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>
    >( Q, 4, 4, Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>(4, 1) );

    std::cout << "eQ = " << std::endl << eQ << std::endl;

    Eigen::Matrix< float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor > eQ_NC = eQ;

    float factor = 1.0f;

    if ( true == flip )
    {
        eQ_NC(1,1) *= -1.0f;
        eQ_NC(1,3) *= -1.0f;
        eQ_NC(2,3) *= -1.0f;

        factor = -1.0f;
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eCoor(4, nPixels);

    int idx = 0;
    float d;

    for ( int i = 0; i < rows; ++i )
    {
        pDisp = disp.ptr<float>(i);

        for ( int j = 0; j < cols; ++j )
        {
            d = pDisp[j];

            if ( d > 0 )
            {
                eCoor(0, idx) = j;
                eCoor(1, idx) = i;
                eCoor(2, idx) = d;
                eCoor(3, idx) = 1.0f;
            }
            else
            {
                eCoor(0, idx) =  0.0f;
                eCoor(1, idx) =  0.0f;
                eCoor(2, idx) = -1.0f;
                eCoor(3, idx) =  1.0f;
            }

            idx++;
        }
    }

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> eCoorWorld(4, nPixels);

    eCoorWorld = eQ_NC * eCoor;

    float dOverB;
    int r, c;

    for ( int i = 0; i < nPixels; ++i )
    {
        if ( eCoorWorld(2, i) * factor > 0 )
        {
            dOverB = eCoorWorld(3, i);

            vertexCoor.push_back({
                eCoorWorld(0, i) / dOverB,
                eCoorWorld(1, i) / dOverB,
                eCoorWorld(2, i) / dOverB });

            r = i / cols;
            c = i % cols;

            pColor = colorRGB.ptr<unsigned char>(r) + c*3;

            vertexColor.push_back({
                pColor[0], pColor[1], pColor[2]
            });
        }
    }

    // Debug.
    std::cout << "Size of coordinates = " << vertexCoor.size() << "." << std::endl;
    std::cout << "Size of colors = " << vertexColor.size() << "." << std::endl;

    // Create a buffer.
    std::filebuf binBuffer;
    if ( true == binary )
    {
        binBuffer.open( fn, std::ios::out | std::ios::binary );
    }
    else
    {
        binBuffer.open( fn, std::ios::out );
    }

    // Create the ostream.
    std::ostream ofs(&binBuffer);
    if ( ofs.fail() )
    {
        throw std::runtime_error("Fail to open " + fn + " for output.");
    }

    // Create the PlyFile object.
    tinyply::PlyFile plyFile;

    plyFile.add_properties_to_element("vertex", { "x", "y", "z" },
            tinyply::Type::FLOAT32,
            vertexCoor.size(),
            reinterpret_cast<uint8_t*>(vertexCoor.data()),
            tinyply::Type::INVALID, 0);

    plyFile.add_properties_to_element("vertex", { "red", "green", "blue" },
            tinyply::Type::UINT8,
            vertexColor.size(),
            reinterpret_cast<uint8_t*>(vertexColor.data()),
            tinyply::Type::INVALID, 0);

    plyFile.get_comments().push_back("generated by tinyply 2.2");

    plyFile.write( ofs, binary );
}