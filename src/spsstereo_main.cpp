/*
    Copyright (C) 2014  Koichiro Yamaguchi

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "SPSStereo.h"
#include "defParameter.h"

#include "nlohmann/json.hpp"

#include "StereoUtils/ImageOutput.hpp"
#include "StereoUtils/PLY.hpp"

namespace bpo = boost::program_options;
namespace bpt = boost::posix_time;

void makeSegmentBoundaryImage(const cv::Mat & inputImage,
							  const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
							  std::vector< std::vector<int> >& boundaryLabels,
							  cv::Mat& segmentBoundaryImage);
void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename);
void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename);

void create_disp_range_maps(int startIdx, int endIdx, int rows, int cols, cv::Mat& pixelDispIdxStart, cv::Mat& pixelDispIdxEnd);

template<typename T>
void read_Q(const std::string& fn, T* array, bool flip=false)
{
    // Open the file.
    std::ifstream ifs(fn);

    if ( !ifs.good() )
    {
        std::stringstream ss;
        ss << "File " << fn << " not good.";
        throw std::runtime_error(ss.str());
    }

    for ( int i = 0; i < 16; ++i )
    {
        ifs >> array[i];
    }

    if ( flip )
    {
        array[5]  *= -1;
        array[7]  *= -1;
        array[11] *= -1;
    }

    ifs.close();
}

/**
 * This function is copied from
 * https://www.boost.org/doc/libs/1_60_0/libs/program_options/example/options_description.cpp
 *
 * @tparam T
 * @param os
 * @param v
 * @return
 */
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    return os;
}

int main(int argc, char* argv[]) {
    std::string inputJSON;

    try
    {
        bpo::options_description optDesc("Allowed options");
        optDesc.add_options()
                ("help", "produce help message")
                ("input-file", bpo::value< std::string >(&inputJSON), "input file");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add("input-file", -1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);

//        if ( optVM.count("input-file") )
//        {
//            std::cout << "input-file is " << optVM["input-file"].as< std::string>() << std::endl;
//        }

    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        return 1;
    }

    std::cout << "inputJson: " << inputJSON << std::endl;

    // Parse the JSON file.
    std::ifstream ifs(inputJSON);
    nlohmann::json J = nlohmann::json::parse(ifs);

    std::cout << "L: " << J["images"]["L"] << std::endl;
    std::cout << "R: " << J["images"]["R"] << std::endl;
    
    std::string leftImageFilename = J["images"]["L"];
    std::string rightImageFilename = J["images"]["R"];

    std::string outputBaseFilename;

    // Prepare the filename.
    boost::filesystem::path leftImageFilenameBoost(leftImageFilename);
    boost::filesystem::path imageFilename = leftImageFilenameBoost.stem();
    boost::filesystem::path outputDir = leftImageFilenameBoost.parent_path();

    outputBaseFilename = outputDir.string() + "/" + imageFilename.string();

    cv::Mat leftImage;
    cv::Mat disparityImage;

    std::cout << "Procesing : " << leftImageFilename << std::endl;
    leftImage = cv::imread(leftImageFilename, cv::IMREAD_UNCHANGED);
    cv::Mat rightImage = cv::imread(rightImageFilename, cv::IMREAD_UNCHANGED);

    // Handle disparity range.
    std::cout << "Minimum disparity: " << J["stereo"]["minDisparity"] << ", "
              << "Maximum disparity: " << J["stereo"]["maxDisparity"] << std::endl;

    cv::Mat pixelDispIdxStart, pixelDispIdxEnd;
    create_disp_range_maps(J["stereo"]["minDisparity"], J["stereo"]["maxDisparity"], leftImage.rows, leftImage.cols, pixelDispIdxStart, pixelDispIdxEnd);

    SPSStereo sps;
    if ( true == J["stereo"]["debug"] )
    {
        sps.enable_debug(outputDir.string());
    }

    sps.set_SGM_disparity_total(static_cast<int>(J["stereo"]["maxDisparity"]) + 1);
    sps.set_SGM_disparity_min(static_cast<int>(J["stereo"]["minDisparity"]));
    sps.setIterationTotal(outerIterationTotal, innerIterationTotal);
    sps.setWeightParameter(lambda_pos, lambda_depth, lambda_bou, lambda_smo);
    sps.setInlierThreshold(lambda_d);
    sps.setPenaltyParameter(lambda_hinge, lambda_occ, lambda_pen);

//    sps.set_SGM_mode( SPSStereo::ORI_SGM );
    sps.set_SGM_mode( SPSStereo::OCV_SGM );

    cv::Mat segmentImage;
//        cv::Mat disparityImage;
    std::vector< std::vector<double> > disparityPlaneParameters;
    std::vector< std::vector<int> > boundaryLabels;

    bpt::ptime start = bpt::microsec_clock::universal_time();

    sps.compute(superpixelTotal, leftImage, rightImage, segmentImage, disparityImage, disparityPlaneParameters, boundaryLabels,
            pixelDispIdxStart.ptr<int>(0), pixelDispIdxEnd.ptr<int>(0));

    bpt::ptime end =  bpt::microsec_clock::universal_time();
    bpt::time_duration td = end - start;

    std::cout <<  "Total time of sps.compute(): "  << td.total_milliseconds() <<  " ms."  << std::endl;

    cv::Mat segmentBoundaryImage;
    makeSegmentBoundaryImage(leftImage, segmentImage, boundaryLabels, segmentBoundaryImage);

    std::string outputDisparityImageFilename = outputBaseFilename + "_left_disparity.png";
    std::string outputSegmentImageFilename = outputBaseFilename + "_segment.png";
    std::string outputBoundaryImageFilename = outputBaseFilename + "_boundary.png";
    std::string outputDisparityPlaneFilename = outputBaseFilename + "_plane.txt";
    std::string outputBoundaryLabelFilename = outputBaseFilename + "_label.txt";

    cv::imwrite(outputDisparityImageFilename, disparityImage);
    cv::imwrite(outputSegmentImageFilename, segmentImage);
    cv::imwrite(outputBoundaryImageFilename, segmentBoundaryImage);
    writeDisparityPlaneFile(disparityPlaneParameters, outputDisparityPlaneFilename);
    writeBoundaryLabelFile(boundaryLabels, outputBoundaryLabelFilename);

    std::cout << "disparityImage.type() = " << disparityImage.type() << "." << std::endl;
    cv::FileStorage file(outputBaseFilename + "_Disparity.yml", cv::FileStorage::WRITE);
    file << "disparityImage" << disparityImage;

    // Output floating point image.
//        write_2_float_image(outputBaseFilename + "_float.png", disparityImage, 140, 180);
    write_2_float_image(outputBaseFilename + "_float.png", disparityImage, 1, 256);

    cv::Mat normalizedSegmentImage;
    normalize(segmentImage, normalizedSegmentImage);

    normalizedSegmentImage = normalizedSegmentImage * 255.0;

    cv::imwrite(outputBaseFilename + "_segment_normalized.png", normalizedSegmentImage);
    cv::FileStorage fileNormalizedSegmentImage("NormalizedSegmentImage.yml", cv::FileStorage::WRITE);
    fileNormalizedSegmentImage << "normalizedSegmentImage" << normalizedSegmentImage;

    float rpjMatrix[16];
//    {
//            1.0,  0.0,                      0.0, -5.068476486206054688e+02,
//            0.0,  1.0,                      0.0, -3.699920768737792969e+02,
//            0.0,  0.0,                      0.0,  1.202625958748662015e+03,
//            0.0,  0.0, 2.611153635327156941e+00,  0.0
//    };

    memset( rpjMatrix, 0, 16 * sizeof(float) );

    // Read the Q matrix.
    std::string camCalibDir = J["camera"]["calibrationDir"];
    std::string camQMat     = J["camera"]["Q"];
    std::cout << "Camera calibration directory: " << camCalibDir << "." << std::endl;
    read_Q<float>( camCalibDir + "/" + camQMat, rpjMatrix, false );

    cv::Mat dispFloat;
    disparityImage.convertTo(dispFloat, CV_32FC1);

    write_ply_with_color(outputBaseFilename + "_Cloud.ply", dispFloat, leftImage, rpjMatrix, true, false);

    return 0;
}


void makeSegmentBoundaryImage(const cv::Mat & inputImage,
							  const /*png::image<png::gray_pixel_16>*/ cv::Mat & segmentImage,
							  std::vector< std::vector<int> >& boundaryLabels,
							  cv::Mat& segmentBoundaryImage)
{
	int width = static_cast<int>(inputImage.cols);
	int height = static_cast<int>(inputImage.rows);
	int boundaryTotal = static_cast<int>(boundaryLabels.size());

	segmentBoundaryImage.create(height, width, CV_8UC3);
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			segmentBoundaryImage.at<cv::Vec3b>(y,x) = inputImage.at<cv::Vec3b>(y, x);
		}
	}

	int boundaryWidth = 2;
	for (int y = 0; y < height - 1; ++y) {
		for (int x = 0; x < width - 1; ++x) {
			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);

			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = cv::Vec3b(128, 128, 128);
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) =  cv::Vec3b(128, 128, 128);
				}
			}
			if (segmentImage.at<uint16_t>( y + 1, x) != pixelLabelIndex) {
				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) =  cv::Vec3b(128, 128, 128);
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (y + w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = cv::Vec3b(128, 128, 128);
				}
			}
		}
	}

	boundaryWidth = 7;
	for (int y = 0; y < height - 1; ++y) {
		for (int x = 0; x < width - 1; ++x) {
			int pixelLabelIndex = segmentImage.at<uint16_t>(y, x);

			if (segmentImage.at<uint16_t>(y, x + 1) != pixelLabelIndex) {
				cv::Vec3b negativeSideColor, positiveSideColor;
				int pixelBoundaryIndex = -1;
				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y, x + 1))
						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y, x + 1) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
					{
						pixelBoundaryIndex = boundaryIndex;
						break;
					}
				}
				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
				} else {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
				}

				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (x - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y, x - w) = negativeSideColor;
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (x + w < width) segmentBoundaryImage.at<cv::Vec3b>(y, x + w) = positiveSideColor;
				}
			}
			if (segmentImage.at<uint16_t>(y + 1, x) != pixelLabelIndex) {
                cv::Vec3b negativeSideColor, positiveSideColor;
				int pixelBoundaryIndex = -1;
				for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
					if ((boundaryLabels[boundaryIndex][0] == pixelLabelIndex && boundaryLabels[boundaryIndex][1] == segmentImage.at<uint16_t>(y + 1, x))
						|| (boundaryLabels[boundaryIndex][0] == segmentImage.at<uint16_t>(y + 1, x) && boundaryLabels[boundaryIndex][1] == pixelLabelIndex))
					{
						pixelBoundaryIndex = boundaryIndex;
						break;
					}
				}
				if (boundaryLabels[pixelBoundaryIndex][2] == 3) continue;
				else if (boundaryLabels[pixelBoundaryIndex][2] == 2) {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 225;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 225;  positiveSideColor.val[0] = 0;
				} else if (pixelLabelIndex == boundaryLabels[pixelBoundaryIndex][boundaryLabels[pixelBoundaryIndex][2]]) {
					negativeSideColor.val[2] = 225;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 0;
					positiveSideColor.val[2] = 0;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 225;
				} else {
					negativeSideColor.val[2] = 0;  negativeSideColor.val[1] = 0;  negativeSideColor.val[0] = 225;
					positiveSideColor.val[2] = 225;  positiveSideColor.val[1] = 0;  positiveSideColor.val[0] = 0;
				}

				for (int w = 0; w < boundaryWidth - 1; ++w) {
					if (y - w >= 0) segmentBoundaryImage.at<cv::Vec3b>(y - w, x) = negativeSideColor;
				}
				for (int w = 1; w < boundaryWidth; ++w) {
					if (y+ w < height) segmentBoundaryImage.at<cv::Vec3b>(y + w, x) = positiveSideColor;
				}
			}
		}
	}
}

void writeDisparityPlaneFile(const std::vector< std::vector<double> >& disparityPlaneParameters, const std::string outputDisparityPlaneFilename) {
	std::ofstream outputFileStream(outputDisparityPlaneFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open file (" << outputDisparityPlaneFilename << ")" << std::endl;
		exit(0);
	}

	int segmentTotal = static_cast<int>(disparityPlaneParameters.size());
	for (int segmentIndex = 0; segmentIndex < segmentTotal; ++segmentIndex) {
		outputFileStream << disparityPlaneParameters[segmentIndex][0] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][1] << " ";
		outputFileStream << disparityPlaneParameters[segmentIndex][2] << std::endl;
	}

	outputFileStream.close();
}

void writeBoundaryLabelFile(const std::vector< std::vector<int> >& boundaryLabels, const std::string outputBoundaryLabelFilename) {
	std::ofstream outputFileStream(outputBoundaryLabelFilename.c_str(), std::ios_base::out);
	if (outputFileStream.fail()) {
		std::cerr << "error: can't open output file (" << outputBoundaryLabelFilename << ")" << std::endl;
		exit(1);
	}

	int boundaryTotal = static_cast<int>(boundaryLabels.size());
	for (int boundaryIndex = 0; boundaryIndex < boundaryTotal; ++boundaryIndex) {
		outputFileStream << boundaryLabels[boundaryIndex][0] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][1] << " ";
		outputFileStream << boundaryLabels[boundaryIndex][2] << std::endl;
	}
	outputFileStream.close();
}

void create_disp_range_maps(int startIdx, int endIdx, int rows, int cols, cv::Mat& pixelDispIdxStart, cv::Mat& pixelDispIdxEnd)
{
    if ( startIdx < 0 || endIdx < 0 || startIdx > endIdx)
    {
        std::stringstream ss;
        ss << "startIdx or endIdx is wrong. startIdx = " << startIdx << ", "
           << "endIdx = " << endIdx << ".";
        throw std::runtime_error(ss.str());
    }

    if ( rows < 0 || cols < 0 )
    {
        std::stringstream ss;
        ss << "rows or cols is wrong. rows = " << rows << ", "
           << "cols = " << cols << ".";
        throw std::runtime_error(ss.str());
    }

    // Create Mat.
    pixelDispIdxStart.create(rows, cols, CV_32SC1);
    pixelDispIdxEnd.create(rows, cols, CV_32SC1);

    // Set value.
    pixelDispIdxStart.setTo(cv::Scalar::all(startIdx));
    pixelDispIdxEnd.setTo(cv::Scalar::all(endIdx));
}