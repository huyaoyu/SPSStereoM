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

#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "StereoBase.hpp"

class SGMStereo : public StereoBase {
public:
	SGMStereo();

	void setDisparityTotal(const int disparityTotal);
	void setDisparityFactor(const double disparityFactor);
	void setDataCostParameters(const int sobelCapValue,
							   const int censusWindowRadius,
							   const double censusWeightFactor,
							   const int aggregationWindowRadius);
	void setSmoothnessCostParameters(const int smoothnessPenaltySmall, const int smoothnessPenaltyLarge);
	void setConsistencyThreshold(const int consistencyThreshold);

	void compute(const cv::Mat& leftImage,
				 const cv::Mat& rightImage,
				 float* disparityImage,
				 const int* pixelDispIdxStart = NULL,
				 const int* pixelDispIdxNum = NULL);

private:
	void initialize(const cv::Mat& leftImage, const cv::Mat& rightImage);
	void setImageSize(const cv::Mat& leftImage, const cv::Mat& rightImage);
	void allocateDataBuffer();
	void freeDataBuffer();
	void computeCostImage(const cv::Mat& leftImage, const cv::Mat& rightImage);
	void convertToGrayscale(const cv::Mat& leftImage,
							const cv::Mat& rightImage,
							unsigned char* leftGrayscaleImage,
							unsigned char* rightGrayscaleImage) const;
	void computeLeftCostImage(const unsigned char* leftGrayscaleImage, const unsigned char* rightGrayscaleImage);
	void computeCappedSobelImage(const unsigned char* image, const bool horizontalFlip, unsigned char* sobelImage) const;
	void computeCensusImage(const unsigned char* image, int* censusImage) const;
	void calcTopRowCost(unsigned char*& leftSobelRow, int*& leftCensusRow,
						unsigned char*& rightSobelRow, int*& rightCensusRow,
						unsigned short* costImageRow);
	void calcRowCosts(unsigned char*& leftSobelRow, int*& leftCensusRow,
					  unsigned char*& rightSobelRow, int*& rightCensusRow,
					  unsigned short* costImageRow);
	void calcPixelwiseSAD(const unsigned char* leftSobelRow, const unsigned char* rightSobelRow, int rowIdx);
	void calcHalfPixelRight(const unsigned char* rightSobelRow);
	void addPixelwiseHamming(const int* leftCensusRow, const int* rightCensusRow, int rowIdx);
	void computeRightCostImage();
	void performSGM(unsigned short* costImage, unsigned short* disparityImage);
	void speckleFilter(const int maxSpeckleSize, const int maxDifference, unsigned short* image) const;
	void enforceLeftRightConsistency(unsigned short* leftDisparityImage, unsigned short* rightDisparityImage) const;

	void copy_pixel_disp_range(const int* fromStart, int* toStart,
                               const int* fromEnd, int* toEnd,
                               int base = 16);
	void initialize_pixel_disp_range(int val);
	int disp_start(int row, int col);
    int disp_start_max(int row, int col, int m);
	int disp_end(int row, int col);
	int disp_end_min(int row, int col, int m);
	int disp_end_1(int row, int col);
	int disp_end_1_min(int row, int col, int m);
	int disp_num(int row, int col);
	int disp_start_algined(int row, int col, int base = 16);

	// Parameter
	int disparityTotal_;
	double disparityFactor_;
	int sobelCapValue_;
	int censusWindowRadius_;
	double censusWeightFactor_;
	int aggregationWindowRadius_;
	int smoothnessPenaltySmall_;
	int smoothnessPenaltyLarge_;
	int consistencyThreshold_;

	// Data
	int width_;
	int height_;
	int widthStep_;
	unsigned short* leftCostImage_;
	unsigned short* rightCostImage_;
	unsigned char* pixelwiseCostRow_;
	unsigned short* rowAggregatedCost_;
	unsigned char* halfPixelRightMin_;
	unsigned char* halfPixelRightMax_;
	int pathRowBufferTotal_;
	int disparitySize_;
	int pathTotal_;
	int pathDisparitySize_;
	int costSumBufferRowSize_;
	int costSumBufferSize_;
	int pathMinCostBufferSize_;
	int pathCostBufferSize_;
	int totalBufferSize_;
	short* sgmBuffer_;
	int* pixelDispIdxStart_;
	int* pixelDispIdxEnd_;
};
