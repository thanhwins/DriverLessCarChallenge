#pragma once

#ifndef RADON_H
#define RADON_H

#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Singleton.h"

namespace framework {
#define W_RANGE 22.5
#define SQRT_2  1.41421356
#define M_PI 3.14159265358979323846
	enum ConvType {
		CONV_FULL, // Full convolution, including border
		CONV_SAME, // Only the part that corresponds to the original image
		CONV_VALID // Only the submatrix containing elements that were not influenced by the border
	};

	class Radon : public Singleton<Radon> {
		friend class Singleton<Radon>;
	public:
		Radon();
		virtual ~Radon();
		void convolution(cv::Mat &res, cv::Mat &img, cv::Mat &kernel, ConvType type);
		void waveletTranform(cv::Mat &res, cv::Mat &img, double threshold);
		void preProcessing(cv::Mat &res, cv::Mat &input, int std_size = 110, int edge = 10);
		bool computeRadon(cv::Mat &res, cv::Mat &img);
		bool computeRadon(double *res, cv::Mat &gray, int index);
		bool initRotateMatrixLookupTable(int &w, int &h, double step = 1, double range = 180);
		bool match(double &res, cv::Mat &r_a, cv::Mat &r_b);
		void computeRSignature(cv::Mat &res, cv::Mat &radon);
		void computeDistance(double &res, cv::Mat &img1, cv::Mat &img2);
	private:
		int m_range;
		std::vector<cv::Mat> rot_mats;
	};
}


#endif // RADON_H
