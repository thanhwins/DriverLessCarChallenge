
#include "Radon.h"
using namespace framework;

Radon::Radon()
{

}

Radon::~Radon() {

}

void Radon::convolution(cv::Mat &res, cv::Mat &img, cv::Mat &kernel, ConvType type) {
	cv::Mat src = img;
	if (CONV_FULL == type) {
		src = cv::Mat();
		const int additional_rows = kernel.rows - 1, additional_cols = kernel.cols - 1;
		cv::copyMakeBorder(img, src, (additional_rows + 1) / 2, additional_rows / 2,
			(additional_cols + 1) / 2, additional_cols / 2,
			cv::BORDER_CONSTANT, cv::Scalar(0));
	}

	cv::Point anchor(kernel.cols - kernel.cols / 2 - 1, kernel.rows - kernel.rows / 2 - 1);
	int border_mode = cv::BORDER_CONSTANT;
	cv::Mat fkernel;
	cv::flip(kernel, fkernel, -1);
	cv::filter2D(src, res, CV_64FC1, fkernel, anchor, 0, border_mode);

	if (CONV_VALID == type) {
		res = res
			.colRange((kernel.cols - 1) / 2, res.cols - kernel.cols / 2)
			.rowRange((kernel.rows - 1) / 2, res.rows - kernel.rows / 2);
	}
}

void Radon::waveletTranform(cv::Mat &res, cv::Mat &img, double threshold = 0.15) {
	cv::Mat src = img;
	if (img.channels() == 3) {
		cv::cvtColor(img, src, CV_BGR2GRAY);
	}
	double pi = M_PI;
	double m = 1.0;
	double dlt = std::pow(2.0, m);
	double x = 0, y = 0;
	double coff = 0;
	int n = 20;
	double a = -1 / sqrt(2 * pi); // M_PI = acos(-1.0)
	cv::Mat phi_x = cv::Mat(n, n, CV_64FC1);
	cv::Mat phi_y = cv::Mat(n, n, CV_64FC1);

	for (int idx = 0; idx < n; idx++) {
		x = idx - (n + 1) / 2.0;
		for (int idy = 0; idy < n; idy++) {
			y = idy - (n + 1) / 2.0;
			coff = a / (dlt * dlt) * exp(-(x*x + y*y) / (2 * dlt*dlt));
			phi_x.at<double>(idx, idy) = (coff * x); // TODO: what the hell?
			phi_y.at<double>(idx, idy) = (coff * y); // TODO: what the hell?
		}
	}
	cv::normalize(phi_x, phi_x);
	cv::normalize(phi_y, phi_y);

	cv::Mat gx, gy;
	this->convolution(gx, src, phi_x, CONV_SAME);
	this->convolution(gy, src, phi_y, CONV_SAME);

	cv::Mat grads = cv::Mat(src.size(), CV_64FC1);
	for (int i = 0; i < gx.rows; i++) {
		for (int j = 0; j < gx.cols; j++) {
			x = gx.at<double>(i, j); // TODO: what the hell?
			y = gy.at<double>(i, j); // TODO: what the hell?
			grads.at<double>(i, j) = sqrt(x*x + y*y);
		}
	}

	double m_esp = 100.0 / (1LL << 52); // matlab esp = 2^-52
	cv::Mat angle_array = cv::Mat::zeros(src.size(), CV_64FC1);
	for (int i = 0; i < src.rows; i++) {
		for (int j = 0; j < src.cols; j++) {
			double p = 90.0;
			if (fabs(gx.at<double>(i, j)) > m_esp) {
				p = atan(gy.at<double>(i, j) / gx.at<double>(i, j)) * 180 / M_PI;
				if (p < 0) p += 360;
				if (gx.at<double>(i, j) < 0 && p > 180) p -= 180;
				else if (gx.at<double>(i, j) < 0 && p < 180) p += 180;
			}
			angle_array.at<double>(i, j) = p;
		}
	}

	cv::Mat edge_array = cv::Mat::zeros(src.size(), CV_64FC1);
	for (int i = 1; i < src.rows - 1; i++) {
		for (int j = 1; j < src.cols - 1; j++) {
			x = angle_array.at<double>(i, j);
			y = grads.at<double>(i, j);
			if (((x >= -W_RANGE) && (x <= W_RANGE)) || ((x >= 180 - W_RANGE) && (x <= 180 + W_RANGE))) {
				if (y > grads.at<double>(i + 1, j) && y > grads.at<double>(i - 1, j)) 
					edge_array.at<double>(i, j) = y;
			}
			else
				if (((x >= 90 - W_RANGE) && (x <= 90 + W_RANGE)) || ((x >= 270 - W_RANGE) && (x <= 270 + W_RANGE))) {
					if (y > grads.at<double>(i, j + 1) && y > grads.at<double>(i, j - 1)) edge_array.at<double>(i, j) = y;
				}
				else
					if (((x >= 45 - W_RANGE) && (x <= 45 + W_RANGE)) || ((x >= 225 - W_RANGE) && (x <= 225 + W_RANGE))) {
						if (y > grads.at<double>(i + 1, j + 1) && y > grads.at<double>(i - 1, j - 1)) edge_array.at<double>(i, j) = y;
					}
					else
						if (y > grads.at<double>(i + 1, j - 1) && y > grads.at<double>(i - 1, j + 1)) edge_array.at<double>(i, j) = y;
		}
	}

	// find max
	double max_e = edge_array.at<double>(0, 0);
	for (int i = 0; i < edge_array.rows; i++) {
		for (int j = 0; j < edge_array.cols; j++) {
			if (max_e < edge_array.at<double>(i, j)) max_e = edge_array.at<double>(i, j);
		}
	}

	res = cv::Mat::zeros(src.size(), CV_8UC1);
	for (int i = 0; i < edge_array.rows; i++) {
		for (int j = 0; j < edge_array.cols; j++) {
			edge_array.at<double>(i, j) /= max_e;
			if (edge_array.at<double>(i, j) > threshold) res.at<uchar>(i, j) = 255;
		}
	}
}

void Radon::preProcessing(cv::Mat &res, cv::Mat &input, int std_size, int edge) {
	cv::Mat gray;
	if (input.channels() == 3) this->waveletTranform(gray, input, 0.1);
	else gray = input.clone();
	for (int i = 0; i < edge; i++) {
		gray.row(i) -= gray.row(i); // clear data?
		gray.row(gray.rows - 1 - i) -= gray.row(gray.rows - 1 - i);
		gray.col(i) -= gray.col(i);
		gray.col(gray.cols - 1 - i) -= gray.col(gray.cols - 1 - i);
	}
	if (gray.cols != std_size || gray.rows != std_size)
		cv::resize(gray, gray, cv::Size(std_size, std_size));

	int w = (int)(SQRT_2 * (double)std_size) + 1;
	int t_pad = (w - std_size) / 2;
	int l_pad = t_pad;
	int d_pad = w - t_pad - std_size;
	int r_pad = d_pad;

	cv::copyMakeBorder(gray, res, t_pad, d_pad, l_pad, r_pad, cv::BORDER_CONSTANT, 0);
}

bool Radon::computeRadon(cv::Mat &res, cv::Mat &img) {
	cv::Mat gray;
	this->preProcessing(gray, img);
	//cv::imshow("gray", gray);
	int i = 0;
#ifdef USE_COL
	res = cv::Mat::zeros(m_range, gray.cols, CV_32FC1);
#else
	res = cv::Mat::zeros(gray.rows, m_range, CV_32FC1);
#endif
	int j = 0;
	for (i = 0; i < m_range; i++) {
#ifdef USE_COL
		double * ran = (double *)std::calloc(gray.cols, sizeof(double));
#else
		double * ran = (double *)std::calloc(gray.rows, sizeof(double));
#endif
		computeRadon(ran, gray, i);
#ifdef USE_COL
		for (j = 0; j < gray.cols; j++) {
			res.at<float>(i, j) = ran[j];
		}
#else
		for (j = 0; j < gray.cols; j++) {
			res.at<float>(j, i) = ran[j];
		}
#endif
	}
	double min, max;
	cv::minMaxLoc(res, &min, &max);
	res /= max;
	res *= 255;
	return true;
}

bool Radon::computeRadon(double *res, cv::Mat &gray, int index) {
	cv::Mat output;
	cv::warpAffine(gray, output, rot_mats[index], gray.size());
	int x, y;
	for (y = 0; y < gray.cols; y++) {
		for (x = 0; x < output.rows; x++) {
			res[y] += output.at<uchar>(y, x);
		}
	}
	return true;
}

bool Radon::initRotateMatrixLookupTable(int &w, int &h, double step, double range) {
	int new_w = (int)(w * SQRT_2) + 1;
	int new_h = (int)(h * SQRT_2) + 1;
	cv::Point center(new_w / 2, new_h / 2);
	double scale = 1.0;
	double angle = 0.0;

	while (angle < range) {
		cv::Mat rot_mat(2, 3, CV_32FC1);
		rot_mat = cv::getRotationMatrix2D(center, angle, scale);
		rot_mats.push_back(rot_mat);
		angle = angle + step;
	}

	m_range = (int)(range / step);
	return true;
}

bool Radon::match(double &res, cv::Mat &r_a, cv::Mat &r_b) {
	cv::Mat m_res = cv::Mat::zeros(1, 1, CV_32FC1);
	cv::matchTemplate(r_a, r_b, m_res, CV_TM_SQDIFF_NORMED);
	std::cout << m_res << std::endl;
	return true;
}

void Radon::computeRSignature(cv::Mat &res, cv::Mat &radon) {
	cv::Mat m_radon;
	cv::multiply(radon, radon, m_radon);
	double mean = 0;
	res = cv::Mat::zeros(1, m_radon.cols, CV_32FC1);
	for (int i = 0; i < m_radon.rows; i++) {
		res += m_radon.row(i);
	}
	for (int i = 0; i < res.cols; i++) {
		mean += res.at<float>(0, i);
	}
	res /= mean / m_range;
}

void Radon::computeDistance(double &res, cv::Mat &img1, cv::Mat &img2) {
	cv::Mat err = img1 - img2;
	cv::multiply(err, err, err);
	res = 0;
	for (int i = 0; i < err.cols; i++) res += err.at<float>(0, i);
}
