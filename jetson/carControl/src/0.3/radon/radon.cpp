#include "radon.h"

//using namespace cv;

namespace radon
{
	void radon(const Mat& image, Mat& image_radon, int theta = 180, int edge = 10)
	{
		Mat gray_img;
//		cout << "convert to gray" << endl;
		if (image.channels() == 3)
		{
			waveletTransform(image, gray_img, 0.1);
//			cvtColor(image, gray_img, CV_BGR2GRAY);
//			cout << "color" << endl;
		}
		else gray_img = image;
		for (int i = 0; i < edge; i++)
		{
			gray_img.row(i) -= gray_img.row(i);
			gray_img.row(gray_img.rows - 1 - i) -= gray_img.row(gray_img.rows - 1 - i);
			gray_img.col(i) -= gray_img.col(i);
			gray_img.col(gray_img.cols - 1 - i) -= gray_img.col(gray_img.cols - 1 - i);
		}
		imshow("gray", gray_img);
		resize(gray_img, gray_img, Size(ksize, ksize));
//		waitKey(0);
//		cout << "padding" << endl;
//		int new_width = int(sqrt(gray_img.rows * gray_img.rows + gray_img.cols * gray_img.cols)) + 1;
		int new_width = int( sqrt(2) * max(gray_img.rows, gray_img.cols)) + 1;
		int top_pad = (new_width - gray_img.rows) / 2;
		int left_pad = (new_width - gray_img.cols) / 2;
		int down_pad = new_width - top_pad - gray_img.rows;
		int right_pad = new_width - left_pad - gray_img.cols;
		Mat img_padded;
		copyMakeBorder(gray_img, img_padded, top_pad, down_pad, left_pad, right_pad, BORDER_CONSTANT, 0);
//		cout << new_width << " "<< top_pad + down_pad << " " << left_pad + right_pad << " " <<img_padded.rows << " " << img_padded.cols << endl; 
//		imshow("padded_img", img_padded);
//		resize(img_padded, img_padded, Size(ksize, ksize));
		Point2f center(img_padded.cols / 2.0, img_padded.rows / 2.0);
//		cout << new_width << " " << gray_img.size() << " " << img_padded.size() << endl;
		image_radon = Mat(img_padded.rows, theta, CV_32FC1);
		image_radon.setTo(0);
		image_radon.convertTo(image_radon, CV_32F);
//		cout << "compute radon" << endl;
		for (int alpha = 1; alpha <= theta; alpha++)
		{
			Mat rotate_matrix = getRotationMatrix2D(center, -alpha, 1.0);
			Mat rotated_img;
			warpAffine(img_padded, rotated_img, rotate_matrix, img_padded.size());
//			cout << alpha << endl;
//			imshow("rotated", rotated_img);
//			waitKey(0);
			rotated_img.convertTo(rotated_img, CV_32F);
			for ( int i = 1; i < rotated_img.rows; i++)
			{
				rotated_img.row(0) += rotated_img.row(i);
			}
			Mat temp = rotated_img.row(0);
			temp = temp.t();
			temp.copyTo(image_radon.col(alpha - 1));
		}
		double min, max;
		minMaxLoc(image_radon, &min, &max);
		image_radon /= max;
		image_radon *= 255;
//		cout << image_radon << endl;
//		imshow("padded", image_radon);
//		waitKey(0);
	}
	
	
	void radon(const Mat& image, Mat& image_radon, int theta = 180, int edge = 10, bool scale = false)
	{
		Mat gray_img;
		if (image.channels() == 3)
		{
			waveletTransform(image, gray_img, 0.1);
		}
		else gray_img = image;
		for (int i = 0; i < edge; i++)
		{
			gray_img.row(i) -= gray_img.row(i);
			gray_img.row(gray_img.rows - 1 - i) -= gray_img.row(gray_img.rows - 1 - i);
			gray_img.col(i) -= gray_img.col(i);
			gray_img.col(gray_img.cols - 1 - i) -= gray_img.col(gray_img.cols - 1 - i);
		}
		imshow("gray", gray_img);
//		waitKey(0);
		int new_width = int( sqrt(2) * max(gray_img.rows, gray_img.cols)) + 1;
		int top_pad = (new_width - gray_img.rows) / 2;
		int left_pad = (new_width - gray_img.cols) / 2;
		int down_pad = new_width - top_pad - gray_img.rows;
		int right_pad = new_width - left_pad - gray_img.cols;
		Mat img_padded;
		copyMakeBorder(gray_img, img_padded, top_pad, down_pad, left_pad, right_pad, BORDER_CONSTANT, 0);
		Point2f center(img_padded.cols / 2.0, img_padded.rows / 2.0);
		image_radon = Mat(img_padded.rows, theta, CV_32FC1);
		image_radon.setTo(0);
		image_radon.convertTo(image_radon, CV_32F);
		for (int alpha = 1; alpha <= theta; alpha++)
		{
			Mat rotate_matrix = getRotationMatrix2D(center, -alpha, 1.0);
			Mat rotated_img;
			warpAffine(img_padded, rotated_img, rotate_matrix, img_padded.size());
			rotated_img.convertTo(rotated_img, CV_32F);
			for ( int i = 1; i < rotated_img.rows; i++)
			{
				rotated_img.row(0) += rotated_img.row(i);
			}
			Mat temp = rotated_img.row(0);
			temp = temp.t();
			temp.copyTo(image_radon.col(alpha - 1));
		}
		double min, max;
		minMaxLoc(image_radon, &min, &max);
		image_radon /= max;
		image_radon *= 255;
	}

	void rsignature(const Mat& radon_img, Mat& rsignature)
	{
//		cout << "begin rsign" << endl;
		Mat radon;
		double max, min;
//		minMaxLoc(radon_img, &min, &max);
//		radon = radon_img / double(max);
		multiply(radon_img, radon_img, radon);
		rsignature = Mat(1, radon.cols, CV_32F);
		rsignature.setTo(0);
		for ( int i = 0; i < radon.rows; i++)
                {
			rsignature += radon.row(i);
                }
//		reduce(radon, rsignature, 0, CV_REDUCE_SUM, -1);
//		double max, min;
//		rsignature.convertTo(rsignature, CV_32F);
//		cout << rsignature << endl;
		float mean = 0;
		for (int i = 0; i < rsignature.cols; i++)
			mean += rsignature.at<float>(0,i);
//		minMaxLoc(rsignature, &min, &max);
		rsignature /= mean / 180;
//		cout << "end rsign" << endl;
//		cout << rsignature << endl;
	}
	
	double distance(const Mat& r1, const Mat& r2)
	{
		Mat error = r1 - r2;
		error.convertTo(error, CV_32F);
		multiply(error, error, error);
//		error.convertTo(error, CV_32F);
		float err = 0;
		for (int i = 0; i < error.cols; i++)
			err += error.at<float>(0,i);
		return err;
	}
}
