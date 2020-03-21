#include "extractInfo.h"
#include "radon.h"
#include <fstream>
#include <stdlib.h>
#include <math.h>

namespace extract
{
	bool sameRegion(Rect r1, Rect r2)
	{
		int width, height;
		width = max(r1.x + r1.width, r2.x + r2.width) + 5;
		height = max(r2.y + r2.height, r1.y + r1.height) + 5;
		Mat mark = Mat::zeros(height, width, CV_8UC1);
		rectangle(mark, r1, Scalar(255), 1, 8, 0);
		rectangle(mark, r2, Scalar(255), 1, 8, 0);
		vector<vector<Point> > contours;
		findContours(mark, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0));
		if (contours.size() > 1) return false;
		Point p1 = Point(-1, -1), p2 = Point(-1, -1);
		for (int i = 0; i < contours[0].size(); i++)
		{
			if (p1.x == -1 || p1.x > contours[0][i].x) p1.x = contours[0][i].x;
			if (p1.y == -1 || p1.y > contours[0][i].y) p1.y = contours[0][i].y;
			if (p2.x == -1 || p2.x < contours[0][i].x) p2.x = contours[0][i].x;
			if (p2.y == -1 || p2.y < contours[0][i].y) p2.y = contours[0][i].y;

		}
		int h = p2.y - p1.y, w = p2.x - p1.x;
		long s = h * w;
		if (r1.width * r1.height >= s * 0.8 || r2.width * r2.height >= s * 0.8) return true;
		return false;
	}
	
	void extractRegions(const Mat& img, vector<Rect> &boxes)
	{
		Mat gray;
		int edge = 5;
		Mat color = img.clone();
		resize(color, color, Size(150, 150));
		if (img.channels() == 3)
		{
			waveletTransform(color, gray, 0.1);
		} else gray = color.clone();
		for (int i = 0; i < edge; i++)
		{
			gray.row(i) -= gray.row(i);
			gray.row(gray.rows - 1 - i) -= gray.row(gray.rows - i - 1);
			gray.col(i) -= gray.col(i);
			gray.col(gray.cols - 1 - i) -= gray.col(gray.cols - i -1);
		}
//		namedWindow("gray", WINDOW_NORMAL);
//		imshow("gray", gray);
//		waitKey(0);
		int morph_size = 1;
		Mat element = getStructuringElement(0, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
//		cout << element << endl;
//		dilate(gray, gray, element);
//		erode(gray, gray, element);
//		imshow("morpholygy", gray);
//		waitKey(0);
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(gray, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		boxes.clear();
		int pad = 5;
		Mat origin = color.clone();
		Point lt(-1, -1), rb(-1, -1);
		vector<int> ptsOfBox;
		ptsOfBox.clear();
		for (int i = 0; i < contours.size(); i ++)
		{
			Point p1, p2;
			p1 = contours[i][0];
			p2 = contours[i][0];
			if (contours[i].size() > 200) continue;
			for (int j = 0; j < contours[i].size(); j++)
			{
				if (p1.x > contours[i][j].x) p1.x = contours[i][j].x;
				if (p1.y > contours[i][j].y) p1.y = contours[i][j].y;
				if (p2.x < contours[i][j].x) p2.x = contours[i][j].x;
				if (p2.y < contours[i][j].y) p2.y = contours[i][j].y;
			}
			int left = 0, right = 0, top = 0, bot = 0;
			if (p1.x - pad >= 0) p1.x -= pad;
			if (p1.y - pad >= 0) p1.y -= pad;
			if (p2.x + pad < color.cols) p2.x += pad;
			if (p2.y + pad < color.rows) p2.y += pad;
			Rect rect = Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
			if (rect.height * 0.25 >  rect.width || rect.height * 0.85 < rect.width) continue;
			boxes.push_back(rect);
			ptsOfBox.push_back(contours[i].size());
		}
		vector<int> labels;
		int k = partition(boxes, labels, sameRegion);
		k = boxes.size();
		vector<Rect> trueBoundary;
		trueBoundary.clear();
		vector<int> ptsOfBox2;
		ptsOfBox2.clear();
		for (int i = 0; i < k; i++)
		{
			int maxComponent = 0;
			Point p1 = Point(-1, -1), p2 = Point(-1, -1);
			for (int j = 0; j < boxes.size(); j ++)
			{
				if (labels[j] == i)
				{
					if (p1.x == -1 || p1.x > boxes[j].x) p1.x = boxes[j].x;
					if (p1.y == -1 || p1.y > boxes[j].y) p1.y = boxes[j].y;
					if (p2.x == -1 || p2.x < boxes[j].x + boxes[j].width) p2.x = boxes[j].x + boxes[j].width;
					if (p2.y == -1 || p2.y < boxes[j].y + boxes[j].height) p2.y = boxes[j].y + boxes[j].height;
					if (ptsOfBox[j] > maxComponent) maxComponent = ptsOfBox[j];
				}
			}
			Rect rec = Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
			if (rec.width < 0.15 * 150 || rec.width > 0.42 * 150 || maxComponent <= 30 || rec.x == 0 || rec.y == 0) continue; 
			trueBoundary.push_back(rec);
			ptsOfBox2.push_back(maxComponent);
			rectangle(color, rec, Scalar(255, 0, 0), 1, 8, 0);
		}
		boxes = trueBoundary;
		ptsOfBox = ptsOfBox2;
		if (boxes.size() > 0)
		{
			for (int i = 0; i < boxes.size() - 1; i++)
				for (int j = i+1; j < boxes.size(); j++)
				{
				if (boxes[i].x > boxes[j].x)
				{
					Rect rTemp = boxes[i];
					boxes[i] = boxes[j];
					boxes[j] = rTemp;
					int temp = ptsOfBox[i];
					ptsOfBox[i] = ptsOfBox[j];
					ptsOfBox[j] = temp;
				}
			}
		}
	}
	

	void extract(const Mat& img, string& output, string method)
	{
		if (method != "debug") method = "templateMatching";
		Mat gray;
//		int edge = 5;
//		output = "";
		Mat color = img.clone();
		resize(color, color, Size(150, 150));
//		cout << "convert to grayscale..." << endl;
		if (img.channels() == 3)
		{
			waveletTransform(color, gray, 0.1);
		} else gray = color.clone();
//		for (int i = 0; i < edge; i++)
//		{
//			gray.row(i) -= gray.row(i);
//			gray.row(gray.rows - 1 - i) -= gray.row(gray.rows - i - 1);
//			gray.col(i) -= gray.col(i);
//			gray.col(gray.cols - 1 - i) -= gray.col(gray.cols - i -1);
//		}
//		cout << "resizing..." << endl;
//		namedWindow("gray", WINDOW_NORMAL);
//		resize(gray, gray, Size(150, 150));
//		gray = 255 - gray;
//		imshow("gray", gray);
//		waitKey(0);
//		int morph_size = 1;
//		Mat element = getStructuringElement(0, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
//		cout << element << endl;
//		dilate(gray, gray, element);
//		erode(gray, gray, element);
//		imshow("morpholygy", gray);
//		waitKey(0);
//		vector<vector<Point> > contours;
//		vector<Vec4i> hierarchy;
//		findContours(gray, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
//		cout << "find contours" << endl;
//		vector<Rect> boxes;
//		boxes.clear();
		int pad = 5;
//		Mat color = img.clone();
//		cout << contours.size() << endl;
		Mat origin = color.clone();
//		Point lt(-1, -1), rb(-1, -1);
//		vector<int> ptsOfBox;
//		ptsOfBox.clear();
//		for (int i = 0; i < contours.size(); i ++)
//		{
//			Point p1, p2;
//			p1 = contours[i][0];
//			p2 = contours[i][0];
//			if (contours[i].size() > 200) continue;
//			for (int j = 0; j < contours[i].size(); j++)
//			{
//				if (p1.x > contours[i][j].x) p1.x = contours[i][j].x;
//				if (p1.y > contours[i][j].y) p1.y = contours[i][j].y;
//				if (p2.x < contours[i][j].x) p2.x = contours[i][j].x;
//				if (p2.y < contours[i][j].y) p2.y = contours[i][j].y;
//			}
//			int left = 0, right = 0, top = 0, bot = 0;
//			if (p1.x - pad >= 0) p1.x -= pad;
//			if (p1.y - pad >= 0) p1.y -= pad;
//			if (p2.x + pad < color.cols) p2.x += pad;
//			if (p2.y + pad < color.rows) p2.y += pad;
//			Rect rect = Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
//			if (rect.height * 0.25 >  rect.width || rect.height * 0.85 < rect.width) continue;
//			boxes.push_back(rect);
//			ptsOfBox.push_back(contours[i].size());
//		}
//		vector<int> labels;
//		int k = partition(boxes, labels, sameRegion);
//		k = boxes.size();
//		vector<Rect> trueBoundary;
//		trueBoundary.clear();
//		vector<int> ptsOfBox2;
//		ptsOfBox2.clear();
//		for (int i = 0; i < k; i++)
//		{
//			int maxComponent = 0;
//			Point p1 = Point(-1, -1), p2 = Point(-1, -1);
//			for (int j = 0; j < boxes.size(); j ++)
//			{
//				if (labels[j] == i)
//				{
//					if (p1.x == -1 || p1.x > boxes[j].x) p1.x = boxes[j].x;
//					if (p1.y == -1 || p1.y > boxes[j].y) p1.y = boxes[j].y;
//					if (p2.x == -1 || p2.x < boxes[j].x + boxes[j].width) p2.x = boxes[j].x + boxes[j].width;
//					if (p2.y == -1 || p2.y < boxes[j].y + boxes[j].height) p2.y = boxes[j].y + boxes[j].height;
//					if (ptsOfBox[j] > maxComponent) maxComponent = ptsOfBox[j];
//				}
//			}
//			Rect rec = Rect(p1.x, p1.y, p2.x - p1.x, p2.y - p1.y);
//			if (rec.width < 0.15 * 150 || rec.width > 0.42 * 150 || maxComponent <= 30 || rec.x == 0 || rec.y == 0) continue; 
//			trueBoundary.push_back(rec);
//			ptsOfBox2.push_back(maxComponent);
//			rectangle(color, rec, Scalar(255, 0, 0), 1, 8, 0);
//		}
//		boxes = trueBoundary;
//		ptsOfBox = ptsOfBox2;
//
//		if (boxes.size() > 0)
//		{
//			for (int i = 0; i < boxes.size() - 1; i++)
//				for (int j = i+1; j < boxes.size(); j++)
//				{
//				if (boxes[i].x > boxes[j].x)
//				{
//					Rect rTemp = boxes[i];
//					boxes[i] = boxes[j];
//					boxes[j] = rTemp;
//					int temp = ptsOfBox[i];
//					ptsOfBox[i] = ptsOfBox[j];
//					ptsOfBox[j] = temp;
//				}
//			}
//		} else 
//		{
//			output = "";
//			return;
//		}
//		cout << "sorted" << endl;
		int border = 4;
		vector<Rect> boxes;
		extractRegions(color, boxes);
/*		if (method == "debug")
		{
			for (int i = 0; i < boxes.size(); i++)
			{
				cout << "box " << i << ": (x, y) = (" << boxes[i].x << ", " <<boxes[i].y << "), size = " << boxes[i].size() << ", numOfPts = " << ptsOfBox[i] << ", ratio = " << boxes[i].width * 1.0 / 150 << endl;
			rectangle(color, boxes[i], Scalar(255, 0, 0), 1, 8, 0);
			}
		}
*/
		if (method == "radon")
		{
			ifstream input("/home/ubuntu/data/template/template.txt");
			int numOfTemp;
			input >> numOfTemp;
			string path;
			input >> path;
			int theta;
			input >> theta;
			Mat rsignature = Mat::zeros(numOfTemp, theta, CV_32F);
			for (int i = 0; i < numOfTemp; i++)
			{
				string filename;
				int label;
				input >> label >> filename;
				Mat temp = imread(path + filename);
				copyMakeBorder(temp, temp, border, border, border, border, BORDER_CONSTANT, 0);
				Mat radonImg;
				radon::radon(temp, radonImg, theta, 3, false);
				Mat rsign;
				radon::rsignature(radonImg, rsign);
				rsignature.row(label) += rsign;
			}
			for (int i = 0; i < boxes.size(); i++)
			{
				Mat roi = gray(boxes[i]);
				char ch;
//				imshow("roi", roi);
//				waitKey(0);
				radonExtract(roi, ch, rsignature, pad - 1);
				output += ch;
			}
		} else
		if (method == "templateMatching")
		{	
			ifstream input("/home/ubuntu/data/template/template.txt");
			int numOfTemp;
			input >> numOfTemp;
			string path;
			input >> path;
			int theta;
			input >> theta;
			vector<Mat> templs;
			border = 4;
			for (int i = 0; i < numOfTemp; i++)
			{
				string filename;
				int label;
				input >> label >> filename;
				Mat temp = imread(path + filename, 0);
				copyMakeBorder(temp, temp, border, border, border, border, BORDER_CONSTANT, 0);
//				imshow("template_befor", temp);
				temp = 255 - temp;
				cvtColor(temp, temp, CV_GRAY2BGR);
//				imshow("template_after", temp);
//				waitKey(0);
				templs.push_back(temp);
			}
			for (int i = 0; i < boxes.size(); i++)
			{
				Mat roi = origin(boxes[i]);
				char ch;
				templateMatching(roi, ch, templs);
				output += ch;
			}
		}
		cout << output << endl;
//		namedWindow("bounding box", WINDOW_NORMAL);
//		imshow("bounding box", color);
//		waitKey(0);
		
	}
	
	void templateMatching(const Mat& img, char &output, const vector<Mat>& templs)
	{
		Mat result;
		double maxMatch = 0;
		double max, min;
		int label = -1;
		for (int i = 0; i < templs.size(); i++)
		{
			Mat sample;
			resize(img, sample, Size(templs[i].cols, templs[i].rows));
			matchTemplate(sample, templs[i], result, CV_TM_CCORR_NORMED);
//			imshow("sample", sample);
//			imshow("template", templs[i]);
			minMaxLoc(result, &min, &max);
			cout << max << " ";
			if (maxMatch < max)
			{
				maxMatch = max;
				label = i;
			}
//			cout << max << endl;
//			waitKey(0);
		}
		cout << endl;
		output = '0' + label;
	}
	
	void radonExtract(const Mat& img, char &output, const Mat& rsignature, int edge = 3)
	{
		Mat radonImg;
		Mat rsign;
		radon::radon(img, radonImg, rsignature.cols, edge, false);
		radon::rsignature(radonImg, rsign);
		double minE = -1;
		int label;
		for (int i = 0; i < rsignature.rows; i++)
		{
			double error = radon::distance(rsign, rsignature.row(i));
			cout << error << " ";
			if (minE == -1 || minE > error)
			{
				minE = error;
				label = i;
			}
		}
		cout << endl << label << " " <<  minE << endl;
//		imshow("img", img);
//		waitKey(0);
		output = '0' + label;
	}
}
