#include "signsRecognizer.h"
#include <iostream>

using namespace std;

signsRecognizer::signsRecognizer(string filePath)
{
	ifstream input;
	input.open(filePath);
	int numOfTemp;
	int redTemps;
	int blueTemps;
	string path;
	input >> numOfTemp >> redTemps >> blueTemps >> theta >> edgePixs >> path;
//	cout << numOfTemp << " " << redTemps << " " << blueTemps << " " << theta << " " << edgePixs << " " << path << endl;
	string signName;
	redRsignTemps = Mat::zeros(redTemps, theta, CV_32F);
	blueRsignTemps = Mat::zeros(blueTemps, theta, CV_32F);
	redSigns.clear();
	blueSigns.clear();
	int iRed = 0, iBlue = 0;
	int colorLabel;
	while (input >> signName >> colorLabel)
	{
//		cout << signName << " " << colorLabel << endl;
		Mat imgTemp = imread(path + signName + ".jpg");
		Mat radonImg;
		Mat rsignature;
		radon::radon(imgTemp, radonImg, theta, edgePixs);
		radon::rsignature(radonImg, rsignature);
//		signNames.push_back(signName);
		if (colorLabel == red)
		{
			redRsignTemps.row(iRed++) += rsignature;
			redSigns.push_back(signName);
		}
		else 
		{
			blueRsignTemps.row(iBlue++) += rsignature;
			blueSigns.push_back(signName);
		}
	}
}

void signsRecognizer::labeling(const vector<Rect> &boxes, const vector<int> &colorLabels, const Mat &color, vector<int> &signLabels, vector<string> &signs)
{
	signLabels.clear();
	signs.clear();
	double thresh = 5;
	for (int i = 0; i < boxes.size(); i++)
	{
		Mat object = color(boxes[i]);
//		imshow("object", object);
//		waitKey(0);
		Mat radonImg;
		radon::radon(object, radonImg, theta, edgePixs);
		Mat rsignature;
		radon::rsignature(radonImg, rsignature);
		int label = -1;
		double minDis = -1;
		if (colorLabels[i] == red)
		{
			for (int j = 0; j < redRsignTemps.rows;  j++)
			{
				double dis = radon::distance(rsignature, redRsignTemps.row(j));
				if (minDis == -1 || (minDis > dis && dis < thresh))
				{
					label = j;
					minDis = dis;
				}
			}
			if (label >= 0) signs.push_back(redSigns[label]);
		} else
		{
			for (int j = 0; j < blueRsignTemps.rows;  j++)
			{
				double dis = radon::distance(rsignature, blueRsignTemps.row(j));
				if (minDis == -1 || (minDis > dis && dis < thresh))
				{
					label = j;
					minDis = dis;
				}
			}
			if (label >= 0) signs.push_back(blueSigns[label]);
		}
		signLabels.push_back(label);
		if (label == -1) signs.push_back("other");
	}
}
