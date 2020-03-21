#include "../radon/radon.h"
#include <fstream>
#include <iostream>
#include <cstring>

#define red 0
#define blue 1

using namespace std;

class signsRecognizer
{
public:
	signsRecognizer() {};
	signsRecognizer(string filePath);
	~signsRecognizer(){};
	void labeling(const vector<Rect> &boxes, const vector<int> &colorLabels, const Mat &color, vector<int> &signLabels, vector<string> &signNames);
public:
	Mat redRsignTemps;
	Mat blueRsignTemps;
	vector<string> redSigns, blueSigns;
	int theta;
	int edgePixs;
};
