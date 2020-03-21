#include "SignProc.h"
#include "fstream"
#include "iostream"

SignProc::OMP::OMP() {
	return ;
}

SignProc::OMP::~OMP() {
	
	return ;
}

void SignProc::OMP::loadDict(char fileName[]) {
	std::ifstream ifs;
	ifs.open(fileName, std::ifstream::in);
	int row, col;
	ifs >> row >> col;
	dictRow = row;
	dictCol = col;
	dictionary.resize(row, col);
	for (int i = 0; i < row; i++) 
	for (int j = 0; j < col; j++) ifs >> dictionary(i,j);
	ifs.close();
}

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}
void SignProc::OMP::setOmpError(float err) {
	ompError = err;
}

void SignProc::OMP::getSparseMat(Eigen::MatrixXd &des) {
	int desCol = des.cols();
	int desRow = des.rows();
	Eigen::MatrixXd a;
	Eigen::MatrixXd proj;
	Eigen::MatrixXd colDes;
	Eigen::MatrixXd clone;
	Eigen::MatrixXd tmp;
	std::vector<int> indx;
	spaMat.resize(dictCol, desCol);
	spaMat.setZero();
	double maxNumCoef = dictRow / 2.0;
	for (int i = 0; i < desCol; i++) {
		colDes = des.col(i);
		clone = colDes;
		indx.clear();
		double curResNorm2 = clone.squaredNorm();
		tmp.resize(dictRow, 0);
		int j = 0;
		while (curResNorm2 > ompError && j < maxNumCoef) {
			j++;
			proj = dictionary.transpose() * clone;
			int pos = 0;
			for (int k = 1; k < dictCol; k++) 
				if (fabs(proj(k,0)) > fabs(proj(pos,0))) pos = k;
			indx.push_back(pos);			
			tmp.conservativeResize(dictRow, j);
			tmp.col(j - 1) = dictionary.col(pos);
			a = pseudoInverse(tmp) * colDes;
			clone = colDes - tmp * a;
			curResNorm2 = clone.squaredNorm();
		}
		
		if (j > 0)
			for (int k = 0; k < j ; k++) 
				spaMat.insert(indx[k], i) = a(k,0); 
	}
}