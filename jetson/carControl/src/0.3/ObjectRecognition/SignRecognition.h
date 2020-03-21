#ifndef __SIGN_RECOGNITION
#define __SIGN_RECOGNITION


#include "SignProc.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include "iostream"
#include "algorithm"
#include "opencv2/opencv.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

#define OMP_ERROR 0.4 // Giá trị mặc định cho sai số khi dùng OMP biểu diễn để biểu diễn các descriptor bằng dictionary.



/** \brief Namespace SignRecognition dùng để nhận diện biển báo. 
*/
namespace SignRecognition {
	
	class SignRecognize {
		public :
			SignRecognize();
			~SignRecognize();

			/** \brief Tải các vector biểu diễn các biển báo có trong thư viện
			  * \param[in] filename Đường dẫn đến file chứa các vector biểu diễn các biển báo trong thư viện.
			  * \param[out] v Lưu các vector biểu diễn các biển báo trong thư viện.
			*/
			void readVDictVector(char filename[], std::vector<Eigen::VectorXd> &v);

			/** \brief Tải vector L từ Data dùng cho nhận dạng biển báo
			  * \param[in] fileName Đường dẫn tới file chứa vector L
			  * \param[out] l Lưu vector L
			*/
			void readLVector(char fileName[], Eigen::VectorXd &l);
			
			/** \brief Chuẩn hóa các descriptor lấy được từ SIFT
			  * \param[in] des Descriptor lấy được từ SIFT.
			  * \param[out] imgDes Descriptor sau khi chuẩn hóa. 
			*/
			void makeDes(cv::Mat &des, Eigen::MatrixXd &imgDes);

			/** \brief Distance giữa 2 vector (sử dụng cosine distance)
			  * \param[in] u Vector thứ nhất
			  * \param[in] v Vector thứ hai
			  * \return Khoảng cách giữa 2 vector.
			*/
			double cosineDistance(Eigen::VectorXd &u, Eigen::VectorXd &v);
			
			/** \brief Tính vector Omega cho request.
			  * \param[in] spa Sparse Matrix biểu diễn cho biển cần nhận dạng
			  * \param[in] numCol số cột của dictionary
			  * \param[out] omega Vector omega kết quả.
			*/
			void calcOmegaRequest(Eigen::SparseMatrix<double> &spa, Eigen::VectorXd &omega, int numCol);
			
			/** \brief Tính vector V biểu diễn cho biển cần nhận dạng.
			  * \param[in] omage Vector omega của biển cần nhận dạng
			  * \param[in] l Vector l dùng cho nhận dạng
			  * \param[out] v Vector v biểu diễn biển cần nhận dạng.
			*/
			void calcVRequest(Eigen::VectorXd &omega, Eigen::VectorXd &L, Eigen::VectorXd &v);

			/** \brief Tính các descriptor của biển cần nhận diện bằng thuật toán SIFT
			  * \param[in] sign ảnh biển cần nhận diện
			  * \param[out] signDes Matrix lưu các descriptor tính được.
			  * \return false nếu không tìm được key points và true trong trường hợp ngược lại.
			*/
			bool getSiftDescriptor(cv::Mat &sign, cv::Mat &signDes);

			/** \brief Nhận dạng biển báo
			  * \param[in] sign Biển báo đầu vào cần nhận dạng
			  * \return Nếu không nhận dạng được return -1, ngược lại trả về 1 số 
			  * trong khoảng từ 1 đến 12 tương ứng với các biển trong thư mục default
			*/
			int recognizeSign(cv::Mat &sign);

			/** \brief Vector omega của biển cần nhận dạng */
			Eigen::VectorXd omegaR;

			/** \brief Vector V biểu diễn biển cần nhận dạng */
			Eigen::VectorXd vReq;

			/** \brief Vector l dùng cho nhận dạng */
			Eigen::VectorXd lVector;

			/** \brief Các vector biểu diễn các biển báo trong thư viện*/
			std::vector<Eigen::VectorXd> vDictVector;

			/** \brief Class xử lý descriptor của biển cần nhận dạng */
			SignProc::OMP signProc;

			/** \brief pointer dùng để xử lý SIFT*/
			cv::Ptr<cv::xfeatures2d::SIFT> siftDes;
			
			/** \brief Vector lưu các keypoint của biển cần nhận dạng */
			std::vector<cv::KeyPoint> kp;

			/** \brief Descriptor của biển cần nhận dạng lấy được bằng SIFT */
			cv::Mat siftDesOfSign;
			
			/** \brief Ảnh gray của biển cần nhận dạng */
			cv::Mat graySign;

			/** \brief Descriptor của biển sau khi chuẩn hóa */
			Eigen::MatrixXd signDes;

	};
}

#endif
