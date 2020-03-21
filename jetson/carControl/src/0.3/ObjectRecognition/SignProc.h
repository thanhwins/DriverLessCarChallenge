#ifndef __SIGN_PROC_H
#define __SIGN_PROC_H


#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>

/** \brief Namespace SignProc dùng để biểu diễn các descriptor của ảnh bằng dictionary.
*/

namespace SignProc {

	class OMP {
		public :
			OMP();
			~OMP();

			/** \brief Tải dictionary đã train từ Data  
			  * \param[in] fileName Đường dẫn đến file chứa dictionary.
			*/
			void loadDict(char fileName[]);

			/** \brief Đặt tham số ompError 
			  * \param[in] err Tham số gán cho ompError.
			*/
			void setOmpError(float err);

			/** \brief Biểu diễn sparse của descriptor bằng dictionary, kết quả lưu vào spaMat 
			  * \param[in] des Ma trận lưu các descriptors của ảnh cần nhận dạng.
			*/
			void getSparseMat(Eigen::MatrixXd &des);

			/** \brief Từ điển xử dụng cho nhận diện */
			Eigen::MatrixXd dictionary;

			/** \brief Ma trận sparse dùng cho biểu diễn descriptor */
			Eigen::SparseMatrix<double> spaMat;
			
			/** \brief Sai số  tối đa trong biểu diễn descriptor bằng dictionary */
			float ompError;
			
			/** \brief Số hàng của ma trận dictionary */
			int dictRow;
			
			/** \brief Số cột của ma trận dictionary */
			int dictCol;

	};
}
#endif
