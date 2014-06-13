#include <ARAM/tools/Intrinsics.hpp>

namespace aram
{
	Intrinsics::Intrinsics(const std::string file)
	{
		load(file);
	}

	void Intrinsics::load(const std::string file)
	{
		cv::FileStorage fs(file,cv::FileStorage::READ);
		
		if(!fs.isOpened()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to open intrinsics parameters");

		fs["Camera_Matrix"] >> _cameraMatrix;
		fs["Distortion_Coefficients"] >> _distorsionCoefficient;
		fs.release();
		
		if(_cameraMatrix.empty()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to read camera matrix");
		if(_distorsionCoefficient.empty()) throw ARAMException(__LINE__, __FILE__, "Intrinsics::Intrinsics", "Failed to read distortions coefficients");
	}

	const cv::Mat & Intrinsics::cameraMatrix()
	{
		return _cameraMatrix;
	}

	const cv::Mat & Intrinsics::distorsionCoefficient()
	{
		return _distorsionCoefficient;
	}

	bool Intrinsics::valid()
	{
		return !_distorsionCoefficient.empty()&&!_cameraMatrix.empty();
	}

	
	void Intrinsics::glGetProjectionMatrix(cv::Size orgImgSize, cv::Size size,double proj_matrix[16],double gnear,double gfar,bool invert)
	{
		//Deterime the resized info
		double Ax=double(size.width)/double(orgImgSize.width);
		double Ay=double(size.height)/double(orgImgSize.height);


		double _fx = cameraMatrix().at<float>(0,0)*Ax;
		double _cx = cameraMatrix().at<float>(0,2)*Ax;
		double _fy = cameraMatrix().at<float>(1,1)*Ay;
		double _cy = cameraMatrix().at<float>(1,2)*Ay;
		double cparam[3][4] =
		{
			{
				_fx,  0,  _cx,  0
			},
			{0,          _fy,  _cy, 0},
			{0,      0,      1,      0}
		};

		argConvGLcpara2(cparam, size.width, size.height, gnear, gfar, proj_matrix, invert );

		return;
	}
	
	void Intrinsics::argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert)
	{

		double   icpara[3][4];
		double   trans[3][4];
		double   p[3][3], q[4][4];
		int      i,j;

		cparam[0][2] *= -1.0;
		cparam[1][2] *= -1.0;
		cparam[2][2] *= -1.0;
		

		for ( i = 0; i < 3; i++ )
		{
			for ( j = 0; j < 3; j++ )
			{
				p[i][j] = icpara[i][j] / icpara[2][2];
			}
		}
		q[0][0] = (2.0 * p[0][0] / width);
		q[0][1] = (2.0 * p[0][1] / width);
		q[0][2] = ((2.0 * p[0][2] / width)  - 1.0);
		q[0][3] = 0.0;

		q[1][0] = 0.0;
		q[1][1] = (2.0 * p[1][1] / height);
		q[1][2] = ((2.0 * p[1][2] / height) - 1.0);
		q[1][3] = 0.0;

		q[2][0] = 0.0;
		q[2][1] = 0.0;
		q[2][2] = (gfar + gnear)/(gfar - gnear);
		q[2][3] = -2.0 * gfar * gnear / (gfar - gnear);

		q[3][0] = 0.0;
		q[3][1] = 0.0;
		q[3][2] = 1.0;
		q[3][3] = 0.0;

		for ( i = 0; i < 4; i++ )
		{
			for ( j = 0; j < 3; j++ )
			{
				m[i+j*4] = q[i][0] * trans[0][j]
						   + q[i][1] * trans[1][j]
						   + q[i][2] * trans[2][j];
			}
			m[i+3*4] = q[i][0] * trans[0][3]
					   + q[i][1] * trans[1][3]
					   + q[i][2] * trans[2][3]
					   + q[i][3];
		}

		if(!invert)
		{
			m[13]=-m[13];
			m[1]=-m[1];
			m[5]=-m[5];
			m[9]=-m[9];
		}
	}
	
};