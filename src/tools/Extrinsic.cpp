#include <ARAM/tools/Extrinsic.hpp>

namespace aram
{
	Extrinsic::Extrinsic(Intrinsic intr, vecPoint2D imgPoints, vecPoint3D objPoints):m_intr(intr)
	{
		m_rvec = cv::Mat::zeros(3,1,CV_64FC1);
		m_tvec = cv::Mat::zeros(3,1,CV_64FC1);
		compute(imgPoints,objPoints);
	}
	
	Extrinsic::Extrinsic(Intrinsic intr, vecPoint2D imgPoints, vecPoint3D objPoints, float &error):m_intr(intr)
	{
		compute(imgPoints,objPoints);

		for(unsigned int i=0;i<objPoints.size();++i)
		{
			Point2D proj = project(objPoints[i]);
			error += (float) cv::norm(proj-imgPoints[i]);
		}
		error/=objPoints.size();
	}

	Extrinsic::Extrinsic(Intrinsic intr, cv::Mat &rmat, cv::Mat &tvec):m_intr(intr)
	{
		m_rmat = rmat.clone();
		m_tvec = tvec.clone();
	}
	
	void Extrinsic::compute(vecPoint2D iP, vecPoint3D oP)
	{
		m_objPts = oP;
		m_imgPts = iP;

		// WARNING : Flag CV_P3P and CV_EPNP return unstable results
		cv::solvePnP(m_objPts, m_imgPts, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), m_rvec, m_tvec, false);

		cv::Rodrigues(m_rvec,m_rmat);
	}

	float Extrinsic::error()
	{
		CV_Assert(m_imgPts.size()==m_objPts.size());
		
		vecPoint2D projPoints = project(m_objPts);

		float err = 0.0;
		for(unsigned int i=0;i<m_imgPts.size();++i)
		{
			err+=std::sqrt((m_imgPts[i].x-projPoints[i].x)*(m_imgPts[i].x-projPoints[i].x)+(m_imgPts[i].y-projPoints[i].y)*(m_imgPts[i].y-projPoints[i].y));
		}

		err/= (float) m_imgPts.size();

		return err;
	}

	const cv::Mat & Extrinsic::rotationMatrix()
	{		
		CV_Assert(!m_rvec.empty() || !m_rmat.empty());

		if(!m_rvec.empty() && m_rmat.empty()) cv::Rodrigues(m_rvec,m_rmat);

		return m_rmat;
	}

	const cv::Mat & Extrinsic::rotationVector()
	{
		CV_Assert(!m_rvec.empty() || !m_rmat.empty());
		
		if(m_rvec.empty() && !m_rmat.empty()) cv::Rodrigues(m_rmat,m_rvec);

		return m_rvec;
	}

	const cv::Mat & Extrinsic::translationVector()
	{
		CV_Assert(!m_rmat.empty());
		
		return m_tvec;
	}

	Point2D Extrinsic::project(Point3D pt)
	{
		CV_Assert(m_intr.valid());
		CV_Assert(!m_tvec.empty()&&!m_rvec.empty());
		
		vecPoint3D objPoints;
		objPoints.push_back(pt);
		
		vecPoint2D imgPoints;
		cv::projectPoints(objPoints, m_rvec, m_tvec, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), imgPoints);

		return imgPoints[0];
	}

	vecPoint2D Extrinsic::project(vecPoint3D objPoints)
	{
		CV_Assert(m_intr.valid());
		CV_Assert(!m_tvec.empty()&&!m_rvec.empty());
		
		vecPoint2D imgPoints;

		cv::projectPoints(objPoints, m_rvec, m_tvec, m_intr.cameraMatrix(), m_intr.distorsionCoefficient(), imgPoints);

		return imgPoints;
	}

	vecPoint3D Extrinsic::objPoints()
	{
		return m_objPts;
	}

	vecPoint2D Extrinsic::imgPoints()
	{
		return m_imgPts;
	}
	
	void Extrinsic::OgreGetPoseParameters(double position[3], double orientation[4])
	{
		cv::Mat Tvec = m_tvec;
		cv::Mat Rvec = m_rvec;
		cv::Mat Rot;
		
		//check if paremeters are valid
		bool invalid=false;
		for (int i=0;i<3 && !invalid ;i++)
		{
			if (Tvec.at<float>(i,0)!=-999999) invalid|=false;
			if (Rvec.at<float>(i,0)!=-999999) invalid|=false;
		}
		if (invalid) throw cv::Exception(9003,"extrinsic parameters are not set","Marker::getModelViewMatrix",__FILE__,__LINE__);  
		
		// calculate position vector
		position[0] = -Tvec.ptr<float>(0)[0];
		position[1] = -Tvec.ptr<float>(0)[1];
		position[2] = +Tvec.ptr<float>(0)[2];
		
		// now calculare orientation quaternion
		Rot = cv::Mat(3,3,CV_32FC1);
		cv::Rodrigues(Rvec, Rot);
		
		// calculate axes for quaternion
		double stAxes[3][3];
		// x axis
		stAxes[0][0] = -Rot.at<float>(0,0);
		stAxes[0][1] = -Rot.at<float>(1,0);
		stAxes[0][2] = +Rot.at<float>(2,0);
		// y axis
		stAxes[1][0] = -Rot.at<float>(0,1);
		stAxes[1][1] = -Rot.at<float>(1,1);
		stAxes[1][2] = +Rot.at<float>(2,1);    
		// for z axis, we use cross product
		stAxes[2][0] = stAxes[0][1]*stAxes[1][2] - stAxes[0][2]*stAxes[1][1];
		stAxes[2][1] = - stAxes[0][0]*stAxes[1][2] + stAxes[0][2]*stAxes[1][0];
		stAxes[2][2] = stAxes[0][0]*stAxes[1][1] - stAxes[0][1]*stAxes[1][0];
		
		// transposed matrix
		double axes[3][3];
		axes[0][0] = stAxes[0][0];
		axes[1][0] = stAxes[0][1];
		axes[2][0] = stAxes[0][2];
		
		axes[0][1] = stAxes[1][0];
		axes[1][1] = stAxes[1][1];
		axes[2][1] = stAxes[1][2];  
		
		axes[0][2] = stAxes[2][0];
		axes[1][2] = stAxes[2][1];
		axes[2][2] = stAxes[2][2];    
		
		// Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
		// article "Quaternion Calculus and Fast Animation".
		double fTrace = axes[0][0]+axes[1][1]+axes[2][2];
		double fRoot;
		  
		if ( fTrace > 0.0 )
		{
		// |w| > 1/2, may as well choose w > 1/2
		fRoot = sqrt(fTrace + 1.0);  // 2w
		orientation[0] = 0.5*fRoot;
		fRoot = 0.5/fRoot;  // 1/(4w)
		orientation[1] = (axes[2][1]-axes[1][2])*fRoot;
		orientation[2] = (axes[0][2]-axes[2][0])*fRoot;
		orientation[3] = (axes[1][0]-axes[0][1])*fRoot;
		}
		else
		{
		// |w| <= 1/2
		static unsigned int s_iNext[3] = { 1, 2, 0 };
		unsigned int i = 0;
		if ( axes[1][1] > axes[0][0] )
			i = 1;
		if ( axes[2][2] > axes[i][i] )
			i = 2;
		unsigned int j = s_iNext[i];
		unsigned int k = s_iNext[j];

		fRoot = sqrt(axes[i][i]-axes[j][j]-axes[k][k] + 1.0);
		double* apkQuat[3] = { &orientation[1], &orientation[2], &orientation[3] };
		*apkQuat[i] = 0.5*fRoot;
		fRoot = 0.5/fRoot;
		orientation[0] = (axes[k][j]-axes[j][k])*fRoot;
		*apkQuat[j] = (axes[j][i]+axes[i][j])*fRoot;
		*apkQuat[k] = (axes[k][i]+axes[i][k])*fRoot;
		}
	}	



	void Extrinsic::glGetProjectionMatrix(cv::Size orgImgSize, cv::Size size,double proj_matrix[16],double gnear,double gfar,bool invert)
	{
		cv::Mat CameraMatrix = m_intr.cameraMatrix();
		
		//Deterime the rsized info
		double Ax=double(size.width)/double(orgImgSize.width);
		double Ay=double(size.height)/double(orgImgSize.height);
		double _fx=CameraMatrix.at<float>(0,0)*Ax;
		double _cx=CameraMatrix.at<float>(0,2)*Ax;
		double _fy=CameraMatrix.at<float>(1,1)*Ay;
		double _cy=CameraMatrix.at<float>(1,2)*Ay;
		double cparam[3][4] =
		{
			{
				_fx,  0,  _cx,  0
			},
			{0,          _fy,  _cy, 0},
			{0,      0,      1,      0}
		};

		argConvGLcpara2( cparam, size.width, size.height, gnear, gfar, proj_matrix, invert );

	}


	void Extrinsic::argConvGLcpara2( double cparam[3][4], int width, int height, double gnear, double gfar, double m[16], bool invert )
	{

		double   icpara[3][4];
		double   trans[3][4];
		double   p[3][3], q[4][4];
		int      i, j;

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

		if (!invert)
		{
			m[13]=-m[13] ;
			m[1]=-m[1];
			m[5]=-m[5];
			m[9]=-m[9];
		}

	}


	void Extrinsic::OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, double proj_matrix[16], double gnear, double gfar, bool invert)
	{
		double temp_matrix[16];
		glGetProjectionMatrix(orgImgSize, size, temp_matrix, gnear, gfar, invert);
		proj_matrix[0]=-temp_matrix[0];
		proj_matrix[1]=-temp_matrix[4];
		proj_matrix[2]=-temp_matrix[8];
		proj_matrix[3]=temp_matrix[12];

		proj_matrix[4]=-temp_matrix[1];
		proj_matrix[5]=-temp_matrix[5];
		proj_matrix[6]=-temp_matrix[9];
		proj_matrix[7]=temp_matrix[13];

		proj_matrix[8]=-temp_matrix[2];
		proj_matrix[9]=-temp_matrix[6];
		proj_matrix[10]=-temp_matrix[10];
		proj_matrix[11]=temp_matrix[14];

		proj_matrix[12]=-temp_matrix[3];
		proj_matrix[13]=-temp_matrix[7];
		proj_matrix[14]=-temp_matrix[11];
		proj_matrix[15]=temp_matrix[15];
	}
};