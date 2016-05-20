#include <ARAM/tag/LocalThreshTagMatcher.hpp>

namespace aram
{
        LocalThreshTagMatcher::LocalThreshTagMatcher(FrameSet *fs):ITagMatcher(fs),m_tagSize(7),m_scale(16)
        {
                m_dst.resize(4);
                float s = (float)m_tagSize*m_scale;
                m_dst[0] = Point2D(0.0, 0.0);
                m_dst[1] = Point2D(0.0, s);
                m_dst[2] = Point2D(s, s);
                m_dst[3] = Point2D(s, 0.0);

                m_bt = TagDictionnary::getInstance();
        }

        bool LocalThreshTagMatcher::checkTag(ROI* roi)
        {
                int64 start = cv::getTickCount();

                vecPoint2D src = roi->corners();

                cv::Mat pers = cv::getPerspectiveTransform(src, m_dst);
                cv::Mat currentFrame, out, otsu, gray;
                currentFrame = load("currentFrame");

                cv::Size size(m_tagSize*m_scale,m_tagSize*m_scale);
                cv::warpPerspective(currentFrame, out, pers, size, cv::INTER_NEAREST);

                cv::cvtColor(out,gray,CV_BGR2GRAY);
                cv::threshold(gray, otsu, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);

                cv::Mat bits = cv::Mat::ones(m_tagSize,m_tagSize,CV_8UC1);

                //compute matrix of 0/1
                int swidth = out.rows/m_tagSize;
                for(int y=0;y<m_tagSize;y++)
                {
                        for(int x=0;x<m_tagSize;x++)
                        {
                                int Xstart=(x)*(swidth);
                                int Ystart=(y)*(swidth);
                                cv::Mat square = otsu(cv::Rect(Xstart,Ystart,swidth,swidth));
                                int nZ = cv::countNonZero(square);

                                if(nZ>(swidth*swidth)/2) bits.at<uchar>(y,x) = 0;
                        }
                }

                //try to find if this tag is in our dictonnary

                int res = -1;
                int nrot = 0;

                nrot = 0;
                while(res==-1&&nrot<4)
                {
                        rotate(bits, bits);
                        res = m_bt->hammingSearch(bits);
                        nrot++;
                }

                roi->rotate(nrot+2);
                roi->id(res);

                double time = (cv::getTickCount()-start)/cv::getTickFrequency()*1000;
                //std::cout << "Local tag time : " << time << std::endl;

                return (res!=-1);
        }
};
