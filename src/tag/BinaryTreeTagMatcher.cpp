#include <ARAM/tag/BinaryTreeTagMatcher.hpp>

namespace aram
{

        BinaryTreeTagMatcher::BinaryTreeTagMatcher(FrameSet *fs):ITagMatcher(fs),m_tagSize(7),m_scale(8)
        {
        }

        bool BinaryTreeTagMatcher::checkTag(ROI *roi)
        {
                cv::Mat out;
                vecPoint2D src = roi->corners();
                vecPoint2D dst;
                dst.resize(4);

                float s = (float) m_tagSize*m_scale;
                dst[0] = Point2D(0.0, 0.0);
                dst[1] = Point2D(0.0, s);
                dst[2] = Point2D(s, s);
                dst[3] = Point2D(s, 0.0);

                cv::Mat pers = cv::getPerspectiveTransform(src, dst);

                cv::Mat currentFrame, grayscale, binary;

                currentFrame = load("currentFrame");

                cv::cvtColor(currentFrame, grayscale,CV_BGR2GRAY);
                cv::threshold(grayscale, binary, 0.0, 255.0, cv::THRESH_BINARY_INV|cv::THRESH_OTSU);

                cv::Size size(m_tagSize*m_scale,m_tagSize*m_scale);
                cv::warpPerspective(binary, out, pers, size, cv::INTER_NEAREST);

                cv::Mat bits = cv::Mat::ones(m_tagSize,m_tagSize,CV_8UC1);

                //compute matrix of 0/1
                int swidth = out.rows/m_tagSize;
                for(int y=0;y<m_tagSize;y++)
                {
                        for(int x=0;x<m_tagSize;x++)
                        {
                                int Xstart=(x)*(swidth);
                                int Ystart=(y)*(swidth);
                                cv::Mat square = out(cv::Rect(Xstart,Ystart,swidth,swidth));
                                int nZ = cv::countNonZero(square);

                                if(nZ>(swidth*swidth)/2) bits.at<uchar>(y,x) = 0;
                        }
                }


                //try to find if this tag is in our dictonnary
                TagDictionnary *td = TagDictionnary::getInstance();

                bool res = false;
                int nrot = 0;

                nrot = 0;
                while(!res&&nrot<4)
                {
                        rotate(bits, bits);
                        res = td->binaryTreeSearch(bits);
                        nrot++;
                }

                roi->rotate(nrot-1);

                return res;
        }
};
