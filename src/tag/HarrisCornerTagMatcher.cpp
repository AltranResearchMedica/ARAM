#include <ARAM/tag/HarrisCornerTagMatcher.hpp>
#define DEBUG

namespace aram
{
        HarrisCornerTagMatcher::HarrisCornerTagMatcher(FrameSet *fs):ITagMatcher(fs),m_tagSize(7),m_scale(8)
        {
        }

        bool HarrisCornerTagMatcher::checkTag(ROI* roi)
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

                cv::Mat otsu;

                cv::Mat currentFrame = load("currentFrame");
                cv::Size size(m_tagSize*m_scale,m_tagSize*m_scale);
                cv::warpPerspective(currentFrame, out, pers, size, cv::INTER_NEAREST);
                cv::cvtColor(out,out,CV_BGR2GRAY);
                cv::threshold(out, otsu, 0, 255, cv::THRESH_BINARY_INV | CV_THRESH_OTSU);

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
                TagDictionnary *bt = TagDictionnary::getInstance();

                int res = -1;
                int nrot = 0;

                nrot = 0;
                while(res==-1&&nrot<4)
                {
                        rotate(bits, bits);
                        res = bt->hammingSearch(bits);
                        nrot++;
                }

                roi->rotate(nrot-1);
                roi->id(res);

                if(res==-1)
                {
                        return false;
                }
                else
                {
                        //Harris corner correction
                        vecPoint2D newCor;
                        newCor.resize(4);

                        for(unsigned int c=0;c<src.size();++c)
                        {
                                int window = 10;
                                aram::Point2D currCorner = src[c];

                                cv::Mat img = load("currentFrame");

                                int a = (int) currCorner.x-window;
                                int b = (int) currCorner.y-window;

                                cv::Mat roi = img(cv::Rect(a,b,2*window,2*window));
                                cv::Mat dst = cv::Mat::zeros(roi.size(), CV_32FC1);
                                cv::cvtColor(roi,roi,CV_BGR2GRAY);
                                int blockSize = 2;
                                int apertureSize = 3;
                                double k = 0.04;
                                cv::cornerHarris(roi, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);

                                cv::Mat dst_norm,dst_norm_scaled;
                                cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
                                cv::convertScaleAbs(dst_norm, dst_norm_scaled);


                                int maxX = 0;
                                int maxY = 0;
                                /// Drawing a circle around corners
                                for(int j=0;j<dst_norm.rows;j++)
                                {
                                        for(int i=0;i<dst_norm.cols;i++)
                                        {
                                                if(dst_norm.at<float>(j,i)>230)
                                                {
                                                        maxX = j;
                                                        maxY = j;
                                                }
                                        }
                                }

                                newCor[c] = Point2D(currCorner.x-window+maxX,currCorner.y-window+maxY);
                        }

                        roi->corners(newCor);

                        return true;
                }
        }
};
