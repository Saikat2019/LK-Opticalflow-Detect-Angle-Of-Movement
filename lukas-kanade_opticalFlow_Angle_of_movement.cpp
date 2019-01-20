#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <cmath>
#include <ctype.h>
using namespace cv;
using namespace std;

int angle(Point2f A, Point2f B) {
    float val = (B.y-A.y)/(B.x-A.x); // calculate slope between the two points
    if(B.x!=A.x)
    {
        val = atan(val); // find arc tan of the slope using taylor series approximation
        val = ((int)(val*180/CV_PI))% 360; // Convert the angle in radians to degrees
        if(B.x < A.x) val+=180;
        if(val < 0) val = 360 + val;
        return val;
    }
    else if(B.y > 0) return 90;    
    else return -90;
}


int main( int argc, char** argv )
{
    VideoCapture cap(0);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 400;
    bool needToInit = false;
    bool nightMode = false;
    namedWindow( "LK Demo", 0 );
    namedWindow( "Histrogram", 0 );
    Mat gray, prevGray, image, frame;
    Mat hist(400, 500, CV_8UC3, Scalar(0,0,0));
    vector<Point2f> points[2];
    while(1)
    {
        cap >> frame;
        if( frame.empty() )
            break;
        frame.copyTo(image);
        cvtColor(image, gray, COLOR_BGR2GRAY);
        if( nightMode )
            image = Scalar::all(0);
        if( needToInit )
        {
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);
            
   
            size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {
                if( !status[i] )
                    continue;
                points[1][k++] = points[1][i];
                line(image, points[0][i], Point(points[0][i].x + ((points[1][i].x - points[0][i].x)*10),points[0][i].y + ((points[1][i].y - points[0][i].y)*10)), Scalar(255,0,0),3);
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
            }
            points[1].resize(k);
        }
        if(points[1].size() < (size_t)MAX_COUNT )
        {
            vector<Point2f> tmp;
            goodFeaturesToTrack(gray, tmp, MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
            for(int m=0;m<tmp.size();m++)
                points[1].push_back(tmp[m]);
        }
        int gradient_hist[36];//={0};
        for(int j=0;j<36;j++)
            gradient_hist[j]=0;
        needToInit = false;
        if(!points[0].empty())
        {
            

            for(int m=0;m<points[1].size();m++)
            {
                cout<<"1\n";

                int theata,r;

                theata = angle(points[0][m],points[1][m]);
                cout << "2\n";
                r = norm(points[0][m] - points[1][m]);
                cout << "3\n";
                cout << "theata : "<< theata <<"\n";
                int index = int(theata/10);
                cout << "4\n";
                cout << "index : "<<index<<"\t";
                cout << "gradient_hist[index] : "<< gradient_hist[index]<< "\n";
                gradient_hist[index] += 1;
                cout << "5\n";

            }
        }
        /*int sum = 0; 
        for(int i=0;i<72;i++)
        {
            sum +=gradient_hist[i];
            cout<<gradient_hist[i]<<' ';//<<endl;
        }*/
        hist = Scalar::all(0); 
        cout << "6\n";
        for(int l=0;l<36;l++)
        {
            int x1,y1,x2,y2;
            x1 = l*10;
            x2 = x1+10;
            y1 = gradient_hist[l];
            y2 = gradient_hist[l+1];
            cout<<"7\n";
            line(hist, Point(x1,y1), Point(x2,y2), Scalar(255,0,0),1);
            cout<<"8\n";
        }

        imshow("LK Demo", image);
        imshow("Histrogram",hist);
        waitKey(0);
        //cout <<sum<<'\n';
        
        std::swap(points[1], points[0]);
        cv::swap(prevGray, gray);
    }
    return 0;
}