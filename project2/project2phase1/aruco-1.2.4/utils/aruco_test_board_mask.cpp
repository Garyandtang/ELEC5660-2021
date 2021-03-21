/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
#include <iostream>
#include <getopt.h>
#include <fstream>
#include <sstream>
#include <opencv2/opencv.hpp>

#include "aruco.h"
#include "boarddetector.h"
#include "arucoboardmask.h"
using namespace cv;
using namespace aruco;

string TheInputVideo;
string TheIntrinsicFile;
string TheBoardConfigFile;
bool The3DInfoAvailable=false;
float TheMarkerSize=-1;
ArMarkerDetector MDetector_Piece;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage,TheInputImageCopy;
CameraParameters camParamsOrg,camParams; 
BoardConfiguration TheBoardConfig;
ArBoardDetector TheBoardDetector;
Board TheBoardDetected;
BoardMask TheBoardMask;
string TheOutVideoFilePath;
cv::VideoWriter VWriter;

void cvTackBarEvents(int pos,void*);
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CP, Size size);
void readArguments ( int argc,char **argv );
void usage();
void draw3dBoardAxis(Mat &Image,Board &m,const CameraParameters &cp);
void draw3dBoardCube(Mat &Image,Board &m,const CameraParameters &cp);
pair<double,double> AvrgTime(0,0) ;//determines the average time required for detection
double ThresParam1,ThresParam2;
int iThresParam1,iThresParam2;
/************************************
 *
 *
 *
 *
 ************************************/
int main(int argc,char **argv)
{
	try
	{
	      if(argc==1) usage();
		//parse arguments
		readArguments (argc,argv);
		//read board config info
		if(TheBoardConfigFile==""){
		  cerr<<"The board configuration info must be provided (-b option)"<<endl;
		  return -1;
		}
		TheBoardConfig.readFromFile(TheBoardConfigFile);
		 //read from camera or from  file
		if (TheInputVideo=="") TheVideoCapturer.open(0);
		else TheVideoCapturer.open(TheInputVideo);
		//check video is open
		if (!TheVideoCapturer.isOpened()){
		  cerr<<"Could not open video"<<endl;
		  return -1;
		  
		}

		//read first image to get the dimensions
		TheVideoCapturer>>TheInputImage;

		//Open outputvideo
		if ( TheOutVideoFilePath!="")
		  VWriter.open(TheOutVideoFilePath,CV_FOURCC('M','J','P','G'),15,TheInputImage.size());
		
		//read camera parameters if passed
		if (TheIntrinsicFile!="")
		{ 
			if (!readCameraParameters(TheIntrinsicFile, camParamsOrg ,TheInputImage.size()))
				{
				  cerr<<"could not open file "<<TheIntrinsicFile<<endl;
				  return -1;				  
				}
			The3DInfoAvailable=true;

		}
		//Create gui
		
		cv::namedWindow("thres",1);
		cv::namedWindow("in",1);
		MDetector.getThresholdParams( ThresParam1,ThresParam2);
		iThresParam1=ThresParam1;iThresParam2=ThresParam2;
		cv::createTrackbar("ThresParam1", "in",&iThresParam1, 13, cvTackBarEvents);
		cv::createTrackbar("ThresParam2", "in",&iThresParam2, 13, cvTackBarEvents);
		char key=0;
		int index=0;
		//capture until press ESC or until the end of the video
		while( key!=27 && TheVideoCapturer.grab())
		{
			TheVideoCapturer.retrieve( TheInputImage);  
			//undistord image if possible
			if (camParamsOrg.isValid()){
			  cv::undistort(TheInputImage,TheInputImageCopy, camParamsOrg.CameraMatrix,camParamsOrg.Distorsion);
			  TheInputImageCopy.copyTo(TheInputImage);
			  camParams=camParamsOrg;
			  camParams.Distorsion=cv::Mat::zeros(4,1,CV_32FC1);
			}
			else{
			  TheInputImage.copyTo(TheInputImageCopy);
			  camParams=camParamsOrg;
			}
			index++; //number of images captured
			double tick = (double)getTickCount();//for checking the speed
			//Detection of markers in the image passed
			MDetector.detect(TheInputImage,TheMarkers,camParams);
			//Detection of the board
			float probDetect=TheBoardDetector.detect( TheMarkers, TheBoardConfig,TheBoardDetected, camParams,TheMarkerSize);
			if (probDetect>0.5)
			  TheBoardMask.update(TheInputImage, TheBoardDetected,camParams);
			//chekc the speed by calculating the mean speed of all iterations
			AvrgTime.first+=((double)getTickCount()-tick)/getTickFrequency();
			AvrgTime.second++;			
			cout<<"Time detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;			
			//print marker borders
			for(unsigned int i=0;i<TheMarkers.size();i++)
				TheMarkers[i].draw(TheInputImageCopy,Scalar(0,0,255),1);
			
			//print board
			if (The3DInfoAvailable){
			  if ( probDetect>0.2)   {
				draw3dBoardAxis( TheInputImageCopy,TheBoardDetected,camParams);
				//draw3dBoardCube( TheInputImageCopy,TheBoardDetected,TheIntriscCameraMatrix,TheDistorsionCameraParams);
				}
			}
			  //DONE! Easy, right?

			cout<<endl<<endl<<endl;
			//show input with augmented information and  the thresholded image
			cv::imshow("in",TheInputImageCopy);			
			cv::imshow("thres",MDetector.getThresholdedImage());
			//write to video if required
			if (  TheOutVideoFilePath!=""){
			  //create a beautiful compiosed image showing the thresholded
			  //first create a small version of the thresholded image
			  cv::Mat smallThres;
			  cv::resize( MDetector.getThresholdedImage(),smallThres,cvSize(TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
			  cv::Mat small3C;
			  cv::cvtColor(smallThres,small3C,CV_GRAY2BGR);
			  cv::Mat roi=TheInputImageCopy(cv::Rect(0,0,TheInputImageCopy.cols/3,TheInputImageCopy.rows/3));
 			  small3C.copyTo(roi);
			  VWriter<<TheInputImageCopy;
// 			 cv::imshow("TheInputImageCopy",TheInputImageCopy);			
			
			}

			key=cv::waitKey(0);//wait for key to be pressed
		}
		
		
	}catch(std::exception &ex)

	{
		cout<<"Exception :"<<ex.what()<<endl;
	}

}
/************************************
 *
 *
 *
 *
 ************************************/

void cvTackBarEvents(int pos,void*)
{
if (iThresParam1<3) iThresParam1=3;
if (iThresParam1%2!=1) iThresParam1++;
if (ThresParam2<1) ThresParam2=1;
ThresParam1=iThresParam1;
ThresParam2=iThresParam2;
MDetector.setThresholdParams(ThresParam1,ThresParam2);
//recompute
MDetector.detect(TheInputImage,TheMarkers,camParams,TheMarkerSize);
//Detection of the board
float probDetect=TheBoardDetector.detect( TheMarkers, TheBoardConfig,TheBoardDetected,camParams);
if (The3DInfoAvailable && probDetect>0.2) 
    draw3dBoardAxis( TheInputImageCopy,TheBoardDetected,camParams);

cv::imshow("in",TheInputImageCopy);			
cv::imshow("thres",MDetector.getThresholdedImage());
}

/************************************
 *
 *
 *
 *
 ************************************/
void usage()
{
	cout<<"This program test the ArUco Library \n\n";
	cout<<"-i <video.avi>: specifies a input video file. If not, images from camera are captures"<<endl;
	cout<<"-b <boardConfiguration.abc>: file with the board configuration"<<endl;
	cout<<"-f <file>: if you have calibrated your camera, pass calibration information here so as to be able to get 3D marker info"<<endl;
	cout<<"-s <size>: size of the marker's sides (expressed in meters!)"<<endl;
	cout<<"-o video.avi: output video"<<endl;
}

/************************************
 *
 *
 *
 *
 ************************************/
static const char short_options [] = "hi:f:s:b:o:";

static const struct option
long_options [] =
{
	{ "help",           no_argument,   NULL,                 'h' },
	{ "input",     required_argument,   NULL,           'i' },
	{ "intFile",     required_argument,   NULL,           'f' },
	{ "boardFile",     required_argument,   NULL,           'b' },
	{ "size",     required_argument,   NULL,           's' },
	{ "output",     required_argument,   NULL,           'o' },

	{ 0, 0, 0, 0 }
};

/************************************
 *
 *
 *
 *
 ************************************/
void readArguments ( int argc,char **argv )
{
	for ( ;; )
	{
		int index;
		int c;
		c = getopt_long ( argc, argv,
			short_options, long_options,
			&index );

		if ( -1 == c )
			break;
		switch ( c )
		{
			case 0:
				break;
			case 'h':
				usage ();
				exit ( EXIT_SUCCESS );
				break;
			case 'i':
				TheInputVideo=optarg;
				break;
			case 'f':
				TheIntrinsicFile=optarg;
				break;
			case 's':
				TheMarkerSize=atof(optarg);
				break;
			case 'b':
				TheBoardConfigFile=optarg;
				break;
			case 'o':
				TheOutVideoFilePath=optarg;
				break;
			default:
				usage ();
				exit ( EXIT_FAILURE );
		};
	}

}


/************************************
 *
 *
 *
 *
 ************************************/

/************************************
 *
 *
 *
 *
 ************************************/
bool readCameraParameters(string TheIntrinsicFile,CameraParameters &CameraParams,Size size)
{
	//open file
	ifstream InFile(TheIntrinsicFile.c_str());
	if (!InFile) return false;
	char line[1024];
	InFile.getline(line,1024);	 //skype first line that should contain only comments
	InFile.getline(line,1024);//read the line with real info

	//transfer to a proper container
	stringstream InLine;
	InLine<<line;
	//Create the matrices
	CameraParams.Distorsion=Mat::zeros(4,1,CV_32FC1);
	CameraParams.CameraMatrix=Mat::eye(3,3,CV_32FC1);


	//read intrinsic matrix
	InLine>>CameraParams.CameraMatrix.at<float>(0,0);//fx
	InLine>>CameraParams.CameraMatrix.at<float>(1,1); //fy
	InLine>>CameraParams.CameraMatrix.at<float>(0,2); //cx
	InLine>>CameraParams.CameraMatrix.at<float>(1,2);//cy
	//read distorion parameters
	for(int i=0;i<4;i++) InLine>>CameraParams.Distorsion.at<float>(i,0);

	//now, read the camera size
	float width,height;
	InLine>>width>>height;
	//resize the camera parameters to fit this image size
	float AxFactor= float(size.width)/ width;
	float AyFactor= float(size.height)/ height;
	CameraParams.CameraMatrix.at<float>(0,0)*=AxFactor;
	CameraParams.CameraMatrix.at<float>(0,2)*=AxFactor;
	CameraParams.CameraMatrix.at<float>(1,1)*=AyFactor;
	CameraParams.CameraMatrix.at<float>(1,2)*=AyFactor;

	//debug
	cout<<"fx="<<CameraParams.CameraMatrix.at<float>(0,0)<<endl;
	cout<<"fy="<<CameraParams.CameraMatrix.at<float>(1,1)<<endl;
	cout<<"cx="<<CameraParams.CameraMatrix.at<float>(0,2)<<endl;
	cout<<"cy="<<CameraParams.CameraMatrix.at<float>(1,2)<<endl;
	cout<<"k1="<<CameraParams.Distorsion.at<float>(0,0)<<endl;
	cout<<"k2="<<CameraParams.Distorsion.at<float>(1,0)<<endl;
	cout<<"p1="<<CameraParams.Distorsion.at<float>(2,0)<<endl;
	cout<<"p2="<<CameraParams.Distorsion.at<float>(3,0)<<endl;

	return true;
}
/************************************
 *
 *
 *
 *
 ************************************/
void draw3dBoardAxis(Mat &Image,Board &B,const CameraParameters &cp)
{

Mat objectPoints (4,3,CV_32FC1);
objectPoints.at<float>(0,0)=0;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=0;
objectPoints.at<float>(1,0)=2*TheMarkerSize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=0;
objectPoints.at<float>(2,0)=0;objectPoints.at<float>(2,1)=2*TheMarkerSize;objectPoints.at<float>(2,2)=0;
objectPoints.at<float>(3,0)=0;objectPoints.at<float>(3,1)=0;objectPoints.at<float>(3,2)=2*TheMarkerSize;

vector<Point2f> imagePoints;
projectPoints( objectPoints, B.Rvec,B.Tvec, cp.CameraMatrix,cp.Distorsion,   imagePoints);
//draw lines of different colours
cv::line(Image,imagePoints[0],imagePoints[1],Scalar(0,0,255),2,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[2],Scalar(0,255,0),2,CV_AA);
cv::line(Image,imagePoints[0],imagePoints[3],Scalar(255,0,0),2,CV_AA);

putText(Image,"X", imagePoints[1],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,255),2);
putText(Image,"Y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 1, Scalar(0,255,0),2);
putText(Image,"Z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 1, Scalar(255,0,0),2);

}

/************************************
 *
 *
 *
 *
 ************************************/
void draw3dBoardCube(Mat &Image,Board &B,const CameraParameters &cp)
{
float cubeSize=B[0].ssize;
float txz=-cubeSize/2;
Mat objectPoints (8,3,CV_32FC1);
objectPoints.at<float>(0,0)=txz;objectPoints.at<float>(0,1)=0;objectPoints.at<float>(0,2)=txz;
objectPoints.at<float>(1,0)=txz+cubeSize;objectPoints.at<float>(1,1)=0;objectPoints.at<float>(1,2)=txz;
objectPoints.at<float>(2,0)=txz+cubeSize;objectPoints.at<float>(2,1)=cubeSize;objectPoints.at<float>(2,2)=txz;
objectPoints.at<float>(3,0)=txz;objectPoints.at<float>(3,1)=cubeSize;objectPoints.at<float>(3,2)=txz;

objectPoints.at<float>(4,0)=txz;objectPoints.at<float>(4,1)=0;objectPoints.at<float>(4,2)=txz+cubeSize;
objectPoints.at<float>(5,0)=txz+cubeSize;objectPoints.at<float>(5,1)=0;objectPoints.at<float>(5,2)=txz+cubeSize;
objectPoints.at<float>(6,0)=txz+cubeSize;objectPoints.at<float>(6,1)=cubeSize;objectPoints.at<float>(6,2)=txz+cubeSize;
objectPoints.at<float>(7,0)=txz;objectPoints.at<float>(7,1)=cubeSize;objectPoints.at<float>(7,2)=txz+cubeSize;

vector<Point2f> imagePoints;
projectPoints( objectPoints,B.Rvec,B.Tvec, cp.CameraMatrix, cp.Distorsion ,   imagePoints);
//draw lines of different colours
for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i+4],imagePoints[4+(i+1)%4],Scalar(0,0,255),1,CV_AA);

for(int i=0;i<4;i++)
  cv::line(Image,imagePoints[i],imagePoints[i+4],Scalar(0,0,255),1,CV_AA);

/*for(unsigned int i=0;i<imagePoints.size();i++)
    cout<<imagePoints[i].x<<" "<<imagePoints[i].y<<endl;*/
}
