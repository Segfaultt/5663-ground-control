#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

void Robot::vision(const double thresh, const double max_val)
{
	//set up camera input (may need to be changed)
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

        cv::Mat image, with_thresh;

        while(true) {
        	cvSink.GrabFrame(image);

		//apply threshold and greyscale to get with_thresh
		cvtColor(image, with_thresh, CV_BGR2GRAY);
		cv::threshold(with_thresh, with_thresh, thresh, max_val, THRESH_BINARY);//tested
        }
}

