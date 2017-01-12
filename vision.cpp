#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

void Robot::vision(const double thresh, const double max_val)
{
	//set up camera input
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
        camera.SetResolution(640, 480);
        cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();

        cv::Mat image, with_thresh;

        while(true) {
        	cvSink.GrabFrame(image);

		//apply threshold to get with_thresh
		cv::threshhold(image, with_thresh, thresh, max_val, THRESH_BINARY);
        }
}
