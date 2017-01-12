#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

//find the centreline where the pin is
unsigned int Robot::average_y_coordinate(Mat& matrix)
{
	unsigned long long cummulative_value = 0, n_of_values = 0;

        for(int i = 0; i < matrix.rows; ++i) {
                const unsigned char* row = matrix.ptr<unsigned char>(i);
                for(int j = 0; j < matrix.cols; j++) {
                        if (row[j] == 255) {
                                cummulative_value += j;
                                n_of_values++;
                        }
                }
        }

        return cummulative_value/n_of_values;
}

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

