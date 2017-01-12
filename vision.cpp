#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>
/* distance = k sqrt(area)
 * retro reflectors are 2x5 inches
 * 10-1/4 inches from outer edge to outer edge
 */

//calculate distance from lift in meters assuming you're right on
double std::find_distance(int area, const int k)
{
	return k * sqrt(area);
}

//find the centreline where the pin is from a greyscale matrix
unsigned int Robot::average_x_coordinate(Mat& matrix)
{
	unsigned long cummulative_value = 0, n_of_values = 0;

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

//find how many white pixels there are right or left of midpoint
int Robot::sum_of_side(Mat& matrix, int midpoint, bool lhs)
{
        unsigned long cummulative_value = 0;

        if (lhs) {
                for(int i = 0; i < matrix.rows; ++i) {
                        const unsigned char* row = matrix.ptr<unsigned char>(i); 
                        for(int j = 0; j < midpoint; j++) {
                                if (row[j] == 255) {
                                        cummulative_value++;
                                }
                        }
                }
        } else {
                
                for(int i = 0; i < matrix.rows; ++i) {
                        const unsigned char* row = matrix.ptr<unsigned char>(i); 
                        for(int j = midpoint; j < matrix.cols; j++) {
                                if (row[j] == 255) {
                                        cummulative_value++;
                                }
                        }
                }
        }

        return cummulative_value;
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
