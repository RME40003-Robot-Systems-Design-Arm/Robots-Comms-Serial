#include <stdio.h>
#include <stdlib.h>
//#include <libserialport.h>

#define BAUD 9600

#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>


using namespace cv;
using namespace std;


class Cube {
public:
	cv::Point pos;
	string colour;

	enum class Location {
		LOADING_BAY = 1,
		BUFFER = 2
	};
	Location location = Location::LOADING_BAY;
};


vector<Cube> get_cubes(Mat image_in) {
	vector<Cube> cubes;

	Mat hue, stauration, value, thresh, blurred, hsv_frame;
	cvtColor(image_in, hsv_frame, COLOR_BGR2HSV);
	medianBlur(hsv_frame, blurred, 11);
	//box(hsv_frame, blurred, 13);

	//split(image_in, colour_channel);
	extractChannel(blurred, stauration, 2);
	extractChannel(blurred, value, 1);
	extractChannel(blurred, hue, 0);

	cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11));
	morphologyEx(value, value, MORPH_CLOSE, kernel_2, cv::Point(-1, -1), 3);
	threshold(value, value, 100, 255, THRESH_BINARY);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(value, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<cv::Moments> mu(contours.size());
	std::vector<cv::Point2f> mc(contours.size());

	imshow("sobelled_images", value);
	for (size_t i = 0; i < contours.size(); ++i) {

		// Get bounding rectangle
		cv::Rect boundingBox = cv::boundingRect(contours[i]);

		int width = boundingBox.width;
		int height = boundingBox.height;

		// Get contour area
		double area = cv::contourArea(contours[i]);

		if (area > 1500.0) {

			// Calculate centre of bounding box
			cv::Point centre(
				boundingBox.x + boundingBox.width / 2,
				boundingBox.y + boundingBox.height / 2
			);

			// Create mask for pixels inside the contour
			cv::Mat mask = cv::Mat::zeros(image_in.size(), CV_8UC1);

			cv::drawContours(
				mask,
				contours,
				static_cast<int>(i),
				cv::Scalar(255),
				cv::FILLED
			);

			// Calculate average BGR colour inside the contour
			cv::Scalar meanColour = cv::mean(image_in, mask);

			int blue = static_cast<int>(meanColour[0]);
			int green = static_cast<int>(meanColour[1]);
			int red = static_cast<int>(meanColour[2]);

			// Determine predominant colour (basic classification)
			std::string colourName;

			if (red > green * 1.7 && red > blue * 1.3) {
				colourName = "Red";
			}
			else if (green > red * 1.3 && green > blue * 1.3) {
				colourName = "Green";
			}
			else if (blue > red * 1.3 && blue > green * 1.3) {
				colourName = "Blue";
			}
			else if (red > 180 && green > 180 && blue < 100) {
				colourName = "Yellow";
			}
			else if (red < 60 && green < 60 && blue < 60) {
				colourName = "Black";
			}
			else if (red > 200 && green > 200 && blue > 200) {
				colourName = "White";
			}
			else if (red > 180 && green > 80 && green < 180 &&
				blue < 100) {
				colourName = "Orange";
			}
			else {
				colourName = "Orange AF";
			}

			Cube detected_cube;
			detected_cube.colour = colourName;
			detected_cube.pos = centre;
			detected_cube.location = Cube::Location::LOADING_BAY;

			cubes.push_back(detected_cube);

			// Print information
			std::cout << "Contour #" << i + 1 << ":\n";
			std::cout << "  Width: " << width << " px\n";
			std::cout << "  Height: " << height << " px\n";
			std::cout << "  Area: " << area << " px^2\n";
			std::cout << "  Centre: (" << centre.x
				<< ", " << centre.y << ")\n";
			std::cout << "  Predominant colour: "
				<< colourName << "\n";

			// Draw bounding box
			cv::rectangle(
				image_in,
				boundingBox,
				cv::Scalar(255, 255, 255),
				2
			);

			// Draw centre point
			cv::circle(
				image_in,
				centre,
				4,
				cv::Scalar(0, 0, 255),
				cv::FILLED
			);

			// Label colour above bounding box
			cv::putText(
				image_in,
				colourName,
				cv::Point(boundingBox.x, boundingBox.y - 10),
				cv::FONT_HERSHEY_SIMPLEX,
				0.7,
				cv::Scalar(0, 255, 255),
				2
			);
		}
	}

return cubes;
}


//vector<int> create_instructions() {
//	// basically the user selects which cube they want moved
//	
//}


int main(int argc, char* argv[])
{
	/*Variables for camera function*/
	int n = 0;
	//char filename[200];
	string window_name = "video | q or esc to quit";
	Mat frame;
	/*Setup camera and check for camera*/
	namedWindow(window_name);

	VideoCapture cap("sample.mp4");
	//VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "cannot open camera";

	}
	/*Variables for serial comms */
	//struct sp_port* port;
	//int err;
	int key = 0;
	//char cmd;

	/* Set up and open the port */
	/* check port usage */
	//if (argc < 2)
	//{
	//	/* return error */
	//	fprintf(stderr, " Port use\n");
	//	exit(1);
	//}

	///* get port name */
	//err = sp_get_port_by_name(argv[1], &port);
	//if (err == SP_OK)
	//	/* open port */
	//	err = sp_open(port, SP_MODE_WRITE);
	//if (err != SP_OK)
	//{
	//	/* return error */
	//	fprintf(stderr, " Can't open port %s\n", argv[1]);
	//	exit(2);
		//}

		///* set Baud rate */
		//sp_set_baudrate(port, BAUD);
		///* set the number of bits */
		//sp_set_bits(port, 8);

		/* specify the comand to send to the port */
		//cmd = 1;

		/* set up to exit when q key is entered */
	while (key != 'q') {
		cap >> frame;
		vector<Cube> cubes_vector = get_cubes(frame);
		

		///*The code contained here reads and outputs a single pixel value at (10,15)*/
		//Vec3b intensity = frame.at<Vec3b>(10, 15);
		//int blue = intensity.val[0];
		//int green = intensity.val[1];
		//int red = intensity.val[2];
		//cout << "Intensity = " << endl << " " << blue << " " << green << " " << red << endl << endl;
		///*End of modifying pixel values*/

		///*The code contained here modifies the output pixel values*/
		//	/* Modify the pixels of the RGB image */
		//for (int i = 150; i < frame.rows; i++)
		//{
		//	for (int j = 150; j < frame.cols; j++)
		//	{
		//		/*The following lines make the red and blue channels zero
		//		(this section of the image will be shades of green)*/
		//		frame.at<Vec3b>(i, j)[0] = 0;
		//		frame.at<Vec3b>(i, j)[2] = 0;
		//	}
		//}

		/*End of modifying pixel values*/

		imshow(window_name, frame);
		char key = (char)waitKey(25);
		/* write the number "cmd" to the port */
		//sp_blocking_write(port, &cmd, 1, 100);

		switch (key) {
		case 'q':
		case 'Q':
		case 27: //escape key
			return 0;
		case ' ': //Save an image
			//sprintf_s(filename, "filename%.3d.jpg", n++);
			//imwrite(filename, frame);
			//cout << "Saved " << filename << endl;

			break;
		default:
			break;
		}
	}
	/* close the port */
	//sp_close(port);
	return 0;
}