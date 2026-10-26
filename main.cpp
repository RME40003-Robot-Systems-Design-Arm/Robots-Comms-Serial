#include <stdio.h>
#include <stdlib.h>
#include <libserialport.h>

#define BAUD 9600

#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <vector>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <mutex>

using namespace cv;
using namespace std;

class Cube {
public:
	enum class Location {
		LOADING_BAY = 1,
		BUFFER = 2
	};
	Location location = Location::LOADING_BAY;
	Scalar colour;
	Point pos;
	int grid_position = -1;
};


class serial_buffer {
public:
	condition_variable thread_status;
	vector<Point, Point> source_to_target_buffer;
};


vector<Cube> get_cubes(Mat image_in) {
	vector<Cube> cubes;
	Mat scaled_input_image;

	float sf = 4;
	float img_width_scaled = round(image_in.size().width / sf);
	float img_height_scaled = round(image_in.size().height / sf);

	resize(image_in, scaled_input_image, cv::Size(img_width_scaled, img_height_scaled));
	Mat hue, stauration, value, thresh, blurred, hsv_frame;

	cvtColor(scaled_input_image, hsv_frame, COLOR_BGR2HSV);
	medianBlur(hsv_frame, blurred, 3);
	extractChannel(blurred, value, 1);

	cv::Mat kernel_2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
	morphologyEx(value, value, MORPH_CLOSE, kernel_2, cv::Point(-1, -1), 3);
	threshold(value, value, 100, 255, THRESH_BINARY);
	
	resize(value, value, cv::Size(img_width_scaled * sf, img_height_scaled * sf));

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(value, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

	imshow("sobelled_images", value);
	for (size_t i = 0; i < contours.size(); ++i) {

		// Get bounding rectangle
		cv::Rect boundingBox = cv::boundingRect(contours[i]);

		int width = boundingBox.width;
		int height = boundingBox.height;

		// Get contour area
		double area = cv::contourArea(contours[i]);

		if (area > 1500) {

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

			Cube detected_cube;
			detected_cube.colour = meanColour;
			detected_cube.pos = centre;
			detected_cube.location = Cube::Location::LOADING_BAY;

			cubes.push_back(detected_cube);

			cv::rectangle(
				image_in,
				boundingBox,
				meanColour*1.5,
				5
			);

			cv::circle(
				image_in,
				centre,
				10,
				Scalar(255,255,255),
				cv::FILLED
			);
		}
	}
return cubes;
}

void drawCenteredRectangleRect(cv::Mat& img, cv::Point center, int width, int height, const cv::Scalar& color, int thickness = -1) {
	cv::Rect rect(center.x - width / 2, center.y - height / 2, width, height);
	cv::rectangle(img, rect, color, thickness);
}

void get_user_input(const vector<Cube>& detected_cubes, const Mat& image_in) {
	if (image_in.empty()) return;

	struct Move {
		Cube cube;
		Point destination;
	};

	struct UIState {
		vector<Cube> cubes;
		vector<Move> buffer;
		Cube selectedCube;
		Point destination;
		bool hasSelection = false;
		bool hasDestination = false;
		Rect submit;
		Rect placementArea;
	};

	static UIState ui;
	ui.cubes = detected_cubes;
	ui.submit = Rect(10, image_in.rows + 10, 120, 40);
	ui.placementArea = Rect(0, 0, image_in.cols, image_in.rows);

	Mat screen(image_in.rows + 100, image_in.cols,
		CV_8UC3, Scalar(0, 0, 0));

	for (const Cube& cube : ui.cubes) {
		drawCenteredRectangleRect(screen, cube.pos, 80, 80, cube.colour);
	}

	// Yellow outline around the selected cube
	if (ui.hasSelection) {
		rectangle(screen,
			Rect(ui.selectedCube.pos.x - 40,
				ui.selectedCube.pos.y - 40, 80, 80),
			Scalar(0, 255, 255), 3);
	}

	// Show the chosen destination
	if (ui.hasDestination) {
		circle(screen, ui.destination, 12, Scalar(0, 255, 255), 3);
	}

	bool ready = ui.hasSelection && ui.hasDestination;

	rectangle(screen, ui.submit,
		ready ? Scalar(0, 120, 0) : Scalar(70, 70, 70), FILLED);

	putText(screen, "Submit", ui.submit.tl() + Point(15, 27),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);

	putText(screen, "Buffered: " + to_string(ui.buffer.size()),
		Point(145, image_in.rows + 37),
		FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);

	string instruction = !ui.hasSelection ? "Select a cube"
		: !ui.hasDestination ? "Click a placement position"
		: "Press Submit, or click to change placement";

	putText(screen, instruction, Point(10, image_in.rows + 80),
		FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255), 1);

	namedWindow("Cube selection", WINDOW_AUTOSIZE);

	setMouseCallback("Cube selection",
		[](int event, int x, int y, int, void* userdata) {
			if (event != EVENT_LBUTTONDOWN) return;

			UIState& state = *static_cast<UIState*>(userdata);
			Point click(x, y);

			if (state.submit.contains(click)) {
				// Submit only works after both steps are complete
				if (!state.hasSelection || !state.hasDestination) return;

				state.buffer.push_back(
					Move{ state.selectedCube, state.destination });

				cout << "Buffer update: move cube from ("
					<< state.selectedCube.pos.x << ", "
					<< state.selectedCube.pos.y << ") to ("
					<< state.destination.x << ", "
					<< state.destination.y << ")\n";

				// Process or send state.buffer here.

				state.hasSelection = false;
				state.hasDestination = false;
				return;
			}

			if (!state.placementArea.contains(click)) return;

			if (!state.hasSelection) {
				// Match the clickable area to the 80 x 80 rectangle
				for (const Cube& cube : state.cubes) {
					Rect button(cube.pos.x - 40, cube.pos.y - 40, 80, 80);

					if (button.contains(click)) {
						state.selectedCube = cube;
						state.hasSelection = true;
						state.hasDestination = false;
						break;
					}
				}
			}
			else {
				// Placement is only available after selecting a cube
				state.destination = click;
				state.hasDestination = true;
			}
		},
		&ui
	);

	imshow("Cube selection", screen);
}



void encode_positions(char* argv[], Point start, Point end) {
	char cmd;


	/*Variables for serial comms */
	struct sp_port* port;
	int err;
	int key = 0;
	//char cmd;
	const char* port_name = "COM17"; 

	/* get port name */
	err = sp_get_port_by_name(port_name, &port);
	if (err == SP_OK)
		/* open port */
		err = sp_open(port, SP_MODE_WRITE);
	if (err != SP_OK)
	{
		/* return error */
		fprintf(stderr, " Can't open port %s\n", argv[1]);
		exit(2);
	}

	/* set Baud rate */
	sp_set_baudrate(port, BAUD);
	/* set the number of bits */
	sp_set_bits(port, 8);

	int small_debouce_wait = 100;
	int prog_register_timeout = 600;

	/* specify the comand to send to the port */
	for (Point point : {start, end}) {
		cmd = static_cast<char>(point.x);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(small_debouce_wait));
		cmd = static_cast<char>(255);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(prog_register_timeout));

		cmd = static_cast<char>(point.y);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(small_debouce_wait));
		cmd = static_cast<char>(255);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(prog_register_timeout));

		cmd = static_cast<char>(1);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(small_debouce_wait));
		cmd = static_cast<char>(255);
		sp_blocking_write(port, &cmd, 1, small_debouce_wait);
		this_thread::sleep_for(std::chrono::milliseconds(prog_register_timeout));
	}

}
//
//void serialise_path() {
//	// So for every path created, 
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

	VideoCapture cap("sample_2.mp4");
	//VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "cannot open camera";
	}

	int key = 0;
	//char cmd;

	Point start, end;

	start.x = 1;
	start.y = 2;

	end.x = 3;
	end.y = 4;

	encode_positions(argv, start, end);


		/* set up to exit when q key is entered */
	//while (key != 'q') {
	//	cap >> frame;
	//	vector<Cube> cubes_vector = get_cubes(frame);
	//	get_user_input(cubes_vector, frame);
	//	imshow(window_name, frame);
	//	
	//	char key = (char)waitKey(25);
	//	/* write the number "cmd" to the port */

	//	switch (key) {
	//	case 'q':
	//	case 'Q':
	//	case 27: //escape key
	//		return 0;
	//	case ' ': //Save an image
	//		//sprintf_s(filename, "filename%.3d.jpg", n++);
	//		//imwrite(filename, frame);
	//		//cout << "Saved " << filename << endl;

	//		break;
	//	default:
	//		break;
	//	}
	//}
	/* close the port */
	//sp_close(port);
	return 0;
}