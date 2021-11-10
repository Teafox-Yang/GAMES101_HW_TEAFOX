#include "Triangle.h"
#include "rasterizer.h"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
		-eye_pos[2], 0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	float sinR = sin(rotation_angle / 180 * MY_PI);
	float cosR = cos(rotation_angle / 180 * MY_PI);
	model << cosR, -sinR, 0, 0,
		sinR, cosR, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	// TODO: Implement this function
	// Create the model matrix for rotating the triangle around the Z axis.
	// Then return it.
	return model;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float rotation_angle)
{
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	float sinR = sin(rotation_angle / 180 * MY_PI);
	float cosR = cos(rotation_angle / 180 * MY_PI);
	Eigen::Vector3f taxis = axis.transpose();
	Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
	Eigen::Matrix3f N;
	N << 0, -axis.z(), axis.y(),
		axis.z(), 0, -axis.x(),
		-axis.y(), axis.x(), 0;
	Eigen::Matrix3f model_3 = identity;
	
	model << model_3.row(0), 0,
		model_3.row(1), 0,
		model_3.row(2), 0,
		0, 0, 0, 1;
	return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	// Students will implement this function

	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

	// TODO: Implement this function
	// Create the projection matrix for the given parameters.
	// Then return it.
	Eigen::Matrix4f Mpersp = Eigen::Matrix4f::Identity();
	Mpersp << zNear, 0, 0, 0,
		0, zNear, 0, 0,
		0, 0, zNear + zFar, -zNear * zFar,
		0, 0, 1, 0;
	Eigen::Matrix4f Mortho = Eigen::Matrix4f::Identity();
	float fov = eye_fov / 180 * MY_PI;
	float t = zNear * tan(fov / 2);
	float r = t;
	Mortho << 1 / r, 0, 0, -r,
		0, 1 / t, 0, -t,
		0, 0, 2 / (zFar - zNear), -(zFar + zNear) / 2,
		0, 0, 0, 1;
	projection = Mortho * Mpersp;
	return projection;
}

int main(int argc, const char** argv)
{
	float angle = 0;
	bool command_line = false;
	std::string filename = "output.png";

	if (argc >= 3) {
		command_line = true;
		angle = std::stof(argv[2]); // -r by default
		if (argc == 4) {
			filename = std::string(argv[3]);
		}
		else
			return 0;
	}

	rst::rasterizer r(700, 700);

	Eigen::Vector3f eye_pos = { 0, 0, 5 };

	std::vector<Eigen::Vector3f> pos{ {2, 0, -2}, {0, 2, -2}, {-2, 0, -2} };

	std::vector<Eigen::Vector3i> ind{ {0, 1, 2} };

	auto pos_id = r.load_positions(pos);
	auto ind_id = r.load_indices(ind);

	int key = 0;
	int frame_count = 0;

	if (command_line) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);
		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);

		cv::imwrite(filename, image);

		return 0;
	}

	while (key != 27) {
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

		r.draw(pos_id, ind_id, rst::Primitive::Triangle);

		cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::imshow("image", image);
		key = cv::waitKey(10);

		std::cout << "frame count: " << frame_count++ << '\n';

		if (key == 'a') {
			angle += 10;
		}
		else if (key == 'd') {
			angle -= 10;
		}
	}

	return 0;
}


