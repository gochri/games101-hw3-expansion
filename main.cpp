#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

float X_max = FLT_MIN; float Y_max = FLT_MIN; float Z_max = FLT_MIN;
float X_min = FLT_MAX; float Y_min = FLT_MAX; float Z_min = FLT_MAX;
bool position_visible = true;

Eigen::Matrix4f get_model_matrix(float angle)
{
	Eigen::Matrix4f rotation;
	angle = angle * MY_PI / 180.f;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f scale;
	scale << 2.5, 0, 0, 0,
		0, 2.5, 0, 0,
		0, 0, 2.5, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return translate * rotation * scale;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, 1, -eye_pos[2],
		0, 0, 0, 1;

	view = translate * view;

	return view;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
	// TODO: Use the same projection matrix from the previous assignments
	Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
	float fovy = eye_fov / 180.0 * MY_PI;
	float top = zNear * tan(fovy / 2);
	float right = top * aspect_ratio;
	projection <<
		zNear / right, 0, 0, 0,
		0, zNear / top, 0, 0,
		0, 0, (zFar + zNear) / (zNear - zFar), 2 * zFar * zNear / (zNear - zFar),
		0, 0, -1, 0;
	return projection;
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {
	if (position_visible) {
		for (int x = 1; x < width; x++) {
			for (int y = 1; y < height; y++) {
				Vector3f background = Vector3f(255, 255, 255);
				set_pixel(Eigen::Vector2i(x, y), background);
			}
		}
	}
	for (const auto& t : TriangleList) {
		Triangle newtri = *t;
		Triangle drawtri = *t;
		for (int i = 0; i < 3; ++i) {
			Vector2f uv = newtri.tex_coords[i];
			float u = uv[0];
			float v = uv[1];
			Vector4f point = Vector4f(u* width, v* height, 50.f, 1.f);
			drawtri.setVertex(i, point);
		}
		if (position_visible) {
			for (int i = 0; i < 3; ++i) {
				Vector4f pos = newtri.v[i];
				float red = 255.f*(pos[0] - X_min) / (X_max - X_min);
				float green = 255.f*(pos[1] - Y_min) / (Y_max - Y_min);
				float blue = 255.f*(pos[2] - Z_min) / (Z_max - Z_min);
				if (blue > 255.f) { blue = blue - 1.f; }
				if (red > 255.f || red < 0 || green>255.f || green<0 || blue>255.f || blue < 0) {
					std::cout << red << "---" << green << "---" << blue << '\n';
				}
				drawtri.setColor(i, red, green, blue);
			}
			rasterize_triangle(drawtri);
		}
		else {
			isvisible(newtri, drawtri);
			}
		}
	}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;
	std::string filename = "position.png";
	bool command_line = false;

	if (false) {
		std::cout << "drawing visible map by default\n";
		filename = "visible.png";
		position_visible = false;
	}
	else {
		std::cout << "drawing position map by default\n";
	}

	if (argc >= 2) {
		command_line = true;
		filename = std::string(argv[1]);
		if (argc == 3 && std::string(argv[2]) == "position") {
			std::cout << "drawing position map\n";
			position_visible = true;
		}
		else {
			if (argc == 3 && std::string(argv[2]) == "visible")
			{
				std::cout << "drawing visible map\n";
				position_visible = false;
			}
		}
	}
    
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");

	for (auto mesh : Loader.LoadedVertices) {
		if (mesh.Position.X > X_max) { X_max = mesh.Position.X; }
		if (mesh.Position.X < X_min) { X_min = mesh.Position.X; }
		if (mesh.Position.Y > Y_max) { Y_max = mesh.Position.Y; }
		if (mesh.Position.Y < Y_min) { Y_min = mesh.Position.Y; }
		if (mesh.Position.Z > Z_max) { Z_max = mesh.Position.Z; }
		if (mesh.Position.Z < Z_min) { Z_min = mesh.Position.Z; }
	}
	std::cout << X_max << " " << X_min << "\n";
	std::cout << Y_max << " " << Y_min << "\n";
	std::cout << Z_max << " " << Z_min << "\n";

    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    //rst::rasterizer r(700, 700);
	rst::rasterizer r(1024, 1024);

    int key = 0;
    int frame_count = 0;
	Eigen::Vector3f eye_pos = { 0,0,10 };
	float angle = 160.0;

	if (command_line) {
		std::cout << "command_line\n";
		r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

		r.draw(TriangleList);
		cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
		image.convertTo(image, CV_8UC3, 1.0f);
		cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
		cv::imwrite(filename, image);

		return 0;
	}

    while(key != 27)
    {
		std::cout << "drawing"<< '\n';
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

		r.set_model(get_model_matrix(angle));
		r.set_view(get_view_matrix(eye_pos));
		r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        //cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
		cv::Mat image(1024, 1024, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

		std::cout << "frame count: " << frame_count  << '\n';
		frame_count = frame_count + 1;

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

    }
    return 0;
}