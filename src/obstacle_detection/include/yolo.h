#pragma once
#define NOMINMAX
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <utility>
#include <string>
//#include <tensorflow/core/platform/env.h>
//#include <tensorflow/core/public/session.h>
#include <sstream>
#include <thread>

#include "my_struct.h"

using namespace std;
//using namespace cv;
//using namespace dnn;



//自定义结构体，分割模型参数配置（预测图片尺寸，输入层输出层名称）
struct Segmentation {
	//预测图片尺寸
	int input_height, input_width;
	//输入层名称
	string input_tensor_name;
	//输出层名称
	string output_tensor_name;
};

struct Output
{
	int id;//结果类别id
	float confidence;//结果置信度
	cv::Rect box;//矩形框

};

void yolov5(cv::Mat image,string model1_path,vector <obstacle> &ob);

class Yolo {
public:
	Yolo() {
	}
	~Yolo() {}
	bool readModel1(cv::dnn::Net& net, std::string& netPath, bool isCuda);
	bool Detect1(cv::Mat& SrcImg, cv::dnn::Net& net, std::vector<Output>& output,cv::Mat &outimg);
	void drawPred1(cv::Mat& img, std::vector<Output> result, std::vector<cv::Scalar> color, cv::Point& middle,vector <obstacle> &ob);

private:
	//计算归一化函数
	float Sigmoid(float x) {
		return static_cast<float>(1.f / (1.f + exp(-x)));
	}
	const float netAnchors[3][6] = { { 10.0, 13.0, 16.0, 30.0, 33.0, 23.0 },{ 30.0, 61.0, 62.0, 45.0, 59.0, 119.0 },{ 116.0, 90.0, 156.0, 198.0, 373.0, 326.0 } };
	const float netStride[3] = { 8, 16.0,32 };
	const int netWidth = 640;//网络模型输入大小
	const int netHeight = 640;
	float nmsThreshold = 0.45;
	float boxThreshold = 0.3;
	float classThreshold = 0.3;

	//类名
	std::vector<std::string> className = { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
		"fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
		"elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
		"skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
		"tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
		"sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
		"potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
		"microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
		"hair drier", "toothbrush"};
	
	//std::vector<std::string> className = { "bucket" };
};
#pragma once
