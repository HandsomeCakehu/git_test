#if 1
//#include "fusion_detection.h"
#include "yolo.h"

mutex mt;

cv::Mat ROI;
cv::Point middle;


vector<double> layersTimes;
double freq;//返回时间
double t;
double refresh_time;
double refresh_time2;
double FPS;
int allframenum, okframenum = 0;
stringstream ssTemp;
string outimgpath ,orgimgpath, numTmp = "";


bool Yolo::readModel1(cv::dnn::Net& net, string& netPath, bool isCuda = true) {
	
	try {
		net = cv::dnn::readNet(netPath);
	}
	catch (const std::exception&) {
		return false;
	}
	//cuda
	if (isCuda) {
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
		cout << "使用CUDA加速中" << endl;
	}
	//cpu
	else {
		net.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
		net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
		//cout << "CPU运行中" << endl;
	}
	return true;
}

//获得网络输出结果
bool Yolo::Detect1(cv::Mat& SrcImg, cv::dnn::Net& net, vector<Output>& output, cv::Mat& outimg) {
	refresh_time = (double)cv::getTickCount();
	refresh_time2= (double)cv::getTickCount();
	allframenum++;

	cv::Mat blob;
	int col = SrcImg.cols;
	int row = SrcImg.rows;
	int maxLen = MAX(col, row);
	cv::Mat netInputImg = SrcImg.clone();
	if (maxLen > 1.2 * col || maxLen > 1.2 * row) {
		cv::Mat resizeImg = cv::Mat::zeros(maxLen, maxLen, CV_8UC3);
		SrcImg.copyTo(resizeImg(cv::Rect(0, 0, col, row)));
		netInputImg = resizeImg;
	}
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(104, 117, 123), true, false);
	//如果在其他设置没有问题的情况下但是结果偏差很大，可以尝试下用下面两句语句
	cv::dnn::blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(0, 0,0), true, false);
	//blobFromImage(netInputImg, blob, 1 / 255.0, cv::Size(netWidth, netHeight), cv::Scalar(114, 114,114), true, false);
	net.setInput(blob);
	std::vector<cv::Mat> netOutputImg;
	//vector<string> outputLayerName{"345","403", "461","output" };
	//net.forward(netOutputImg, outputLayerName[3]); //获取output的输出


	net.forward(netOutputImg, net.getUnconnectedOutLayersNames());

	//遍历网络输出获取结果，遍历每一行的长度为85的一维数组，并且获取符合条件的结果
	std::vector<int> classIds;//结果id数组
	std::vector<float> confidences;//结果每个id对应置信度数组
	std::vector<cv::Rect> boxes;//每个id矩形框
	float ratio_h = (float)netInputImg.rows / netHeight;
	float ratio_w = (float)netInputImg.cols / netWidth;
	int net_width = className.size() + 5;  //输出的网络宽度是类别数+5
	float* pdata = (float*)netOutputImg[0].data;
	for (int stride = 0; stride < 4; stride++) {    //stride
		int grid_x = (int)(netWidth / netStride[stride]);
		int grid_y = (int)(netHeight / netStride[stride]);
		for (int anchor = 0; anchor < 3; anchor++) { //anchors
			const float anchor_w = netAnchors[stride][anchor * 2];
			const float anchor_h = netAnchors[stride][anchor * 2 + 1];
			for (int i = 0; i < grid_y; i++) {
				for (int j = 0; j < grid_x; j++) {
					float box_score = pdata[4]; //Sigmoid(pdata[4]);//获取每一行的box框中含有某个物体的概率
					if (box_score > boxThreshold) {
						cv::Mat scores(1, className.size(), CV_32FC1, pdata + 5);
						cv::Point classIdPoint;
						double max_class_socre;
						minMaxLoc(scores, 0, &max_class_socre, 0, &classIdPoint);
						max_class_socre = (float)max_class_socre; //Sigmoid((float)max_class_socre);
						if (max_class_socre > classThreshold) {
							//rect [x,y,w,h]
							float x = pdata[0];// (Sigmoid(pdata[0]) * 2.f - 0.5f + j) * netStride[stride];  //x
							float y = pdata[1];// (Sigmoid(pdata[1]) * 2.f - 0.5f + i) * netStride[stride];   //y
							float w = pdata[2];// powf(Sigmoid(pdata[2]) * 2.f, 2.f) * anchor_w;   //w
							float h = pdata[3];// powf(Sigmoid(pdata[3]) * 2.f, 2.f) * anchor_h;  //h
							int left = (x - 0.5 * w) * ratio_w;
							int top = (y - 0.5 * h) * ratio_h;
							classIds.push_back(classIdPoint.x);
							confidences.push_back(max_class_socre * box_score);
							boxes.push_back(cv::Rect(left, top, int(w * ratio_w), int(h * ratio_h)));
						}
					}
					pdata += net_width;//下一行
				}
			}
		}
	}

	//执行非最大抑制以消除具有较低置信度的冗余重叠框（NMS）
	vector<int> nms_result;
	cv::dnn::NMSBoxes(boxes, confidences, classThreshold, nmsThreshold, nms_result);

	cv::Point target=cv::Point(0,0);
	float score = 0;
	float maxscore = 0;
	double start, end;


	start = clock();
	for (int i = 0; i < nms_result.size(); i++) 
	{
		int idx = nms_result[i];
		Output result;

		result.id = classIds[idx];
		result.confidence = confidences[idx];
		result.box = boxes[idx];
		output.push_back(result);
		
		score = result.box.height * result.box.width * result.confidence;
		end = clock();

	}
	if (nms_result.size() > 0)
	{
		circle(outimg, target, 10, cv::Scalar(255, 0, 0), 10, 8, 0);
	}


	//左上角推理时间
    //vector<double> layersTimes;
	freq = cv::getTickFrequency() / 1000;//1ms的帧数
	t = net.getPerfProfile(layersTimes) / freq;//总时间/总帧数=每一帧图像的停留时间

	 //图像左上角输出每帧的推理时间
	string label1 = cv::format("Inference time for a frame : %.2f ms", t);//在图像上绘制的标签
	putText(outimg, label1, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);//在图像上绘制标签

	/*namedWindow("预测图片", 1);
	imshow("预测图片", outimg);*/
	if (!output.size())
	{
		/*******************每帧时间*******************************/


		refresh_time2 = ((double)cv::getTickCount() - refresh_time2) / cv::getTickFrequency();
		FPS = 1 / refresh_time2;
		//cout << "每一帧用时:" << refresh_time2 << "s" << "  " << "FPS:" << FPS << endl;
		// 图像左上角输出每帧时间和帧率
		string label2 = cv::format("refresh time per frame : %.2f s    FPS : %.2f  ", refresh_time2, FPS);//在图像上绘制的标签
		putText(outimg, label2, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);//在图像上绘制标签

	}


	if (output.size())
		return true;
	else
		return false;
}

//对结果进行画框显示输出
void Yolo::drawPred1(cv::Mat& img, vector<Output> result, vector<cv::Scalar> color,cv::Point &middle,vector <obstacle> &ob) {
	okframenum++;
	cv::Mat imgcopy;
	img.copyTo(imgcopy);
	//Mat ROI1;

	vector <obstacle> ().swap(ob);
	for (int i = 0; i < result.size(); i++) {
		int left, top;
		int width,height;
		left = result[i].box.x;
		top = result[i].box.y;
		width = result[i].box.width;
		height = result[i].box.height;

		int color_num = i;
		if (result[i].box.empty())
		{
			cout<<"box is empty"<<endl;
		}

		// if (className[result[i].id]  == "car" )
		// {
			
		// 	cout << " 识别到目标物为：" <<className[result[i].id] << endl;

		// 	cout << "目标物左上角像素坐标为： " << "(" << left << "," << top << ")" << endl;
		// 	cout << "目标物右下角像素坐标为： " << "(" << left + width <<"," << top + height << ")" << endl;
		// 	cout << "目标物的中心像素坐标为： " << "(" << left + width/2 << "," << top + height/2 << ")" << endl;

		// }

		std::string id = std::to_string(i);

		obstacle ob1;
		ob1.name = className[result[i].id] + ' ' + id;
		ob1.Address =0;
		ob1.u_left = left;
		ob1.v_left = top;
		ob1.u_right = left + width;
		ob1.v_right = top + height;
		ob1.mass_x = 0;
		ob1.mass_y = 0;
		ob1.mass_z = 0;
		ob1.depth = 0;
		ob1.height = 0;
		ob1.length = 0;
		ob.push_back(ob1);
		// for(auto a: id)
		// {
		// 	ob[i].name.push_back(a);
		// 	//cout << a << endl;
		// }
		//ob[i].name.append(id);
		
		

		rectangle(img, result[i].box, color[result[i].id], 2, 8);


		string label = className[result[i].id] + ":" + to_string(result[i].confidence);
		int baseLine;
		cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
		top = max(top, labelSize.height);
		//rectangle(frame, Point(left, top - int(1.5 * labelSize.height)), Point(left + int(1.5 * labelSize.width), top + baseLine), Scalar(0, 255, 0), FILLED);
		putText(img, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 1, color[result[i].id], 2);


	}

	// 图像左上角输出每帧的推理时间
	string label1 = cv::format("Inference time for a frame : %.2f ms", t);//在图像上绘制的标签
	putText(img, label1, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);//在图像上绘制标签

	refresh_time = ((double)cv::getTickCount() - refresh_time) / cv::getTickFrequency();
	FPS = 1 / refresh_time;
	//cout << "每一帧用时:" << refresh_time << "s" << "  " << "FPS:" << FPS << endl;
	// 图像左上角输出每帧时间和帧率
	string label2 = cv::format("refresh time per frame : %.2f s    FPS : %.2f  ", refresh_time, FPS);//在图像上绘制的标签
	putText(img, label2, cv::Point(10, 45), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);//在图像上绘制标签
	cv::namedWindow("yolov5识别",cv::WINDOW_NORMAL);
	cv::imshow("yolov5识别", img);


}


void yolov5(cv::Mat image,string model1_path,vector <obstacle> &ob)
{
	Yolo test;
	cv::dnn::Net net;
   
    

	if (test.readModel1(net, model1_path, false)) {
		//cout << "read net ok!" << endl;
	}
	else {
		cout << "read net is not ok!" << endl;
		system("pause");
		//return -1;
	}
	// 
	//生成随机颜色
	vector<cv::Scalar> color;
	//srand(time(0));
	//for (int i = 0; i < 80; i++) {
	//	int b = rand() % 256;
	//	int g = rand() % 256;
	//	int r = rand() % 256;
	//	color.push_back(Scalar(b, g, r));
	//}
	for (int i = 0; i < 4; i++) {
		int b = 160;
		int g = 80;
		int r = 50;
		color.push_back(cv::Scalar(b, g, r));
	}
	
	cv::Mat frame=image;
	// 如果到达视频结尾，停止程序 
		if (frame.empty()) {
		cout << "Don't load picture !!!" << endl;
			cv::waitKey();
		}

	vector<Output> result;

	cv::Mat img, outimg;
	frame.copyTo(img);
	frame.copyTo(outimg);

	if (test.Detect1(img, net, result, outimg)) {
		test.drawPred1(outimg, result, color, middle,ob);
	}
	else {
		//cout << "Detect Failed!" << endl;
		middle = cv::Point(0, 0);
		cv::namedWindow("yolov5识别", 0);
		cv::imshow("yolov5识别", outimg);
	}

	cv::waitKey(1);

}


#endif