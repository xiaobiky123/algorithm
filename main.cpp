/*
 *  Copyright 2010-2011 ZXing authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http: *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <fstream>
#include <string>
#include "ImageReaderSource.h"
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>
#include <zxing/datamatrix/detector/Detector.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>

#include <zxing/MatSource.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<io.h>
#include<time.h>
#include<fstream>


#include <zxing/datamatrix/DataMatrixReader.h>
using namespace cv;
using namespace std;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;

#define HJ_DEBUG 0
#define STATISTIC_BINA 1
#define STATISTIC_GRAY 1
#define STATISTIC_LDET 1
#define STATISTIC_RES 1
#define STATISTIC_FILT 1
#define STATISTIC_DILA 1
#define STATISTIC_FETCH 1
#define STATISTIC_TINYBINA 1
#define STATISTIC_CONTOUR 1
#define  STATISTIC_ROTATE 1
#define  STATISTIC_TOTAL 1
#define  STATISTIC_ZXING 1
#define STATISTIC_DS 1
#define deepJudge 0
#define DEBUG912 0
#define PRYD 0
#define DOWNSAMPLE 1


ofstream debugout;

ofstream finput_bina("G:\\AGV_ZXING_PROJECT\\statistic\\bina.txt");ofstream finput_gray("G:\\AGV_ZXING_PROJECT\\statistic\\gray.txt");ofstream finput_ldet("G:\\AGV_ZXING_PROJECT\\statistic\\ldet.txt");ofstream finput_res("G:\\AGV_ZXING_PROJECT\\statistic\\res.txt");ofstream finput_filt("G:\\AGV_ZXING_PROJECT\\statistic\\filt.txt");ofstream finput_dila("G:\\AGV_ZXING_PROJECT\\statistic\\dila.txt");ofstream finput_fetch("G:\\AGV_ZXING_PROJECT\\statistic\\fetch.txt");ofstream finput_tinybina("G:\\AGV_ZXING_PROJECT\\statistic\\tinybina.txt");ofstream finput_contour("G:\\AGV_ZXING_PROJECT\\statistic\\contour.txt");ofstream finput_rotate("G:\\AGV_ZXING_PROJECT\\statistic\\rotate.txt");ofstream finput_total("G:\\AGV_ZXING_PROJECT\\statistic\\total.txt");ofstream finput_cal("G:\\AGV_ZXING_PROJECT\\statistic\\cal.txt");ofstream finput_zxing("G:\\AGV_ZXING_PROJECT\\statistic\\zxing.txt");ofstream finput_ds("G:\\AGV_ZXING_PROJECT\\statistic\\ds.txt");ofstream finput_anglepos("G:\\AGV_ZXING_PROJECT\\statistic\\anglepos.txt");double deg_compensation = 0;
int posx_compensation=0;
int posy_compensation=0;
Mat huitu;
int rotate_center_x;
int rotate_center_y;


Mat Binary(Mat input, int maxVal, int adaptiveMethod, \
	int thresholdType, int blockSize, int constValue)
{
	Mat output;
	cv::adaptiveThreshold(input, output,
		maxVal, adaptiveMethod,
		thresholdType, blockSize,
		constValue);
	return output;
}



typedef struct class_data_
{
	double degg;
	int cla;
}class_data;


Point GetCenterPoint(Rect rect)
{
	Point cpt;
	cpt.x = rect.x + cvRound(rect.width / 2.0);
	cpt.y = rect.y + cvRound(rect.height / 2.0);
	return cpt;
}


void Morphology(int shape, int type, \
	int element_size, Mat input, Mat &output)
{
	Mat element = getStructuringElement(shape, Size(element_size, element_size));
	morphologyEx(input, output, type, element);
}

void Dilation(int shape,int element_size, Mat input, Mat &output)
{
	Mat element = getStructuringElement(MORPH_RECT, Size(element_size, element_size)); 																   	dilate(input, output, element);
}




int FindOptimumContourForBarcode(vector<std::vector<cv::Point>> contours, Rect &rect_res)
{
	if (contours.size() > 0)
	{
		double max = 0;
		int maxIndex = 0;

		vector<Moments> mom(contours.size());
		vector<Point2f> m(contours.size());
		for (int index = 0; index < contours.size(); index++)
		{
			if (contours.size() == 0)
			{
				break;
			}
			double tmp = fabs(contourArea(contours[index]));
			Rect rect = boundingRect(contours[index]);
			Point ct = GetCenterPoint(rect);
			mom[index] = moments(contours[index], false);
			m[index] = Point(static_cast<float>(mom[index].m10 / mom[index].m00), static_cast<float>(mom[index].m01 / mom[index].m00));
			if (abs(ct.x - m[index].x) < rect.width / 10.0)
			{
				if ((double)abs(rect.width - rect.height) < 0.5*(rect.width + rect.height))
				{
					if (tmp >= max)
					{
						max = tmp;
						maxIndex = index;
					}
				}
			}
		}
		Rect rect = boundingRect(contours[maxIndex]);

		if ((abs((double)rect.width / (double)rect.height - 1)) < 0.7)
		{
			rect_res = rect;
			return maxIndex;

		}
		else
			return -1;
	}
	else
		return -1;

}

void RotatePic(Mat input, Mat &output, double angle)
{
	copyMakeBorder(input, output, 20, 20, 20, 20, BORDER_CONSTANT, 255);
	int width = output.size().width;
	int height = output.size().height;
	Mat M;
	M = getRotationMatrix2D(Point(width / 2, height / 2), angle, 1);
	Mat img_ro;
	warpAffine(output, output, M, Size(width, height));
}



namespace {

bool more = false;
bool test_mode = false;
bool try_harder = false;
bool search_multi = false;
bool use_hybrid = false;
bool use_global = false;
bool verbose = false;

}


message upload_message;

message upload_message_real;

void my_dilation3(void *src0, void* dst0, int N)
{
	CvMat  *src = (CvMat*)src0;
	CvMat  *dst = (CvMat*)dst0;
	int flag = 0;
	for (int i = N / 2; i < src->rows - N / 2; i++)
	{
		 uchar* ptr = ( uchar*)(dst->data.ptr + i*dst->step);
		for (int j = N / 2; j < src->cols - N / 2; j++)
		{
			flag = 0;
			*ptr = 0;
			for (int m = -N / 2; m < N / 2; m++)
			{
				uchar* ptr_src = (uchar*)(src->data.ptr + (i + m)*src->step + j);
				for (int n = -N / 2; n < N / 2; n++)
				{
					if(*ptr_src==255)
					{
						flag = 1;
						break;
					}
					
					ptr_src= ptr_src+n;				}
				if (flag == 1)
					break;
			}
			if (flag)
				*ptr = 255;
			ptr = ptr + 1;
		}
	}
}



void my_dilation2(CvMat *src, CvMat* dst, int N)
{

	int flag = 0;
	for (int j = N / 2; j < src->cols - N / 2; j++)
	{
		for (int i = N / 2; i < src->rows - N / 2; i++)
		{
			flag = 0;
			*( (uchar*)CV_MAT_ELEM_PTR(*dst,i, j) )=0;
						for (int m = -N / 2; m < N / 2; m++)
			{
				for (int n = -N / 2; n < N / 2; n++)
				{
					if(CV_MAT_ELEM(*src, uchar, i+m,j+n)==255)
					{
						flag = 1;
					}
				}
			}
			if (flag)
				*((uchar*)CV_MAT_ELEM_PTR(*dst, i, j)) = 255;
		}
	}
	
}





void my_dilation(Mat &src, Mat &dst0, Mat element)
{
	Mat dst = dst0.clone();
	int N = element.size().width;
	int flag = 0;
	for (int j = N / 2; j < src.size().width- N / 2; j++)
	{
		for (int i = N / 2; i < src.size().height - N / 2; i++)
		{
			flag = 0;
			dst.at<uchar>(i, j) = 0;
			for (int m = -N / 2; m < N / 2; m++)
			{
				for (int n = -N / 2; n < N / 2; n++)
				{
										
															if (src.at<uchar>(i + m, j + n) == 255)
						{
							flag = 1;
						}
									}
			}
			if(flag)
				dst.at<uchar>(i, j) = 255;

		}
	}
	dst0 = dst;
}




void getFiles1(string path, vector<string>& files)
{
	intptr_t hFile = 0;   						  
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
			{
		do
		{
									if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles1(p.assign(path).append("\\").append(fileinfo.name), files);
			}
			else
			{
				files.push_back(path + "\\" + fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}
void getFiles2(string path, vector<string>& files, vector<string> &ownname)
{
	/*files存储文件的路径及名称(eg.   C:\Users\WUQP\Desktop\test_devided\data1.txt)
	ownname只存储文件的名称(eg.     data1.txt)*/
	long   hFile = 0;
	struct _finddata_t fileinfo;
	string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles2(p.assign(path).append("\\").append(fileinfo.name), files, ownname);
			}
			else
			{
				files.push_back(path + "\\" + fileinfo.name);
				ownname.push_back(fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

string pathConvert_Single2Double(string& s) {
	string news;
	string::size_type pos = 0;
	while ((pos = s.find('\\', pos)) != string::npos) {
		s.insert(pos, "\\");
		pos = pos + 2;
	}
	news = string(s);
	return  news;
}

Point getCenterPoint(Rect rect)
{
	Point cpt;
	cpt.x = rect.x + cvRound(rect.width / 2.0);
	cpt.y = rect.y + cvRound(rect.height / 2.0);
	return cpt;
}

int cv_process(Mat image, Mat &output,string filename )
{
	Mat final_locate;
	Mat blackWhitePicq;
	Mat openImg;
	Mat s_down;

//1.降采样
	pyrDown(image, s_down, Size(image.cols / 2, image.rows / 2));
#if STATISTIC_BINA
	double start1 = clock();
#endif
//2.二值化
	const int const_value = 10;
	const int binary_block_size1 = 55;
	const int max = 255;
	Mat res_final = Binary(s_down, max, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, binary_block_size1, const_value);

#if STATISTIC_BINA
	double finish1 = clock();
	double totaltime1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
	finput_bina << totaltime1 << endl;
#endif
#if STATISTIC_FILT
	double start2 = clock();
#endif
#if !DOWNSAMPLE
	Mat element = getStructuringElement(MORPH_RECT, Size(7, 7)); 	
	morphologyEx(res_final, res_final, MORPH_OPEN, element);
#endif

//3.开闭运算代替滤波效果
	const int elem_size1 = 3;
#if DOWNSAMPLE
	Morphology(MORPH_RECT, MORPH_OPEN, elem_size1, res_final, res_final);
#endif

#if STATISTIC_FILT
	double finish2 = clock();
	double totaltime2 = (double)(finish2 - start2) / CLOCKS_PER_SEC;
	finput_filt << totaltime2 << endl;
#endif

#if STATISTIC_DILA
	double start3 = clock();
#endif
#if !DOWNSAMPLE
	Mat element5 = getStructuringElement(MORPH_RECT, Size(37, 37)); 	dilate(er, er, element5);   
#endif

//4.膨胀突显二维码轮廓
#if DOWNSAMPLE	
	const int elem_size2 = 13;
	Dilation(MORPH_RECT, elem_size2, res_final, res_final);
#endif

#if STATISTIC_DILA
	double finish3 = clock();
	double totaltime3 = (double)(finish3 - start3) / CLOCKS_PER_SEC;
	finput_dila << totaltime3 << endl;
#endif

#if STATISTIC_CONTOUR
	double start4 = clock();
#endif
//5.找出最佳轮廓
	vector<std::vector<cv::Point>> contours;
	vector<Vec4i> hierarchy;
	Rect rect;
	findContours(res_final, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);  	
	int maxIndex=FindOptimumContourForBarcode(contours, rect);
	RotatedRect box;
	
#if STATISTIC_CONTOUR	
	double finish4 = clock();
	double totaltime4 = (double)(finish4 - start4) / CLOCKS_PER_SEC;
	finput_contour << totaltime4 << endl;
#endif

#if STATISTIC_FETCH
	double start5 = clock();
#endif
#if STATISTIC_FETCH																				
	double finish5 = clock();
	double totaltime5 = (double)(finish5 - start5) / CLOCKS_PER_SEC;
	finput_fetch << totaltime5 << endl;
#endif

//6.由最佳轮廓抠出二维码
	if (maxIndex != -1)
	{
		box = minAreaRect(contours[maxIndex]);
		final_locate = s_down(rect).clone();

#if !DOWNSAMPLE
		int blockSize01 = 65;
#endif
#if DOWNSAMPLE
#endif

//7.对抠出二维码进行二值化
		const int const_value2 = 10;
		const int binary_block_size2 = 25;
		const int max2 = 255;
		blackWhitePicq = Binary(final_locate, max2, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, binary_block_size2, const_value2);
#if !DOWNSAMPLE
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); 			
		morphologyEx(blackWhitePicq, openImg, MORPH_CLOSE, element);
#endif

//8.对抠出二维码进行闭运算代替滤波
		const int elem_size3 = 3;
#if DOWNSAMPLE
		Morphology(MORPH_RECT, MORPH_CLOSE, elem_size3, blackWhitePicq, openImg);
#endif		

	}
#if STATISTIC_TINYBINA
	double start6 = clock();
#endif
#if STATISTIC_TINYBINA																				
	double finish6 = clock();
	double totaltime6 = (double)(finish6 - start6) / CLOCKS_PER_SEC;
	finput_tinybina << totaltime6 << endl;
#endif
	if (blackWhitePicq.empty())
	{
		return -1;
	}
	else
	{
		if (openImg.empty())
			return -1;
		output = openImg;

//9.如果有需要旋转的，进行旋转
		const int above_45 = 60;
		const int below_45 = 30;
		if (((box.angle > below_45) && (box.angle < above_45)) || ((box.angle > -above_45) && (box.angle < -below_45)))
		{
			RotatePic(output, output, box.angle);
			deg_compensation = box.angle;
			rotate_center_x = output.size().width / 2;
			rotate_center_y = output.size().height / 2;
		}

		return 0;
	}
}
class OpenCVBitmapSource : public LuminanceSource
{
private:
	cv::Mat m_pImage;

public:
	OpenCVBitmapSource(cv::Mat &image);

	~OpenCVBitmapSource();

	int getWidth() const;
	int getHeight() const;

	ArrayRef<char> getRow(int y, ArrayRef<char> row) const; 
	ArrayRef<char> getMatrix() const;
};

OpenCVBitmapSource::OpenCVBitmapSource(cv::Mat& image)
	: LuminanceSource(image.cols, image.rows)
{
	this->m_pImage = image.clone();
}

OpenCVBitmapSource::~OpenCVBitmapSource()
{
}

int OpenCVBitmapSource::getWidth() const
{
	return this->m_pImage.cols;
}

int OpenCVBitmapSource::getHeight() const
{
	return this->m_pImage.rows;
}

ArrayRef<char> OpenCVBitmapSource::getRow(int y, ArrayRef<char> row) const
{
	int width_ = getWidth();
	if (!row)
		row = ArrayRef<char>(width_);
	const char *p = this->m_pImage.ptr<char>(y);
	for (int x = 0; x<width_; ++x, ++p)
		row[x] = *p;
	return row;
}

ArrayRef<char> OpenCVBitmapSource::getMatrix() const
{
	int width_ = getWidth();
	int height_ = getHeight();
	ArrayRef<char> matrix = ArrayRef<char>(width_ * height_);
	for (int y = 0; y < height_; ++y)
	{
		const char *p = this->m_pImage.ptr<char>(y);
		int yoffset = y * width_;
		for (int x = 0; x < width_; ++x, ++p)
		{
			matrix[yoffset + x] = *p;
		}
	}
	return matrix;
}


	Ref<Result>  decode(Ref<BinaryBitmap> image, DecodeHints hints) {
  	Ref<Reader> reader(new zxing::datamatrix::DataMatrixReader());   	return  reader->decode(image, hints);
}

vector<Ref<Result> > decode_multi(Ref<BinaryBitmap> image, DecodeHints hints) {
  MultiFormatReader delegate;
  GenericMultipleBarcodeReader reader(delegate);
  return reader.decodeMultiple(image, hints);
}

int read_image(Ref<LuminanceSource> source, bool hybrid, string expected,string &code_result) {
   string cell_result;
  int res = -1;
  Ref<Result>  results;
  try {
    Ref<Binarizer> binarizer;
      binarizer = new GlobalHistogramBinarizer(source);
    	DecodeHints hints(DecodeHints::DATA_MATRIX_HINT);
	
    hints.setTryHarder(try_harder);
    Ref<BinaryBitmap> binary(new BinaryBitmap(binarizer));
               results = decode(binary, hints);
    
	if (expected.empty()) {
#if HJ_DEBUG
		cout << "  Expected text or binary data for image missing." << endl
			<< "  Detected: " << result << endl;
#endif
		std::string result0 = results->getText()->getText();
		code_result = result0;
		res = -61;
	}
	
  }
  catch (const ReaderException& e) {
	  cell_result = "zxing::ReaderException: " + string(e.what());
	  res = -2;
  }
  return res;

}

string read_expected(string imagefilename) {
  string textfilename = imagefilename;
  string::size_type dotpos = textfilename.rfind(".");

  textfilename.replace(dotpos + 1, textfilename.length() - dotpos - 1, "txt");
  ifstream textfile(textfilename.c_str(), ios::binary);
  textfilename.replace(dotpos + 1, textfilename.length() - dotpos - 1, "bin");
  ifstream binfile(textfilename.c_str(), ios::binary);
  ifstream *file = 0;
  if (textfile.is_open()) {
    file = &textfile;
  } else if (binfile.is_open()) {
    file = &binfile;
  } else {
    return std::string();
  }
  file->seekg(0, ios_base::end);
  size_t size = size_t(file->tellg());
  file->seekg(0, ios_base::beg);

  if (size == 0) {
    return std::string();
  }

  char* data = new char[size + 1];
  file->read(data, size);
  data[size] = '\0';
  string expected(data);
  delete[] data;

  return expected;
}

int main(int argc, char** argv) {
	
	debugout.open("G:\\AGV_ZXING_PROJECT\\debugF\\log.txt");

	string dir = "G:\\AGV_ZXING_PROJECT\\video\\1231a\\wrong3\\";
	string dir1 = "G:\\AGV_ZXING_PROJECT\\video\\1231a\\right3\\";
	string dir2 = "G:\\AGV_ZXING_PROJECT\\video\\1231a\\biao\\";

	int total = 0;
	int gonly = 0;
	int honly = 0;
	int both = 0;
	int neither = 0;
	
	use_global = use_hybrid = true;
	test_mode = true;

	char * filePath = "G:\\AGV_ZXING_PROJECT\\video\\1231xx\\";
	vector<string> files;
	vector<string> files_value;
	vector<string> filesname;
	getFiles2(filePath, files, filesname);
	int files_num = files.size();
	char str[30];
	bool isBarcodeExist = false;
	for (int i = 0; i < files.size(); i++)
	{
		files_value.push_back(pathConvert_Single2Double(files[i]));
	}
	int gresult = 1;
	int hresult = 1;
	string expected = "";
	string filePath_a = "G:\\AGV_ZXING_PROJECT\\video\\1231xx\\";
	vector<string> label;
	ifstream in("G:\\AGV_ZXING_PROJECT\\video\\1231xx_label\\label.txt");
	string line;

	while (getline(in, line)) 	
	{
		label.push_back(line);
	}
	string fname;
	string realfname;
	Mat gray0;
	Mat image;
	vector<string> calculate_value;

	for (int i = 0; i < files_value.size(); i++)
	{

#if STATISTIC_TOTAL
		double start_total = clock();
#endif

		string calculate_="";
		upload_message_real.degg = 10000;
		upload_message_real.x = 10000;
		upload_message_real.y = 10000;
		upload_message_real.tlx = 0;
		upload_message_real.tly = 0;
		upload_message_real.blx = 0;
		upload_message_real.bly = 0;
		upload_message_real.brx = 0;
		upload_message_real.bry = 0;
		deg_compensation = 0;
		posx_compensation = 0;
		posy_compensation = 0;
		rotate_center_x=0;
		rotate_center_y=0;

		fname = filePath_a + to_string(i + 1) + ".jpg";
		realfname = to_string(i + 1) + ".jpg";

		image = imread(fname.c_str(), 0);

		if (image.empty())
			return -1;

#if STATISTIC_DS
		double start_ds = clock();
#endif

#if STATISTIC_DS
		double finish_ds = clock();
		double totaltime_ds = (double)(finish_ds - start_ds) / CLOCKS_PER_SEC;
		if (!finput_ds.is_open())
		{
			cout << "failure0ds" << endl;
		}
		finput_ds << totaltime_ds << endl;
#endif	

#if STATISTIC_GRAY
		double start = clock();
#endif

#if DOWNSAMPLE
#endif

#if !DOWNSAMPLE
		cv::cvtColor(image, gray0, CV_BGR2GRAY);
#endif

#if STATISTIC_GRAY
		double finish = clock();
		double totaltime = (double)(finish - start) / CLOCKS_PER_SEC;

		if (!finput_gray.is_open())
		{

			cout << "failure0" << endl;
			
		}
		finput_gray << totaltime << endl;

#endif	
		Mat output;
		int kres = cv_process(image, output, fname);
		if (kres == -1)
		{
			isBarcodeExist = false;
		}

		if (kres == 0)
		{
			isBarcodeExist = true;
			string name = filesname[i] + "edge";
#if STATISTIC_LDET
			double start0 = clock();
#endif
#if STATISTIC_LDET
			double finish0 = clock();
			double totaltime0 = (double)(finish0 - start0) / CLOCKS_PER_SEC;
			finput_ldet << totaltime0 << endl;

#endif

#if STATISTIC_ROTATE
			double start_r = clock();
#endif
#if STATISTIC_ROTATE
			double finish_r = clock();
			double totaltime_r = (double)(finish_r - start_r) / CLOCKS_PER_SEC;

			finput_rotate << totaltime_r << endl;
#endif

		}
				
#if STATISTIC_ZXING
		double start_z = clock();
#endif
		Ref<LuminanceSource> source_cv = MatSource::create(output); 							
		if (!isBarcodeExist)
		{
			calculate_ = "-1";
			calculate_value.push_back(calculate_);
			if(label[i]== calculate_)
				finput_res << "1" << endl;
			else
				finput_res << "-1" << endl;
		}
		else 
		{
			string coderes = "";
			hresult = read_image(source_cv, false, expected, coderes);			
			if (hresult == -61)
			{																			
#if STATISTIC_RES
				calculate_ = coderes;
				calculate_value.push_back(calculate_);
				if (label[i] == calculate_)
					finput_res << "1" << endl;
				else
					finput_res << "-1" << endl;
#endif
			}
			else
			{	

				upload_message_real.degg = 10000;
				upload_message_real.x = 10000;
				upload_message_real.y = 10000;
#if STATISTIC_RES
				calculate_ = "-1";
				calculate_value.push_back(calculate_);
				if (label[i] == calculate_)
					finput_res << "1" << endl;
				else
					finput_res << "-1" << endl;
#endif			
		    }
		}

#if STATISTIC_ZXING
		double finish_z= clock();
		double totaltime_z = (double)(finish_z - start_z) / CLOCKS_PER_SEC;

		if (!finput_zxing.is_open())
			cout << "failure0" << endl;
		finput_zxing << totaltime_z << endl;
#endif

#if STATISTIC_TOTAL
		double finish_total = clock();
		double totaltime_total = (double)(finish_total - start_total) / CLOCKS_PER_SEC;
		finput_total << totaltime_total << endl;
#endif	
		finput_anglepos << upload_message_real.degg << '\t' << upload_message_real.x << '\t' << upload_message_real.y << endl;	
	}
	
	finput_res.close();
	finput_bina.close();
	finput_gray.close();
	finput_ldet.close();
	finput_filt.close();
	finput_dila.close();
	finput_fetch.close();
	finput_tinybina.close();
	finput_contour.close();
	finput_rotate.close();
	debugout.close();
	finput_total.close();
	finput_zxing.close();
	finput_ds.close();
	finput_anglepos.close();

	for (int kk = 0; kk < files_num; kk++)
	{
		finput_cal << calculate_value[kk] << endl;
	}
	finput_cal.close();

	waitKey(0);

#if HJ_DEBUG
	waitKey(0);
#endif
	/*
    if (use_global && (verbose || hresult != 0)) {
      gresult = read_image(source, false, expected);
      if (!verbose && gresult != 0) {
        cout << "decoding failed" << endl;
      }
    }
	*/
	/*
    gresult = gresult == 0;
    hresult = hresult == 0;
    gonly += gresult && !hresult;
    honly += hresult && !gresult;
    both += gresult && hresult;
    neither += !gresult && !hresult;
    total = total + 1;
	*/
   /*
  if (test_mode) {
    cout << endl
         << "Summary:" << endl
         << " " << total << " images tested total," << endl
         << " " << (honly + both)  << " passed hybrid, " << (gonly + both)
         << " passed global, " << both << " pass both, " << endl
         << " " << honly << " passed only hybrid, " << gonly
         << " passed only global, " << neither << " pass neither." << endl;

 }  
 */
  return 0;
}
