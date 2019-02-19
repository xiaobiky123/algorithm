
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

typedef struct coordinate
{
	float x;
	float y;
}array_coordinate;


array_coordinate vector_coordinate(Vec4f line)
{
	array_coordinate arr_coor;
	arr_coor.x = line[2] - line[0] ;
	arr_coor.y = line[3] - line[1];
	return arr_coor;
}

float inner_product(Vec4f line1, Vec4f line2)
{
	array_coordinate aline = vector_coordinate(line1);
	array_coordinate bline = vector_coordinate(line2);
	//cout << "aline.x" << aline.x << endl;
	//cout << "aline.y" << aline.y << endl;

	//cout << "bline.x" << bline.x << endl;
	//cout <<" bline.y" << bline.y << endl;

	return aline.x * bline.x + aline.y * bline.y;
}

float norm(Vec4f line0)
{
	array_coordinate line = vector_coordinate(line0);
	
	return sqrt(line.x * line.x + line.y *line.y);
}



float cos_value_to_horizon(Vec4f line0)
{
	array_coordinate line = vector_coordinate(line0);
	return line.x / norm(line0);
}

float sin_value_to_horizon(Vec4f line0)
{
	array_coordinate line = vector_coordinate(line0);
	return line.y / norm(line0);
}

//r = x*cos(theta) + y*sin(theta);
//sin（π / 2－α） = cosα
//cos（π / 2－α） = sinα
float normal_equation_para(Vec4f line0)
{
	float x = line0[0], y = line0[1];
	//cout << "x" << x << endl;
	//cout << "y" << y << endl;
	float cosa = abs(sin_value_to_horizon(line0)), sina = abs(cos_value_to_horizon(line0));  //加ABS修正 解决上挑斜线sin值是负数的问题
	//cout << "cosa" << cosa << endl;
	//cout << "sina" << sina << endl;
	cout << endl;
	cout << "polar angle:" << asin(sina)*180/3.1415926 << endl;

	cout << "sinnnnnnnnnn" << sin_value_to_horizon(line0) << endl;
	cout <<"cossssssssss"<< cos_value_to_horizon(line0) << endl;
	
	if (sin_value_to_horizon(line0) < 0)
	{
		float r = x*cosa + y*sina;
		return r;
	}
	else
	{
		float a=asin(sina)*180/3.1415926;
		a=180-a;

		a=a*3.1415926/180.0;
		float r = x*cos(a) + y*sin(a);
		return r;
	}
	
	//float a = 105*3.1415926/180;
	//float r = x*cos(a) + y*sin(a);
	//float r = abs(-x*sina + y*cosa);
	//float r = x*cosa + y*sina;
	
}



float intersection_angle(Vec4f aline, Vec4f bline)
{
	cout << endl;
	cout << "a[0]" <<aline[0]<< endl;
	cout <<" a[1]" << aline[1] << endl;
	cout << "a[2]" << aline[2] << endl;
	cout << "a[3]" << aline[3] << endl;

	cout << "a[0]" << bline[0] << endl;
	cout << "a[1]" << bline[1] << endl;
	cout << "a[2]" << bline[2] << endl;
	cout << "a[3]" << bline[3] << endl;


	cout <<"inner_product" <<inner_product(aline, bline) << endl;
	cout <<"norm"<< norm(aline) << endl;
	cout <<"norm"<< norm(bline) << endl;
	cout << "test"<<inner_product(aline, bline) / (norm(aline)*norm(bline)) << endl;
	float deg=acos(inner_product(aline, bline) /( norm(aline)*norm(bline) ));
	deg = deg / 3.1415926 * 180;
	return deg;
}
//r=x*cos(theta)+y*sin(theta);
//(y-y1)/(y2-y1)=(x-x1)/(x2-x1);

//typedef vector<float,4> Vec4f 
float distance_between_lines(Vec4f line1, Vec4f line2)
{
	if ((intersection_angle(line1, line2) > 3) && (intersection_angle(line1, line2) < 177))
	{
		cout << "not paralell,return " << endl;
		return -1;
	}
	float distance = abs(normal_equation_para(line1) - normal_equation_para(line2));
	/*
	float distance;
	if ((acos(cos_value_to_horizon(line1)) < 3) || (acos(cos_value_to_horizon(line1)) > 177))
	{
		distance = abs(line1[1] - line2[1]);
	}
	else
	{
		float xmid, yline1, yline2;
		xmid = (line1[0] + line2[0]) / 2;
		yline1 = (xmid - line1[0]) / vector_coordinate(line1).x*vector_coordinate(line1).y + line1[1];
		yline2 = (xmid - line2[0]) / vector_coordinate(line2).x*vector_coordinate(line2).y + line2[1];
		distance = abs(yline1 - yline2)*cos_value_to_horizon(line1);
	}
	return distance;
	*/
}
