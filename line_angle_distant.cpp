
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
	arr_coor.x = line[2] - line[0];
	arr_coor.y = line[3] - line[1];
	return arr_coor;
}

float inner_product(Vec4f line1, Vec4f line2)
{
	array_coordinate aline = vector_coordinate(line1);
	array_coordinate bline = vector_coordinate(line2);
	cout << "aline.x" << aline.x << endl;
	cout << "aline.y" << aline.y << endl;

	cout << "bline.x" << bline.x << endl;
	cout << " bline.y" << bline.y << endl;

	return aline.x * bline.x + aline.y * bline.y;
}

float norm(Vec4f line0)
{
	array_coordinate line = vector_coordinate(line0);

	return sqrt(line.x * line.x + line.y *line.y);
}



float cos_value(Vec4f line0)
{
	array_coordinate line = vector_coordinate(line0);
	return line.x / norm(line0);
}


float intersection_angle(Vec4f aline, Vec4f bline)
{
	cout << endl;
	cout << "a[0]" << aline[0] << endl;
	cout << " a[1]" << aline[1] << endl;
	cout << "a[2]" << aline[2] << endl;
	cout << "a[3]" << aline[3] << endl;

	cout << "a[0]" << bline[0] << endl;
	cout << "a[1]" << bline[1] << endl;
	cout << "a[2]" << bline[2] << endl;
	cout << "a[3]" << bline[3] << endl;


	cout << "inner_product" << inner_product(aline, bline) << endl;
	cout << "norm" << norm(aline) << endl;
	cout << "norm" << norm(bline) << endl;
	cout << "test" << inner_product(aline, bline) / (norm(aline)*norm(bline)) << endl;
	float deg = acos(inner_product(aline, bline) / (norm(aline)*norm(bline)));
	deg = deg / 3.1415926 * 180;
	return deg;
}
//typedef vector<float,4> Vec4f 
float distance_between_lines(Vec4f line1, Vec4f line2)   //Î´ÑéÖ¤
{
	if ((intersection_angle(line1, line2) > 3) && (intersection_angle(line1, line2) < 177))
	{
		cout << "not paralell,return " << endl;
		return -1;
	}
	float distance;
	if ((acos(cos_value(line1)) < 3) || (acos(cos_value(line1)) > 177))
	{
		distance = abs(line1[1] - line2[1]);
	}
	else
	{
		float xmid, yline1, yline2;
		xmid = (line1[0] + line2[0]) / 2;
		yline1 = (xmid - line1[0]) / vector_coordinate(line1).x*vector_coordinate(line1).y + line1[1];
		yline2 = (xmid - line2[0]) / vector_coordinate(line2).x*vector_coordinate(line2).y + line2[1];
		distance = abs(yline1 - yline2)*cos_value(line1);
	}
	return distance;
}
