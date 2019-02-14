/*Bubble Sort*/
template<typename T>
void bubblesort(vector<T> &inputarray)
{
	int i = 0, j = 0;
	int len = inputarray.size();
	bool swapflag = false;
	for (i = 0; i < len - 1; i++)
	{
		swapflag = false;
		for (j = 0; j < len - 1 - i; j++)
		{
			if (inputarray[j] > inputarray[j + 1])
			{
				T temp = inputarray[j];
				inputarray[j] = inputarray[j + 1];
				inputarray[j + 1] = temp;
				swapflag = true;
			}
		}
		if (swapflag == false)
			break;
	}
}

/*insertsort Sort*/
template<typename T>
void insertsort(vector<T> &inputarray)
{
	int len = inputarray.size();
	int i = 0, j = 0;
	T inserter;
	for (i = 1; i < len; i++)
	{
		inserter = inputarray[i];
		for (j = i - 1; j > -1; j--)
		{
			if (inputarray[j] > inserter)
				inputarray[j + 1] = inputarray[j];
			else
				break;
		}
		inputarray[j + 1] = inserter;
	}
}

/*Select Sort*/

template<typename T>
void selectsort(vector<T> &inputarray)
{
	int i, j;
	int len = inputarray.size();
	for (i = 1; i < len; i++)
	{
		j = i - 1;
		int index = j;
		T min = inputarray[j];
		for (; j < len - 1; j++)
		{
			if (min > inputarray[j + 1])
			{
				min = inputarray[j + 1];
				index = j + 1;
			}
		}
		T temp = inputarray[i - 1];
		inputarray[i - 1] = inputarray[index];
		inputarray[index] = temp;
	}
}

/*Merge Sort*/

template<typename T>
void mergesort(vector<T> &inputarray)
{
	int begin = 0;
	int end = inputarray.size() - 1;
	mergesort_recur(begin, end, inputarray);
}
template<typename T>  //插入 冒泡 选择排序都是慢慢逼近有序的 而归并排序是每个小数组同时分别有序 再组合 一个是时间分解 一个是空间分解 空间的分解让时间也少了 时间的分解让空间也少了
void mergesort_recur(int begin, int end, vector<T> &inputarray)
{
	if ((end - begin) < 1)
		return;
	int mid = (begin + end) / 2;   //不能改这个
	mergesort_recur(begin, mid, inputarray);
	mergesort_recur(mid + 1, end, inputarray);
	mergearray(begin, mid, end, inputarray);
}



template<typename T>  //notice refine index  无论数组长度奇数和偶数都可以
void mergearray(int begin, int mid, int end, vector<T> &inputarray)
{
	int i, j, k;
	vector<T> temparray(end - begin + 1);
	for (i = begin, j = mid + 1, k = 0; (i <= mid) && (j <= end); k++)
	{
		if (inputarray[i] > inputarray[j])
			temparray[k] = inputarray[j++];
		else
			temparray[k] = inputarray[i++];
	}

	for (; i<mid + 1; k++)
	{
		temparray[k] = inputarray[i++];
	}
	for (; j<end + 1; j++, k++)
	{
		temparray[k] = inputarray[j++];
	}

	for (i = begin, k = 0; i <= end; k++, i++)
	{
		inputarray[i] = temparray[k];
	}

}