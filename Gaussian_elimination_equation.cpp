#include <iostream>
#include <cstdio>

using namespace std;

int main()
{
    double martix[100][100];
    int n, m; // n行m列

    scanf("%d %d", &n, &m);

    // 输入
    for(int i = 0; i < n; i++)
        for(int j = 0; j < m; j++)
            scanf("%lf", &martix[i][j]);

    // 向前步骤
    for(int i = 0; i < n - 1; i++)
    {
        // 找主元
        int pos = 0;
        for(int j = 0; j < m; j++)
            if(martix[i][j])
            {
                pos = j;
                break;
            }

        if(martix[i][pos] != 1 && martix[i][pos] != 0)
        {
            double tmp = martix[i][pos];
            for(int j = pos; j < m; j++)
            {
                martix[i][j] = martix[i][j] / tmp;
            }
        }
        for(int j = i + 1; j < n; j++)
        {
            if(!martix[j][pos])
                continue;
            double tmp = martix[j][pos];
            for(int k = pos; k < m; k++)
            {
                martix[j][k] = martix[j][k] - martix[i][k] * tmp;
            }
        }
    }

    // 向后步骤
    for(int i = n - 1; i > 0; i--)
    {
        int pos = 0;
        for(int j = 0; j < m; j++)
            if(martix[i][j])
            {
                pos = j;
                break;
            }

        if(martix[i][pos] != 1 && martix[i][pos] != 0)
        {
            double tmp = martix[i][pos];
            for(int j = pos; j < m; j++)
            {
                martix[i][j] = martix[i][j] / tmp;
            }
        }

        for(int j = 0; j < i; j++)
        {
            if(!martix[j][pos])
                continue;
            double tmp = martix[j][pos];
            for(int k = pos; k < m; k++)
            {
                martix[j][k] = martix[j][k] - martix[i][k] * tmp;
            }
        }
    }

    // 输出
    for(int i = 0; i < n; i++)
    {
        for(int j = 0; j < m; j++)
            printf("%-10.2f", martix[i][j]);
        printf("\n");
    }
    return 0;
}