#include "../include/rm_cv/LinearRegression.hpp"
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main()
{
    LinearRegression<3> lr(2);
    int k[3][3];
    double decay;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            cin >> k[i][j];

    cin >> decay;

    for (int i = 0; i < 10; i++)
    {
        cv::Vec3d v = {
            k[0][2] * pow(i, 2) + k[0][1] * i + k[0][0],
            k[1][2] * pow(i, 2) + k[1][1] * i + k[1][0],
            k[2][2] * pow(i, 2) + k[2][1] * i + k[2][0]};
        lr.addValuePair(i, v);
    }
    cout << lr.getResult();
}
