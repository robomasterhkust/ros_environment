#include <iostream>
#include "../include/rm_cv/LinearRegression.hpp"

using namespace std;

int main()
{
    LinearRegression regret(2);
    cout << "input decay factor:\n";
    double decayFactor;
    cin >> decayFactor;
    double x1, y1;
    for (int i = 0; i < 5; i++)
    {
        regret.decay(decayFactor);
        cin >> x1 >> y1;
        regret.addValuePair(x1, y1);
    }
    regret.find_y(0, y1);
    cout << "f(0)=" << y1 << endl;
    regret.find_dydx(0, y1);
    cout << "f'(0)=" << y1 << endl;
    regret.find_y(1, y1);
    cout << "f(1)=" << y1 << endl;
    regret.find_dydx(1, y1);
    cout << "f'(1)=" << y1 << endl;
    cout << regret.getResult().at<double>(0, 0);
    cout <<" + ";
    cout << regret.getResult().at<double>(1, 0);
    cout <<"x + ";
    cout << regret.getResult().at<double>(2, 0);
    cout <<"x^2";
};
