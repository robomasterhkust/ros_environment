/**
 * @brief a object to handle polynomial approximation for data sets
 * 
 * @file LinearRegression.hpp
 * @author Alex Au
 * @date 2018-03-26
 */
#pragma once
#include "defines.hpp"
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>

using namespace std;
using namespace cv;

/**
  * @brief 
  * 
  * @tparam dim the dimenstion of the input vector 
 */
template <unsigned int dim>
class LinearRegression
{
public:
  /**
 * @brief Construct a new Linear Regression:: Linear Regression object
 * 
 * @param degree The degree of x for the polynominal used to approxiamte the value,
 *  for example, degree = 0 means y = c, degree = 1 means y = mx + c
 */
  LinearRegression(const unsigned int &degree)
      : degree(degree),
        inputCount(0),
        solvedDegree(-1),
        tempSigmaX(2 * degree + 1, 1, CV_64F, double(0)),
        A(degree + 1, degree + 1, CV_64F, double(0)),
        B(degree + 1, dim, CV_64F, double(0)),
        coeffs(degree + 1, dim, CV_64F, double(0))
  {
    tempSigmaX.zeros(2 * degree + 1, 1, CV_64F);
#ifdef DEBUG
    cout << "new linear regression has been created\n";

    cout << tempSigmaX << endl;
#endif
  };

  //TODO: check copy constructors
  LinearRegression(const LinearRegression &copy_src)
      : degree(copy_src.degree)
  {
    std::lock_guard<std::mutex> lg(mu);
    this->inputCount = copy_src.inputCount;
    this->solvedDegree = copy_src.solvedDegree;
    this->A = copy_src.A.clone();
    this->B = copy_src.B.clone();
    this->coeffs = copy_src.coeffs.clone();
    this->tempSigmaX = copy_src.tempSigmaX;
  }

  LinearRegression &operator=(const LinearRegression<dim> &copy_src)
  {
    std::lock_guard<std::mutex> lg(mu);
    this->inputCount = copy_src.inputCount;
    this->solvedDegree = copy_src.solvedDegree;
    this->A = copy_src.A.clone();
    this->B = copy_src.B.clone();
    this->coeffs = copy_src.coeffs.clone();
    this->tempSigmaX = copy_src.tempSigmaX;
    return *this;
  }

  void addValuePair(const double &x, const cv::Vec<double, dim> &y)
  {
    std::lock_guard<mutex> lock(mu);
    solvedDegree = -1;
    inputCount++;
#ifdef DEBUG
    // cout << "LinearRegression::addValuePair(" << x << "," << y << ")" << endl;
    // cout << "\tBEFORE tempSigmaX:\n"
    //      << tempSigmaX << endl
    //      << "\tBEFORE B:\n"
    //      << B << endl;
#endif
    for (int i = 0; i < 2 * degree + 1; i++)
    {
      tempSigmaX.at<double>(i, 0) += pow(x, i);
    }
    for (int i = 0; i < degree + 1; i++)
    {
      double temp = pow(x, i);
      B.at<double>(i, 0) += y[0] * temp;
      B.at<double>(i, 1) += y[1] * temp;
      B.at<double>(i, 2) += y[2] * temp;
    }
#ifdef DEBUG
      // cout << "\tAFTER tempSigmaX:\n"
      //      << tempSigmaX << endl
      //      << "\tAFTER B:\n"
      //      << B << endl;
#endif
  };

  void decay(const double multiplier)
  {
    solvedDegree = -1;
    tempSigmaX *= multiplier;
    B *= multiplier;
  };

  bool find_y(const double &x, cv::Vec<double, dim> &y)
  {
    y = 0;
    if (this->solve())
    {
      for (int i = 0; i < solvedDegree + 1; i++)
      {
        for (int j = 0; j < dim; j++)
        {
          y[j] += pow(x, i) * coeffs.at<double>(i, j);
        }
      }
      return true;
    }
    else
      return false;
  };

  //calculate the dy/dx at x
  bool find_dydx(const double &x, cv::Vec<double, dim> &y)
  {
    y = 0;
    if (this->solve())
    {
      for (int i = 0; i < solvedDegree; i++)
      {
        y += pow(x, i) * coeffs.at<double>(i + 1, 0) * (i + 1);
      }
      return true;
    }
    else
      return false;
  };

  /**
 * @brief Get the solved coefficients for the polynominal as a n*1 matrix,
 *  if there is no solution, a zero matrix will be returned
 * 
 * @return Mat 
 */
  Mat getResult()
  {
    std::lock_guard<mutex> lock(mu);
    if (this->solve())
      return coeffs;
    else
    {
      return Mat().zeros(1, 1, CV_64F);
    }
  };

private:
  //return whether the result can be calculated, puts the output at y
  bool solve()
  {
    if (solvedDegree < 0) //if the solution is not found yet
    {
      //form the matrix A
      for (int i = 0; i < degree + 1; i++)
      {
        //A.row(i) = tempSigmaX.rowRange(i, i + degree);
        transpose(tempSigmaX.rowRange(i, i + 1 + degree).col(0), A.row(i));
      }
      solvedDegree = (this->inputCount > degree) ? (degree) : this->inputCount;
#ifdef DEBUG
      // cout << "solving linear regression with " << inputCount << " data\n";
      // cout << "Degree to solve = " << solvedDegree << endl;
      cout << "A: " << A << endl;
      cout << "B: " << B << endl;
#endif
      while (solvedDegree >= 0)
      {
        if (cv::solve(A.colRange(0, solvedDegree + 1).rowRange(0, solvedDegree + 1),
                      B.rowRange(0, solvedDegree + 1),
                      coeffs))
          break;
        solvedDegree--;
      }
      if (solvedDegree < 0)
        return false;
    }
    return true;
  };

  const int degree;
  int inputCount; //number of inputed pairs
  int solvedDegree;
  cv::Mat tempSigmaX;

  /**
    * @brief 
    * the matrix where Aij = summation(1 to n) [Xn^(i+j)].....(i, j starts at 0)
    * only used when solving
   */
  cv::Mat A;

  cv::Mat B; //the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
  cv::Mat coeffs;
  mutex mu; //not sure if needed
};
