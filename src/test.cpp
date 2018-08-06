#include <iostream>
// #include "Eigen/Dense"
// #include "tools.h"
// #include <vector>

using namespace std;
// using Eigen::MatrixXd;
// using Eigen::VectorXd;
// using std::vector;


// test RMSE
/*
int main(){
  Tools tools;
  vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	//the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	//call the CalculateRMSE and print out the result
	cout << tools.CalculateRMSE(estimations, ground_truth) << endl;

  return 0;
}
*/

// test Jacobian
/*
int main(){
  Tools tools;
  VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = tools.CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;
  return 0;
}
*/

int main(){


 cout << 3 % 2.5 << endl;


  return 0;
}
