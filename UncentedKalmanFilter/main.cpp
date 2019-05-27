//
//  main.cpp
//  UncentedKalmanFilter
//
//  Created by Bajrang Chapola on 20/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#include <iostream>
#include "ukf.hpp"

int main(int argc, const char * argv[]) {
    // insert code here...
   
    UKF ukf;
//    MatrixXd Xsig_out =MatrixXd(5,11);
//    ukf.GenerateSigmaPoints(&Xsig_out);
//    ukf.AugmentedSigmaPoints(&Xsig_out);
//    ukf.SigmaPointPrediction(&Xsig_out);
    VectorXd x_pred = VectorXd(5);
    MatrixXd P_pred = MatrixXd(5, 5);
    ukf.PredictMeanAndCovariance(&x_pred, &P_pred);
    // print result
//    std::cout << "Xsig = " << std::endl << Xsig_out << std::endl;
    return 0;
}
