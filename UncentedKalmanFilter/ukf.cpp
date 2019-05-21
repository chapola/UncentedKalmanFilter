//
//  ukf.cpp
//  UncentedKalmanFilter
//
//  Created by Bajrang Chapola on 20/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#include "ukf.hpp"
UKF::UKF(){
    
}
UKF::~UKF(){
    
}
void UKF::GenerateSigmaPoints(MatrixXd *Xsig_out){
    // set state dimension
    int n_x=5;
    
    // define spreading parameter lambda
    double lambda = 3-n_x;
     // set  state vector
    
    VectorXd x = VectorXd(n_x);
    x <<   5.7441,
    1.3800,
    2.2049,
    0.5015,
    0.3528;
    
    // set example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
    -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
    0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
    -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
    -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // create sigma point matrix
    MatrixXd Xsig = MatrixXd(n_x,2*n_x+1);
    
    // calculate square root of P
    MatrixXd A = P.llt().matrixL();
    
    // set first column of sigma point matrix
    
    Xsig.col(0)=x;
    
    for (int i=0; i<n_x; ++i) {
        Xsig.col(i+1)=x+sqrt(lambda+n_x)*A.col(i);
        Xsig.col(i+1+n_x)=x-sqrt(lambda+n_x)*A.col(i);
    }
    
    *Xsig_out=Xsig;
    
}
void UKF::AugmentedSigmaPoints(MatrixXd *Xsig_out){
    // set state dimension
    int n_x=5;
    
    // set augmentation dimension
    int n_aug=7;
    
    // Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a = 0.2;
    
    // Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd=0.2;
    
    // define spreading parameter
    int lambda = 3-n_aug;
    
    // set example state
    VectorXd x = VectorXd(n_x);
    x <<   5.7441,
    1.3800,
    2.2049,
    0.5015,
    0.3528;
    // create example covariance matrix
    MatrixXd P = MatrixXd(n_x, n_x);
    P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
    -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
    0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
    -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
    -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

    // create augmented mean vector
    VectorXd X_aug=VectorXd(7);
     // create augmented state covariance
    MatrixXd P_aug = MatrixXd(7,7);
    // create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug,2*n_aug+1);
    
     // create augmented mean state
    X_aug.head(5)=x;
    X_aug(5)=0;
    X_aug(6)=0;
    
     // create augmented covariance matrix
    P_aug.fill(0.0);
    P_aug.topLeftCorner(5, 5)=P;
    
    P_aug(5,5)=std_a*std_a;
    P_aug(6,6)=std_yawdd*std_yawdd;
    
    // create square root matrix
    MatrixXd L = P_aug.llt().matrixL();
     Xsig_aug.col(0)  = X_aug;
    for (int i=0; i<n_aug; ++i) {
        Xsig_aug.col(i+1)=X_aug+sqrt(lambda+n_aug)*L.col(i);
        Xsig_aug.col(i+1+n_aug)=X_aug-sqrt(lambda+n_aug)*L.col(i);
    }
    
  
    // write result
    *Xsig_out = Xsig_aug;
    
}
