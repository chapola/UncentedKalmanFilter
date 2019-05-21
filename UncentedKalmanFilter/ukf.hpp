//
//  ukf.hpp
//  UncentedKalmanFilter
//
//  Created by Bajrang Chapola on 20/05/19.
//  Copyright © 2019 Bajrang Chapola. All rights reserved.
//

#ifndef ukf_hpp
#define ukf_hpp

#include <stdio.h>
#include "Eigen/Dense"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF{
public:
    /**
     * Constructor
     */
    UKF();
    /**
     * Destructor
     */
    virtual ~UKF();
    /**
     * Init Initializes Unscented Kalman filter
     */
    void Init();
    
    /**
    * Generate Sigma points
    */
    
    void GenerateSigmaPoints(MatrixXd* Xsig_out);
    void AugmentedSigmaPoints(MatrixXd* Xsig_out);
    
};

#endif /* ukf_hpp */
