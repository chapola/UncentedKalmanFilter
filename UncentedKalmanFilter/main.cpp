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
    MatrixXd Xsig_out =MatrixXd(5,11);
//    ukf.GenerateSigmaPoints(&Xsig_out);
    ukf.AugmentedSigmaPoints(&Xsig_out);
    
    // print result
    std::cout << "Xsig = " << std::endl << Xsig_out << std::endl;
    return 0;
}
