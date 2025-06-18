#include <iostream>
#include "ukf.h"

int main() {
    std::cout << "Unscented Kalman Filter Implementation" << std::endl;
    
    // UKF 示例代码
    UKF ukf;
    ukf.initialize();
    
    std::cout << "UKF initialized successfully!" << std::endl;
    
    return 0;
}
