#ifndef BUZZ_LAMBDA_2_H
#define BUZZ_LAMBDA_2_H

#include "/usr/include/eigen3/Eigen/Dense"
#include "/usr/include/eigen3/Eigen/Eigenvalues"
#include <vector>
#include <algorithm>
#include <iostream>
#include <buzz/buzzvm.h>
#include <sys/time.h>
// #include <iostream>
// #include <fstream>
// #include <string>
// #include <sstream>


class lambda_2_class {
private:

    std::vector< std::vector<float> > shared_matrix;
    int robot_id_int;
    float R;
    std::vector<float> result;

public:

    lambda_2_class(const std::vector< std::vector<float> > &shared_matrix, const int &robot_id_int, const float &R);
    void demo();
    virtual ~lambda_2_class();

    std::vector<float> calculate();
    std::vector<float> calculate_with_delay(const float &delay, const float &time_step, const float &max_vel);

    const std::vector<float> &get_solution() const;
};


#endif //BUZZ_LAMBDA_2_H
