#include "include/lambda_2.h"

lambda_2_class::lambda_2_class(const std::vector< std::vector<float> > &shared_matrix, const int &robot_id_int, const float &R) :
                 shared_matrix(shared_matrix), robot_id_int(robot_id_int), R(R) {}


lambda_2_class::~lambda_2_class() {

}

void lambda_2_class::demo(){
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3,3);

    // A(0,0) = 1;
    // A(0,1) = 2;
    // A(1,0) = 3;
    // A(1,1) = 4;

    A(0,0) = 1;
    A(0,1) = 2;
    A(0,2) = 3;
    A(1,0) = 4;
    A(1,1) = 5;
    A(1,2) = 6;
    A(2,0) = 7;
    A(2,1) = 8;
    A(2,2) = 9;

    Eigen::EigenSolver<Eigen::MatrixXd> es(A); //define Laplacian matrix
    Eigen::VectorXd eigenvalues_vector = es.eigenvalues().real(); //compute the eigenvalues
    std::vector <std::pair<double,int> > V;
    for(int i=0;i<eigenvalues_vector.size();i++){
        std::pair<double,int>P=std::make_pair(eigenvalues_vector(i),i);
        V.push_back(P);
    }

    //order the vector containing the values and the index 
    //I order based on the values contained in first element of pairs (default for sort())
    sort(V.begin(),V.end());
    
    //lambda_2 is the second smallest value. To access the index I need .second
    float l2 = eigenvalues_vector(V[1].second);
    printf("lambda_2 demo is = %f\n",l2);
}

std::vector<float> lambda_2_class::calculate()
{
    // demo();
    // To measure the time of the function
    // static struct timeval t1, t2;
    // gettimeofday(&t1, NULL);

    //get robots array
    std::vector <int> robots_array;

    // robots_array.push_back((int) shared_matrix[0][0]);
    printf("shared_matrix size:  %d,\n",shared_matrix.size());
    if(shared_matrix.size() > 0)
    {

        robots_array.push_back((int) shared_matrix.at(0).at(0));
        for (int i=0;i<shared_matrix.size();i++)
        {
            if((int) shared_matrix[i][0]!=robots_array.back())
            {
                robots_array.push_back((int) shared_matrix[i][0]); //array with ids of robot in the network
            }
        }

        // Sort array of IDs and create a new one that we can exploit to access correctly the matrix after
        std::vector <int> new_robots_array;
        sort(robots_array.begin(), robots_array.end());  
        for (int i = 0; i  < robots_array.size(); i++)
        {
            new_robots_array.push_back(i);
        }

        
        //---------- Calculate A, D, L, lambda_2 -----------------//

        float sigma=pow(R,4.0)/log(2);
        float distance = 0.0;
        float accum = 0.0;
        float lambda_2 = 0.0;
        float d_lambda_2_x = 0.0;
        float d_lambda_2_y = 0.0;
        int index_i = 0;
        int index_j = 0;
        

        Eigen::MatrixXd adjacency_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());
        Eigen::MatrixXd degree_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());
        Eigen::MatrixXd distance_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());
      

        //compute adjacency matrix
        for (int line_of_matrix=0;line_of_matrix<shared_matrix.size();line_of_matrix++){
                
                distance=sqrt(pow(shared_matrix[line_of_matrix][2],2)+pow(shared_matrix[line_of_matrix][3],2));   
                // printf("the distance is %f\n",distance);
                // Substitution of index for preventing segmentation_fault        
                for (int i = 0; i < robots_array.size(); i++)
                {
                    if((int) shared_matrix[line_of_matrix][0] == robots_array[i])
                        index_i = new_robots_array[i];
                    
                    if((int) shared_matrix[line_of_matrix][1] == robots_array[i])
                        index_j = new_robots_array[i];
                }
                distance_matrix(index_i, index_j) = distance;
                //fprintf(stdout, "Distance [%f][%f]= %f \n", shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1], distance);
                if(distance<R)
                {
                    adjacency_matrix(index_i, index_j)=exp(pow(pow(R,2.0) - pow(distance,2.0),2.0)/sigma) - 1.0;
                //fprintf(stdout, "Calcolo= %f \n", exp((pow(distance,2) - pow(distance,2))/sigma) - 1);
                //if (shared_matrix[line_of_matrix][0] == robot_id_int)
                   //fprintf(stdout, "A[%f][%f]= %f \n", shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1], adjacency_matrix(shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1]));
                }
                //else //I initialize the matrix to zero then I do not need to put zero in "else"
                //adjacency_matrix(i,j)=0.0;
                //adjacency_matrix(shared_matrix[line_of_matrix][1],shared_matrix[line_of_matrix][0])=adjacency_matrix(shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1]);
            adjacency_matrix(index_j, index_i)=adjacency_matrix(index_i, index_j);
        }
        
        

        /* for(int i=0;i<robots_array.size();i++){
            for(int j=0;j<robots_array.size();j++){
                fprintf(stdout, "%f ", adjacency_matrix(i,j));
            }
            fprintf(stdout, "\n");
        } */
        

        //fprintf(stdout, "\n -------------------------\n");

        //fprintf(stdout, "Degree matrix");
        //compute degree matrix
        for (int i=0;i<robots_array.size();i++){
            accum=0.0;
            for (int j=0;j<robots_array.size();j++){
                accum=accum+adjacency_matrix(i,j);
            }
            degree_matrix(i,i)=accum;
        }

        //fprintf(stdout, "lambda_2");

        //compute lambda_2
        std::cout << "Here is the Laplacian:\n" << (degree_matrix-adjacency_matrix) << std::endl;
        Eigen::EigenSolver<Eigen::MatrixXd> es(degree_matrix-adjacency_matrix); //define Laplacian matrix
        Eigen::VectorXd eigenvalues_vector = es.eigenvalues().real(); //compute the eigenvalues

        //Create a vector with eigenvalues and index
        std::vector <std::pair<double,int> > V;
        for(int i=0;i<eigenvalues_vector.size();i++){
            std::pair<double,int>P=std::make_pair(eigenvalues_vector(i),i);
            V.push_back(P);
        }

        //order the vector containing the values and the index 
        //I order based on the values contained in first element of pairs (default for sort())
        sort(V.begin(),V.end());
        
        //lambda_2 is the second smallest value. To access the index I need .second
        lambda_2 = eigenvalues_vector(V[1].second);

        //calculate gradient of lambda_2
        Eigen::MatrixXd eigenvectors_matrix = es.eigenvectors().real(); //compute eigenvectors
        Eigen::VectorXd eigenvector_2 = eigenvectors_matrix.col(V[1].second);//take second smallest eigenvector

        
        for (int line_of_matrix=0;line_of_matrix<shared_matrix.size();line_of_matrix++)
        {
            if ((int) shared_matrix[line_of_matrix][0] == robot_id_int)
            {
                // Substitution of index for preventing segmentation_fault 
                for (int i = 0; i < robots_array.size(); i++)
                {
                    if((int) shared_matrix[line_of_matrix][0] == robots_array[i])
                        index_i = new_robots_array[i];
                    
                    if((int) shared_matrix[line_of_matrix][1] == robots_array[i])
                        index_j = new_robots_array[i];
                }
                float d_i_j = distance_matrix(index_i, index_j);
                d_lambda_2_x = d_lambda_2_x -  exp(pow((pow(R,2.0) - pow(d_i_j,2.0)),2.0) / sigma) * 4.0 * (pow(R, 2.0) - pow(d_i_j,2.0)) / sigma * (shared_matrix[line_of_matrix][2]) * pow((eigenvector_2(index_i) - eigenvector_2(index_j)),2.0);
                d_lambda_2_y = d_lambda_2_y -  exp(pow((pow(R,2.0) - pow(d_i_j,2.0)),2.0) / sigma) * 4.0 * (pow(R, 2.0) - pow(d_i_j,2.0)) / sigma * (shared_matrix[line_of_matrix][3]) * pow((eigenvector_2(index_i) - eigenvector_2(index_j)),2.0);
                //break; //If I found my gradient I don't need to check again the matrix -> WRONG
            }
        }
        //fprintf(stdout, "I am: %d, d_lambda_2_x: %f, d_lambda_2_y: %f\n", robot_id_int, d_lambda_2_x, d_lambda_2_y);
        /* for (int i = 0; i < eigenvectors_matrix.cols(); i++)
        {
            //for (int j = 0 ; j < eigenvectors_matrix.rows() ; j++) 
            //{
                fprintf(stdout, " %f,", eigenvectors_matrix(i,index[1]));
            //}
            
        } */
        //fprintf(stdout, "\n------------\n");

        // std::ofstream myfile;
        // std::string temp;
        // std::stringstream ss;
        // ss << robot_id_int;
        // std::string numberAsString(ss.str());
        // temp = "saved_data/frequency_" + numberAsString + ".csv";
        // myfile.open(temp, std::ofstream::out | std::ofstream::app);

        // gettimeofday(&t2, NULL);
        // double elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec); //[us]
        // fprintf(stdout, "lambda_2 elapsed time: %f [us]\n", elapsedTime);
        
        // myfile << "Elapsed " << elapsedTime << "\n";
        // myfile.close();
            
        // fprintf(stdout,"lambda_2: %f \n", lambda_2); // print results
        result.push_back(lambda_2);
        result.push_back(d_lambda_2_x);
        result.push_back(d_lambda_2_y);
    }
    else
    {
        result.push_back(0);
        result.push_back(0);
        result.push_back(0);
    }

    
    return result;
}

std::vector<float> lambda_2_class::calculate_with_delay(const float &delay, const float &time_step, const float &max_vel)
{
    // To measure the time of the function
    // static struct timeval t1, t2;
    // gettimeofday(&t1, NULL);

    //get robots array
    std::vector <int> robots_array;

    robots_array.push_back((int) shared_matrix[0][0]);
    for (int i=0;i<shared_matrix.size();i++)
    {
        if((int) shared_matrix[i][0]!=robots_array.back())
        {
            robots_array.push_back((int) shared_matrix[i][0]); //array with ids of robot in the network
        }
    }

    // Sort array of IDs and create a new one that we can exploit to access correctly the matrix after
    std::vector <int> new_robots_array;
    sort(robots_array.begin(), robots_array.end());  
    for (int i = 0; i  < robots_array.size(); i++)
    {
        new_robots_array.push_back(i);
    }

    
    //---------- Calculate A, D, L, lambda_2 -----------------//

    float sigma=pow(R,4.0)/log(2);
    float distance = 0.0;
    float accum = 0.0;
    float lambda_2 = 0.0;
    float d_lambda_2_x = 0.0;
    float d_lambda_2_y = 0.0;
    int index_i = 0;
    int index_j = 0;
    

    Eigen::MatrixXd adjacency_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());
    Eigen::MatrixXd degree_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());
    Eigen::MatrixXd distance_matrix = Eigen::MatrixXd::Zero(robots_array.size(),robots_array.size());

    //compute adjacency matrix
    for (int line_of_matrix=0;line_of_matrix<shared_matrix.size();line_of_matrix++){
            
            distance=sqrt(pow(shared_matrix[line_of_matrix][2],2)+pow(shared_matrix[line_of_matrix][3],2));   
            // Substitution of index for preventing segmentation_fault        
            for (int i = 0; i < robots_array.size(); i++)
            {
                if((int) shared_matrix[line_of_matrix][0] == robots_array[i])
                    index_i = new_robots_array[i];
                
                if((int) shared_matrix[line_of_matrix][1] == robots_array[i])
                    index_j = new_robots_array[i];
            }
            distance_matrix(index_i, index_j) = distance;
            //fprintf(stdout, "Distance [%f][%f]= %f \n", shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1], distance);
            // I have to calculate the fictitious distance before the if or I'll have non consistent matrix
            float distance_modified = distance + 2.0 * max_vel * delay * time_step;
            // fprintf(stdout, "max_vel %f, delay %f, time_step %f\n", max_vel, delay, time_step);

            if(distance_modified < R)
            {
                // fprintf(stdout, "distance %f, distance_modified %f\n", distance, distance_modified);
                adjacency_matrix(index_i, index_j)=exp(pow(pow(R,2.0) - pow(distance_modified,2.0),2.0)/sigma) - 1.0;
                //fprintf(stdout, "Calcolo= %f \n", exp((pow(distance,2) - pow(distance,2))/sigma) - 1);
                //if (shared_matrix[line_of_matrix][0] == robot_id_int)
                //fprintf(stdout, "A[%f][%f]= %f \n", shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1], adjacency_matrix(shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1]));
            }
            //else //I initialize the matrix to zero then I do not need to put zero in "else"
            //adjacency_matrix(i,j)=0.0;
            //adjacency_matrix(shared_matrix[line_of_matrix][1],shared_matrix[line_of_matrix][0])=adjacency_matrix(shared_matrix[line_of_matrix][0],shared_matrix[line_of_matrix][1]);
        adjacency_matrix(index_j, index_i)=adjacency_matrix(index_i, index_j);
    }
    
    

    /* for(int i=0;i<robots_array.size();i++){
        for(int j=0;j<robots_array.size();j++){
            fprintf(stdout, "%f ", adjacency_matrix(i,j));
        }
        fprintf(stdout, "\n");
    } */
    

    //fprintf(stdout, "\n -------------------------\n");

    //fprintf(stdout, "Degree matrix");
    //compute degree matrix
    for (int i=0;i<robots_array.size();i++){
        accum=0.0;
        for (int j=0;j<robots_array.size();j++){
            accum=accum+adjacency_matrix(i,j);
        }
        degree_matrix(i,i)=accum;
    }

    //fprintf(stdout, "lambda_2");

    //compute lambda_2
    Eigen::EigenSolver<Eigen::MatrixXd> es(degree_matrix-adjacency_matrix); //define Laplacian matrix
    Eigen::VectorXd eigenvalues_vector = es.eigenvalues().real(); //compute the eigenvalues

    //Create a vector with eigenvalues and index
    std::vector <std::pair<double,int> > V;
    for(int i=0;i<eigenvalues_vector.size();i++){
        std::pair<double,int>P=std::make_pair(eigenvalues_vector(i),i);
        V.push_back(P);
    }

    //order the vector containing the values and the index 
    //I order based on the values contained in first element of pairs (default for sort())
    sort(V.begin(),V.end());
    
    //lambda_2 is the second smallest value. To access the index I need .second
    lambda_2 = eigenvalues_vector(V[1].second);

    //calculate gradient of lambda_2
    Eigen::MatrixXd eigenvectors_matrix = es.eigenvectors().real(); //compute eigenvectors
    Eigen::VectorXd eigenvector_2 = eigenvectors_matrix.col(V[1].second);//take second smallest eigenvector

    
    for (int line_of_matrix=0;line_of_matrix<shared_matrix.size();line_of_matrix++)
    {
        if ((int) shared_matrix[line_of_matrix][0] == robot_id_int)
        {
            // Substitution of index for preventing segmentation_fault 
            for (int i = 0; i < robots_array.size(); i++)
            {
                if((int) shared_matrix[line_of_matrix][0] == robots_array[i])
                    index_i = new_robots_array[i];
                
                if((int) shared_matrix[line_of_matrix][1] == robots_array[i])
                    index_j = new_robots_array[i];
            }
            float d_i_j = distance_matrix(index_i, index_j);
            d_lambda_2_x = d_lambda_2_x -  exp(pow((pow(R,2.0) - pow(d_i_j,2.0)),2.0) / sigma) * 4.0 * (pow(R, 2.0) - pow(d_i_j,2.0)) / sigma * (shared_matrix[line_of_matrix][2]) * pow((eigenvector_2(index_i) - eigenvector_2(index_j)),2.0);
            d_lambda_2_y = d_lambda_2_y -  exp(pow((pow(R,2.0) - pow(d_i_j,2.0)),2.0) / sigma) * 4.0 * (pow(R, 2.0) - pow(d_i_j,2.0)) / sigma * (shared_matrix[line_of_matrix][3]) * pow((eigenvector_2(index_i) - eigenvector_2(index_j)),2.0);
            //break; //If I found my gradient I don't need to check again the matrix -> WRONG
        }
    }
    //fprintf(stdout, "I am: %d, d_lambda_2_x: %f, d_lambda_2_y: %f\n", robot_id_int, d_lambda_2_x, d_lambda_2_y);
    /* for (int i = 0; i < eigenvectors_matrix.cols(); i++)
    {
        //for (int j = 0 ; j < eigenvectors_matrix.rows() ; j++) 
        //{
            fprintf(stdout, " %f,", eigenvectors_matrix(i,index[1]));
        //}
        
    } */
    //fprintf(stdout, "\n------------\n");

    // std::ofstream myfile;
    // std::string temp;
    // std::stringstream ss;
    // ss << robot_id_int;
    // std::string numberAsString(ss.str());
    // temp = "saved_data/frequency_" + numberAsString + ".csv";
    // myfile.open(temp, std::ofstream::out | std::ofstream::app);

    // gettimeofday(&t2, NULL);
    // double elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000000 + (t2.tv_usec - t1.tv_usec); //[us]
    // fprintf(stdout, "lambda_2 elapsed time: %f [us]\n", elapsedTime);
    
    // myfile << "Elapsed " << elapsedTime << "\n";
    // myfile.close();
        
    // fprintf(stdout,"lambda_2: %f \n", lambda_2); // print results


    result.push_back(lambda_2);
    result.push_back(d_lambda_2_x);
    result.push_back(d_lambda_2_y);

    return result;
}




const std::vector<float> &lambda_2_class::get_solution() const {
    return result;
}
