#include "intermediate.h"
#include "include/QPSolver.h"
#include "include/lambda_2.h"

#ifdef __cplusplus
extern "C"
{
#endif

int qp_solver(buzzvm_t vm)
{
    int argn = buzzvm_lnum(vm);

    std::vector<double> a;
    std::vector<double> b;
    std::vector<double> c;

    buzzvm_lload(vm, 1);
    buzzvm_type_assert(vm, 1, BUZZTYPE_TABLE);
    buzzobj_t ta = buzzvm_stack_at(vm, 1);
    int size_a = buzzdict_size(ta->t.value);

    for(int32_t i = 0; i < size_a; ++i)
    {
        buzzvm_dup(vm);
        buzzvm_pushi(vm, i);
        buzzvm_tget(vm);
        buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
        a.push_back(buzzvm_stack_at(vm, 1)->f.value);
        buzzvm_pop(vm);
//        printf("a[%i]= \t %f\n", i, a[i]);
    }

    buzzvm_lload(vm, 2);
    buzzvm_type_assert(vm, 2, BUZZTYPE_TABLE);
    buzzobj_t tb = buzzvm_stack_at(vm, 1);
    int size_b = buzzdict_size(tb->t.value);

    for(int32_t i = 0; i < size_b; ++i)
    {
        buzzvm_dup(vm);
        buzzvm_pushi(vm, i);
        buzzvm_tget(vm);
        buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
        b.push_back(buzzvm_stack_at(vm, 1)->f.value);
        buzzvm_pop(vm);
//        printf("b[%i]= \t %f\n", i, b[i]);
    }
    buzzvm_lload(vm, 3);
    buzzvm_type_assert(vm, 3, BUZZTYPE_TABLE);
    buzzobj_t tc = buzzvm_stack_at(vm, 1);
    int size_c = buzzdict_size(tc->t.value);

    for(int32_t i = 0; i < size_c; ++i)
    {
        buzzvm_dup(vm);
        buzzvm_pushi(vm, i);
        buzzvm_tget(vm);
        buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
        c.push_back(buzzvm_stack_at(vm, 1)->f.value);
        buzzvm_pop(vm);
//        printf("c[%i]= \t %f\n", i, c[i]);
    }
    if (argn == 5)
    {
        std::vector<double> bndl;
        std::vector<double> bndu;

        buzzvm_lload(vm, 4);
        buzzvm_type_assert(vm, 4, BUZZTYPE_TABLE);
        buzzobj_t tbndl = buzzvm_stack_at(vm, 1);
        int size_bndl = buzzdict_size(tbndl->t.value);

        for(int32_t i = 0; i < size_bndl; ++i)
        {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, i);
            buzzvm_tget(vm);
            buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
            bndl.push_back(buzzvm_stack_at(vm, 1)->f.value);
            buzzvm_pop(vm);
        }

        buzzvm_lload(vm, 5);
        buzzvm_type_assert(vm, 5, BUZZTYPE_TABLE);
        buzzobj_t tbndu = buzzvm_stack_at(vm, 1);
        int size_bndu = buzzdict_size(tbndu->t.value);

        for(int32_t i = 0; i < size_bndu; ++i)
        {
            buzzvm_dup(vm);
            buzzvm_pushi(vm, i);
            buzzvm_tget(vm);
            buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
            bndu.push_back(buzzvm_stack_at(vm, 1)->f.value);
            buzzvm_pop(vm);
        }
        QPSolver solver = QPSolver(a, b, c, bndl, bndu);
        solver.solve();
        buzzvm_pusht(vm);
        buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
        for(size_t i = 0; i < solver.get_solution().size(); ++i) {
           buzzvm_push(vm, tProxTable);
		   buzzvm_pushi(vm, i);
		   buzzvm_pushf(vm, solver.get_solution()[i]);
		   buzzvm_tput(vm);
            // TablePut_real1(vm, tProxTable, i, solver.get_solution()[i]);
            // TablePutF(tProxTable, i,solver.get_solution()[i] , vm);
//        printf("solution[%i] %d\n", i, solver.get_solution()[i]);
        }
    } else {
        QPSolver solver = QPSolver(a, b, c);
        solver.solve();
        buzzvm_pusht(vm);
        buzzobj_t tProxTable = buzzvm_stack_at(vm, 1);
        for(size_t i = 0; i < solver.get_solution().size(); ++i) {
           buzzvm_push(vm, tProxTable);
		   buzzvm_pushi(vm, i);
		   buzzvm_pushf(vm, solver.get_solution()[i]);
		   buzzvm_tput(vm);
            // TablePut_real1(vm, tProxTable, i, solver.get_solution()[i]);
//        printf("solution[%i] %d\n", i, solver.get_solution()[i]);
        }
    }
    return buzzvm_ret1(vm);
}


//please notice that the next function overflows after 10000 seconds (2.77 hours)
int get_time_stamp(buzzvm_t vm) //the input to this function is a time (since 1970 in seconds) and this function returns the difference between the time since 1970 and the given time in seconds
{
    buzzvm_lload(vm, 1);
    buzzvm_type_assert(vm, 1, BUZZTYPE_INT);
    int start_time = buzzvm_stack_at(vm, 1)->i.value;

    time_t t;
    t = time(NULL);
    // int dt = t - start_time;
    int dt = t%10000 - start_time; //this returns the last four digits in t (which is a huge number) as an integer
    // printf("start time is: %d\nthe time elapsed now is:  %d \n",start_time,dt);

    buzzvm_pushs(vm, buzzvm_string_register(vm, "time_elapsed", 1));
    buzzvm_pushi(vm,dt);
    buzzvm_gstore(vm);


    return buzzvm_ret0(vm);
}

// int get_time_stamp_millis(buzzvm_t vm)
// {
//     buzzvm_lload(vm, 1);
//     buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
//     float start_time = buzzvm_stack_at(vm, 1)->f.value;

//     struct timeval tv;
//     gettimeofday(&tv, NULL);
//     int secs = tv.tv_sec%10000;
//     unsigned long ret = tv.tv_usec;
//     ret /= 1000;
//     float ms = ((float)ret)/1000.0;
//     float final_time = (double)secs + ms;
//     float dt = final_time - start_time;

//     // printf("start time is: %f\nthe time elapsed now is:  %f \n",start_time,dt);
//     buzzvm_pushs(vm, buzzvm_string_register(vm, "time_elapsed_ms", 1));
//     buzzvm_pushf(vm,dt);
//     buzzvm_gstore(vm);
//     return buzzvm_ret0(vm);
// }

int get_time_stamp_millis(buzzvm_t vm)
                    
{
    buzzvm_lload(vm, 1);
    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    float start_time = buzzvm_stack_at(vm, 1)->f.value;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    int secs = tv.tv_sec%10000;
    unsigned long ret = tv.tv_usec;
    ret /= 1000;
    float ms = ((float)ret)/1000.0;
    float final_time = (double)secs + ms;
    float dt = final_time - start_time;
    // printf("start time is: %f\nthe time elapsed now is:  %f \n",start_time,dt);
    buzzvm_pushf(vm,dt);
    
    return buzzvm_ret1(vm);
}

int BuzzLambda2Function(buzzvm_t vm) { //function that calculates lambda_2


    //first read the value pass from BUZZ
    buzzvm_lnum_assert(vm, 3);

    buzzvm_lload(vm, 1); 
    buzzvm_lload(vm, 2);
    buzzvm_lload(vm, 3); 

    // NB: if I have more then one data passed I access them from the last passed!
    buzzobj_t shared_table_table_obj = buzzvm_stack_at(vm, 3);

    buzzvm_type_assert(vm, 2, BUZZTYPE_INT);
    int robot_id_int = buzzvm_stack_at(vm, 2)->i.value;   

    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    float R = buzzvm_stack_at(vm, 1)->f.value;

    buzzobj_t t;


    //get shared table
    int32_t shared_table_table_obj_size = (shared_table_table_obj->t.value)->size;
    int shared_matrix_lines_number=(int)(shared_table_table_obj_size/5);
    //fprintf(stdout, "Number of rows of the shared_matrix %d \n", shared_matrix_lines_number);

    std::vector< std::vector<float> > shared_matrix;

    t = shared_table_table_obj;

    int i=0; //line counter
    int j=0; //column counter
    
    shared_matrix.resize(shared_matrix_lines_number);
    for(int idx = 0; idx < shared_table_table_obj_size; ++idx) {        
        buzzvm_push(vm,t);// push table       
        buzzvm_pushi(vm,idx);// push index        
        buzzvm_tget(vm);// get element at index i (this pops the table and the index)      
        buzzobj_t object_value = buzzvm_stack_at(vm,1); // now the stack has the value at the top// do something with buzzvm_stack_at(vm,1), which is a buzzobj_t
        if (object_value->o.type == BUZZTYPE_FLOAT) {
            shared_matrix[i].push_back(object_value->f.value);
        } 
        else if (object_value->o.type == BUZZTYPE_INT) {
            shared_matrix[i].push_back((float) object_value->i.value);
        }
        else
            fprintf(stdout,"\n-------unrecognized variable type-------\n");
        j=j+1;
        if(j==5){
            j=0;
            i++;
        }
    }

    printf("printing the shared matrix\n");
    for ( std::vector<std::vector<int> >::size_type i = 0; i < shared_matrix.size(); i++ )
    {
       for ( std::vector<int>::size_type j = 0; j < shared_matrix[i].size(); j++ )
       {
          std::cout << shared_matrix[i][j] << ' ';
       }
       std::cout << std::endl;
    }

    //end of reading

    lambda_2_class lambda_2_obj = lambda_2_class(shared_matrix, robot_id_int, R);
    lambda_2_obj.calculate();
    

    //return lambda_2, d_lambda_2_x and d_lambda_2_y values to BUZZ

    buzzvm_pushs(vm, buzzvm_string_register(vm, "lambda_2", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[0]);
    buzzvm_gstore(vm);

    buzzvm_pushs(vm, buzzvm_string_register(vm, "d_lambda_2_x", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[1]);
    buzzvm_gstore(vm);

    buzzvm_pushs(vm, buzzvm_string_register(vm, "d_lambda_2_y", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[2]);
    buzzvm_gstore(vm);


    return buzzvm_ret0(vm);
   
}

int BuzzLambda2DelayFunction(buzzvm_t vm) { //function that calculates lambda_2


    //first read the value pass from BUZZ
    buzzvm_lnum_assert(vm, 6);

    buzzvm_lload(vm, 1); 
    buzzvm_lload(vm, 2);
    buzzvm_lload(vm, 3); 
    buzzvm_lload(vm, 4);
    buzzvm_lload(vm, 5);
    buzzvm_lload(vm, 6);

    // NB: if I have more then one data passed I access them from the last passed!
    buzzobj_t shared_table_table_obj = buzzvm_stack_at(vm, 6);

    buzzvm_type_assert(vm, 5, BUZZTYPE_INT);
    int robot_id_int = buzzvm_stack_at(vm, 5)->i.value;   

    buzzvm_type_assert(vm, 4, BUZZTYPE_FLOAT);
    float R = buzzvm_stack_at(vm, 4)->f.value;

    buzzvm_type_assert(vm, 3, BUZZTYPE_INT);
    int delay = buzzvm_stack_at(vm, 3)->i.value;

    buzzvm_type_assert(vm, 2, BUZZTYPE_FLOAT);
    float time_step = buzzvm_stack_at(vm, 2)->f.value;

    buzzvm_type_assert(vm, 1, BUZZTYPE_FLOAT);
    float max_vel = buzzvm_stack_at(vm, 1)->f.value;

    buzzobj_t t;


    //get shared table
    int32_t shared_table_table_obj_size = (shared_table_table_obj->t.value)->size;
    int shared_matrix_lines_number=(int)(shared_table_table_obj_size/5);
    //fprintf(stdout, "Number of rows of the shared_matrix %d \n", shared_matrix_lines_number);

    std::vector< std::vector<float> > shared_matrix;

    t = shared_table_table_obj;

    int i=0; //line counter
    int j=0; //column counter
    
    shared_matrix.resize(shared_matrix_lines_number);
    for(int idx = 0; idx < shared_table_table_obj_size; ++idx) {        
        buzzvm_push(vm,t);// push table       
        buzzvm_pushi(vm,idx);// push index        
        buzzvm_tget(vm);// get element at index i (this pops the table and the index)      
        buzzobj_t object_value = buzzvm_stack_at(vm,1); // now the stack has the value at the top// do something with buzzvm_stack_at(vm,1), which is a buzzobj_t
        if (object_value->o.type == BUZZTYPE_FLOAT) {
            shared_matrix[i].push_back(object_value->f.value);
        } 
        else if (object_value->o.type == BUZZTYPE_INT) {
            shared_matrix[i].push_back((float) object_value->i.value);
        }
        else
            fprintf(stdout,"\n-------unrecognized variable type-------\n");
        j=j+1;
        if(j==5){
            j=0;
            i++;
        }
    }

    

    //end of reading

    lambda_2_class lambda_2_obj = lambda_2_class(shared_matrix, robot_id_int, R);
    lambda_2_obj.calculate_with_delay((float)delay, time_step, max_vel);
    

    //return lambda_2, d_lambda_2_x and d_lambda_2_y values to BUZZ

    buzzvm_pushs(vm, buzzvm_string_register(vm, "lambda_2", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[0]);
    buzzvm_gstore(vm);

    buzzvm_pushs(vm, buzzvm_string_register(vm, "d_lambda_2_x", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[1]);
    buzzvm_gstore(vm);

    buzzvm_pushs(vm, buzzvm_string_register(vm, "d_lambda_2_y", 1));
    buzzvm_pushf(vm,lambda_2_obj.get_solution()[2]);
    buzzvm_gstore(vm);


    return buzzvm_ret0(vm);
   
}


#ifdef __cplusplus
} // extern "C"
#endif
