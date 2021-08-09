#include "intermediate.h"
#include "include/QPSolver.h"

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

#ifdef __cplusplus
} // extern "C"
#endif
