#ifndef MAGIC_H
#define MAGIC_H
// #include "include/VoronoiDiagramGenerator.h"
// #include "buzz/buzz_utility.c"

#ifdef __cplusplus
extern "C"
{
#endif
#include <buzz/buzzvm.h>
#include "buzzcognifly_closures.h"
#include <sys/time.h>
#include <stdlib.h> 



int qp_solver(buzzvm_t vm);
int get_time_stamp(buzzvm_t vm);
int get_time_stamp_millis(buzzvm_t vm);
int random_number_generator(buzzvm_t vm);

int BuzzLambda2Function(buzzvm_t vm);
int BuzzLambda2DelayFunction(buzzvm_t vm);

#ifdef __cplusplus
} // extern "C"
#endif

#endif