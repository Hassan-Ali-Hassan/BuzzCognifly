#ifndef MAGIC_H
#define MAGIC_H
// #include "include/VoronoiDiagramGenerator.h"
// #include "buzz/buzz_utility.c"
#ifdef __cplusplus
extern "C"
{
#endif
#include <buzz/buzzvm.h>
#include "buzzkh4_closures.h"
// #include "include/QPSolver.h"

int qp_solver(buzzvm_t vm);

#ifdef __cplusplus
} // extern "C"
#endif

#endif