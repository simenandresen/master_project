#include "__cf_kinematics.h"
#ifndef RTW_HEADER_kinematics_acc_h_
#define RTW_HEADER_kinematics_acc_h_
#ifndef kinematics_acc_COMMON_INCLUDES_
#define kinematics_acc_COMMON_INCLUDES_
#include <stdlib.h>
#include <stddef.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#endif
#include "kinematics_acc_types.h"
typedef struct { real_T B_7_1_0 [ 6 ] ; real_T B_7_6_0 [ 6 ] ; real_T B_7_7_0
[ 3 ] ; real_T B_7_8_0 [ 3 ] ; real_T B_7_9_0 [ 3 ] ; real_T B_7_13_0 [ 12 ]
; real_T B_7_15_0 [ 6 ] ; real_T B_7_18_0 [ 6 ] ; real_T B_7_19_0 [ 6 ] ;
real_T B_7_20_0 [ 6 ] ; real_T B_6_0_1 [ 6 ] ; real_T B_5_0_1 [ 6 ] ; real_T
B_4_0_1 [ 6 ] ; real_T B_3_0_1 [ 72 ] ; real_T B_2_0_1 [ 144 ] ; real_T
B_1_0_1 [ 12 ] ; real_T B_0_0_1 [ 72 ] ; } BlockIO_kinematics ; typedef
struct { real_T UnitDelay2_DSTATE [ 3 ] ; real_T UnitDelay_DSTATE [ 3 ] ;
real_T UnitDelay1_DSTATE [ 3 ] ; void * ToWorkspace1_1_PWORK ; void *
ToWorkspace_1_PWORK ; void * ToWorkspace2_1_PWORK ; char
pad_ToWorkspace2_1_PWORK [ 4 ] ; } D_Work_kinematics ; typedef struct {
real_T Integrator1_CSTATE [ 6 ] ; real_T Integrator_CSTATE [ 6 ] ; real_T
Integrator2_CSTATE [ 6 ] ; real_T Integrator2_CSTATE_b [ 6 ] ; }
ContinuousStates_kinematics ; typedef struct { real_T Integrator1_CSTATE [ 6
] ; real_T Integrator_CSTATE [ 6 ] ; real_T Integrator2_CSTATE [ 6 ] ; real_T
Integrator2_CSTATE_b [ 6 ] ; } StateDerivatives_kinematics ; typedef struct {
boolean_T Integrator1_CSTATE [ 6 ] ; boolean_T Integrator_CSTATE [ 6 ] ;
boolean_T Integrator2_CSTATE [ 6 ] ; boolean_T Integrator2_CSTATE_b [ 6 ] ; }
StateDisabled_kinematics ; struct Parameters_kinematics_ { real_T P_0 [ 2 ] ;
real_T P_1 [ 6 ] ; real_T P_2 [ 2 ] ; real_T P_3 [ 6 ] ; real_T P_4 [ 6 ] ;
real_T P_5 [ 6 ] ; real_T P_6 [ 3 ] ; real_T P_7 [ 3 ] ; real_T P_8 [ 3 ] ;
real_T P_9 [ 6 ] ; real_T P_10 [ 6 ] ; real_T P_11 [ 6 ] ; } ; extern
Parameters_kinematics kinematics_rtDefaultParameters ;
#endif
