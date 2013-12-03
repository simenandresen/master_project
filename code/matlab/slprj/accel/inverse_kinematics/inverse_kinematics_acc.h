#include "__cf_inverse_kinematics.h"
#ifndef RTW_HEADER_inverse_kinematics_acc_h_
#define RTW_HEADER_inverse_kinematics_acc_h_
#ifndef inverse_kinematics_acc_COMMON_INCLUDES_
#define inverse_kinematics_acc_COMMON_INCLUDES_
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "mwmathutil.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#endif
#include "inverse_kinematics_acc_types.h"
typedef struct { real_T B_8_0_0 [ 6 ] ; real_T B_8_6_0 [ 12 ] ; real_T
B_8_11_0 [ 3 ] ; real_T B_8_15_0 [ 6 ] ; real_T B_8_17_0 [ 6 ] ; real_T
B_8_18_0 [ 6 ] ; real_T B_8_23_0 [ 6 ] ; real_T B_8_25_0 [ 12 ] ; real_T
B_8_28_0 [ 6 ] ; real_T B_8_34_0 [ 3 ] ; real_T B_8_35_0 [ 6 ] ; real_T
B_7_0_1 [ 6 ] ; real_T B_6_0_1 [ 72 ] ; real_T B_5_0_1 [ 144 ] ; real_T
B_4_0_1 [ 12 ] ; real_T B_3_0_1 [ 6 ] ; real_T B_2_0_1 [ 6 ] ; real_T B_1_0_1
[ 6 ] ; real_T B_0_0_1 [ 72 ] ; real_T B_8_20_0 [ 12 ] ; real_T B_8_26_0 [ 6
] ; uint8_T B_8_13_0 ; uint8_T B_8_14_0 ; uint8_T B_8_21_0 ; uint8_T B_8_22_0
; char pad_B_8_22_0 [ 4 ] ; } BlockIO_inverse_kinematics ; typedef struct {
real_T NextOutput ; void * ToWorkspace1_PWORK ; void * ToWorkspace_PWORK ;
void * ToWorkspace1_PWORK_b ; void * ToWorkspace2_PWORK ; void *
ToWorkspace_PWORK_f ; void * ToWorkspace1_PWORK_p ; void *
ToWorkspace2_PWORK_f ; void * ToWorkspace_PWORK_g ; void *
ToWorkspace1_PWORK_m ; void * ToWorkspace2_PWORK_a ; uint32_T RandSeed ; char
pad_RandSeed [ 4 ] ; } D_Work_inverse_kinematics ; typedef struct { real_T
Integrator_CSTATE [ 6 ] ; real_T TransferFcn_CSTATE ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn2_CSTATE ; real_T Integrator1_CSTATE
[ 6 ] ; real_T Integrator_CSTATE_a [ 6 ] ; real_T Integrator_CSTATE_c [ 6 ] ;
} ContinuousStates_inverse_kinematics ; typedef struct { real_T
Integrator_CSTATE [ 6 ] ; real_T TransferFcn_CSTATE ; real_T
TransferFcn1_CSTATE ; real_T TransferFcn2_CSTATE ; real_T Integrator1_CSTATE
[ 6 ] ; real_T Integrator_CSTATE_a [ 6 ] ; real_T Integrator_CSTATE_c [ 6 ] ;
} StateDerivatives_inverse_kinematics ; typedef struct { boolean_T
Integrator_CSTATE [ 6 ] ; boolean_T TransferFcn_CSTATE ; boolean_T
TransferFcn1_CSTATE ; boolean_T TransferFcn2_CSTATE ; boolean_T
Integrator1_CSTATE [ 6 ] ; boolean_T Integrator_CSTATE_a [ 6 ] ; boolean_T
Integrator_CSTATE_c [ 6 ] ; } StateDisabled_inverse_kinematics ; struct
Parameters_inverse_kinematics_ { real_T P_0 [ 6 ] ; real_T P_1 [ 12 ] ;
real_T P_2 ; real_T P_3 ; real_T P_4 ; real_T P_7 ; real_T P_8 ; real_T P_9 ;
real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_17 [ 3 ] ; real_T P_18 ;
real_T P_19 [ 6 ] ; real_T P_20 [ 6 ] ; real_T P_21 [ 6 ] ; real_T P_22 [ 6 ]
; real_T P_23 ; real_T P_24 ; real_T P_25 ; real_T P_26 [ 3 ] ; real_T P_27 [
6 ] ; uint8_T P_28 ; uint8_T P_29 ; uint8_T P_30 ; uint8_T P_31 ; char
pad_P_31 [ 4 ] ; } ; extern Parameters_inverse_kinematics
inverse_kinematics_rtDefaultParameters ;
#endif
