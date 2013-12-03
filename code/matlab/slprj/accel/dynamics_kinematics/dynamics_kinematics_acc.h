#include "__cf_dynamics_kinematics.h"
#ifndef RTW_HEADER_dynamics_kinematics_acc_h_
#define RTW_HEADER_dynamics_kinematics_acc_h_
#ifndef dynamics_kinematics_acc_COMMON_INCLUDES_
#define dynamics_kinematics_acc_COMMON_INCLUDES_
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#define S_FUNCTION_NAME simulink_only_sfcn 
#define S_FUNCTION_LEVEL 2
#define RTW_GENERATED_S_FUNCTION
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#endif
#include "dynamics_kinematics_acc_types.h"
typedef struct { real_T B_15_1_0 [ 6 ] ; real_T B_15_4_0 [ 144 ] ; real_T
B_15_5_0 [ 144 ] ; real_T B_15_6_0 [ 12 ] ; real_T B_15_7_0 [ 12 ] ; real_T
B_15_8_0 [ 6 ] ; real_T B_15_9_0 [ 6 ] ; real_T B_15_10_0 [ 72 ] ; real_T
B_15_11_0 ; real_T B_15_12_0 [ 6 ] ; real_T B_15_13_0 ; real_T B_15_14_0 ;
real_T B_15_15_0 [ 12 ] ; real_T B_15_16_0 [ 6 ] ; real_T B_15_17_0 ; real_T
B_15_18_0 ; real_T B_15_20_0 [ 12 ] ; real_T B_15_21_0 [ 12 ] ; real_T
B_15_22_0 ; real_T B_15_24_0 [ 6 ] ; real_T B_15_25_0 [ 6 ] ; real_T
B_15_26_0 [ 12 ] ; real_T B_15_27_0 [ 12 ] ; real_T B_15_30_0 [ 6 ] ; real_T
B_15_32_0 [ 12 ] ; real_T B_15_33_0 [ 6 ] ; real_T B_15_34_0 [ 6 ] ; real_T
B_15_37_0 [ 6 ] ; real_T B_15_41_0 [ 12 ] ; real_T B_15_42_0 [ 12 ] ; real_T
B_15_43_0 [ 12 ] ; real_T B_15_44_0 [ 12 ] ; real_T B_15_45_0 [ 12 ] ; real_T
B_15_46_0 [ 12 ] ; real_T B_15_50_0 ; real_T B_15_55_0 ; real_T B_15_57_0 [
12 ] ; real_T B_15_58_0 [ 12 ] ; real_T B_15_63_0 [ 6 ] ; real_T B_14_0_1 [ 6
] ; real_T B_14_0_2 ; real_T B_14_0_3 ; real_T B_13_0_1 [ 144 ] ; real_T
B_12_0_1 [ 6 ] ; real_T B_11_0_1 [ 6 ] ; real_T B_10_0_1 [ 12 ] ; real_T
B_9_0_1 [ 6 ] ; real_T B_8_0_1 ; real_T B_7_0_1 [ 6 ] ; real_T B_6_0_1 ;
real_T B_5_0_1 [ 6 ] ; real_T B_4_0_1 [ 72 ] ; real_T B_3_0_1 [ 144 ] ;
real_T B_2_0_1 [ 6 ] ; real_T B_1_0_1 [ 12 ] ; real_T B_0_0_1 [ 72 ] ;
uint8_T B_15_28_0 ; uint8_T B_15_29_0 ; char pad_B_15_29_0 [ 6 ] ; }
BlockIO_dynamics_kinematics ; typedef struct { real_T UnitDelay_DSTATE ; void
* ToWorkspace1_1_PWORK ; void * ToWorkspace_1_PWORK ; void *
ToWorkspace2_1_PWORK ; void * ToWorkspace_1_PWORK_o ; void *
ToWorkspace2_1_PWORK_n ; char pad_ToWorkspace2_1_PWORK_n [ 4 ] ; }
D_Work_dynamics_kinematics ; typedef struct { real_T Integrator1_CSTATE [ 6 ]
; real_T Integrator1_CSTATE_c [ 6 ] ; real_T TransferFcn_CSTATE ; real_T
Integrator2_CSTATE [ 6 ] ; real_T Integrator_CSTATE [ 12 ] ; real_T
Integrator2_CSTATE_b [ 6 ] ; real_T Integrator1_CSTATE_b [ 6 ] ; real_T
Integrator_CSTATE_f [ 6 ] ; real_T Integrator_CSTATE_a [ 6 ] ; real_T
TransferFcn_CSTATE_h ; } ContinuousStates_dynamics_kinematics ; typedef
struct { real_T Integrator1_CSTATE [ 6 ] ; real_T Integrator1_CSTATE_c [ 6 ]
; real_T TransferFcn_CSTATE ; real_T Integrator2_CSTATE [ 6 ] ; real_T
Integrator_CSTATE [ 12 ] ; real_T Integrator2_CSTATE_b [ 6 ] ; real_T
Integrator1_CSTATE_b [ 6 ] ; real_T Integrator_CSTATE_f [ 6 ] ; real_T
Integrator_CSTATE_a [ 6 ] ; real_T TransferFcn_CSTATE_h ; }
StateDerivatives_dynamics_kinematics ; typedef struct { boolean_T
Integrator1_CSTATE [ 6 ] ; boolean_T Integrator1_CSTATE_c [ 6 ] ; boolean_T
TransferFcn_CSTATE ; boolean_T Integrator2_CSTATE [ 6 ] ; boolean_T
Integrator_CSTATE [ 12 ] ; boolean_T Integrator2_CSTATE_b [ 6 ] ; boolean_T
Integrator1_CSTATE_b [ 6 ] ; boolean_T Integrator_CSTATE_f [ 6 ] ; boolean_T
Integrator_CSTATE_a [ 6 ] ; boolean_T TransferFcn_CSTATE_h ; }
StateDisabled_dynamics_kinematics ; struct Parameters_dynamics_kinematics_ {
real_T P_0 [ 6 ] ; real_T P_1 [ 12 ] ; real_T P_2 [ 6 ] ; real_T P_3 ; real_T
P_4 ; real_T P_5 ; real_T P_6 ; real_T P_9 [ 6 ] ; real_T P_10 ; real_T P_11
; real_T P_12 ; real_T P_13 ; real_T P_14 ; real_T P_15 ; real_T P_16 [ 6 ] ;
real_T P_17 [ 6 ] ; real_T P_18 [ 6 ] ; real_T P_19 [ 6 ] ; real_T P_20 [ 144
] ; real_T P_21 [ 144 ] ; real_T P_22 ; real_T P_23 ; real_T P_24 ; real_T
P_27 ; real_T P_28 ; real_T P_29 [ 12 ] ; uint8_T P_30 ; uint8_T P_31 ; char
pad_P_31 [ 6 ] ; } ; extern Parameters_dynamics_kinematics
dynamics_kinematics_rtDefaultParameters ;
#endif
