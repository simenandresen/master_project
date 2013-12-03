#include "__cf_dynamics_kinematics.h"
#if 0
This file is not available for use in any application other than as a MATLAB
( R ) MEX file for use with the Simulink ( R ) product . If you do not have
the Simulink Coder licensed , this file contains encrypted block names , etc
. If you purchase the Simulink Coder , this file will contain full block
descriptions and improved variable names .
#endif
#include <math.h>
#include "dynamics_kinematics_acc.h"
#include "dynamics_kinematics_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
static void mdlOutputs ( SimStruct * S , int_T tid ) { real_T B_15_52_0 ;
real_T B_15_53_0 ; int32_T i ; int32_T i_0 ; int32_T i_1 ;
BlockIO_dynamics_kinematics * _rtB ; Parameters_dynamics_kinematics * _rtP ;
D_Work_dynamics_kinematics * _rtDW ; _rtDW = ( ( D_Work_dynamics_kinematics *
) ssGetRootDWork ( S ) ) ; _rtP = ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 3 , 0 , SS_CALL_MDL_OUTPUTS
) ; { int_T i1 ; real_T * y0 = ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) -> B_15_1_0 ; real_T * xc = & ( (
ContinuousStates_dynamics_kinematics * ) ssGetContStates ( S ) ) ->
Integrator1_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [
i1 ] ; } } ssCallAccelRunBlock ( S , 0 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i <
12 ; i ++ ) { for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { _rtB -> B_15_4_0 [ i + 12
* i_1 ] = 0.0 ; for ( i_0 = 0 ; i_0 < 6 ; i_0 ++ ) { _rtB -> B_15_4_0 [ i +
12 * i_1 ] = _rtB -> B_4_0_1 [ 12 * i_0 + i ] * _rtB -> B_0_0_1 [ 6 * i_1 +
i_0 ] + _rtB -> B_15_4_0 [ 12 * i_1 + i ] ; } } } for ( i = 0 ; i < 144 ; i
++ ) { _rtB -> B_15_5_0 [ i ] = _rtB -> B_3_0_1 [ i ] - _rtB -> B_15_4_0 [ i
] ; } if ( ssIsSampleHit ( S , 1 , 0 ) ) { memcpy ( & _rtB -> B_15_6_0 [ 0 ]
, & _rtP -> P_1 [ 0 ] , 12U * sizeof ( real_T ) ) ; } for ( i = 0 ; i < 12 ;
i ++ ) { _rtB -> B_15_7_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) {
_rtB -> B_15_7_0 [ i ] += _rtB -> B_15_5_0 [ 12 * i_1 + i ] * _rtB ->
B_15_6_0 [ i_1 ] ; } } if ( ssIsSampleHit ( S , 1 , 0 ) ) { for ( i = 0 ; i <
6 ; i ++ ) { _rtB -> B_15_8_0 [ i ] = _rtP -> P_2 [ i ] ; } } { int_T i1 ;
real_T * y0 = ( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) ->
B_15_9_0 ; real_T * xc = & ( ( ContinuousStates_dynamics_kinematics * )
ssGetContStates ( S ) ) -> Integrator1_CSTATE_c [ 0 ] ; for ( i1 = 0 ; i1 < 6
; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } for ( i = 0 ; i < 6 ; i ++ ) { for (
i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { _rtB -> B_15_10_0 [ i_1 + 12 * i ] = _rtB ->
B_0_0_1 [ 6 * i_1 + i ] ; } } ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) -> B_15_11_0 = ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) -> P_6 * ( ( ContinuousStates_dynamics_kinematics *
) ssGetContStates ( S ) ) -> TransferFcn_CSTATE ; { int_T i1 ; real_T * y0 =
( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_12_0 ;
real_T * xc = & ( ( ContinuousStates_dynamics_kinematics * ) ssGetContStates
( S ) ) -> Integrator2_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [
i1 ] = xc [ i1 ] ; } } _rtB -> B_15_13_0 = _rtB -> B_15_11_0 - _rtB ->
B_15_12_0 [ 1 ] ; _rtB -> B_15_14_0 = _rtP -> P_10 * _rtB -> B_15_13_0 ; {
int_T i1 ; real_T * y0 = ( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO (
S ) ) -> B_15_15_0 ; real_T * xc = & ( ( ContinuousStates_dynamics_kinematics
* ) ssGetContStates ( S ) ) -> Integrator_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 <
12 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } for ( i = 0 ; i < 6 ; i ++ ) {
_rtB -> B_15_16_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { _rtB ->
B_15_16_0 [ i ] += _rtB -> B_0_0_1 [ 6 * i_1 + i ] * _rtB -> B_15_15_0 [ i_1
] ; } } _rtB -> B_15_17_0 = _rtP -> P_12 * _rtB -> B_15_16_0 [ 1 ] ; _rtB ->
B_15_18_0 = _rtB -> B_15_14_0 + _rtB -> B_15_17_0 ; ssCallAccelRunBlock ( S ,
7 , 0 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i < 12 ; i ++ ) { _rtB ->
B_15_20_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 6 ; i_1 ++ ) { _rtB ->
B_15_20_0 [ i ] += _rtB -> B_15_10_0 [ 12 * i_1 + i ] * _rtB -> B_7_0_1 [ i_1
] ; } _rtB -> B_15_21_0 [ i ] = _rtP -> P_13 * _rtB -> B_15_20_0 [ i ] ; } if
( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> B_15_22_0 = _rtDW ->
UnitDelay_DSTATE ; } ssCallAccelRunBlock ( S , 14 , 0 , SS_CALL_MDL_OUTPUTS )
; for ( i = 0 ; i < 6 ; i ++ ) { _rtB -> B_15_24_0 [ i ] = _rtP -> P_15 *
_rtB -> B_14_0_1 [ i ] ; _rtB -> B_15_25_0 [ i ] = _rtB -> B_15_8_0 [ i ] +
_rtB -> B_15_24_0 [ i ] ; } for ( i = 0 ; i < 12 ; i ++ ) { _rtB -> B_15_26_0
[ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 6 ; i_1 ++ ) { _rtB -> B_15_26_0 [ i ] +=
_rtB -> B_4_0_1 [ 12 * i_1 + i ] * _rtB -> B_15_25_0 [ i_1 ] ; } _rtB ->
B_15_27_0 [ i ] = _rtB -> B_15_7_0 [ i ] + _rtB -> B_15_26_0 [ i ] ; } if (
ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> B_15_28_0 = _rtP -> P_30 ; ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_29_0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_28_0 ; } {
int_T i1 ; real_T * y0 = ( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO (
S ) ) -> B_15_30_0 ; real_T * xc = & ( ( ContinuousStates_dynamics_kinematics
* ) ssGetContStates ( S ) ) -> Integrator2_CSTATE_b [ 0 ] ; for ( i1 = 0 ; i1
< 6 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } ssCallAccelRunBlock ( S , 1 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 15 , 32 ,
SS_CALL_MDL_OUTPUTS ) ; { int_T i1 ; real_T * y0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_33_0 ; real_T *
xc = & ( ( ContinuousStates_dynamics_kinematics * ) ssGetContStates ( S ) )
-> Integrator1_CSTATE_b [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] =
xc [ i1 ] ; } } { int_T i1 ; real_T * y0 = ( ( BlockIO_dynamics_kinematics *
) _ssGetBlockIO ( S ) ) -> B_15_34_0 ; real_T * xc = & ( (
ContinuousStates_dynamics_kinematics * ) ssGetContStates ( S ) ) ->
Integrator_CSTATE_f [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc
[ i1 ] ; } } ssCallAccelRunBlock ( S , 15 , 35 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 2 , 0 , SS_CALL_MDL_OUTPUTS ) ; { int_T i1 ; real_T
* y0 = ( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_37_0
; real_T * xc = & ( ( ContinuousStates_dynamics_kinematics * )
ssGetContStates ( S ) ) -> Integrator_CSTATE_a [ 0 ] ; for ( i1 = 0 ; i1 < 6
; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } ssCallAccelRunBlock ( S , 15 , 38 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_OUTPUTS
) ; ssCallAccelRunBlock ( S , 13 , 0 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ;
i < 12 ; i ++ ) { _rtB -> B_15_41_0 [ i ] = _rtB -> B_15_32_0 [ i ] - _rtB ->
B_15_15_0 [ i ] ; } for ( i = 0 ; i < 12 ; i ++ ) { _rtB -> B_15_42_0 [ i ] =
0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { _rtB -> B_15_42_0 [ i ] += _rtP
-> P_20 [ 12 * i_1 + i ] * _rtB -> B_15_41_0 [ i_1 ] ; } } for ( i = 0 ; i <
6 ; i ++ ) { _rtB -> B_15_43_0 [ i ] = _rtB -> B_15_33_0 [ i ] - _rtB ->
B_15_30_0 [ i ] ; } for ( i = 0 ; i < 6 ; i ++ ) { _rtB -> B_15_43_0 [ i + 6
] = _rtB -> B_15_34_0 [ i ] - _rtB -> B_15_1_0 [ i ] ; } for ( i = 0 ; i < 12
; i ++ ) { _rtB -> B_15_44_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++
) { _rtB -> B_15_44_0 [ i ] += _rtP -> P_21 [ 12 * i_1 + i ] * _rtB ->
B_15_43_0 [ i_1 ] ; } _rtB -> B_15_45_0 [ i ] = _rtB -> B_15_42_0 [ i ] +
_rtB -> B_15_44_0 [ i ] ; } for ( i = 0 ; i < 12 ; i ++ ) { _rtB -> B_15_46_0
[ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { _rtB -> B_15_46_0 [ i ]
+= _rtB -> B_13_0_1 [ 12 * i_1 + i ] * _rtB -> B_15_45_0 [ i_1 ] ; } }
ssCallAccelRunBlock ( S , 15 , 47 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 15 , 48 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 6 , 0 , SS_CALL_MDL_OUTPUTS ) ; ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_50_0 = ( (
Parameters_dynamics_kinematics * ) ssGetDefaultParam ( S ) ) -> P_24 * ( (
ContinuousStates_dynamics_kinematics * ) ssGetContStates ( S ) ) ->
TransferFcn_CSTATE_h ; B_15_52_0 = ( _rtB -> B_15_50_0 - _rtB -> B_15_12_0 [
5 ] ) * _rtP -> P_27 ; B_15_53_0 = _rtP -> P_28 * _rtB -> B_15_16_0 [ 5 ] ;
ssCallAccelRunBlock ( S , 8 , 0 , SS_CALL_MDL_OUTPUTS ) ; _rtB -> B_15_55_0 =
B_15_52_0 + B_15_53_0 ; ssCallAccelRunBlock ( S , 9 , 0 , SS_CALL_MDL_OUTPUTS
) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { memcpy ( & _rtB -> B_15_57_0 [ 0 ] ,
& _rtP -> P_29 [ 0 ] , 12U * sizeof ( real_T ) ) ; } for ( i = 0 ; i < 12 ; i
++ ) { _rtB -> B_15_58_0 [ i ] = ( _rtB -> B_15_21_0 [ i ] + _rtB ->
B_15_57_0 [ i ] ) + _rtB -> B_15_46_0 [ i ] ; } ssCallAccelRunBlock ( S , 10
, 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 11 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 12 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 15 , 62 ,
SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i < 6 ; i ++ ) { _rtB -> B_15_63_0 [ i
] = _rtB -> B_15_8_0 [ i ] - _rtB -> B_15_16_0 [ i ] ; } UNUSED_PARAMETER (
tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) {
BlockIO_dynamics_kinematics * _rtB ; _rtB = ( ( BlockIO_dynamics_kinematics *
) _ssGetBlockIO ( S ) ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { ( (
D_Work_dynamics_kinematics * ) ssGetRootDWork ( S ) ) -> UnitDelay_DSTATE =
_rtB -> B_14_0_2 ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { BlockIO_dynamics_kinematics *
_rtB ; _rtB = ( ( BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) ; { {
int_T i1 ; const real_T * u0 = & ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) -> B_15_15_0 [ 6 ] ; real_T * xdot = & ( (
StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator1_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] = u0
[ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_63_0 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator1_CSTATE_c [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S
) ) -> TransferFcn_CSTATE = ( ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) -> P_4 ) * ( ( ContinuousStates_dynamics_kinematics
* ) ssGetContStates ( S ) ) -> TransferFcn_CSTATE ; ( (
StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn_CSTATE += ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) -> P_5 * ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) -> B_6_0_1 ; } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_11_0_1 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator2_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] = u0
[ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_10_0_1 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 12 ; i1 ++ ) { xdot [ i1 ] = u0
[ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_12_0_1 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator2_CSTATE_b [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_2_0_1 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator1_CSTATE_b [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = & ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_15_32_0 [ 6 ] ;
real_T * xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S )
) -> Integrator_CSTATE_f [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1
] = u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_dynamics_kinematics * ) _ssGetBlockIO ( S ) ) -> B_5_0_1 ; real_T *
xdot = & ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S ) ) ->
Integrator_CSTATE_a [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX ( S
) ) -> TransferFcn_CSTATE_h = ( ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) -> P_22 ) * ( (
ContinuousStates_dynamics_kinematics * ) ssGetContStates ( S ) ) ->
TransferFcn_CSTATE_h ; ( ( StateDerivatives_dynamics_kinematics * ) ssGetdX (
S ) ) -> TransferFcn_CSTATE_h += ( ( Parameters_dynamics_kinematics * )
ssGetDefaultParam ( S ) ) -> P_23 * ( ( BlockIO_dynamics_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_0_1 ; } } static void mdlInitializeSizes (
SimStruct * S ) { ssSetChecksumVal ( S , 0 , 1131637354U ) ; ssSetChecksumVal
( S , 1 , 1132714696U ) ; ssSetChecksumVal ( S , 2 , 2445376483U ) ;
ssSetChecksumVal ( S , 3 , 36873434U ) ; { mxArray * slVerStructMat = NULL ;
mxArray * slStrMat = mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ;
int status = mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" )
; if ( status == 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 ,
"Version" ) ; if ( slVerMat == NULL ) { status = 1 ; } else { status =
mxGetString ( slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "7.9" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
D_Work_dynamics_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( BlockIO_dynamics_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
Parameters_dynamics_kinematics ) ) { static char msg [ 256 ] ; sprintf ( msg
, "Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & dynamics_kinematics_rtDefaultParameters ) ; rt_InitInfAndNaN (
sizeof ( real_T ) ) ; } static void mdlInitializeSampleTimes ( SimStruct * S
) { { SimStruct * childS ; SysOutputFcn * callSysFcns ; childS =
ssGetSFunction ( S , 0 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 1 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 2 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 3 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 4 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 5 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 6 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 7 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 8 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 9 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 10 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 11 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 12 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 13 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS =
ssGetSFunction ( S , 14 ) ; callSysFcns = ssGetCallSystemOutputFcnList (
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; } } static
void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
