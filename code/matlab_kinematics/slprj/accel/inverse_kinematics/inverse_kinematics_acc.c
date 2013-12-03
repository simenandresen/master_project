#include "__cf_inverse_kinematics.h"
#if 0
This file is not available for use in any application other than as a MATLAB
( R ) MEX file for use with the Simulink ( R ) product . If you do not have
the Simulink Coder licensed , this file contains encrypted block names , etc
. If you purchase the Simulink Coder , this file will contain full block
descriptions and improved variable names .
#endif
#include <math.h>
#include "inverse_kinematics_acc.h"
#include "inverse_kinematics_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) { uint32_T lo ; uint32_T
hi ; lo = * u % 127773U * 16807U ; hi = * u / 127773U * 2836U ; if ( lo < hi
) { * u = 2147483647U - ( hi - lo ) ; } else { * u = lo - hi ; } return (
real_T ) * u * 4.6566128752457969E-10 ; } real_T rt_nrand_Upu32_Yd_f_pw_snf (
uint32_T * u ) { real_T sr ; real_T si ; do { sr = 2.0 *
rt_urand_Upu32_Yd_f_pw_snf ( u ) - 1.0 ; si = 2.0 *
rt_urand_Upu32_Yd_f_pw_snf ( u ) - 1.0 ; si = sr * sr + si * si ; } while (
si > 1.0 ) ; return muDoubleScalarSqrt ( - 2.0 * muDoubleScalarLog ( si ) /
si ) * sr ; } static void mdlOutputs ( SimStruct * S , int_T tid ) { real_T
B_8_8_0 ; real_T B_8_9_0 ; real_T B_8_10_0 ; real_T B_8_7_0 [ 12 ] ; int32_T
i ; real_T tmp [ 144 ] ; int32_T i_0 ; real_T tmp_0 ; int32_T i_1 ;
BlockIO_inverse_kinematics * _rtB ; Parameters_inverse_kinematics * _rtP ;
D_Work_inverse_kinematics * _rtDW ; _rtDW = ( ( D_Work_inverse_kinematics * )
ssGetRootDWork ( S ) ) ; _rtP = ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) ; { int_T i1 ; real_T * y0 = ( (
BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_8_0_0 ; real_T * xc
= & ( ( ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
Integrator_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [
i1 ] ; } } ssCallAccelRunBlock ( S , 0 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 6 , 0 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) {
memcpy ( & _rtB -> B_8_6_0 [ 0 ] , & _rtP -> P_1 [ 0 ] , 12U * sizeof (
real_T ) ) ; } for ( i = 0 ; i < 12 ; i ++ ) { for ( i_1 = 0 ; i_1 < 12 ; i_1
++ ) { tmp_0 = 0.0 ; for ( i_0 = 0 ; i_0 < 6 ; i_0 ++ ) { tmp_0 += _rtB ->
B_6_0_1 [ 12 * i_0 + i ] * _rtB -> B_0_0_1 [ 6 * i_1 + i_0 ] ; } tmp [ i + 12
* i_1 ] = _rtB -> B_5_0_1 [ 12 * i_1 + i ] - tmp_0 ; } } for ( i = 0 ; i < 12
; i ++ ) { B_8_7_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ ) {
B_8_7_0 [ i ] += tmp [ 12 * i_1 + i ] * _rtB -> B_8_6_0 [ i_1 ] ; } } B_8_8_0
= ( ( Parameters_inverse_kinematics * ) ssGetDefaultParam ( S ) ) -> P_4 * (
( ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
TransferFcn_CSTATE ; B_8_9_0 = ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_9 * ( ( ContinuousStates_inverse_kinematics *
) ssGetContStates ( S ) ) -> TransferFcn1_CSTATE ; B_8_10_0 = ( (
Parameters_inverse_kinematics * ) ssGetDefaultParam ( S ) ) -> P_14 * ( (
ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
TransferFcn2_CSTATE ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> B_8_11_0 [
0 ] = _rtP -> P_17 [ 0 ] ; _rtB -> B_8_11_0 [ 1 ] = _rtP -> P_17 [ 1 ] ; _rtB
-> B_8_11_0 [ 2 ] = _rtP -> P_17 [ 2 ] ; } _rtB -> B_8_26_0 [ 0 ] = _rtP ->
P_18 * B_8_8_0 ; _rtB -> B_8_26_0 [ 1 ] = _rtP -> P_18 * B_8_9_0 ; _rtB ->
B_8_26_0 [ 2 ] = _rtP -> P_18 * B_8_10_0 ; _rtB -> B_8_26_0 [ 3 ] = _rtP ->
P_18 * _rtB -> B_8_11_0 [ 0 ] ; _rtB -> B_8_26_0 [ 4 ] = _rtP -> P_18 * _rtB
-> B_8_11_0 [ 1 ] ; _rtB -> B_8_26_0 [ 5 ] = _rtP -> P_18 * _rtB -> B_8_11_0
[ 2 ] ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB -> B_8_13_0 = _rtP -> P_28
; ( ( BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_8_14_0 = ( (
BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_8_13_0 ; for ( i =
0 ; i < 6 ; i ++ ) { _rtB -> B_8_15_0 [ i ] = _rtP -> P_19 [ i ] ; } }
ssCallAccelRunBlock ( S , 8 , 16 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit
( S , 1 , 0 ) ) { for ( i = 0 ; i < 6 ; i ++ ) { _rtB -> B_8_17_0 [ i ] =
_rtP -> P_20 [ i ] ; } } for ( i = 0 ; i < 6 ; i ++ ) { _rtB -> B_8_18_0 [ i
] = _rtB -> B_8_26_0 [ i ] + _rtB -> B_8_17_0 [ i ] ; } for ( i = 0 ; i < 12
; i ++ ) { tmp_0 = 0.0 ; for ( i_1 = 0 ; i_1 < 6 ; i_1 ++ ) { tmp_0 += _rtB
-> B_6_0_1 [ 12 * i_1 + i ] * _rtB -> B_8_18_0 [ i_1 ] ; } _rtB -> B_8_20_0 [
i ] = B_8_7_0 [ i ] + tmp_0 ; } if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtB ->
B_8_21_0 = _rtP -> P_30 ; ( ( BlockIO_inverse_kinematics * ) _ssGetBlockIO (
S ) ) -> B_8_22_0 = ( ( BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) )
-> B_8_21_0 ; } { int_T i1 ; real_T * y0 = ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_23_0 ; real_T * xc = & ( (
ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
Integrator1_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [
i1 ] ; } } ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 8 , 25 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i <
6 ; i ++ ) { _rtB -> B_8_26_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++
) { _rtB -> B_8_26_0 [ i ] += _rtB -> B_0_0_1 [ 6 * i_1 + i ] * _rtB ->
B_8_25_0 [ i_1 ] ; } } ssCallAccelRunBlock ( S , 8 , 27 , SS_CALL_MDL_OUTPUTS
) ; { int_T i1 ; real_T * y0 = ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_28_0 ; real_T * xc = & ( (
ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
Integrator_CSTATE_a [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc
[ i1 ] ; } } ssCallAccelRunBlock ( S , 8 , 29 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 8 , 30 , SS_CALL_MDL_OUTPUTS ) ; if ( ssIsSampleHit
( S , 1 , 0 ) ) { ssCallAccelRunBlock ( S , 8 , 31 , SS_CALL_MDL_OUTPUTS ) ;
} ssCallAccelRunBlock ( S , 1 , 0 , SS_CALL_MDL_OUTPUTS ) ; if (
ssIsSampleHit ( S , 2 , 0 ) ) { _rtB -> B_8_34_0 [ 0 ] = _rtP -> P_26 [ 0 ] *
_rtDW -> NextOutput ; _rtB -> B_8_34_0 [ 1 ] = _rtP -> P_26 [ 1 ] * _rtDW ->
NextOutput ; _rtB -> B_8_34_0 [ 2 ] = _rtP -> P_26 [ 2 ] * _rtDW ->
NextOutput ; } { int_T i1 ; real_T * y0 = ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_35_0 ; real_T * xc = & ( (
ContinuousStates_inverse_kinematics * ) ssGetContStates ( S ) ) ->
Integrator_CSTATE_c [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc
[ i1 ] ; } } ssCallAccelRunBlock ( S , 8 , 36 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 8 , 37 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 8 , 38 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 2 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 3 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 8 , 41 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 8 , 42 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 8 , 43 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 7 , 0 , SS_CALL_MDL_OUTPUTS
) ; UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) {
BlockIO_inverse_kinematics * _rtB ; Parameters_inverse_kinematics * _rtP ;
D_Work_inverse_kinematics * _rtDW ; _rtDW = ( ( D_Work_inverse_kinematics * )
ssGetRootDWork ( S ) ) ; _rtP = ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) ; _rtB = ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) ; if ( ssIsSampleHit ( S , 2 , 0 ) ) { _rtDW ->
NextOutput = rt_nrand_Upu32_Yd_f_pw_snf ( & _rtDW -> RandSeed ) * _rtP ->
P_24 + _rtP -> P_23 ; } UNUSED_PARAMETER ( tid ) ; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { BlockIO_inverse_kinematics *
_rtB ; _rtB = ( ( BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) ; { {
int_T i1 ; const real_T * u0 = & ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_25_0 [ 6 ] ; real_T * xdot = & ( (
StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) -> Integrator_CSTATE
[ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] = u0 [ i1 ] ; } } } { (
( StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn_CSTATE = ( ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_2 ) * ( ( ContinuousStates_inverse_kinematics
* ) ssGetContStates ( S ) ) -> TransferFcn_CSTATE ; ( (
StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) -> TransferFcn_CSTATE
+= ( ( Parameters_inverse_kinematics * ) ssGetDefaultParam ( S ) ) -> P_3 * (
( BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_8_34_0 [ 0 ] ; }
{ ( ( StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn1_CSTATE = ( ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_7 ) * ( ( ContinuousStates_inverse_kinematics
* ) ssGetContStates ( S ) ) -> TransferFcn1_CSTATE ; ( (
StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn1_CSTATE += ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_8 * ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_34_0 [ 1 ] ; } { ( (
StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn2_CSTATE = ( ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_12 ) * ( ( ContinuousStates_inverse_kinematics
* ) ssGetContStates ( S ) ) -> TransferFcn2_CSTATE ; ( (
StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
TransferFcn2_CSTATE += ( ( Parameters_inverse_kinematics * )
ssGetDefaultParam ( S ) ) -> P_13 * ( ( BlockIO_inverse_kinematics * )
_ssGetBlockIO ( S ) ) -> B_8_34_0 [ 2 ] ; } { { int_T i1 ; const real_T * u0
= ( ( BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_7_0_1 ;
real_T * xdot = & ( ( StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) )
-> Integrator1_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_1_0_1 ; real_T *
xdot = & ( ( StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
Integrator_CSTATE_a [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_inverse_kinematics * ) _ssGetBlockIO ( S ) ) -> B_2_0_1 ; real_T *
xdot = & ( ( StateDerivatives_inverse_kinematics * ) ssGetdX ( S ) ) ->
Integrator_CSTATE_c [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] =
u0 [ i1 ] ; } } } } static void mdlInitializeSizes ( SimStruct * S ) {
ssSetChecksumVal ( S , 0 , 2391252363U ) ; ssSetChecksumVal ( S , 1 ,
3281140950U ) ; ssSetChecksumVal ( S , 2 , 328231528U ) ; ssSetChecksumVal (
S , 3 , 1389789180U ) ; { mxArray * slVerStructMat = NULL ; mxArray *
slStrMat = mxCreateString ( "simulink" ) ; char slVerChar [ 10 ] ; int status
= mexCallMATLAB ( 1 , & slVerStructMat , 1 , & slStrMat , "ver" ) ; if (
status == 0 ) { mxArray * slVerMat = mxGetField ( slVerStructMat , 0 ,
"Version" ) ; if ( slVerMat == NULL ) { status = 1 ; } else { status =
mxGetString ( slVerMat , slVerChar , 10 ) ; } } mxDestroyArray ( slStrMat ) ;
mxDestroyArray ( slVerStructMat ) ; if ( ( status == 1 ) || ( strcmp (
slVerChar , "7.9" ) != 0 ) ) { return ; } } ssSetOptions ( S ,
SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork ( S ) != sizeof (
D_Work_inverse_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( BlockIO_inverse_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
Parameters_inverse_kinematics ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & inverse_kinematics_rtDefaultParameters ) ; rt_InitInfAndNaN (
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
childS ) ; callSysFcns [ 3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; } } static
void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
