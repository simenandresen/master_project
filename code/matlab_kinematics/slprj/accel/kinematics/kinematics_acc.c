#include "__cf_kinematics.h"
#if 0
This file is not available for use in any application other than as a MATLAB
( R ) MEX file for use with the Simulink ( R ) product . If you do not have
the Simulink Coder licensed , this file contains encrypted block names , etc
. If you purchase the Simulink Coder , this file will contain full block
descriptions and improved variable names .
#endif
#include <math.h>
#include "kinematics_acc.h"
#include "kinematics_acc_private.h"
#include <stdio.h>
#include "simstruc.h"
#include "fixedpoint.h"
#define CodeFormat S-Function
#define AccDefine1 Accelerator_S-Function
static void mdlOutputs ( SimStruct * S , int_T tid ) { real_T B_7_5_0 [ 144 ]
; int32_T i ; int32_T i_0 ; real_T tmp ; real_T B [ 12 ] ; real_T tmp_0 [ 12
] ; int32_T i_1 ; BlockIO_kinematics * _rtB ; D_Work_kinematics * _rtDW ;
_rtDW = ( ( D_Work_kinematics * ) ssGetRootDWork ( S ) ) ; _rtB = ( (
BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) ; ssCallAccelRunBlock ( S , 2 ,
0 , SS_CALL_MDL_OUTPUTS ) ; { int_T i1 ; real_T * y0 = ( ( BlockIO_kinematics
* ) _ssGetBlockIO ( S ) ) -> B_7_1_0 ; real_T * xc = & ( (
ContinuousStates_kinematics * ) ssGetContStates ( S ) ) -> Integrator1_CSTATE
[ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } }
ssCallAccelRunBlock ( S , 0 , 0 , SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock
( S , 3 , 0 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i < 12 ; i ++ ) { for (
i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { tmp = 0.0 ; for ( i_0 = 0 ; i_0 < 6 ; i_0 ++
) { tmp += _rtB -> B_3_0_1 [ 12 * i_0 + i ] * _rtB -> B_0_0_1 [ 6 * i_1 + i_0
] ; } B_7_5_0 [ i + 12 * i_1 ] = _rtB -> B_2_0_1 [ 12 * i_1 + i ] - tmp ; } }
if ( ssIsSampleHit ( S , 1 , 0 ) ) { for ( i = 0 ; i < 6 ; i ++ ) { _rtB ->
B_7_6_0 [ i ] = ( ( Parameters_kinematics * ) ssGetDefaultParam ( S ) ) ->
P_5 [ i ] ; } _rtB -> B_7_7_0 [ 0 ] = _rtDW -> UnitDelay2_DSTATE [ 0 ] ; _rtB
-> B_7_7_0 [ 1 ] = _rtDW -> UnitDelay2_DSTATE [ 1 ] ; _rtB -> B_7_7_0 [ 2 ] =
_rtDW -> UnitDelay2_DSTATE [ 2 ] ; _rtB -> B_7_8_0 [ 0 ] = _rtDW ->
UnitDelay_DSTATE [ 0 ] ; _rtB -> B_7_8_0 [ 1 ] = _rtDW -> UnitDelay_DSTATE [
1 ] ; _rtB -> B_7_8_0 [ 2 ] = _rtDW -> UnitDelay_DSTATE [ 2 ] ; _rtB ->
B_7_9_0 [ 0 ] = _rtDW -> UnitDelay1_DSTATE [ 0 ] ; _rtB -> B_7_9_0 [ 1 ] =
_rtDW -> UnitDelay1_DSTATE [ 1 ] ; _rtB -> B_7_9_0 [ 2 ] = _rtDW ->
UnitDelay1_DSTATE [ 2 ] ; } ssCallAccelRunBlock ( S , 1 , 0 ,
SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i < 12 ; i ++ ) { B [ i ] = 0.0 ; for (
i_1 = 0 ; i_1 < 12 ; i_1 ++ ) { B [ i ] += B_7_5_0 [ 12 * i_1 + i ] * _rtB ->
B_1_0_1 [ i_1 ] ; } tmp_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 6 ; i_1 ++ ) {
tmp_0 [ i ] += _rtB -> B_3_0_1 [ 12 * i_1 + i ] * _rtB -> B_7_6_0 [ i_1 ] ; }
_rtB -> B_7_13_0 [ i ] = B [ i ] + tmp_0 [ i ] ; } ssCallAccelRunBlock ( S ,
7 , 14 , SS_CALL_MDL_OUTPUTS ) ; { int_T i1 ; real_T * y0 = ( (
BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) -> B_7_15_0 ; real_T * xc = & (
( ContinuousStates_kinematics * ) ssGetContStates ( S ) ) ->
Integrator_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [
i1 ] ; } } ssCallAccelRunBlock ( S , 7 , 16 , SS_CALL_MDL_OUTPUTS ) ;
ssCallAccelRunBlock ( S , 4 , 0 , SS_CALL_MDL_OUTPUTS ) ; for ( i = 0 ; i < 6
; i ++ ) { _rtB -> B_7_18_0 [ i ] = 0.0 ; for ( i_1 = 0 ; i_1 < 12 ; i_1 ++ )
{ _rtB -> B_7_18_0 [ i ] += _rtB -> B_0_0_1 [ 6 * i_1 + i ] * _rtB ->
B_7_13_0 [ i_1 ] ; } } { int_T i1 ; real_T * y0 = ( ( BlockIO_kinematics * )
_ssGetBlockIO ( S ) ) -> B_7_19_0 ; real_T * xc = & ( (
ContinuousStates_kinematics * ) ssGetContStates ( S ) ) -> Integrator2_CSTATE
[ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } { int_T
i1 ; real_T * y0 = ( ( BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) ->
B_7_20_0 ; real_T * xc = & ( ( ContinuousStates_kinematics * )
ssGetContStates ( S ) ) -> Integrator2_CSTATE_b [ 0 ] ; for ( i1 = 0 ; i1 < 6
; i1 ++ ) { y0 [ i1 ] = xc [ i1 ] ; } } ssCallAccelRunBlock ( S , 7 , 21 ,
SS_CALL_MDL_OUTPUTS ) ; ssCallAccelRunBlock ( S , 5 , 0 , SS_CALL_MDL_OUTPUTS
) ; ssCallAccelRunBlock ( S , 6 , 0 , SS_CALL_MDL_OUTPUTS ) ;
UNUSED_PARAMETER ( tid ) ; }
#define MDL_UPDATE
static void mdlUpdate ( SimStruct * S , int_T tid ) { BlockIO_kinematics *
_rtB ; D_Work_kinematics * _rtDW ; _rtDW = ( ( D_Work_kinematics * )
ssGetRootDWork ( S ) ) ; _rtB = ( ( BlockIO_kinematics * ) _ssGetBlockIO ( S
) ) ; if ( ssIsSampleHit ( S , 1 , 0 ) ) { _rtDW -> UnitDelay2_DSTATE [ 0 ] =
_rtB -> B_7_20_0 [ 0 ] ; _rtDW -> UnitDelay2_DSTATE [ 1 ] = _rtB -> B_7_20_0
[ 1 ] ; _rtDW -> UnitDelay2_DSTATE [ 2 ] = _rtB -> B_7_20_0 [ 2 ] ; _rtDW ->
UnitDelay_DSTATE [ 0 ] = _rtB -> B_7_20_0 [ 3 ] ; _rtDW -> UnitDelay_DSTATE [
1 ] = _rtB -> B_7_20_0 [ 4 ] ; _rtDW -> UnitDelay_DSTATE [ 2 ] = _rtB ->
B_7_20_0 [ 5 ] ; _rtDW -> UnitDelay1_DSTATE [ 0 ] = _rtB -> B_7_19_0 [ 0 ] ;
_rtDW -> UnitDelay1_DSTATE [ 1 ] = _rtB -> B_7_19_0 [ 1 ] ; _rtDW ->
UnitDelay1_DSTATE [ 2 ] = _rtB -> B_7_19_0 [ 2 ] ; } UNUSED_PARAMETER ( tid )
; }
#define MDL_DERIVATIVES
static void mdlDerivatives ( SimStruct * S ) { BlockIO_kinematics * _rtB ;
_rtB = ( ( BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) ; { { int_T i1 ;
const real_T * u0 = & ( ( BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) ->
B_7_13_0 [ 6 ] ; real_T * xdot = & ( ( StateDerivatives_kinematics * )
ssGetdX ( S ) ) -> Integrator1_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ )
{ xdot [ i1 ] = u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( (
BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) -> B_4_0_1 ; real_T * xdot = & (
( StateDerivatives_kinematics * ) ssGetdX ( S ) ) -> Integrator_CSTATE [ 0 ]
; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] = u0 [ i1 ] ; } } } { { int_T
i1 ; const real_T * u0 = ( ( BlockIO_kinematics * ) _ssGetBlockIO ( S ) ) ->
B_5_0_1 ; real_T * xdot = & ( ( StateDerivatives_kinematics * ) ssGetdX ( S )
) -> Integrator2_CSTATE [ 0 ] ; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ]
= u0 [ i1 ] ; } } } { { int_T i1 ; const real_T * u0 = ( ( BlockIO_kinematics
* ) _ssGetBlockIO ( S ) ) -> B_6_0_1 ; real_T * xdot = & ( (
StateDerivatives_kinematics * ) ssGetdX ( S ) ) -> Integrator2_CSTATE_b [ 0 ]
; for ( i1 = 0 ; i1 < 6 ; i1 ++ ) { xdot [ i1 ] = u0 [ i1 ] ; } } } } static
void mdlInitializeSizes ( SimStruct * S ) { ssSetChecksumVal ( S , 0 ,
2383459634U ) ; ssSetChecksumVal ( S , 1 , 902794377U ) ; ssSetChecksumVal (
S , 2 , 3424020369U ) ; ssSetChecksumVal ( S , 3 , 1631087587U ) ; { mxArray
* slVerStructMat = NULL ; mxArray * slStrMat = mxCreateString ( "simulink" )
; char slVerChar [ 10 ] ; int status = mexCallMATLAB ( 1 , & slVerStructMat ,
1 , & slStrMat , "ver" ) ; if ( status == 0 ) { mxArray * slVerMat =
mxGetField ( slVerStructMat , 0 , "Version" ) ; if ( slVerMat == NULL ) {
status = 1 ; } else { status = mxGetString ( slVerMat , slVerChar , 10 ) ; }
} mxDestroyArray ( slStrMat ) ; mxDestroyArray ( slVerStructMat ) ; if ( (
status == 1 ) || ( strcmp ( slVerChar , "7.9" ) != 0 ) ) { return ; } }
ssSetOptions ( S , SS_OPTION_EXCEPTION_FREE_CODE ) ; if ( ssGetSizeofDWork (
S ) != sizeof ( D_Work_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal DWork sizes do "
"not match for accelerator mex file." ) ; } if ( ssGetSizeofGlobalBlockIO ( S
) != sizeof ( BlockIO_kinematics ) ) { ssSetErrorStatus ( S ,
"Unexpected error: Internal BlockIO sizes do "
"not match for accelerator mex file." ) ; } { int ssSizeofParams ;
ssGetSizeofParams ( S , & ssSizeofParams ) ; if ( ssSizeofParams != sizeof (
Parameters_kinematics ) ) { static char msg [ 256 ] ; sprintf ( msg ,
"Unexpected error: Internal Parameters sizes do "
"not match for accelerator mex file." ) ; } } _ssSetDefaultParam ( S , (
real_T * ) & kinematics_rtDefaultParameters ) ; rt_InitInfAndNaN ( sizeof (
real_T ) ) ; } static void mdlInitializeSampleTimes ( SimStruct * S ) { {
SimStruct * childS ; SysOutputFcn * callSysFcns ; childS = ssGetSFunction ( S
, 0 ) ; callSysFcns = ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [
3 + 0 ] = ( SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 1 ) ;
callSysFcns = ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ]
= ( SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 2 ) ; callSysFcns
= ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 3 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 4 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 5 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; childS = ssGetSFunction ( S , 6 ) ; callSysFcns =
ssGetCallSystemOutputFcnList ( childS ) ; callSysFcns [ 3 + 0 ] = (
SysOutputFcn ) ( NULL ) ; } } static void mdlTerminate ( SimStruct * S ) { }
#include "simulink.c"
