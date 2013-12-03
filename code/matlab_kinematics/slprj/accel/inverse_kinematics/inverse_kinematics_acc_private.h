#include "__cf_inverse_kinematics.h"
#ifndef RTW_HEADER_inverse_kinematics_acc_private_h_
#define RTW_HEADER_inverse_kinematics_acc_private_h_
#include "rtwtypes.h"
#ifndef RTW_COMMON_DEFINES_
#define RTW_COMMON_DEFINES_
#define rt_VALIDATE_MEMORY(S, ptr)   if(!(ptr)) {\
  ssSetErrorStatus(S, RT_MEMORY_ALLOCATION_ERROR);\
  }
#if !defined(_WIN32)
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((ptr));\
  (ptr) = (NULL);\
  }
#else
#define rt_FREE(ptr)   if((ptr) != (NULL)) {\
  free((void *)(ptr));\
  (ptr) = (NULL);\
  }
#endif
#endif
#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#else
#ifdef TMWTYPES_PREVIOUSLY_INCLUDED
#error This file requires rtwtypes.h to be included before tmwtypes.h
#else
#ifndef RTWTYPES_ID_C08S16I32L32N32F1
#error This code was generated with a different "rtwtypes.h" than the file included
#endif
#endif
#endif
extern real_T rt_urand_Upu32_Yd_f_pw_snf ( uint32_T * u ) ; extern real_T
rt_nrand_Upu32_Yd_f_pw_snf ( uint32_T * u ) ;
#endif
