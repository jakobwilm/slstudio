/*
 * dlpc350_error.h
 *
 * This module defines the error handling related definitions
 *
 * Copyright (C) {2015} Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef ERROR_H_
#define ERROR_H_

#include <stdio.h>
#include <errno.h>

#include "dlpc350_common.h"

#define DEBUG_LEVEL_ERR  1
#define DEBUG_LEVEL_WRN  2
#define DEBUG_LEVEL_MSG  3
#define DEBUG_LEVEL_NONE 0

#define DEBUG_PRN(...)      printf(__VA_ARGS__)

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_MSG
#define DEBUG_MSG(...)      printf(__VA_ARGS__)
#else
#define DEBUG_MSG(...)
#endif

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_WRN
#define DEBUG_WRN(...)      printf(__VA_ARGS__)
#else
#define DEBUG_WRN(...)
#endif

#if CFG_DEBUG_LEVEL >= DEBUG_LEVEL_ERR
#define DEBUG_ERR(...)      printf(__VA_ARGS__)
#else
#define DEBUG_ERR(...)		(void)0
#endif

#define DEBUG_TRACE()	ERR_LOG_MSG("Trace ..."), getchar()

#define ERR_GET_STR(f, l)		ERR_GET_STR_(f, l)
#define ERR_GET_STR_(f, l)		f ":" #l " >> "
#define ERR_LOC_STR				ERR_GET_STR(__FILE__,__LINE__)

#define ERR_BLOCK_BEGIN
#define ERR_BLOCK_END      

#define ERR_THROW_MSG(x, ...)  do{ ERR_LOG_MSG(__VA_ARGS__); return (x); } while(0)

#define ERR_THROW(x)	   ERR_THROW_MSG(x, #x)

#define ERR_THROW_PREV()   ERR_THROW_MSG((void)0, "")

#define THROW(x)		   do { ERR_LOG_MSG(#x); if(x == ERR_DEVICE_FAIL) \
    DEBUG_ERR("System Error : %s\n", strerror(errno)); \
    return (x); \
    } while(0)
#define THROW_MSG(x,...)   do { ERR_LOG_MSG(__VA_ARGS__);  return (x); } while(0)
#define THROW_S(x)		   return (x)

#define TRY(x)			   do { ErrorCode_t x__ = (x); if(x__ != SUCCESS) { ERR_LOG_MSG(#x); return (x__); } } while(0)

#define ERR_BREAK()		   return (FAIL)
#define ERR_BREAK_MSG(m)   do { DEBUG_MSG(m); return (FAIL); } while(0)

#define ERR_LOG_MSG(...)    do { DEBUG_ERR(ERR_LOC_STR __VA_ARGS__); DEBUG_ERR("\n");} while(0)



#endif /* ERROR_H_ */
