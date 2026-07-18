/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
/*                                                                           */
/*                  This file is part of the program and library             */
/*         SCIP --- Solving Constraint Integer Programs                      */
/*                                                                           */
/*  Copyright (c) 2002-2026 Zuse Institute Berlin (ZIB)                      */
/*                                                                           */
/*  Licensed under the Apache License, Version 2.0 (the "License");          */
/*  you may not use this file except in compliance with the License.         */
/*  You may obtain a copy of the License at                                  */
/*                                                                           */
/*      http://www.apache.org/licenses/LICENSE-2.0                           */
/*                                                                           */
/*  Unless required by applicable law or agreed to in writing, software      */
/*  distributed under the License is distributed on an "AS IS" BASIS,        */
/*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/*  See the License for the specific language governing permissions and      */
/*  limitations under the License.                                           */
/*                                                                           */
/*  You should have received a copy of the Apache-2.0 license                */
/*  along with SCIP; see the file LICENSE. If not visit scipopt.org.         */
/*                                                                           */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/**@file   scipbuildflags.c
 * @brief  build flags methods
 * @author Felipe Serrano
 */

/*---+----1----+----2----+----3----+----4----+----5----+----6----+----7----+----8----+----9----+----0----+----1----+----2*/

#include "scip/scipbuildflags.h"

/** returns the flags that were used to build SCIP */
const char* SCIPgetBuildFlags(
   void
   )
{
   return " ARCH=arm64\n\
 OSTYPE=Darwin-25.5.0\n\
 COMP=AppleClang 21.0.0.21000101\n\
 BUILD=Release\n\
 CHECKSTAGE=\n\
 CHECKNAME=\n\
 DEBUGSOL=\n\
 EXPRINT=\n\
 SYM=\n\
 GMP=\n\
 CONOPT=\n\
 IPOPT=\n\
 WORHP=\n\
 LPS=\n\
 LPSCHECK=\n\
 LTO=\n\
 NOBLKBUFMEM=\n\
 NOBLKMEM=\n\
 NOBUFMEM=\n\
 THREADSAFE=\n\
 READLINE=\n\
 SANITIZE=\n\
 SHARED=OFF\n\
 VERSION=9.0.0\n\
 API_VERSION=102\n\
 ZIMPL=\n\
 ZLIB=OFF";
}
