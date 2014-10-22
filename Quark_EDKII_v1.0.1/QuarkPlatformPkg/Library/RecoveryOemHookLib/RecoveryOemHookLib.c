/** @file
  This file includes the function that can be customized by OEM.

Copyright (c) 2013 Intel Corporation.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
* Neither the name of Intel Corporation nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**/

#include "CommonHeader.h"

/**
  This function allows the user to force a system recovery

**/
VOID
EFIAPI
OemInitiateRecovery (
  VOID
  ) 
{
  UINT32          Data32;

  //
  // Set 'B_CFG_STICKY_RW_FORCE_RECOVERY' sticky bit so we know we need to do a recovery following warm reset
  //
  Data32 = QNCAltPortRead (QUARK_SCSS_SOC_UNIT_SB_PORT_ID, QUARK_SCSS_SOC_UNIT_CFG_STICKY_RW);
  Data32 |= B_CFG_STICKY_RW_FORCE_RECOVERY;
  QNCAltPortWrite (QUARK_SCSS_SOC_UNIT_SB_PORT_ID, QUARK_SCSS_SOC_UNIT_CFG_STICKY_RW, Data32);

  //
  // Initialte the warm reset
  //
  ResetWarm ();
}

/**
  This function allows the user to force a system recovery and deadloop.

  Deadloop required since system should not execute beyond this point.
  Deadloop should never happen since OemInitiateRecovery () called within
  this routine should never return since it executes a Warm Reset.

**/
VOID
EFIAPI
OemInitiateRecoveryAndWait (
  VOID
  )
{
  volatile UINTN  Index;

  OemInitiateRecovery ();
  for (Index = 0; Index == 0;);
}
