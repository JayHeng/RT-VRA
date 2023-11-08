/*--------------------------------------------------------------------------*/
/* Copyright 2021 NXP                                                       */
/*                                                                          */
/* NXP Confidential. This software is owned or controlled by NXP and may    */
/* only be used strictly in accordance with the applicable license terms.   */
/* By expressly accepting such terms or by downloading, installing,         */
/* activating and/or otherwise using the software, you are agreeing that    */
/* you have read, and that you agree to comply with and are bound by, such  */
/* license terms. If you do not agree to be bound by the applicable license */
/* terms, then you may not retain, install, activate or otherwise use the   */
/* software.                                                                */
/*--------------------------------------------------------------------------*/

/**
 * @file  mcuxClEcc_Mont_Internal_SecureScalarMult_XZMontLadder_FUP.c
 * @brief mcuxClEcc: FUP programs for implementation of ECC internal secure scalar multiplication function montgomery ladder based
 */


#include <internal/mcuxClPkc_FupMacros.h>
#include <internal/mcuxClEcc_Mont_Internal.h>
#include <internal/mcuxClEcc_Mont_Internal_SecureScalarMult_XZMontLadder_FUP.h>

const mcuxClPkc_FUPEntry_t mcuxClEcc_FUP_SecureScalarMult_XZMontLadder_LadderStep[22] MCUX_FUP_ATTRIBUTE = {{0x10u,0x00u,0x7du,0x8cu,0x47u,0x56u},{0x80u,0x21u,0x10u,0x0cu,0x0du,0x19u},{0x80u,0x00u,0x19u,0x19u,0x00u,0x1du},{0x80u,0x2au,0x10u,0x0cu,0x0du,0x1bu},{0x80u,0x00u,0x1bu,0x1bu,0x00u,0x1fu},{0x80u,0x00u,0x1du,0x1fu,0x00u,0x0cu},{0x80u,0x2au,0x10u,0x1du,0x1fu,0x1du},{0x80u,0x00u,0x12u,0x1du,0x00u,0x0du},{0x80u,0x21u,0x10u,0x1fu,0x0du,0x1fu},{0x80u,0x00u,0x1du,0x1fu,0x00u,0x0du},{0x80u,0x21u,0x10u,0x0eu,0x0fu,0x1du},{0x80u,0x00u,0x1du,0x1bu,0x00u,0x1fu},{0x80u,0x2au,0x10u,0x0eu,0x0fu,0x1bu},{0x80u,0x00u,0x1bu,0x19u,0x00u,0x1du},{0x80u,0x2au,0x10u,0x1du,0x1fu,0x19u},{0x80u,0x00u,0x19u,0x19u,0x00u,0x1bu},{0x80u,0x00u,0x20u,0x1bu,0x00u,0x0fu},{0x80u,0x21u,0x10u,0x1du,0x1fu,0x19u},{0x80u,0x00u,0x19u,0x19u,0x00u,0x0eu},{0x10u,0x00u,0x4bu,0x90u,0x6au,0xfcu},{0x00u,0x1eu,0x00u,0x0eu,0x03u,0x1bu},{0x80u,0x00u,0x21u,0x1bu,0x00u,0x0eu}};



