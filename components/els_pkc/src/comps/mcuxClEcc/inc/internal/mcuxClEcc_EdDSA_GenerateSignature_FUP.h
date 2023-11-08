/*--------------------------------------------------------------------------*/
/* Copyright 2022-2023 NXP                                                  */
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
 * @file  mcuxClEcc_EdDSA_GenerateSignature_FUP.h
 * @brief defines FUP programs byte arrays
 */


#ifndef MCUXCLECC_EDDSA_GENERATESIGNATURE_FUP_H_
#define MCUXCLECC_EDDSA_GENERATESIGNATURE_FUP_H_

#include <mcuxClConfig.h> // Exported features flags header
#include <internal/mcuxClPkc_FupMacros.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * FUP program declaration mcuxClEcc_FUP_EdDSA_GenerateSignature_ReduceScalarModN
 */
#define mcuxClEcc_FUP_EdDSA_GenerateSignature_ReduceScalarModN_LEN  10u
extern const mcuxClPkc_FUPEntry_t mcuxClEcc_FUP_EdDSA_GenerateSignature_ReduceScalarModN[mcuxClEcc_FUP_EdDSA_GenerateSignature_ReduceScalarModN_LEN];

/**
 * FUP program declaration mcuxClEcc_FUP_EdDSA_GenerateSignature_Compute_S
 */
#define mcuxClEcc_FUP_EdDSA_GenerateSignature_Compute_S_LEN  10u
extern const mcuxClPkc_FUPEntry_t mcuxClEcc_FUP_EdDSA_GenerateSignature_Compute_S[mcuxClEcc_FUP_EdDSA_GenerateSignature_Compute_S_LEN];

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MCUXCLECC_EDDSA_GENERATESIGNATURE_FUP_H_ */
