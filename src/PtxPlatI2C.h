/** \file
    ---------------------------------------------------------------
    SPDX-License-Identifier: BSD-3-Clause

    Copyright (c) 2024, Renesas Electronics Corporation and/or its affiliates


    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of Renesas nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.



   THIS SOFTWARE IS PROVIDED BY Renesas "AS IS" AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL RENESAS OR CONTRIBUTORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ---------------------------------------------------------------

    Project     : PTX105R Arduino
    Module      : I2C
    File        : PtxPlatI2C.h

    Description : I2C interface
*/

#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
typedef uint16_t ptxStatus_t;

/**
 * \brief GPIO structure.
 */
typedef struct ptxPLAT_GPIO {
  int pinIRQ; /**< IRQ pin. */
} ptxPLAT_GPIO_t;

/**
 * \brief Initialize the I2C interface.
 * \note This function shall be successfully executed before any other call to
 * the functions in this module. It initializes SPI hardware wise.
 * \param[out] gpio Pointer to pointer where the allocated GPIO context is going
 * to be provided.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_i2cInit(ptxPLAT_GPIO_t **gpio);

/**
 * \brief Deinitialize the I2C interface.
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_i2cDeinit();

/**
 * \brief I2C transmit and receive function.
 * \note Wrapper function for \ref ptxPLAT_TRx. See it for detailed description
 * \param[in]     i2c          Pointer to an initialized I2C context.
 * \param[in]     txBuf        Array of buffers to transmit.
 * \param[in]     txLen        Array of lengths of buffers to transmit.
 * \param[in]     numTxBuffers Number of buffers to transmit.
 * \param[out]    rxBuf        Array of buffers to receive.
 * \param[in,out] rxLen        Array of lengths of buffers to receive.
 * \param[in]     numTxBuffers Number of buffers to receive.
 * \param[in]     flags        General purpose flags for TRx-operation (specific
 * to used host-interface).
 * \return Status, indicating whether the operation was successful. See \ref
 * ptxStatus_t.
 */
ptxStatus_t ptxPlat_i2cTrx(const uint8_t *txBuf[], size_t txLen[],
                           size_t numTxBuffers, uint8_t *rxBuf[],
                           size_t *rxLen[], size_t numRxBuffers, uint8_t flags);

#ifdef __cplusplus
}
#endif