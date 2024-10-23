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
    File        : PtxPlatI2C.cpp

    Description : I2C interface implementation
*/

/*
 * ####################################################################################################################
 * INCLUDES
 * ####################################################################################################################
 */

#include "PtxPlatI2C.h"

#include <Arduino.h>
#include <Wire.h>

#include <algorithm>

#include "ptxPLAT_INT.h"

namespace {
ptxPLAT_GPIO_t gpioCtx;
const int pinIRQ = 9;
const int clockSpeed = 100000;
const uint8_t deviceAddress = 0x4C;
}  // namespace

ptxStatus_t ptxPlat_i2cInit(ptxPLAT_GPIO_t **gpio) {
  if (gpio != nullptr) {
    memset(&gpioCtx, 0, sizeof(gpioCtx));
    gpioCtx.pinIRQ = pinIRQ;
    *gpio = &gpioCtx;
    pinMode(gpioCtx.pinIRQ, INPUT_PULLUP);
    Wire.setClock(clockSpeed);  // Sets the I2C Communication to 100 KHz
    Wire.begin();               // Join I2C bus
    return ptxStatus_Success;
  } else {
    return PTX_STATUS(ptxStatus_Comp_PLAT, ptxStatus_InvalidParameter);
  }
}

ptxStatus_t ptxPlat_i2cDeinit() {
  memset(&gpioCtx, 0, sizeof(gpioCtx));
  Wire.end();

  return ptxStatus_Success;
}

ptxStatus_t ptxPlat_i2cTrx(const uint8_t *txBuf[], size_t txLen[],
                           size_t numTxBuffers, uint8_t *rxBuf[],
                           size_t *rxLen[], size_t numRxBuffers,
                           uint8_t flags) {
  auto status = ptxStatus_Success;

  /** Tx part of the overall transaction (Attention: I2C may not send anything
   * at all - just receive). */
  for (size_t i = 0; i < numTxBuffers; i++) {
    Wire.beginTransmission(deviceAddress);
    if (txBuf[i] && txLen[i]) {
      const bool restartRequired =
          (flags & PTX_PLAT_TRX_FLAGS_I2C_RESTART_CONDITION) &&
          (i == numTxBuffers - 1);

      Wire.write(txBuf[i], txLen[i]);          // Sends Bytes in the Buffer
      Wire.endTransmission(!restartRequired);  // restart required
    }
  }

  if (!status && rxBuf && rxLen) {
    for (size_t i = 0; i < numRxBuffers; i++) {
      const size_t length = *rxLen[i];
      uint8_t *memIndex = rxBuf[i];
      if (memIndex && length) {
        Wire.requestFrom(deviceAddress, length, true);

        while (Wire.available()) {
          *memIndex = static_cast<uint8_t>(Wire.read());  // Receives a Byte
          memIndex++;
        }
      }
    }
  }

  return status;
}