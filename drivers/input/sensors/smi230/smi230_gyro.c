// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2020-2021 Robert Bosch GmbH. All rights reserved.
 * Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*! \file smi230_gyro.c
 * \brief Sensor Driver for SMI230 sensors */

/****************************************************************************/

/**\name        Header files
 ****************************************************************************/
#include "smi230.h"
#include "smi230_driver.h"
#define MODULE_TAG MODULE_NAME
#include "smi230_log.h"

/****************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/****************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 *
 * @param[in] dev : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t null_ptr_check(const struct smi230_dev *dev);

/*!
 *  @brief This API reads the data from the given register address of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address from where the data to be read
 *  @param[out]reg_data  : Pointer to data buffer to store the read data.
 *  @param[in] len       : No. of bytes of data to be read.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *data, uint16_t len,
		       const struct smi230_dev *dev);

/*!
 *  @brief This API writes the given data to the register address
 *  of gyro sensor.
 *
 *  @param[in] reg_addr  : Register address to where the data to be written.
 *  @param[in] reg_data  : Pointer to data buffer which is to be written
 *  in the sensor.
 *  @param[in] len       : No. of bytes of data to write.
 *  @param[in] dev       : Structure instance of smi230_dev.
 *
 *  @return Result of API execution status
 *  @retval zero -> Success / -ve value -> Error
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_dev *dev);

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 *
 * @param[in] int_config  : Structure instance of smi230_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t
set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
			const struct smi230_dev *dev);

static int8_t
set_gyro_fifo_int(const struct smi230_gyro_int_channel_cfg *int_config,
		  const struct smi230_dev *dev);
/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 *
 * @param[in] int_config  : Structure instance of smi230_gyro_int_channel_cfg.
 * @param[in] dev         : Structure instance of smi230_dev.
 *
 * @return Result of API execution status
 * @retval zero -> Success / -ve value -> Error
 */
static int8_t
set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config,
		   const struct smi230_dev *dev);

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 *
 *  @param[in] selftest : Variable used to enable or disable
 *  the Gyro self test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | SMI230_DISABLE
 *  0x01    | SMI230_ENABLE
 *
 *  @param[in] dev : Structure instance of smi230_dev
 *
 *  @return Result of API execution status
 *  @retval zero -> Success  / -ve value -> Error
 *
 */
static int8_t set_gyro_selftest(uint8_t selftest, const struct smi230_dev *dev);

/****************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/****************************************************************************/

/**\name        Globals
 ****************************************************************************/

/****************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point for gyro sensor.
 *  It performs the selection of I2C/SPI read mechanism according to the
 *  selected interface and reads the chip-id of gyro sensor.
 */
int8_t smi230_gyro_init(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t chip_id = 0;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		/* Read gyro chip id */
		rslt = get_regs(SMI230_GYRO_CHIP_ID_REG, &chip_id, 1, dev);

		if (rslt == SMI230_OK) {
			if (chip_id == SMI230_GYRO_CHIP_ID) {
				/* Store the chip ID in dev structure */
				dev->gyro_chip_id = chip_id;
			} else {
				rslt = SMI230_E_DEV_NOT_FOUND;
			}
		}
	}

	return rslt;
}

/*!
 *  @brief This API resets the gyro fifo.
 */
int8_t smi230_gyro_fifo_reset(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		data = SMI230_GYRO_FIFO_MODE;

		/* Reset accel device */
		rslt = set_regs(SMI230_GYRO_FIFO_CONFIG_1_ADDR, &data, 1, dev);

		if (rslt == SMI230_OK) {
			/* Delay 1 ms after reset value is written to its register */
			dev->delay_ms(1);
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address
 * of gyro sensor.
 */
int8_t smi230_gyro_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
			    const struct smi230_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == SMI230_OK) && (reg_data != NULL)) {
		if (len > 0) {
			/* Reading from the register */
			rslt = get_regs(reg_addr, reg_data, len, dev);
		} else {
			rslt = SMI230_E_RD_WR_LENGTH_INVALID;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of gyro sensor.
 */
int8_t smi230_gyro_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
			    const struct smi230_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == SMI230_OK) && (reg_data != NULL)) {
		if (len > 0) {
			/* Writing to the register */
			rslt = set_regs(reg_addr, reg_data, len, dev);
		} else {
			rslt = SMI230_E_RD_WR_LENGTH_INVALID;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API resets the gyro sensor.
 */
int8_t smi230_gyro_soft_reset(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		/* Reset gyro device */
		data = SMI230_SOFT_RESET_CMD;
		rslt = set_regs(SMI230_GYRO_SOFTRESET_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			/* delay 30 ms after writing reset value to its register */
			dev->delay_ms(SMI230_GYRO_SOFTRESET_DELAY);
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the gyro odr and range from the sensor, store it in the smi230_dev
 * structure instance passed by the user.
 */
int8_t smi230_gyro_get_meas_conf(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[2];

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_RANGE_REG, data, 2, dev);

		if (rslt == SMI230_OK) {
			dev->gyro_cfg.range = data[0];
			dev->gyro_cfg.odr = (data[1] & SMI230_GYRO_BW_MASK);
			dev->gyro_cfg.bw = dev->gyro_cfg.odr;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the output data rate, range and bandwidth
 * of gyro sensor.
 */
int8_t smi230_gyro_set_meas_conf(const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;
	uint8_t odr, range;
	uint8_t is_range_invalid = FALSE, is_odr_invalid = FALSE;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		odr = dev->gyro_cfg.odr;
		range = dev->gyro_cfg.range;

		if (odr > SMI230_GYRO_BW_32_ODR_100_HZ) {
			/* Updating the status */
			is_odr_invalid = TRUE;
		}

		if (range > SMI230_GYRO_RANGE_125_DPS) {
			/* Updating the status */
			is_range_invalid = TRUE;
		}

		/* If ODR and Range is valid, write it to gyro config. registers */
		if ((!is_odr_invalid) && (!is_range_invalid)) {
			/* Read range value from the range register */
			rslt = get_regs(SMI230_GYRO_BANDWIDTH_REG, &data, 1,
					dev);

			if (rslt == SMI230_OK) {
				data = SMI230_SET_BITS_POS_0(
					data, SMI230_GYRO_BW, odr);

				/* Write odr value to odr register */
				rslt = set_regs(SMI230_GYRO_BANDWIDTH_REG,
						&data, 1, dev);

				if (rslt == SMI230_OK) {
					/* Read range value from the range register */
					rslt = get_regs(SMI230_GYRO_RANGE_REG,
							&data, 1, dev);

					if (rslt == SMI230_OK) {
						data = SMI230_SET_BITS_POS_0(
							data, SMI230_GYRO_RANGE,
							range);

						/* Write range value to range register */
						rslt = set_regs(
							SMI230_GYRO_RANGE_REG,
							&data, 1, dev);
					}
				}
			}

		} else {
			/* Invalid configuration present in ODR, Range */
			rslt = SMI230_E_INVALID_CONFIG;
		}
	}

	return rslt;
}

/*!
 * @brief This API reads the gyro power mode from the sensor,
 * store it in the smi230_dev structure instance
 * passed by the user.
 *
 */
int8_t smi230_gyro_get_power_mode(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		rslt = get_regs(SMI230_GYRO_LPM1_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			/* Updating the power mode in the dev structure */
			dev->gyro_cfg.power = data;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the power mode of the gyro sensor.
 */
int8_t smi230_gyro_set_power_mode(struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t power_mode, prev;
	uint8_t is_power_switching_mode_valid = TRUE;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (rslt == SMI230_OK) {
		/* Use the last power mode actually applied to the hardware as
		 * the "previous" state. The LPM1 register cannot be trusted as
		 * the current power mode: field logs show that while the device
		 * is settled in deep suspend (read taken ~400 ms after entry,
		 * well past the 200 ms switching time) LPM1 still reads back
		 * 0x00, which is indistinguishable from NORMAL. Relying on that
		 * value made a redundant deep-suspend request (e.g. app layer
		 * and PM suspend racing to request it) mis-detect a
		 * NORMAL->DEEP_SUSPEND transition and re-run save_context while
		 * already in deep suspend, where the configuration registers are
		 * lost (read as 0). That zeroed interrupt/FIFO config was then
		 * written back on resume, disabling the interrupts (IRQ halt). */
		prev = dev->gyro_power_applied;
		power_mode = dev->gyro_cfg.power;

		{
			/*switching between normal mode and the suspend modes is allowed, it is not possible to switch
			 * between suspend and deep suspend and vice versa. Check for invalid power switching (i.e)
			 * deep suspend to suspend */
			if ((power_mode == SMI230_GYRO_PM_SUSPEND) &&
			    (prev == SMI230_GYRO_PM_DEEP_SUSPEND)) {
				/* Updating the status */
				is_power_switching_mode_valid = FALSE;
			}

			/* Check for invalid power switching (i.e) from suspend to deep suspend */
			if ((power_mode == SMI230_GYRO_PM_DEEP_SUSPEND) &&
			    (prev == SMI230_GYRO_PM_SUSPEND)) {
				/* Updating the status */
				is_power_switching_mode_valid = FALSE;
			}

			/* Save the context only on a genuine NORMAL -> DEEP_SUSPEND
			 * transition. A redundant deep-suspend request (prev already
			 * DEEP_SUSPEND) is correctly ignored, so the valid saved
			 * context is never overwritten with the lost config. */
			if ((power_mode == SMI230_GYRO_PM_DEEP_SUSPEND) &&
			    (prev == SMI230_GYRO_PM_NORMAL)) {
				PINFO("SMI230 gyro save context");
				rslt = smi230_gyro_save_context(dev);
				if (rslt != SMI230_OK) {
					return rslt;
				}
			}

			/* Check if power switching mode is valid*/
			if (is_power_switching_mode_valid) {
				uint8_t attempt;
				uint8_t readback;

				/* Write the power mode and verify it was
				 * accepted by reading the register back. Retry
				 * on mismatch to guard against a silently
				 * dropped write (e.g. bus glitch/NACK). */
				rslt = SMI230_E_COM_FAIL;
				for (attempt = 0;
				     attempt < SMI230_POWER_MODE_SET_RETRY;
				     attempt++) {
					rslt = set_regs(SMI230_GYRO_LPM1_REG,
							&power_mode, 1, dev);
					if (rslt != SMI230_OK)
						continue;

					/* Time required to switch the power mode */
					dev->delay_ms(
						SMI230_GYRO_POWER_MODE_CONFIG_DELAY);

					/* LPM1 read-back is not reliable in deep
					 * suspend, so it cannot be used to verify
					 * this write. Cross-check instead that the
					 * configuration section was actually
					 * powered down: in deep suspend the config
					 * registers are lost and read back as the
					 * reset value 0. Use INT_CTRL as a witness
					 * because it is non-zero whenever the
					 * new-data/FIFO interrupt is enabled (its
					 * value was captured by save_context just
					 * before this write, while still alive). */
					if (power_mode ==
					    SMI230_GYRO_PM_DEEP_SUSPEND) {
						uint8_t witness =
							dev->gyro_regs
								.gyro_int_ctrl_reg;

						/* No non-zero witness available:
						 * cannot cross-check, accept the
						 * write. */
						if (witness == 0) {
							rslt = SMI230_OK;
							break;
						}

						rslt = get_regs(
							SMI230_GYRO_INT_CTRL_REG,
							&readback, 1, dev);
						if (rslt != SMI230_OK)
							continue;

						/* Config lost -> deep suspend
						 * confirmed. */
						if (readback != witness) {
							rslt = SMI230_OK;
							break;
						}

						/* Config still intact -> the
						 * write did not take effect,
						 * retry. */
						rslt = SMI230_E_COM_FAIL;
						continue;
					}

					rslt = get_regs(SMI230_GYRO_LPM1_REG,
							&readback, 1, dev);
					if (rslt != SMI230_OK)
						continue;

					if (readback == power_mode) {
						rslt = SMI230_OK;
						break;
					}

					/* Register did not take the value */
					rslt = SMI230_E_COM_FAIL;
				}

				if (rslt != SMI230_OK)
					return rslt;

				/* Write succeeded: record the applied power mode so
				 * the next call detects transitions correctly. */
				dev->gyro_power_applied = power_mode;

				/* Restore on a genuine DEEP_SUSPEND -> NORMAL
				 * transition. */
				if ((power_mode == SMI230_GYRO_PM_NORMAL) &&
				    (prev == SMI230_GYRO_PM_DEEP_SUSPEND)) {
					PINFO("SMI230 gyro restore context");
					rslt = smi230_gyro_restore_context(dev);
					if (rslt != SMI230_OK) {
						return rslt;
					}
				}

			} else {
				/* Direct suspend <-> deep-suspend switching is
				 * not supported by the hardware (datasheet
				 * 7.3.2). Rather than failing the caller (which
				 * would abort a system suspend), treat the
				 * request as a no-op: the device is already in a
				 * low-power state, so leave it there and keep the
				 * tracked applied mode unchanged. */
				PINFO("SMI230 gyro ignoring unsupported suspend<->deep-suspend switch");
				rslt = SMI230_OK;
			}
		}
	}
	return rslt;
}

int8_t smi230_gyro_save_context(struct smi230_dev *dev)
{
	int8_t ret;
	uint8_t data[2];

	ret = null_ptr_check(dev);
	if (ret)
		return ret;

	ret = get_regs(SMI230_GYRO_RANGE_REG, data, 2, dev);
	if (ret)
		return ret;
	dev->gyro_regs.range_reg = data[0];
	dev->gyro_regs.bw_reg = data[1];

	PINFO("SMI230 save  context range %d", dev->gyro_regs.range_reg);

	ret = get_regs(SMI230_GYRO_RATE_HBW_REG, data, 1, dev);
	if (ret)
		return ret;
	dev->gyro_regs.rate_hbw_reg = data[0];

	ret = get_regs(SMI230_GYRO_INT_CTRL_REG, data, 2, dev);
	if (ret)
		return ret;
	dev->gyro_regs.gyro_int_ctrl_reg = data[0];
	dev->gyro_regs.int_en_1_reg = data[1];

	ret = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, data, 1, dev);
	if (ret)
		return ret;
	dev->gyro_regs.int3_int4_io_map_reg = data[0];

	ret = get_regs(SMI230_GYRO_WM_INT_REG, data, 1, dev);
	if (ret)
		return ret;
	dev->gyro_regs.fifo_wm_enable_reg = data[0];

	ret = get_regs(SMI230_GYRO_FIFO_EXT_INT_S_REG, data, 1, dev);
	if (ret)
		return ret;
	dev->gyro_regs.bgw_spi3_wdt_fifo_reg = data[0];

	ret = get_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, data, 2, dev);
	if (ret)
		return ret;
	dev->gyro_regs.fifo_config_0_reg = data[0];
	dev->gyro_regs.fifo_config_1_reg = data[1];

	dev->gyro_regs_saved = true;

	return 0;
}

int8_t smi230_gyro_restore_context(struct smi230_dev *dev)
{
	int8_t ret;
	uint8_t power;
	uint8_t data[2];
	uint8_t reg[2];

	ret = null_ptr_check(dev);
	if (ret)
		return ret;

	if (dev->gyro_regs_saved) {
		ret = get_regs(SMI230_GYRO_LPM1_REG, &power, 1, dev);
		if (ret)
			return ret;
		if (power != 0)
			return -1;

		data[0] = dev->gyro_regs.range_reg;
		ret = set_regs(SMI230_GYRO_RANGE_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_RANGE_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_RANGE_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		// ignore the reserved bits
		data[0] = (dev->gyro_regs.bw_reg) & 0x0F;
		ret = set_regs(SMI230_GYRO_BANDWIDTH_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_BANDWIDTH_REG, reg, 1, dev);
		reg[0] = reg[0] & 0x0F; // ignore the reserved bits
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_BANDWIDTH_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.rate_hbw_reg;
		ret = set_regs(SMI230_GYRO_RATE_HBW_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_RATE_HBW_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_RATE_HBW_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.gyro_int_ctrl_reg;
		ret = set_regs(SMI230_GYRO_INT_CTRL_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_INT_CTRL_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_INT_CTRL_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.int_en_1_reg;
		ret = set_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_INT3_INT4_IO_CONF_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.int3_int4_io_map_reg;
		ret = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_INT3_INT4_IO_MAP_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.fifo_wm_enable_reg;
		ret = set_regs(SMI230_GYRO_WM_INT_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_WM_INT_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_WM_INT_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.bgw_spi3_wdt_fifo_reg;
		ret = set_regs(SMI230_GYRO_FIFO_EXT_INT_S_REG, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_FIFO_EXT_INT_S_REG, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_FIFO_EXT_INT_S_REG failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.fifo_config_0_reg;
		ret = set_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_FIFO_CONFIG_0_ADDR failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		data[0] = dev->gyro_regs.fifo_config_1_reg;
		ret = set_regs(SMI230_GYRO_FIFO_CONFIG_1_ADDR, data, 1, dev);
		if (ret)
			return ret;
		get_regs(SMI230_GYRO_FIFO_CONFIG_1_ADDR, reg, 1, dev);
		if (data[0] != reg[0]) {
			PINFO("restore SMI230_GYRO_FIFO_CONFIG_1_ADDR failed val%d readback %d",
			      data[0], reg[0]);
			return -1;
		}

		dev->gyro_regs_saved = false;
	}

	return 0;
}

/*!
 * @brief This API reads the gyro data from the sensor,
 * store it in the smi230_sensor_data structure instance
 * passed by the user.
 */
int8_t smi230_gyro_get_data(struct smi230_sensor_data *gyro,
			    const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data[6];
	uint8_t lsb, msb;
	uint16_t msblsb;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == SMI230_OK) && (gyro != NULL)) {
		/* read gyro sensor data */
		rslt = get_regs(SMI230_GYRO_X_LSB_REG, data, 6, dev);

		if (rslt == SMI230_OK) {
			lsb = data[0];
			msb = data[1];
			msblsb = (msb << 8) | lsb;
			gyro->x = (int16_t)msblsb; /* Data in X axis */

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			gyro->y = (int16_t)msblsb; /* Data in Y axis */

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			gyro->z = (int16_t)msblsb; /* Data in Z axis */
		}

	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API configures the necessary gyro interrupt
 * based on the user settings in the smi230_int_cfg
 * structure instance.
 */
int8_t
smi230_gyro_set_int_config(const struct smi230_gyro_int_channel_cfg *int_config,
			   const struct smi230_dev *dev)
{
	int8_t rslt;

	/* Check for null pointer in the device structure*/
	rslt = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if ((rslt == SMI230_OK) && (int_config != NULL)) {
		switch (int_config->int_type) {
		case SMI230_GYRO_DATA_RDY_INT: {
			/* Data ready interrupt */
			rslt = set_gyro_data_ready_int(int_config, dev);
		} break;
		case SMI230_GYRO_FIFO_INT: {
			/* Data ready interrupt */
			rslt = set_gyro_fifo_int(int_config, dev);
		} break;

		default:
			rslt = SMI230_E_INVALID_CONFIG;
			break;
		}
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API checks whether the self test functionality of the
 *  gyro sensor is working or not.
 */
int8_t smi230_gyro_perform_selftest(const struct smi230_dev *dev)
{
	int8_t ret, rslt = 0;
	uint8_t data = 1;

	/* Check for null pointer in the device structure*/
	ret = null_ptr_check(dev);

	/* Proceed if null check is fine */
	if (ret == SMI230_OK) {
		/* Enable the gyro self-test */
		ret = set_gyro_selftest(SMI230_ENABLE, dev);

		smi230_delay(80);

		if (ret == SMI230_OK) {
			/* Read self-test register */
			ret = get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1,
				       dev);

			if (ret == SMI230_OK) {
				rslt = data & SMI230_GYRO_SELF_TEST_RESULT_MASK;
			}
		}
	}

	return rslt;
}

/*****************************************************************************/
/* Static function definition */

/*!
 * @brief This API is used to validate the device structure pointer for
 * null conditions.
 */
static int8_t null_ptr_check(const struct smi230_dev *dev)
{
	int8_t rslt;

	if ((dev == NULL) || (dev->read == NULL) || (dev->write == NULL) ||
	    (dev->delay_ms == NULL)) {
		/* Device structure pointer is not valid */
		rslt = SMI230_E_NULL_PTR;
	} else {
		/* Device structure is fine */
		rslt = SMI230_OK;
	}

	return rslt;
}

/*!
 * @brief This API reads the data from the given register address of gyro sensor.
 */
static int8_t get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_SPI_INTF) {
		/* Configuring reg_addr for SPI Interface */
		reg_addr = (reg_addr | SMI230_SPI_RD_MASK);
	}

	/* read a gyro register */
	rslt = dev->read(dev->gyro_id, reg_addr, reg_data, len);

	if (rslt != SMI230_OK) {
		/* Updating the error */
		rslt = SMI230_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API writes the given data to the register address of gyro sensor.
 */
static int8_t set_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len,
		       const struct smi230_dev *dev)
{
	int8_t rslt;

	if (dev->intf == SMI230_SPI_INTF) {
		/* Configuring reg_addr for SPI Interface */
		reg_addr = (reg_addr & SMI230_SPI_WR_MASK);
	}

	/* write to a gyro register */
	rslt = dev->write(dev->gyro_id, reg_addr, reg_data, len);

	if (rslt != SMI230_OK) {
		/* Updating the error */
		rslt = SMI230_E_COM_FAIL;
	}

	return rslt;
}

/*!
 * @brief This API sets the data ready interrupt for gyro sensor.
 */
static int8_t
set_gyro_data_ready_int(const struct smi230_gyro_int_channel_cfg *int_config,
			const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t conf, data[2] = { 0 };

	/* read interrupt map register */
	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

	if (rslt == SMI230_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_3:

			/* Data to enable new data ready interrupt */
			data[0] = SMI230_SET_BITS_POS_0(
				data[0], SMI230_GYRO_INT3_MAP, conf);
			PINFO("ch3 data 0 = %hx", data[0]);
			break;

		case SMI230_INT_CHANNEL_4:

			/* Data to enable new data ready interrupt */
			data[0] = SMI230_SET_BITS(data[0], SMI230_GYRO_INT4_MAP,
						  conf);
			PINFO("ch4 data 0 = %hx", data[0]);
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			/*condition to check disabling the interrupt in single channel when both
			 * interrupts channels are enabled*/
			if (data[0] & SMI230_GYRO_MAP_DRDY_TO_BOTH_INT3_INT4) {
				/* Updating the data */
				/* Data to enable new data ready interrupt */
				data[1] = SMI230_GYRO_DRDY_INT_ENABLE_VAL;
			} else {
				data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;
			}

			/* write data to interrupt map register */
			rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG,
					&data[0], 1, dev);

			if (rslt == SMI230_OK) {
				/* Configure interrupt pin */
				rslt = set_int_pin_config(int_config, dev);

				if (rslt == SMI230_OK) {
					/* write data to interrupt control register */
					rslt = set_regs(
						SMI230_GYRO_INT_CTRL_REG,
						&data[1], 1, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the fifo interrupt for gyro sensor.
 */
static int8_t
set_gyro_fifo_int(const struct smi230_gyro_int_channel_cfg *int_config,
		  const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t conf, data[2] = { 0 };

	/* read interrupt map register */
	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG, &data[0], 1, dev);

	if (rslt == SMI230_OK) {
		conf = int_config->int_pin_cfg.enable_int_pin;

		switch (int_config->int_channel) {
		case SMI230_INT_CHANNEL_3:
			/* Data to enable new data ready interrupt */
			data[0] = SMI230_SET_BITS(
				data[0], SMI230_GYRO_FIFO_INT3_MAP, conf);
			break;

		case SMI230_INT_CHANNEL_4:

			/* Data to enable new data ready interrupt */
			data[0] = SMI230_SET_BITS(
				data[0], SMI230_GYRO_FIFO_INT4_MAP, conf);
			break;

		default:
			rslt = SMI230_E_INVALID_INPUT;
			break;
		}

		if (rslt == SMI230_OK) {
			/*condition to check disabling the interrupt in single channel when both
			 * interrupts channels are enabled*/
			if (data[0] & SMI230_GYRO_MAP_FIFO_TO_BOTH_INT3_INT4) {
				/* Updating the data */
				/* Data to enable new data ready interrupt */
				data[1] = SMI230_GYRO_FIFO_INT_ENABLE_VAL;
			} else {
				data[1] = SMI230_GYRO_DRDY_INT_DISABLE_VAL;
			}

			/* write data to interrupt map register */
			rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_MAP_REG,
					&data[0], 1, dev);

			if (rslt == SMI230_OK) {
				/* Configure interrupt pin */
				rslt = set_int_pin_config(int_config, dev);

				if (rslt == SMI230_OK) {
					/* write data to interrupt control register */
					rslt = set_regs(
						SMI230_GYRO_INT_CTRL_REG,
						&data[1], 1, dev);
				}
			}
		}
	}

	return rslt;
}

/*!
 * @brief This API configures the pins which fire the
 * interrupt signal when any interrupt occurs.
 */
static int8_t
set_int_pin_config(const struct smi230_gyro_int_channel_cfg *int_config,
		   const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data;

	/* Read interrupt configuration register */
	rslt = get_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1, dev);

	if (rslt == SMI230_OK) {
		switch (int_config->int_channel) {
		/* Interrupt pin or channel 3 */
		case SMI230_INT_CHANNEL_3:

			/* Update data with user configured smi230_int_cfg structure */
			data = SMI230_SET_BITS_POS_0(
				data, SMI230_GYRO_INT3_LVL,
				int_config->int_pin_cfg.lvl);
			data = SMI230_SET_BITS(
				data, SMI230_GYRO_INT3_OD,
				int_config->int_pin_cfg.output_mode);
			break;

		case SMI230_INT_CHANNEL_4:

			/* Update data with user configured smi230_int_cfg structure */
			data = SMI230_SET_BITS(data, SMI230_GYRO_INT4_LVL,
					       int_config->int_pin_cfg.lvl);
			data = SMI230_SET_BITS(
				data, SMI230_GYRO_INT4_OD,
				int_config->int_pin_cfg.output_mode);
			break;

		default:
			break;
		}

		/* write to interrupt configuration register */
		rslt = set_regs(SMI230_GYRO_INT3_INT4_IO_CONF_REG, &data, 1,
				dev);
	}

	return rslt;
}

/*!
 *  @brief This API enables or disables the Gyro Self test feature in the
 *  sensor.
 */
static int8_t set_gyro_selftest(uint8_t selftest, const struct smi230_dev *dev)
{
	int8_t rslt;
	uint8_t data = 0;

	/* Check for valid selftest input */
	if ((selftest == SMI230_ENABLE) || (selftest == SMI230_DISABLE)) {
		/* Read self test register */
		rslt = get_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1, dev);

		if (rslt == SMI230_OK) {
			/* Enable self-test */
			data = SMI230_SET_BITS_POS_0(
				data, SMI230_GYRO_SELF_TEST_EN, selftest);

			/* write self test input value to self-test register */
			rslt = set_regs(SMI230_GYRO_SELF_TEST_REG, &data, 1,
					dev);
		}

	} else {
		rslt = SMI230_E_INVALID_INPUT;
	}

	return rslt;
}

/*!
 * @brief This API sets the FIFO configuration in the sensor.
 * mode: SMI230_GYRO_FIFO_MODE or SMI230_GYRO_STREAM_MODE
 */
int8_t smi230_gyro_set_fifo_config(struct gyro_fifo_config *config,
				   const struct smi230_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	//uint8_t data = 0;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);

	if (rslt == SMI230_OK) {
		rslt = smi230_gyro_set_regs(SMI230_GYRO_FIFO_CONFIG_1_ADDR,
					    &config->mode, 1, dev);
		rslt |= smi230_gyro_set_regs(SMI230_GYRO_WM_INT_REG,
					     &config->wm_en, 1, dev);

		// if (config->int3_en ^ config->int4_en) {
		// 	if (config->int3_en)
		// 		data = 0x10;
		// 	else
		// 		data = 0x18;
		// 	rslt |= smi230_gyro_set_regs(
		// 		SMI230_GYRO_FIFO_EXT_INT_S_REG, &data, 1, dev);
		// } else
		// 	rslt |= smi230_gyro_set_regs(
		// 		SMI230_GYRO_FIFO_EXT_INT_S_REG, &data, 1, dev);
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API reads the FIFO data.
 */
int8_t smi230_gyro_read_fifo_data(struct smi230_fifo_frame *fifo,
				  const struct smi230_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;
	/* Variable to define FIFO address */
	uint8_t addr = SMI230_GYRO_FIFO_DATA_ADDR;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (fifo != NULL)) {
		/* Read FIFO data */
		rslt = smi230_gyro_get_regs(addr, fifo->data, fifo->length,
					    dev);
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This API gets the length of FIFO data available in the sensor in
 * bytes.
 */
int8_t smi230_gyro_get_fifo_length(uint16_t *fifo_frames,
				   const struct smi230_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Array to store FIFO data length */
	uint8_t data;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (fifo_frames != NULL)) {
		/* read fifo length */
		rslt = smi230_gyro_get_regs(SMI230_GYRO_FIFO_STATUS_ADDR, &data,
					    1, dev);
		if (rslt == SMI230_OK) {
			(*fifo_frames) = (uint16_t)(data & 0x7F);
		} else {
			rslt = SMI230_E_NULL_PTR;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t smi230_gyro_get_fifo_wm(uint8_t *wm, const struct smi230_dev *dev)
{
	int8_t rslt;

	uint8_t data;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		rslt = smi230_gyro_get_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR,
					    &data, 1, dev);
		if ((rslt == SMI230_OK) && (wm != NULL)) {
			*wm = data & 0x7F;
		} else {
			rslt = SMI230_E_NULL_PTR;
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the FIFO water-mark level in the sensor.
 */
int8_t smi230_gyro_set_fifo_wm(uint8_t wm, const struct smi230_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	if (rslt == SMI230_OK) {
		/* Set the FIFO water-mark level */
		rslt = smi230_gyro_set_regs(SMI230_GYRO_FIFO_CONFIG_0_ADDR, &wm,
					    1, dev);
	}

	return rslt;
}

/*!
 * @brief This internal API is used to parse gyroerometer data from the
 * FIFO data.
 */
static void unpack_gyro_data(struct smi230_sensor_data *gyro,
			     uint16_t data_start_index,
			     const struct smi230_fifo_frame *fifo)
{
	/* Variables to store LSB value */
	uint16_t data_lsb;

	/* Variables to store MSB value */
	uint16_t data_msb;

	/* Accelerometer raw x data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->x = (int16_t)((data_msb << 8) | data_lsb);

	/* Accelerometer raw y data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->y = (int16_t)((data_msb << 8) | data_lsb);

	/* Accelerometer raw z data */
	data_lsb = fifo->data[data_start_index++];
	data_msb = fifo->data[data_start_index++];
	gyro->z = (int16_t)((data_msb << 8) | data_lsb);
}

/*!
 * @brief This internal API is used to parse the gyro data from the
 * FIFO in headerless mode.
 */
static int8_t extract_gyro_headerless_mode(struct smi230_sensor_data *gyro,
					   uint16_t *fifo_length,
					   struct smi230_fifo_frame *fifo)
{
	/* Variable to define error */
	int8_t rslt = SMI230_OK;

	/* Variable to index the data bytes */
	uint16_t data_index;

	/* Variable to index gyroerometer frames */
	uint16_t gyro_index = 0;

	/* Variable to indicate gyroerometer frames read */
	uint16_t frame_to_read = *fifo_length;

	/* Only iterate while a full gyro frame (SMI230_FIFO_GYRO_FRAME_LENGTH
	 * bytes) is still available in the supplied buffer. A FIFO byte count
	 * that is not an exact multiple of the frame length would otherwise
	 * cause unpack_gyro_data() to read past the end of fifo->data. */
	for (data_index = 0;
	     (data_index + SMI230_FIFO_GYRO_FRAME_LENGTH) <= fifo->length;) {
		unpack_gyro_data(&gyro[gyro_index], data_index, fifo);

		data_index += SMI230_FIFO_GYRO_FRAME_LENGTH;
		gyro_index++;

		/* Break if Number of frames to be read is complete or FIFO is mpty */
		if ((frame_to_read == gyro_index) ||
		    (rslt == SMI230_W_FIFO_EMPTY)) {
			break;
		}
	}

	/* Update the gyroerometer frame index */
	(*fifo_length) = gyro_index;

	return rslt;
}

/*!
 * @brief This API parses and extracts the gyroscope frames from FIFO data
 * read by the "smi230_read_fifo_data" API and stores it in the "gyro_data"
 * structure instance.
 */
int8_t smi230_gyro_extract_fifo(struct smi230_sensor_data *gyro_data,
				uint16_t *fifo_length,
				struct smi230_fifo_frame *fifo,
				const struct smi230_dev *dev)
{
	/* Variable to define error */
	int8_t rslt;

	/* Check for null pointer in the device structure */
	rslt = null_ptr_check(dev);
	if ((rslt == SMI230_OK) && (gyro_data != NULL) &&
	    (fifo_length != NULL) && (fifo != NULL)) {
		/* Parsing the FIFO data in header mode */
		rslt = extract_gyro_headerless_mode(gyro_data, fifo_length,
						    fifo);
	} else {
		rslt = SMI230_E_NULL_PTR;
	}

	return rslt;
}

/** @}*/
