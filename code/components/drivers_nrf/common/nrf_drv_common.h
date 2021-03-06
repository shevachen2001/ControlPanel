/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#ifndef NRF_DRV_COMMON_H__
#define NRF_DRV_COMMON_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "sdk_errors.h"
#include "nrf_drv_config.h"


/**
 * @brief Offset of event registers in every peripheral instance
 *
 * This is the offset where event registers start in the every peripheral.
 */
#define NRF_DRV_COMMON_EVREGS_OFFSET 0x100U

/**
 * @brief Driver state.
 */
typedef enum 
{ 
    NRF_DRV_STATE_UNINITIALIZED, /**< Uninitialized. */
    NRF_DRV_STATE_INITIALIZED, /**< Initialized but powered off. */
    NRF_DRV_STATE_POWERED_ON
} nrf_drv_state_t;

/**
 * @brief Driver power state selection.
 */
typedef enum
{
    NRF_DRV_PWR_CTRL_ON,   /**< Power on request. */
    NRF_DRV_PWR_CTRL_OFF   /**< Power off request. */
} nrf_drv_pwr_ctrl_t;

/**
 * @brief IRQ handler type.
 */
typedef void (*nrf_drv_irq_handler_t)(void);


#if PERIPHERAL_RESOURCE_SHARING_ENABLED

/**
 * @brief Function for acquiring shared peripheral resources associated with
 *        the specified peripheral.
 *
 * Certain resources and registers are shared among peripherals that have
 * the same ID (for example: SPI0, SPIM0, SPIS0, TWI0, TWIM0, and TWIS0).
 * Only one of them can be utilized at a given time. This function reserves
 * proper resources to be used by the specified peripheral.
 * If PERIPHERAL_RESOURCE_SHARING_ENABLED is set to a non-zero value, IRQ
 * handlers for peripherals that are sharing resources with others are
 * implemented by the nrf_drv_common module instead of individual drivers.
 * The drivers must then specify their interrupt handling routines and
 * register them by using this function.
 *
 * @param[in] p_per_base Requested peripheral base pointer.
 * @param[in] handler    Interrupt handler to register. May be NULL
 *                       if interrupts are not used for the peripheral.
 *
 * @retval NRF_SUCCESS             If resources were acquired successfully.
 * @retval NRF_ERROR_BUSY          If resources were already acquired.
 * @retval NRF_ERROR_INVALID_PARAM If the specified peripheral is not enabled 
 *                                 or the peripheral does not share resources 
 *                                 with other peripherals.
 */
ret_code_t nrf_drv_common_per_res_acquire(void const * p_per_base,
                                          nrf_drv_irq_handler_t handler);

/**
 * @brief Function for releasing shared resources reserved previously by
 *        @ref nrf_drv_common_per_res_acquire() for the specified peripheral.
 *
 * @param[in] p_per_base Requested peripheral base pointer.
 */
void nrf_drv_common_per_res_release(void const * p_per_base);

#endif // PERIPHERAL_RESOURCE_SHARING_ENABLED


/**
 * @brief Function sets priority and enables NVIC interrupt
 *
 * @note Function checks if correct priority is used when softdevice is present
 *
 * @param[in] IRQn     Interrupt id
 * @param[in] priority Interrupt priority
 */
void nrf_drv_common_irq_enable(IRQn_Type IRQn, uint8_t priority);

/**
 * @brief Function disables NVIC interrupt
 *
 * @param[in] IRQn     Interrupt id
 */
__STATIC_INLINE void nrf_drv_common_irq_disable(IRQn_Type IRQn);

/**
 * @brief Convert bit position to event code
 *
 * Function for converting the bit position in INTEN register to event code
 * that is equivalent to the offset of the event register from the beginning
 * of peripheral instance.
 *
 * For example the result of this function can be casted directly to
 * the types like @ref nrf_twis_event_t or @ref nrf_rng_events_t...
 *
 * @param bit Bit position in INTEN register
 * @return Event code to be casted to the right enum type or to be used in functions like
 * @ref nrf_rng_event_get
 *
 * @sa nrf_drv_event_to_bitpos
 */
__STATIC_INLINE uint32_t nrf_drv_bitpos_to_event(uint32_t bit);

/**
 * @brief Convert event code to bit position
 *
 * This function can be used to get bit position in INTEN register from event code.
 *
 * @param event Event code that may be casted from enum values from types like
 * @ref nrf_twis_event_t or @ref nrf_rng_events_t
 * @return Bit position in INTEN register that corresponds to the given code.
 *
 * @sa nrf_drv_bitpos_to_event
 */
__STATIC_INLINE uint32_t nrf_drv_event_to_bitpos(uint32_t event);

/**
 * @brief Get interrupt number connected with given instance
 *
 * Function returns interrupt number for a given instance of any peripheral.
 * @param[in] pinst Pointer to peripheral registry
 * @return Interrupt number
 */
__STATIC_INLINE IRQn_Type nrf_drv_get_IRQn(void const * const pinst);

/**
 * @brief Check if given object is in RAM
 *
 * Function for analyzing if given location is placed in RAM.
 * This function is used to determine if we have address that can be supported by EasyDMA.
 * @param[in] ptr Pointer to the object
 * @retval true  Object is located in RAM
 * @retval false Object is not located in RAM
 */
__STATIC_INLINE bool nrf_drv_is_in_RAM(void const * const ptr);


#ifndef SUPPRESS_INLINE_IMPLEMENTATION

__STATIC_INLINE void nrf_drv_common_irq_disable(IRQn_Type IRQn)
{
    NVIC_DisableIRQ(IRQn);
}

__STATIC_INLINE uint32_t nrf_drv_bitpos_to_event(uint32_t bit)
{
    return NRF_DRV_COMMON_EVREGS_OFFSET + bit * sizeof(uint32_t);
}

__STATIC_INLINE uint32_t nrf_drv_event_to_bitpos(uint32_t event)
{
    return (event - NRF_DRV_COMMON_EVREGS_OFFSET) / sizeof(uint32_t);
}

__STATIC_INLINE IRQn_Type nrf_drv_get_IRQn(void const * const pinst)
{
    uint8_t ret = (uint8_t)((uint32_t)pinst>>12U);
    return (IRQn_Type) ret;
}

__STATIC_INLINE bool nrf_drv_is_in_RAM(void const * const ptr)
{
    return ((((uintptr_t)ptr) & 0xE0000000u) == 0x20000000u);
}

#endif // SUPPRESS_INLINE_IMPLEMENTATION

#endif // NRF_DRV_COMMON_H__
