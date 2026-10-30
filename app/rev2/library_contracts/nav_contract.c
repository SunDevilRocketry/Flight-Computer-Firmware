/**
  ******************************************************************************
  * @file           : nav_contract.c
  * @brief          : Contains nav library contract functions.
  * @note			: Criticality = Flight Qualified
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE
  * file in the root directory of this software component.
  * If no LICENSE file comes with this software, it is covered under the
  * BSD-3-Clause.
  *
  * https://opensource.org/license/bsd-3-clause
  *
  ******************************************************************************
  */

/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>

#include "main.h"
#include "nav_contract.h"

/*------------------------------------------------------------------------------
 Public Procedures                                                               
------------------------------------------------------------------------------*/

/**
 * @brief Reserve this nav state instance.
 * 
 * @param nav_state The state to reserve.
 * 
 * @retval NAV_BUSY Could not reserve state.
 * @retval NAV_OK State reservation succeeded.
 */
NAV_STATUS nav_reserve_state
    (
    NAV_PUBLIC_STATE* nav_state
    )
{
/**
 * The flight computer runs as a single-threaded system. Only one thread will
 * ever call into stateful APIs of the NAV library, so we do not need to 
 * synchronize its state.
 */
(void)nav_state;
return NAV_OK;

} /* nav_reserve_state */


/**
 * @brief Release this nav state instance.
 * 
 * @param nav_state The state to release.
 * 
 * @retval NAV_FAIL Could not release state.
 * @retval NAV_OK State reservation succeeded.
 */
NAV_STATUS nav_release_state
    (
    NAV_PUBLIC_STATE* nav_state
    )
{
/**
 * The flight computer runs as a single-threaded system. Only one thread will
 * ever call into stateful APIs of the NAV library, so we do not need to 
 * synchronize its state.
 */
(void)nav_state;
return NAV_OK;

} /* nav_release_state */


/**
 * @brief Notify the integrating project of a change in NAV's phase of flight.
 *
 * @param[in,out] nav_state The state of the NAV library
 * @param[in] original_state The starting state during this update
 * @param[in] new_state The new NAV state.
 */
void nav_pof_transition_event
    (
    NAV_PUBLIC_STATE* nav_state,
    NAV_POF original_state,
    NAV_POF new_state
    )
{
/**
 * This function is part of NAV's design to allow event driven programming for
 * state changes. On the flight computer, we have a specific point we expect
 * to update this state, so this function is a no-op.
 */
(void)nav_state;
(void)original_state;
(void)new_state;

} /* nav_pof_transition_event */

/**
 * END OF FILE
 */