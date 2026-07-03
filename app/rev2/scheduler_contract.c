/**
  ******************************************************************************
  * @file           : scheduler_contract.c
  * @brief          : TODO
  * @attention      : Criticality: FQ - Flight Qualified
  ******************************************************************************
  * @copyright
  *
  * Copyright (c) 2025 Sun Devil Rocketry.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
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


/*------------------------------------------------------------------------------
 Project Includes
------------------------------------------------------------------------------*/
#include "main.h"
#include "scheduler.h"
#include "pool_allocator.h"


/*------------------------------------------------------------------------------
 Scheduler Pool                                                                
------------------------------------------------------------------------------*/
#define SCHEDULER_POOL_SIZE ( 32 * sizeof(TASK_LIST) ) // TODO MOVE
uint8_t scheduler_pool_mem[SCHEDULER_POOL_SIZE];
Pool scheduler_pool;

/**
 * @brief Initializes the scheduler's memory pool
 */
void scheduler_pool_init
    (
    void
    )
{
scheduler_pool = pool_init(scheduler_pool_mem, SCHEDULER_POOL_SIZE);
}


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/