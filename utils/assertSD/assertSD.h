/*******************************************************************************
*
* FILE: 
* 		assertSD.h
*
* DESCRIPTION: 
* 		Contains asserts methods that output to a log file in SD card 
*
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ASSERTSD_H
#define ASSERTSD_H

/*------------------------------------------------------------------------------
 Project Includes  
------------------------------------------------------------------------------*/
#include "assertSD.h"
#include "main.h"
#include <stdbool.h>
#include "sd_card.h"

void assert
    (
    bool condition, 
    char* message_ptr
    );

#endif /* ASSERTSD_H */