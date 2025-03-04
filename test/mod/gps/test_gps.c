/*******************************************************************************
*
* FILE: 
*      test_gps.c
*
* DESCRIPTION: 
*      Unit tests for functions in the sensor module 
*
*******************************************************************************/

/*------------------------------------------------------------------------------
Standard Includes                                                                     
------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*------------------------------------------------------------------------------
Project Includes                                                                     
------------------------------------------------------------------------------*/
#include "main.h" /* mock */
#include "sensor.h"
#include "gps.h"
#include "unity.h"

/*------------------------------------------------------------------------------
Global Variables 
------------------------------------------------------------------------------*/
UART_HandleTypeDef huart4;  /* GPS            */

/*------------------------------------------------------------------------------
Macros
------------------------------------------------------------------------------*/
#define NUM_TESTS_readings_to_bytes ( 3 )

/*------------------------------------------------------------------------------
Procedures 
------------------------------------------------------------------------------*/

void gpsStructToFile(GPS_DATA* data, FILE* f) {
	// calculated values
	fprintf(f, "{");
    fprintf(f, "%f,", data->dec_longitude);
	fprintf(f, "%f,", data->dec_latitude);
	fprintf(f, "%f,", data->altitude_ft);

    // GGA - Global Positioning System Fixed Data
	fprintf(f, "%f,", data->nmea_longitude);
	fprintf(f, "%f,", data->nmea_latitude);
	fprintf(f, "%f,", data->utc_time);
	fprintf(f, "%c,", data->ns);
	fprintf(f, "%c,", data->ew);
	fprintf(f, "%d,", data->lock);
	fprintf(f, "%d,", data->satelites);
	fprintf(f, "%f,", data->hdop);
	fprintf(f, "%f,", data->msl_altitude);
	fprintf(f, "%c,", data->msl_units);

    // RMC - Recommended Minimmum Specific GNS Data
	fprintf(f, "%c,", data->rmc_status);
	fprintf(f, "%f,", data->speed_k);
	fprintf(f, "%f,", data->course_d);
	fprintf(f, "%d,", data->date);

    // GLL
	fprintf(f, "%c,", data->gll_status);

    // VTG - Course over ground, ground speed
	fprintf(f, "%f,", data->course_t);
	fprintf(f, "%c,", data->course_t_unit);
	fprintf(f, "%f,", data->course_m);
	fprintf(f, "%c,", data->course_m_unit);
	fprintf(f, "%c,", data->speed_k_unit);
    fprintf(f, "%f,", data->speed_km);
	fprintf(f, "%c", data->speed_km_unit);

	fprintf(f, "}");
	fprintf(f, "\n");
}

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       setUp                                                                  *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Code to run prior to any test                                          *
*                                                                              *
*******************************************************************************/
void setUp
	(
	void
    )
{
printf("-----------------------\n");
printf("	GPS UNIT TESTING\n");
printf("-----------------------\n");
} /* setUp */


/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       tearDown                                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Code to run after tests                                                *
*                                                                              *
*******************************************************************************/
void tearDown 
	(
	void
    )
{
} /* tearDown */

/*******************************************************************************
*                                                                              *
* PROCEDURE:                                                                   * 
*       test_GPS_parse			                                               *
*                                                                              *
* DESCRIPTION:                                                                 * 
*       Test converting extracted dataframe to string in dataframe_to_string.c *
*                                                                              *
*******************************************************************************/
void test_GPS_parse 
	(
	void
    )
{
/*------------------------------------------------------------------------------
Initializations
------------------------------------------------------------------------------*/
char buffer_str[175];

// Source: https://github.com/esutton/gps-nmea-log-files/blob/master/AMOD_AGL3080_20121104_134730.txt

char* input_strings[] = 
{
#include "cases/NMEA_inputs.txt"
};
GPS_DATA data;
FILE* f = fopen("cases/expected_gps_data.txt", "w");
for (int i = 0; i < 12; i++) {
	GPS_parse(&data, input_strings[i]);
	gpsStructToFile(&data, f);
	printf("%d", i);
}
fclose(f);

/*
GPS_DATA expected_data[] = 
{
#include "cases/expected_gps_data.txt"
};
*/

// TEST_ASSERT_EQUAL_STRING(input_strings[0], expected_data[0]);


/*------------------------------------------------------------------------------
Run Tests
------------------------------------------------------------------------------*/

//for ( int test_num = 0; test_num < 3; test_num++ )
//	{
//    /* Initialize input/output */
//	memset(&buffer_str, 0, sizeof(char) * 175); /* Clean buffer string every test */
//	dataframe_to_string( &(sensor_data_test[test_num]), 0, &buffer_str[0]); /* Call a testing function */
//	TEST_ASSERT_EQUAL_STRING(expected_data[test_num], buffer_str); /* Test begins */
//  }
//
//

TEST_ASSERT_EQUAL_CHAR('a', 'a');
}


/*------------------------------------------------------------------------------
Run Tests
------------------------------------------------------------------------------*/
int main
	(
	void
	)
{
UNITY_BEGIN();
RUN_TEST( test_GPS_parse );

return UNITY_END();
} /* main */


/*******************************************************************************
* END OF FILE                                                                  * 
*******************************************************************************/