#ifndef INC_GNSS_NMEA_PARSER_H_
#define INC_GNSS_NMEA_PARSER_H_

#include <string.h>
#include <stdio.h>

#define LINEMAX 200
#define FIELD_MAX 20

/**
 * @brief Data structure that contains the coordinates information
 */
typedef struct
{
   double lat;   /**< Latitude */
   double lon;   /**< Longitude */
   double alt;   /**< Altitude */
} GPS_Coords_t;


//Extract GPS information from the UART received frame
void ProcessNMEALine(char *s, GPS_Coords_t * coords, int * gpsQuality);

#endif /* INC_GNSS_NMEA_PARSER_H_ */
