#ifndef TRAMES_NMEA_H_
#define TRAMES_NMEA_H_

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

#endif /* TRAMES_NMEA_H_ */
