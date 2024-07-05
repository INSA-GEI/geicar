#include <trames_nmea.h>

//Interprets gps frames to retrieve useful data (latitude, longitude, altitude ...)
void ProcessNMEALine(char *s, GPS_Coords_t * coords, int * quality)
{
  char *field[FIELD_MAX];
  int f;
  int i;
  char c;
  int x;


  // Check the line synchronization

  if (s[0] != '$')
    return;

  // Verify the line checksum integrity

  c = 0; // checksum

  i = 1; // Xor bytes between $ and *, but not including those bytes

  while((s[i] != 0) && (s[i] != '*'))
    c ^= s[i++];

  if (s[i] == 0)
    return;

  sscanf(&s[i + 1],"%x",&x); // Checksum byte - Note sscanf needs this to be an int, rather than a single byte

  if (c != (char)(x & 0xFF)) // Leave if checksum fails
    return;

  // Parse out fields on , and *

  f = 0;

  while(1)
  {
    field[f++] = s;

    while((*s != 0) && (*s != ',') && (*s != '*') && (*s != 0x0D) && (*s != 0x0A))
      s++;

    if ((*s == 0) || (*s == '*') || (*s == 0x0D) || (*s == 0x0A) || (f == (FIELD_MAX - 1)))
    {
      *s = 0;
      field[f] = NULL;
      break;
    }

    *s++ = 0;
  }


  // Process a couple of NMEA sentences for illustration

  if (strncmp(field[0],"$G",2) == 0) // Allow for different Talker ID's ie $GP, $GN, $GL, etc
  {
    if ((strcmp(&field[0][3],"GLL") == 0) && (f > 6)) // Geographic Position, Latitude, Longitude and Time
    {
      double lat, lon;
      int lat_deg, lon_deg;
      double lat_min, lon_min;
      double fix_time;
      int fix_hour, fix_minute;
      double fix_second;
      char lat_hemi, lon_hemi, valid;

      // Field 1 Latitude DDMM.mmmmmm
      // Field 2 Lat Hemi N/S
      // Field 3 Longitude DDMMM.mmmmm
      // Field 4 Lon Hemi E/W
      // Field 5 UTC Time HHMMSS.SSS
      // Field 6 Fix A=Valid, V=Not Valid
      // Field 7 Mode A=Autonomous, D=Differential, E=Estimated, N=Not Valid [Optional] - Simulator Does not report this

      sscanf(field[1],"%lf",&lat);

      lat_hemi = field[2][0];

      sscanf(field[3],"%lf",&lon);

      lon_hemi = field[4][0];

      sscanf(field[5],"%lf",&fix_time);

      valid = field[6][0];

      if (valid == 'A')
      {
        // Extract Time-of-Fix

        fix_minute = (int)fix_time / 100;

        fix_second = fix_time - (fix_minute * 100);

        fix_hour = fix_minute / 100;

        fix_minute = fix_minute % 100;

        // Process Latitude DDMM.mmmmm

        lat_deg = (int)lat / 100; // Decompose NMEA form ASCII into DEGREES and MINUTES

        lat_min = lat - (lat_deg * 100);

        lat = (double)lat_deg + (lat_min / 60.0); // Computed Latitude in DECIMAL DEGREES

        if (lat_hemi == 'S')
        {
          lat_deg = -lat_deg;
          lat = -lat;
        }

        // Process Longitude DDDMM.mmmmm

        lon_deg = (int)lon / 100; // Decompose NMEA form ASCII into DEGREES and MINUTES

        lon_min = lon - (lon_deg * 100);

        lon = (double)lon_deg + (lon_min / 60.0); // Computed Longitude in DECIMAL DEGREES

        if (lon_hemi == 'W')
        {
          lon_deg = -lon_deg;
          lon = -lon;
        }


        coords->lat=lat;
        coords->lon=lon;
      }

    }
    else if ((strcmp(&field[0][3],"GSA") == 0) && (f > 17)) // GPS DOP and Active Satellites
    {
      char mode, fix;
      double pdop, hdop, vdop;
      int i, sv;

      // Field 1 A=Automatic(3D/2D), M=Manual
      // Field 2 Fix 1=No Fix, 2=2D, 3=3D
      // Field 3 SV List#1
      // Field 14 SV List#12
      // Field 15 PDOP
      // Field 16 HDOP
      // Field 17 VDOP

      mode = field[1][0];
      fix = field[2][0];

      sscanf(field[15],"%lf",&pdop); // Position Dilution of precision (PDOP)
      sscanf(field[16],"%lf",&hdop); // Horizontal Dilution of precision (HDOP)
      sscanf(field[17],"%lf",&vdop); // Vertical Dilution of precision (VDOP)


      for(i=0; i<12; i++)
      {
        if (field[3+i][0])
        {
          sscanf(field[3+i],"%d",&sv);
        }
      }


    }
    else  if ((strcmp(&field[0][3],"GGA") == 0) && (f > 14))
    {
      double lat, lon, alt, msl;
      int lat_deg, lon_deg;
      double lat_min, lon_min;
      double fix_time;
      int fix_hour, fix_minute;
      double fix_second;
      char lat_hemi, lon_hemi;
      int valid = 1;
      double hdop;
      int sv;

      // Field  1 UTC Time HHMMSS.SSS
      // Field  2 Latitude
      // Field  3 Lat Hemi
      // Field  4 Longitude
      // Field  5 Lon Hemi
      // Field  6 Position Fix Indicator - 0=Fix Not available, 1=GPS SPS, 2=GPS SPS DIFF, 3=GPS PPS,
      //              4=RTK, 5=Float RTK, 6=Estimate/Dead Reckoning, 7=Manual, 8=Simulator
      // Field  7 Satellites Used
      // Field  8 HDOP
      // Field  9 MSL Altitude
      // Field 10 Units M
      // Field 11 Geoid Separation
      // Field 12 Units M
      // Field 13 Age of Differential (NULL when not used)
      // Field 14 Differential Reference Station ID

      sscanf(field[2],"%lf",&lat);

      lat_hemi = field[3][0];

      sscanf(field[4],"%lf",&lon);

      lon_hemi = field[5][0];

      sscanf(field[9],"%lf",&msl);

      sscanf(field[11],"%lf",&alt);

      sscanf(field[1],"%lf",&fix_time);

      if (sscanf(field[6],"%d",quality) == 0)// 0=Fix Not available, 1=GPS SPS, 2=GPS SPS DIFF
        valid = 0;

      sscanf(field[7],"%d",&sv);    // Satellites used (or in view, depends on receiver)

      sscanf(field[8],"%lf",&hdop); // Horizontal Dilution of precision (HDOP)

      if (valid != 0)
      {
        lat_deg = (int)lat / 100;

        lat_min = lat - (lat_deg * 100);

        lat = (double)lat_deg + (lat_min / 60.0);

        if (lat_hemi == 'S')
          lat = -lat;


        lon_deg = (int)lon / 100;

        lon_min = lon - (lon_deg * 100);

        lon = (double)lon_deg + (lon_min / 60.0);

        if (lon_hemi == 'W')
          lon = -lon;

        alt += msl; // Convert to height above datum

        fix_minute = (int)fix_time / 100;

        fix_second = fix_time - (fix_minute * 100);

        fix_hour = fix_minute / 100;

        fix_minute = fix_minute % 100;

        coords->lat=lat;
        coords->lon=lon;
        coords->alt=alt;
      }
    }
  }
  return;
}
