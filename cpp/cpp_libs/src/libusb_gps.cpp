/**
 *  @file libusb_gps.h
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 23.12.2016
 *
 *  @brief Used to read uBlox EVKM8N.
 *         Has been included in ROS in gnss_protocols.h
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *
 *  @todo Plot data in Gnuplot (visualize GNSS data) see "int main()"
 *
 *
 */

#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <boost/algorithm/string.hpp>


/* GGA.
 * Der GGA-Datensatz (GPS Fix Data) beinhaltet Informationen bezüglich Zeit, geographische Länge
 * und Breite, Qualität des Systems, Anzahl der genutzten Satelliten und Höhe.
*/
struct NMEA_GGA
{
  std::string id_dts;
  double utc_time;
  double latitude;
  std::string lat_dir;
  double longitude;
  std::string long_dir;
  int gps_quality;
  int num_sat;
  double HDOP;
  int height;
  std::string height_unit;
  int dh_ellip;
  std::string dh_unit;
  double date_DGPS;
  int id_dgps;
  int checksum;
};

/* GSA.
 * Der GSA-Datensatz (GNSS DOP and Active Satellites) beinhaltet Informationen bezüglich Messmodus
 * (2D oder 3D), Anzahl der zur Bestimmung der Position verwendeten Satelliten und Genauigkeit der
 * Messungen (DOP: Dilution of Precision).
*/
struct NMEA_GSA
{
  std::string id_dts; /* Protocoll type */
  std::string c_mode;
  int i_mode;
  int sat_id[12]; /* up to 12 satellites specified */
  double PDOP;
  double HDOP;
  double VDOP;
  int checksum;
};

/* RMC.
 * Der RMC-Datensatz (Recommended Minimum Specific GNSS Data) beinhaltet Informationen bezüglich
 * Zeit, geographische Breite, Länge, Status des Systems, Geschwindigkeit, Kurs und Datum. Dieser
 * Datensatz wird von allen GPS-Empfängern übermittelt.
 */
struct NMEA_RMC
{
  std::string id_dts;
  double utc_time;
  std::string quality;
  double latitude;
  std::string lat_dir;
  double longitude;
  std::string long_dir;
  double velocity;
  double vel_cir;
  int date;
  double declination;
  std::string decl_dir;
  int checksum;
};

/* VTG.
 * Der VTG-Datensatz (Course over Ground and Ground Speed) beinhaltet Informationen bezüglich Kurs
 * und Geschwindigkeit.
*/
struct NMEA_VTG
{
  std::string id_dts;
  double dir_hor;
  std::string dir_id; /* T=map rel., M=Magn. Nordpol */
  double dir_hor2;
  std::string dir_id2;
  double vel_hor;
  std::string vel_unit; /* N=Knoten, K=km/h */
  double vel_hor2;
  std::string vel_unit2;
  int checksum;
};

/* NMEA message has to be separated between $ and * */
char nmea_checksum(std::string gnss_check)
{
  char check = 0;
  /* iterate, XOR each byte with the total sum: */
  for (int i = 0; i < gnss_check.size(); i++) {
    check = char(check ^ gnss_check.at(i));
  }
  return check;
}

std::string assign_nmea(struct NMEA_GGA * nmea_gga, struct NMEA_GSA * nmea_gsa, struct NMEA_RMC * nmea_rmc,
                struct NMEA_VTG * nmea_vtg, std::string gnss_string)
{
  int checksum;
  std::vector<std::string> gnss_sep;
  std::vector<std::string> gnss_check;
  std::vector<std::string> gnss_buf;

  /* Divide comma separated string */
  boost::split(gnss_sep, gnss_string, boost::is_any_of(","));
  boost::split(gnss_check, gnss_string, boost::is_any_of("*"));

  /* Compute checksum */
  checksum = (int)nmea_checksum(gnss_check[0]);

  /* Divide checksum and last element */
  boost::split(gnss_buf, gnss_sep[gnss_sep.size()-1], boost::is_any_of("*"));
  gnss_sep[gnss_sep.size()-1] = gnss_buf[0];
  gnss_sep.push_back(gnss_buf[1]);

  /* Exchange empty string with -1. Prevents error in std::stoi() later */
  for (int i=0; i<gnss_sep.size();i++)
  {
    if (gnss_sep[i]=="")
    { gnss_sep[i]="-1";}
  }

  /* Check checksum */
  if (!strtoul(gnss_sep[gnss_sep.size()-1].c_str(),NULL,16) == checksum)
  {
    std::cout << "Checksum wrong\n";
  }

   /* Update GGA struct */
  if (gnss_string.find("GGA")==2)
  {
    nmea_gga->id_dts = gnss_sep[0];
    nmea_gga->utc_time = std::stod(gnss_sep[1]);
    nmea_gga->latitude = std::stod(gnss_sep[2]);
    nmea_gga->lat_dir = gnss_sep[3];
    nmea_gga->longitude = std::stod(gnss_sep[4]);
    nmea_gga->long_dir = gnss_sep[5];
    nmea_gga->gps_quality = std::stoi(gnss_sep[6]);
    nmea_gga->num_sat = std::stoi(gnss_sep[7]);
    nmea_gga->HDOP = std::stod(gnss_sep[8]);
    nmea_gga->height = std::stoi(gnss_sep[9]);
    nmea_gga->height_unit = gnss_sep[10];
    nmea_gga->dh_ellip = std::stoi(gnss_sep[11]);
    nmea_gga->dh_unit = gnss_sep[12];
    nmea_gga->date_DGPS = std::stod(gnss_sep[13]);
    nmea_gga->id_dgps = std::stoi(gnss_sep[14]);
    nmea_gga->checksum = std::stoi(gnss_sep[15]);
    return "GGA";
  }

  /* Update GSA struct */
  else if (gnss_string.find("GSA")==2)
  {
    nmea_gsa->id_dts = gnss_sep[0];
    nmea_gsa->c_mode = gnss_sep[1];
    nmea_gsa->i_mode = std::stoi(gnss_sep[2]);
    for (int i=0; i<12; i++){ nmea_gsa->sat_id[i] = std::stoi(gnss_sep[3+i]); }
    nmea_gsa->PDOP = std::stod(gnss_sep[15]);
    nmea_gsa->HDOP = std::stod(gnss_sep[16]);
    nmea_gsa->VDOP = std::stod(gnss_sep[17]);
    nmea_gsa->checksum = checksum;
    return "GSA";
  }

  /* Update RMC struct */
  else if (gnss_string.find("RMC")==2)
  {
    nmea_rmc->id_dts = gnss_sep[0];
    nmea_rmc->utc_time = std::stod(gnss_sep[1]);
    nmea_rmc->quality = gnss_sep[2];
    nmea_rmc->latitude = std::stod(gnss_sep[3]);
    nmea_rmc->lat_dir = gnss_sep[4];
    nmea_rmc->longitude = std::stod(gnss_sep[5]);
    nmea_rmc->long_dir = gnss_sep[6];
    nmea_rmc->velocity = std::stod(gnss_sep[7]);
    nmea_rmc->vel_cir = std::stod(gnss_sep[8]);
    nmea_rmc->date = std::stoi(gnss_sep[9]);
    nmea_rmc->declination = std::stod(gnss_sep[10]);
    nmea_rmc->decl_dir = gnss_sep[11];
    nmea_rmc->checksum = checksum;
    return "RMC";
  }

  /* Update VTG struct */
  else if (gnss_string.find("VTG")==2)
  {
    nmea_vtg->id_dts = gnss_sep[0];
    nmea_vtg->dir_hor = std::stod(gnss_sep[1]);
    nmea_vtg->dir_id = gnss_sep[2];
    nmea_vtg->dir_hor2 = std::stod(gnss_sep[3]);
    nmea_vtg->dir_id2 = gnss_sep[4];
    nmea_vtg->vel_hor = std::stod(gnss_sep[5]);
    nmea_vtg->vel_unit = gnss_sep[6];
    nmea_vtg->vel_hor2 = std::stod(gnss_sep[7]);
    nmea_vtg->vel_unit2 = gnss_sep[8];
    nmea_vtg->checksum = checksum;
    return "VTG";
  }
  else
  {
    return "";
  }
}

int plot_gnuplot( double xvals[], double yvals[], FILE * gnuplot_pipe)
{
  for (int i=0; i < 5; i++)
  {
    fprintf(gnuplot_pipe, "%lf %lf \n", xvals[i], yvals[i]); //Send commands to gnuplot one by one.
  }
  fprintf(gnuplot_pipe, "e");
  return 0;
}



int main()
{
  double xvals[] = {1.0, 2.0, 3.0, 4.0, 5.0};
  double yvals[] = {5.0 ,3.0, 1.0, 3.0, 5.0};
  /*Opens an interface that one can use to send commands as if they were typing into the
  *     gnuplot command line.  "The -persistent" keeps the plot open even after your
  *     C program terminates.
  */
  FILE * gnuplot_pipe = popen ("gnuplot -persistent", "w");
  fprintf(gnuplot_pipe, "%s \n", "set title \"Hello\"");
  plot_gnuplot(xvals, yvals, gnuplot_pipe);

  std::string gnss_string="";
  std::string nmea_type;
  char gnss_message[82]; /* maximal size of NMEA message */
  NMEA_GGA nmea_gga;
  NMEA_GSA nmea_gsa;
  NMEA_RMC nmea_rmc;
  NMEA_VTG nmea_vtg;

  int fd = open("/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00", O_RDWR);

  while(1)
  {
    for (int i=0; i<82; i++)
    {
      read(fd, &gnss_message[i], 1);
      gnss_string.push_back(gnss_message[i]);
      /* Loop until Carriage return <CR> and new line <NL> found */
      if (gnss_message[i]=='\n' && gnss_message[i-1]=='\r')
      {
        gnss_string.pop_back(); /* Delete <NL> */
        gnss_string.pop_back(); /* Delete <CR< */
        gnss_string.erase(gnss_string.begin());
        nmea_type = assign_nmea(&nmea_gga, &nmea_gsa, &nmea_rmc, &nmea_vtg, gnss_string);
        if (nmea_type == "RMC")
        {
          std::cout << nmea_rmc.utc_time << '\n';
        }
        gnss_string = ""; /* Set String back to NULL */
        break;
      }
    }

  }
  return 0;
}
