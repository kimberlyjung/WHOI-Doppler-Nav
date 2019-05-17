/************************************************************/
/*    NAME: Kimberly Jung                                              */
/*    ORGN: MIT                                             */
/*    FILE: HydrophoneSensor.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef HydrophoneSensor_HEADER
#define HydrophoneSensor_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include <string>
#include <cstdlib>
#include <iostream>
#include <cmath>
#include "AngleUtils.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include <vector>
#include <map>
#include "VarDataPair.h"
//#include "XYRangePulse.h"
//#include "XYArc.h"

class HydrophoneSensor : public AppCastingMOOSApp
{
 public:
   HydrophoneSensor();
   ~HydrophoneSensor();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool    handleNodeReport(const std::string&);
   bool    handleFrequencyRequest(const std::string&);
   double  getTrueNodeNodeRange(const std::string&, const std::string&);
   double  getTrueNodeNodeFrequency(const std::string&, const std::string&);
   double  getTrueNodeNodeHeading(const std::string&, const std::string&);
   double  getTrueNodeNodeBearing(const std::string&, const std::string&);
   bool    allowableEchoType(std::string);

 private: // Configuration variables
  double               m_default_node_push_dist;
  double               m_default_node_pull_dist;
  double               m_default_node_ping_wait;
  std::vector<double>  m_left_arcs;
  std::vector<double>  m_right_arcs;
  std::vector<std::string> m_allow_echo_types;

  // Map is keyed on the name of the vehicle
  std::map<std::string, double> m_map_node_push_dist;
  std::map<std::string, double> m_map_node_pull_dist;
  std::map<std::string, double> m_map_node_ping_wait;

  std::string m_ping_color;
  std::string m_echo_color;
  std::string m_report_vars;
  bool        m_ground_truth;

  std::string m_rn_algorithm;   // Empty string = no random noise
  double      m_rn_uniform_pct;
  double      m_rn_gaussian_sigma;

  // Added by Alon Yaari Jan 2013
  bool        m_display_range_pulse;

  bool m_first_reading;
  double m_range_prev, m_range_now;
  int m_set_frequency; // set leader's emitting frequency
  int m_sensor_frequency; //frequency at which sensor reads a signal
  const unsigned int m_c = 1500; //speed of sound in water

 private: // State variables
  std::string m_leader;

  std::map<std::string, NodeRecord>   m_map_node_records;
  std::map<std::string, unsigned int> m_map_node_reps_recd;
  std::map<std::string, unsigned int> m_map_node_pings_gend;
  std::map<std::string, unsigned int> m_map_node_echos_recd;
  std::map<std::string, unsigned int> m_map_node_echos_sent;
  std::map<std::string, unsigned int> m_map_node_xping_freq;
  std::map<std::string, unsigned int> m_map_node_xping_dist;
  std::map<std::string, double>       m_map_node_last_ping;
};

#endif 
