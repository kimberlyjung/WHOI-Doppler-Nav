/************************************************************/
/*    NAME: Kimberly Jung                                              */
/*    ORGN: MIT                                             */
/*    FILE: HydrophoneSensor.cpp                                        */
/*    DATE:                                                 */
/************************************************************/

#include "HydrophoneSensor.h"

using namespace std;

//---------------------------------------------------------
// Constructor

HydrophoneSensor::HydrophoneSensor()
{
  m_set_frequency = 25000;
  m_sensor_frequency = 1;  // 1 Hz 
  m_range_prev = 0;
  m_range_now = 0;
  m_first_reading=true;
}

//---------------------------------------------------------
// Destructor

HydrophoneSensor::~HydrophoneSensor()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool HydrophoneSensor::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  bool handled;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval = msg.GetString();

    if(key == "FREQUENCY_REQUEST") {
      handled = handleFrequencyRequest(sval);
    }
    else if(key == "NODE_REPORT") {
      handled = handleNodeReport(sval);
    }
    

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool HydrophoneSensor::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool HydrophoneSensor::Iterate()
{
  AppCastingMOOSApp::Iterate();
  AppCastingMOOSApp::PostReport();
  //***** KYJ added this line below because otherwise FREQUENCY_REQUESTS are stale for just one vehicle!
  bool handled = handleFrequencyRequest("gilda");
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool HydrophoneSensor::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "leader") {
      m_leader = value;
      handled = true;
    }
    else if(param == "set_frequency") {
      m_set_frequency = atoi(value.c_str());
      handled = true;
    }
    else if(param == "sensor_frequency") {
      m_sensor_frequency = atoi(value.c_str());
      handled = true;
      //currently this variable is doing nothing
    }
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void HydrophoneSensor::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NODE_REPORT", 0);
  Register("FREQUENCY_REQUEST", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool HydrophoneSensor::buildReport() 
{
  m_msgs << "============================================" << endl;

  //ACTable actab(4);
  //actab << "Alpha | Bravo | Charlie | Delta";
  //actab.addHeaderLines();
  //actab << "one" << "two" << "three" << "four";
  //m_msgs << actab.getFormattedString();

  return(true);
}

bool HydrophoneSensor::handleNodeReport(const string& node_report_str)
{
  NodeRecord new_node_record = string2NodeRecord(node_report_str);
  if(!new_node_record.valid()) return(false);
  string vname = new_node_record.getName();
  //Notify("NEW_NODE_RECORD", vname);
  //if(vname == "") {
    //reportRunWarning("Unhandled NODE_REPORT. Missing Vehicle/Node name.");
    //return(false);
  //}

  m_map_node_records[vname] = new_node_record;
    //m_map_node_reps_recd[vname]++;

    // If node push_dist not pre-configured, set to node default
    //if(m_map_node_push_dist.count(vname) == 0)
      //m_map_node_push_dist[vname] = m_default_node_push_dist;
    // If node pull_dist not pre-configured, set to node default
    //if(m_map_node_pull_dist.count(vname) == 0)
      //m_map_node_pull_dist[vname] = m_default_node_pull_dist;
    // If node ping_wait time not pre-configured, set to node default
    //if(m_map_node_ping_wait.count(vname) == 0)
      //m_map_node_ping_wait[vname] = m_default_node_ping_wait;
  return(true);
}

//---------------------------------------------------------
// Procedure: handleFrequencyRequest
//   Example: vname=alpha

bool HydrophoneSensor::handleFrequencyRequest(const string& request)
{
  string vname = request;
  string capVNAME = toupper(vname);
  
  // Phase 1: Confirm this request is coming from a known vehicle.
  //if(vname == "") {
    //reportRunWarning("Failed Range Request: Unknown vehicle["+vname+"]");
    //reportEvent("Failed Range Request: Unknown vehicle["+vname+"]");
    //return(false);
  //}

  //m_map_node_pings_gend[vname]++;

  // Phase 4: Handle range reports to target nodes. Generate FREQUENCY_ANSWER
  
  //double push_dist = m_map_node_push_dist[vname];
  //map<string, NodeRecord>::iterator p;
  //for(p=m_map_node_records.begin(); p!=m_map_node_records.end(); p++) {
    //string contact_name = p->first;
    //string contact_type = tolower(p->second.getType());
    
    // Check if the contact type is allowed to be pinged on.
    //bool allowed_type = allowableEchoType(contact_type);
    if(vname=="gilda" && m_map_node_records.size()>1) {
      Notify("GILDA_NODE_RECORD", m_map_node_records["gilda"].getSpec());
      Notify("HENRY_NODE_RECORD", m_map_node_records["henry"].getSpec());
      double actual_frequency_now = getTrueNodeNodeFrequency(vname, "henry");
      //                            getTrueNodeNodeFrequency(follower,leader)
      Notify("FREQUENCY_ANSWER_" + capVNAME, actual_frequency_now);
      //double actual_heading_now = getTrueNodeHeading(vname);
      //double actual_bearing_now =  getTrueNodeNodeBearing(vname, contact_name);
      //double pull_dist = m_map_node_pull_dist[contact_name];
    }

  //}
  return(true);
}

//------------------------------------------------------------
// Procedure: getTrueNodeNodeFrequency()

double HydrophoneSensor::getTrueNodeNodeFrequency(const string& node_a, const string& node_b)
{
  if((m_map_node_records.count(node_a) == 0) ||
     (m_map_node_records.count(node_b) == 0))
    return(-1);

  double anode_x       = m_map_node_records[node_a].getX();
  double anode_y       = m_map_node_records[node_a].getY();
  double bnode_x       = m_map_node_records[node_b].getX();
  double bnode_y       = m_map_node_records[node_b].getY();
  double anode_speed   = m_map_node_records[node_a].getSpeed();
  double bnode_speed   = m_map_node_records[node_b].getSpeed();
  double anode_heading = m_map_node_records[node_a].getHeading();
  double bnode_heading = m_map_node_records[node_b].getHeading();

  double bearing = getTrueNodeNodeBearing(node_a, node_b); // relative bearing
  double heading = getTrueNodeNodeHeading(node_a, node_b); // relative heading 
  Notify("REL_BEARING", bearing);
  Notify("REL_HEADING", heading);

  if(m_first_reading) {
    m_first_reading =false;
    m_range_prev = getTrueNodeNodeRange(node_a, node_b);
    return m_set_frequency;
  }
  m_range_now = getTrueNodeNodeRange(node_a, node_b);
  if(m_range_now < m_range_prev) { //towards 
    m_range_prev = m_range_now;
    return(m_set_frequency*(m_c+anode_speed)/(m_c-(bnode_speed*cos(heading))));
  }
  else { //away
    m_range_prev = m_range_now;
    return(m_set_frequency*(m_c-anode_speed)/(m_c+(bnode_speed*cos(heading))));
  }
}

double HydrophoneSensor::getTrueNodeNodeRange(const string& node_a,
               const string& node_b)
{
  if((m_map_node_records.count(node_a) == 0) ||
     (m_map_node_records.count(node_b) == 0))
    return(-1);

  double anode_x       = m_map_node_records[node_a].getX();
  double anode_y       = m_map_node_records[node_a].getY();
  double bnode_x       = m_map_node_records[node_b].getX();
  double bnode_y       = m_map_node_records[node_b].getY();
  double range = abs(hypot((anode_x-bnode_x), (anode_y-bnode_y)));
  Notify("RANGE", range);
  return(range);
}

//------------------------------------------------------------
// Procedure: getTrueNodeHeading()

double HydrophoneSensor::getTrueNodeNodeHeading(const string& node_a, 
               const string& node_b)
{
  if((m_map_node_records.count(node_a) == 0)||m_map_node_records.count(node_b) == 0)
    return(-1);

  double heading1 = m_map_node_records[node_a].getHeading();
  double heading2 = m_map_node_records[node_b].getHeading();
  double finalheading = heading2-heading1;
  if(finalheading<0) finalheading+=360;
  if(finalheading>=360) finalheading-=360;

  return(finalheading);
}

//------------------------------------------------------------
// Procedure: getTrueNodeNodeBearing()

double HydrophoneSensor::getTrueNodeNodeBearing(const string& node_a,
               const string& node_b)
{
  if((m_map_node_records.count(node_a) == 0) ||
     (m_map_node_records.count(node_b) == 0))
    return(-1);

  double anode_x = m_map_node_records[node_a].getX();
  double anode_y = m_map_node_records[node_a].getY();
  double bnode_x = m_map_node_records[node_b].getX();
  double bnode_y = m_map_node_records[node_b].getY();
  double bearing = relAng(anode_x,anode_y,bnode_x,bnode_y);

  return(bearing);
}

bool HydrophoneSensor::allowableEchoType(string vehicle_type)
{
  string vtype = tolower(vehicle_type);

  // If no list is provided, then all types are allowed
  if(m_allow_echo_types.size() == 0)
    return(true);

  // If there is indeed a non-empty list of allowed types, check if
  // the given type is on the list.
  if(vectorContains(m_allow_echo_types, vtype))
    return(true);

  // Otherwise....
  return(false);
}