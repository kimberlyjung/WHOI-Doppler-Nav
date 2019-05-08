/************************************************************/
/*    NAME: Kimberly Jung                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                    */
/*    DATE: 22 April 2019                                                */
/************************************************************/

//Note: must add in average many frequencies

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_ZigLeg.h"
#include "ZAIC_PEAK.h"
#include <math.h>

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "default_zigleg");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Initializing variables
  m_priority_wt = 201;
  m_curr_time = 0;
 	m_zig_offset = 45;
  m_first_heading = 0;
	m_freq0 = 25000;   //known set frequency of leader
	m_freq1 = 0;   //measured frequency by hydrophone in Hz
	m_c = 1500;    //sound speed in water in m/s
	m_v_f = 1;  //follower speed in m/s
	m_v_l = 1;  //leader speed in m/s
  m_toward_angle_1 = 0;
  m_away_angle_1 = 0;
	m_toward_angle_2 = 0;
  m_away_angle_2 = 0;
  m_post_time_1 = 5;
  m_post_time_2 = 5;
  m_freq_counter = 0;
  m_sample_size = 10;
  m_bool_right = true;
  m_acceptable_angle_difference = 5;

  // Add any variables this behavior needs to subscribe for
  //addInfoVars("NAV_X, NAV_Y");
  addInfoVars("NAV_HEADING", "no_warning");
  addInfoVars("FREQUENCY_ANSWER", "no_warning");
  //addInfoVars("CYCLE_COMPLETE",	 "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "zig_offset") && isNumber(val)) {
    m_zig_offset = double_val;
    return(true);
  }
  else if ((param == "pwt") && isNumber(val)){
    m_priority_wt= double_val;
    return(true);
  }
  else if ((param == "sample_size") && isNumber(val)){
  	m_sample_size = double_val;
  	return(true);
  }
  else if ((param == "acceptable_angle_difference") && isNumber(val)) {
  	m_acceptable_angle_difference = double_val;
  	return(true);
  }

  // If not handled above, then just return false;
  return(false);
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ZigLeg::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_ZigLeg::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  bool ok = true;
  //if(ok) m_ox = getBufferDoubleVal("NAV_X", ok);
  //if(!ok) postEMessage("NAV_X not found.");
  //if(ok) m_oy = getBufferDoubleVal("NAV_Y", ok);
  //if(!ok) postEMessage("NAV_Y not found.");

  postMessage("FREQUENCY_REQUEST", m_us_name);
  if(ok) m_curr_time = getBufferCurrTime();
  if(!ok) postEMessage("curr buffer time not found");
  if(ok) m_vec_freqs.push_back(getBufferDoubleVal("FREQUENCY_ANSWER", ok));
  if(!ok) postEMessage("FREQUENCY not measured");

  if(m_vec_freqs.size()>m_sample_size) {
  	  m_freq1 = averageFrequency();
	  if(m_curr_time >= m_post_time_1) { //time to take 1st doppler measurement
	  	m_first_heading = getBufferDoubleVal("NAV_HEADING", ok);
	  	m_post_time_1 = m_curr_time + 10; //next time to take 1st doppler measurement is 10 seconds later
	  	m_post_time_2 = m_curr_time + 5; //time for 2nd doppler measurement
        //doppler_shift = m_freq0 - m_freq1;
		m_toward_angle_1 = calcTowardAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
		m_away_angle_1 = calcAwayAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
		if((m_toward_angle_1 < m_acceptable_angle_difference) || (m_away_angle_1 < m_acceptable_angle_difference)) {
			return ipf;
		}
	  }
	  else if(m_curr_time <= m_post_time_2) { // Actuate temp heading to take 2nd Doppler measurement 
	  	if(m_bool_right) ipf = buildIvPFxnZAICRight(); //actuate right: CW
	  	else ipf = buildIvPFxnZAICLeft(); // actuate left: CCW
	  }
	  else if(m_curr_time >= m_post_time_2) {//time to take second doppler measurement and calc new heading
	  	m_toward_angle_2 = calcTowardAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
	  	m_away_angle_2 = calcAwayAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
	  	
	  	//calc new heading from logic
	  	//--if actuation was right-CW  --if actuation was left-CCW
	  	if((m_freq0 - m_freq1) > 0) {// positive dopppler shift: leader is moving away from follower 
	  		if(m_bool_right) { 
	  			if(m_away_angle_1 > m_away_angle_2) m_first_heading = m_first_heading - m_away_angle_1;
	  			else {
	  				m_first_heading = m_first_heading + m_away_angle_1;
	  				m_bool_right = false;
	  			}
	  		}
	  		else
	  		{
	  			if(m_away_angle_1 < m_away_angle_2) {
	  				m_first_heading = m_first_heading - m_away_angle_1;
	  				m_bool_right = true;
	  			}
	  			else m_first_heading = m_first_heading + m_away_angle_1;
	  		}
	  	}
	  	else { // negative doppler shift: leader is moving towards follower
	  		if(m_bool_right) {
	  			if(m_toward_angle_1 > m_toward_angle_2) m_first_heading = m_first_heading + 180 - m_toward_angle_1;
	  			else {
	  				m_first_heading = m_first_heading + 180 + m_toward_angle_1;
	  				m_bool_right = false;
	  			}
	  		}
	  		else {
	  			if(m_toward_angle_1 < m_toward_angle_2) {
	  				m_first_heading = m_first_heading + 180 - m_toward_angle_1;
	  				m_bool_right = true;
	  			}
	  			else m_first_heading = m_first_heading + 180 + m_toward_angle_1;
	  		}
	  		
	  	}
	   	postMessage("NAV_HEADING_NEW", m_first_heading);
	  }
	}
  /*m_end_time = m_post_time + m_zig_time;
  m_bool_new_heading = false;

  if(m_post_time <= m_curr_time) {
	m_first_heading = getBufferDoubleVal("NAV_HEADING", ok);
	m_bool_new_heading=true;
    }

  if(m_end_time <= m_curr_time) m_bool_new_heading=false;
  if((m_curr_time >= m_post_time) && (m_curr_time <= m_end_time)) {
  	ipf = buildIvPFxnZAIC();
  }
  */

  // Part N: Prior to returning the IvP function, apply the priority wt
  // Actual weight applied may be some value different than the configured
  // m_priority_wt, depending on the behavior author's insite.
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

IvPFunction *BHV_ZigLeg::buildIvPFxnZAICRight()
{
  // Step 1 - Create the IvPDomain, the function's domain
  //IvPDomain domain;
  //domain.addDomain("depth", 0, 600, 601);
     
  // Step 2 - Create the ZAIC_PEAK with the domain and variable name
  ZAIC_PEAK  zaic_peak(m_domain, "course");
   
  // Step 3 - Configure the ZAIC_LEQ parameters
  zaic_peak.setSummit(m_first_heading + m_zig_offset);
  zaic_peak.setPeakWidth(0);
  zaic_peak.setBaseWidth(180);
  zaic_peak.setSummitDelta(0);
  zaic_peak.setValueWrap(true);
  if(!zaic_peak.stateOK()) {
  	string warnings = "ZAIC not working: " + zaic_peak.getWarnings();
  	postWMessage(warnings);
    return(0);
  }
  
  // Step 4 - Extract the IvP function
  IvPFunction *ivp_function = zaic_peak.extractIvPFunction();
  return(ivp_function);    
}

IvPFunction *BHV_ZigLeg::buildIvPFxnZAICLeft()
{
  // Step 1 - Create the IvPDomain, the function's domain
  //IvPDomain domain;
  //domain.addDomain("depth", 0, 600, 601);
     
  // Step 2 - Create the ZAIC_PEAK with the domain and variable name
  ZAIC_PEAK  zaic_peak(m_domain, "course");
   
  // Step 3 - Configure the ZAIC_LEQ parameters
  zaic_peak.setSummit(m_first_heading - m_zig_offset);
  zaic_peak.setPeakWidth(0);
  zaic_peak.setBaseWidth(180);
  zaic_peak.setSummitDelta(0);
  zaic_peak.setValueWrap(true);
  if(!zaic_peak.stateOK()) {
  	string warnings = "ZAIC not working: " + zaic_peak.getWarnings();
  	postWMessage(warnings);
    return(0);
  }
  
  // Step 4 - Extract the IvP function
  IvPFunction *ivp_function = zaic_peak.extractIvPFunction();
  return(ivp_function);    
}


//---------------------------------------------------------
//Procedure: Compute Towards Angle from Doppler Shift

double BHV_ZigLeg::calcTowardAngle(double c, double f_0, double f_m, double v_l, double v_f)
{
  /*   c = sound speed
     f_0 = original frequency w
     f_m = measured frequency w'
     v_l = leader's speed
     v_f = follower's speed */

  return(acos((c-((f_0/f_m)*(c+v_f)))/v_l));
}

double BHV_ZigLeg::calcAwayAngle(double c, double f_0, double f_m, double v_l, double v_f)
{
  /*   c = sound speed
     f_0 = original frequency w
     f_m = measured frequency w'
     v_l = leader's speed
     v_f = follower's speed */

  return(acos((-c+((f_0/f_m)*(c-v_f)))/v_l));
}

int BHV_ZigLeg::averageFrequency() // return average frequency over set amount of data points based on size of vector
{
	int size = m_vec_freqs.size();
	int sum = 0;
	for(unsigned int k=0; k < size; k++) {
		sum += m_vec_freqs[k];
	}
	m_vec_freqs.clear();     // clears the current list of frequencies
	return(sum/size);
}
