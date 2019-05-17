/************************************************************/
/*    NAME: Kimberly Jung                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigFollow.cpp                                    */
/*    DATE: 22 April 2019                                                */
/************************************************************/

//Note: must add in average many frequencies

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_ZigFollow.h"
#include "ZAIC_PEAK.h"
#include <math.h>

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigFollow::BHV_ZigFollow(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "default_ZigFollow");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Initializing variables
  m_priority_wt = 201;
  m_curr_time = 0;
 	m_zig_offset = 45;
  m_first_heading = 90;
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
  m_sample_size = 1;
  m_bool_right = true;
  m_aad = 2;

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_HEADING", "no_warning");
  addInfoVars("FREQUENCY_ANSWER", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigFollow::setParam(string param, string val)
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
  	m_aad = double_val;
  	return(true);
  }
  else if ((param == "desired_speed") && isNumber(val)) {
    m_v_f = double_val;
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

void BHV_ZigFollow::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigFollow::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigFollow::onIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigFollow::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigFollow::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigFollow::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigFollow::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_ZigFollow::onRunState()
{
  // Part 1: Build the IvP function
  IvPFunction *ipf = 0;
  bool ok = true;

  postMessage("FREQUENCY_REQUEST", m_us_name);
  if(ok) m_curr_time = getBufferCurrTime();
  if(!ok) postEMessage("curr buffer time not found");
  string myFrequencyString = getBufferStringVal("FREQUENCY_ANSWER", ok);
  double myFrequency = atof(myFrequencyString.c_str());
  postMessage("FREQUENCY_ANSWER_TEST", myFrequency);
  if(!ok) postEMessage("FREQUENCY not measured");
  if(ok) m_vec_freqs.push_back(myFrequency);
  
  if(m_vec_freqs.size()>m_sample_size) {
  	m_freq1 = averageFrequency();
    postMessage("AVERAGE_FREQUENCY", m_freq1);
    m_vec_freqs.clear();
	  if(m_curr_time >= m_post_time_1) { //time to take 1st doppler measurement
      m_first_heading = getBufferDoubleVal("NAV_HEADING", ok);
	  	m_post_time_1 = m_curr_time + 10; //next time to take 1st doppler measurement is 10 seconds later
	  	m_post_time_2 = m_curr_time + 5; //time for 2nd doppler measurement
        //doppler_shift = m_freq0 - m_freq1;
		  m_toward_angle_1 = calcTowardAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
      m_toward_angle_1 = m_toward_angle_1*180/3.14159;
		  m_away_angle_1 = calcAwayAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
      m_away_angle_1 = m_away_angle_1*180/3.14159;
		  postMessage("m_toward_angle_1", m_toward_angle_1);
      postMessage("m_away_angle_1", m_away_angle_1);
      postMessage("TEST_VAR", 1);
      if((m_toward_angle_1 <= m_aad) || (m_away_angle_1 <= m_aad)) {
        return buildIvPFxnZAIC(0);
      }
	  }
	  else if(m_curr_time <= m_post_time_2) { // Actuate temp heading to take 2nd Doppler measurement 
	  	if(m_bool_right) ipf = buildIvPFxnZAIC(m_zig_offset); //actuate right: CW
	  	else ipf = buildIvPFxnZAIC(-m_zig_offset); // actuate left: CCW
      postMessage("TEST_VAR", 2);
	  }
	  else if(m_curr_time > m_post_time_2) {//time to take second doppler measurement and calc new heading
	  	m_toward_angle_2 = calcTowardAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
      m_toward_angle_2 = m_toward_angle_2 *180/3.14159;
	  	m_away_angle_2 = calcAwayAngle(m_c, m_freq0, m_freq1, m_v_l, m_v_f);
      m_away_angle_2 = m_away_angle_2 * 180/3.14159;
      postMessage("m_toward_angle_2", m_toward_angle_2);
      postMessage("m_away_angle_2", m_away_angle_2);
      postMessage("TEST_VAR", 3);
	  	
	  	//calc new heading from logic
	  	//--if actuation was right-CW  --if actuation was left-CCW
	  	if(m_freq0 > m_freq1) {// positive dopppler shift: leader is moving away from follower 
	  		if(m_bool_right) { 
	  			if(m_away_angle_1 > m_away_angle_2) {
            m_first_heading = m_first_heading - m_away_angle_1;
            postMessage("TEST_VAR", 4);
          }
	  			else {
	  				m_first_heading = m_first_heading + m_away_angle_1;
	  				m_bool_right = false;
            postMessage("TEST_VAR", 5);
	  			}
	  		}
	  		else
	  		{
	  			if(m_away_angle_1 < m_away_angle_2) {
	  				m_first_heading = m_first_heading - m_away_angle_1;
	  				m_bool_right = true;
            postMessage("Calculated_heading_1", m_first_heading);
            postMessage("TEST_VAR", 6);
	  			}
	  			else {
            m_first_heading = m_first_heading + m_away_angle_1;
            postMessage("TEST_VAR", 7);
          }
	  		}
	  	}
	  	else { // negative doppler shift: leader is moving towards follower
	  		if(m_bool_right) {
	  			if(m_toward_angle_1 > m_toward_angle_2) {
            m_first_heading = m_first_heading + 180 - m_toward_angle_1;
            postMessage("TEST_VAR", 8);
          }
	  			else {
	  				m_first_heading = m_first_heading + 180 + m_toward_angle_1;
	  				m_bool_right = false;
            postMessage("Calculated_heading_2", m_first_heading);
            postMessage("TEST_VAR", 9);
	  			}
	  		}
	  		else {
	  			if(m_toward_angle_1 < m_toward_angle_2) {
	  				m_first_heading = m_first_heading + 180 - m_toward_angle_1;
	  				m_bool_right = true;
            postMessage("TEST_VAR", 10);
	  			}
	  			else {
            m_first_heading = m_first_heading + 180 + m_toward_angle_1;
            postMessage("TEST_VAR", 11);
          }
	  		}
	  		
	  	}
	  }
    if(m_first_heading>=360) m_first_heading-=360;
    if(m_first_heading<0) m_first_heading+=360;
    postMessage("NAV_HEADING_NEW", m_first_heading);
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
  else ipf = buildIvPFxnZAIC(0);
  return(ipf);
}

IvPFunction *BHV_ZigFollow::buildIvPFxnZAIC(double f_offset)
{
  IvPFunction *ipf = 0;

  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_v_f);
  spd_zaic.setBaseWidth(0.3);
  spd_zaic.setPeakWidth(0.0);
  spd_zaic.setSummitDelta(0.0);
  spd_zaic.setValueWrap(true);
  IvPFunction *spd_of = spd_zaic.extractIvPFunction();
    
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(m_first_heading + f_offset);
  crs_zaic.setBaseWidth(180);
  crs_zaic.setPeakWidth(10);
  crs_zaic.setSummitDelta(25);
  crs_zaic.setValueWrap(true);
  IvPFunction *crs_of = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;

  ipf = coupler.couple(crs_of, spd_of, 50, 50); // 50 used to be m_patience, see BHV_Loiter

  if(ipf)
    ipf->setPWT(m_priority_wt);
  else 
    postEMessage("Unable to generate constant-heading IvP function");
 
  return(ipf);    
}


//---------------------------------------------------------
//Procedure: Compute Towards Angle from Doppler Shift

double BHV_ZigFollow::calcTowardAngle(double c, double f_0, double f_m, double v_l, double v_f)
{
  /*   c = sound speed
     f_0 = original frequency w
     f_m = measured frequency w'
     v_l = leader's speed
     v_f = follower's speed */

  return(acos((c-((f_0/f_m)*(c+v_f)))/v_l));
}

double BHV_ZigFollow::calcAwayAngle(double c, double f_0, double f_m, double v_l, double v_f)
{
  /*   c = sound speed
     f_0 = original frequency w
     f_m = measured frequency w'
     v_l = leader's speed
     v_f = follower's speed */

  return(acos((-c+((f_0/f_m)*(c-v_f)))/v_l));
}

int BHV_ZigFollow::averageFrequency() // return average frequency over set amount of data points based on size of vector
{
	double size = m_vec_freqs.size();
  if(size==0) return 0;
	double sum = 0;
	for(unsigned int k=0; k < size; k++) {
		sum += m_vec_freqs[k];
	}
	return(sum/size);
}
