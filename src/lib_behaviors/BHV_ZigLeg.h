/************************************************************/
/*    NAME: Kimberly Jung                                              */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.h                                      */
/*    DATE:                                                 */
/************************************************************/

#ifndef ZigLeg_HEADER
#define ZigLeg_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions
  IvPFunction* buildIvPFxnZAICRight();
  IvPFunction* buildIvPFxnZAICLeft();
  double       calcTowardAngle(double c, double f_0, double f_m, double v_l, double v_f);
  double       calcAwayAngle(double c, double f_0, double f_m, double v_l, double v_f);
  int          averageFrequency();
  
protected: // Configuration parameters

protected: // State variables

  bool m_debug;

  double m_priority_wt;
  double m_curr_time;
  double m_zig_offset; //default is 45 degrees for 2nd doppler measurement
  double m_first_heading; //original heading before the 2nd doppler measurement
  double m_freq0, m_freq1;   //known set frequency of leader
  double m_c;    //sound speed in water in m/s
  double m_v_f;  //follower speed in m/s
  double m_v_l;  //leader speed in m/s 
  double m_toward_angle_1, m_away_angle_1;
  double m_toward_angle_2, m_away_angle_2;
  double m_post_time_1, m_post_time_2;
  double m_acceptable_angle_difference;
  unsigned int m_freq_counter;
  std::vector<int> m_vec_freqs;
  unsigned int m_sample_size;  
  bool m_bool_right;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ZigLeg(domain);}
}
#endif
