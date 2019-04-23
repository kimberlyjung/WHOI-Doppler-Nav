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
IvPFunction* buildIvPFxnZAIC();

protected: // Configuration parameters

protected: // State variables

  bool m_debug;

  double m_priority_wt;
  double m_curr_time;
  double m_wpt_index_old;
  double m_wpt_index_new;
  double m_ox;
  double m_oy;
  double m_post_time;
  double m_true_heading;
  double m_zig_offset;
  double m_zig_time;
  double m_end_time;
  double m_cycle_complete;
  bool m_bool_true_heading;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ZigLeg(domain);}
}
#endif