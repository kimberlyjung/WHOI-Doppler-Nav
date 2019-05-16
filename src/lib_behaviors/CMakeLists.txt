#--------------------------------------------------------
# The CMakeLists.txt for:              lib_behaviors-test
# Author(s):                                
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  # Windows Libraries
  SET(SYSTEM_LIBS
      )
else (${WIN32})
  # Linux and Apple Libraries
  SET(SYSTEM_LIBS
      m )
endif (${WIN32})


MACRO(ADD_BHV BHV_NAME)
  ADD_LIBRARY(${BHV_NAME} SHARED ${BHV_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${BHV_NAME} 
    geometry
    helmivp
    behaviors 
    ivpbuild 
    logic 
    ivpcore 
    bhvutil 
    mbutil 
    ${SYSTEM_LIBS} )
ENDMACRO(ADD_BHV)

#--------------------------------------------------------
#                                      BHV_ZigFollow
#--------------------------------------------------------

ADD_LIBRARY(BHV_ZigFollow SHARED 
   BHV_ZigFollow.cpp)
TARGET_LINK_LIBRARIES(BHV_ZigFollow
   geometry
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   ${SYSTEM_LIBS} )
#--------------------------------------------------------
#                                      BHV_ConstantKimHeading
#--------------------------------------------------------

ADD_LIBRARY(BHV_ConstantKimHeading SHARED 
   BHV_ConstantKimHeading.cpp)
TARGET_LINK_LIBRARIES(BHV_ConstantKimHeading
   geometry
   helmivp
   behaviors 
   ivpbuild 
   logic 
   ivpcore 
   bhvutil 
   mbutil 
   ${SYSTEM_LIBS} )


