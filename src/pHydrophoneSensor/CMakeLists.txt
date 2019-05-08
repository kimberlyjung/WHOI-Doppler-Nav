#--------------------------------------------------------
# The CMakeLists.txt for:                       pHydrophoneSensor
# Author(s):                              Kimberly Jung
#--------------------------------------------------------

SET(SRC
  HydrophoneSensor.cpp
  HydrophoneSensor_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pHydrophoneSensor ${SRC})

TARGET_LINK_LIBRARIES(pHydrophoneSensor
   ${MOOS_LIBRARIES}
   geometry
   apputil
   contacts
   mbutil
   m
   pthread)

