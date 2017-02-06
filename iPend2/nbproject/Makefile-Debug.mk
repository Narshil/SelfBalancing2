#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/PID_v1.o \
	${OBJECTDIR}/pend1.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-Wl,-rpath,'/usr/local/include' -Wl,-rpath,'/usr/local/include/opencv2' -Wl,-rpath,'../../Haz/MotionSensorExample' `pkg-config --libs opencv` ../../Haz/MotionSensorExample/MotionSensor/libMotionSensor.a ../../Haz/MotionSensorExample/libs/libI2Cdev.a  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ipend2

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ipend2: ../../Haz/MotionSensorExample/MotionSensor/libMotionSensor.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ipend2: ../../Haz/MotionSensorExample/libs/libI2Cdev.a

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ipend2: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/ipend2 ${OBJECTFILES} ${LDLIBSOPTIONS} -lpigpio -pthread

${OBJECTDIR}/PID_v1.o: PID_v1.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -I/usr/local/include/opencv2 -I../../Haz/MotionSensorExample -I../../Haz/MotionSensorExample/MotionSensor `pkg-config --cflags opencv` -std=c++14  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PID_v1.o PID_v1.cpp

${OBJECTDIR}/pend1.o: pend1.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -I/usr/local/include -I/usr/local/include/opencv2 -I../../Haz/MotionSensorExample -I../../Haz/MotionSensorExample/MotionSensor `pkg-config --cflags opencv` -std=c++14  -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/pend1.o pend1.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
