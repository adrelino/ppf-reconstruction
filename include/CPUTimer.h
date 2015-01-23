// ###
// ###
// ### Practical Course: GPU Programming in Computer Vision
// ### Final Project: Variational Depth from Focus
// ###
// ### Technical University Munich, Computer Vision Group
// ### Summer Semester 2014, September 8 - October 10
// ###
// ###
// ### Maria Klodt, Jan Stuehmer, Mohamed Souiai, Thomas Moellenhoff
// ###
// ###

// ### Dennis Mack, dennis.mack@tum.de, p060
// ### Adrian Haarbach, haarbach@in.tum.de, p077
// ### Markus Schlaffer, markus.schlaffer@in.tum.de, p070

#ifndef CPU_TIMER_H
#define CPU_TIMER_H

#include <iostream>
#include <string>
#include <map>
#include <time.h>

#ifndef CLOCK_REALTIME
  #define CLOCK_REALTIME CLOCK_MONOTONIC
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef _WIN32
#include <wintime.h>
#else
#include <sys/time.h>
#endif

class CPUTimer {
 private:
  timespec startTime;
  timespec endTime;

  bool startWasSet;
  bool stopWasSet;

  void getTime(struct timespec *ts);

  std::map<std::string,float> timingsMap;

 public:
  CPUTimer();
  ~CPUTimer();

  void tic();
  timespec toc();
  void toc(std::string name);

  void printAllTimings();
};
#endif
