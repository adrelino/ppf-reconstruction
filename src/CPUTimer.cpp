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

#include <cassert>
#include <CPUTimer.h>

#include <stdio.h>
#include <iomanip> 

using namespace std;

CPUTimer::CPUTimer() : startWasSet(false), stopWasSet(true)  {
}

CPUTimer::~CPUTimer() {
}

void CPUTimer::getTime(struct timespec *ts){
#ifdef __MACH__ 
  //https://gist.github.com/jbenet/1087739
  // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
#else
#ifdef _WIN32
  clock_gettime(ts);
#else
  clock_gettime(CLOCK_REALTIME, ts);
#endif
#endif

}

void CPUTimer::tic() {
  assert(stopWasSet && !startWasSet);

  getTime(&startTime);
  startWasSet = true;
  stopWasSet = false;
}

timespec CPUTimer::toc() {
  assert(startWasSet && !stopWasSet);
  getTime(&endTime);

  timespec temp;

  if ((endTime.tv_nsec - startTime.tv_nsec)<0) {
    temp.tv_sec = endTime.tv_sec - startTime.tv_sec-1;
    temp.tv_nsec = 1000000000 + endTime.tv_nsec - startTime.tv_nsec;
  } else {
    temp.tv_sec = endTime.tv_sec - startTime.tv_sec;
    temp.tv_nsec = endTime.tv_nsec - startTime.tv_nsec;
  }

  stopWasSet = true;
  startWasSet = false;

  return temp;
}

void CPUTimer::toc(std::string name){
  timespec elapsedTimeCPUTimer = this->toc();
  cout<<endl;
  cout<<"=====  TIMING["<<name<<"] is ";
  cout<< elapsedTimeCPUTimer.tv_sec << "." << elapsedTimeCPUTimer.tv_nsec << " s" << endl;          
  cout << endl;

  float seconds=elapsedTimeCPUTimer.tv_sec + elapsedTimeCPUTimer.tv_nsec / 1e9f;

  timingsMap[name]=seconds;
}

void CPUTimer::printAllTimings(){
  typedef std::map<std::string,float>::iterator it_type;

  for(it_type it = timingsMap.begin(); it != timingsMap.end(); ++it) {
    cout<< left << setw(20) << it->first<<":";
    printf("%0.3f\n",it->second);
  }
}

