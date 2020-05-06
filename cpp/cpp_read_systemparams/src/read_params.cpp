/**
 *  @file read_params.cpp
 *  @author Maximilian Harr <maximilian.harr@daimler.com>
 *  @date 20.11.2017
 *
 *  @brief	Read system parameters > Necessary to benchmark programms for their performance
 * 		    https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
 *          Better use python script print_system_usage.py
 *
 *
 *          Coding Standard:
 *          wiki.ros.org/CppStyleGuide
 *          https://google.github.io/styleguide/cppguide.html
 *
 *
 *  @bug
 *
 *  @todo
 *
 */

#include <iostream>
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "sys/sysinfo.h"
#include "sys/types.h"
#include "sys/times.h"
#include "sys/vtimes.h"
#include <unistd.h>

/** @brief 
 *  @param
 *  @return
 */ 
int parseLine(char* line);

/** @brief 
 *  @param
 *  @return
 */ 
int getVirtualMemCurrProc();

/** @brief 
 *  @param
 *  @return
 */ 
int getPhysMemCurrProc();

/** @brief 
 *  @param
 *  @return
 */ 
/* Read /proc/stat file for the first time 
 * http://www.linuxhowtos.org/System/procstat.htm
 * The meanings of the columns are as follows, from left to right:
 *   user: normal processes executing in user mode
 *   nice: niced processes executing in user mode
 *   system: processes executing in kernel mode
 *   idle: twiddling thumbs
 *   iowait: waiting for I/O to complete
 *   irq: servicing interrupts
 *   softirq: servicing softirqs 
 */
double getCPUCurrUsed();

static clock_t lastCPU, lastSysCPU, lastUserCPU;
static int numProcessors;

/** @brief 
 *  @param
 *  @return
 */ 
void initCPU2();

/** @brief 
 *  @param
 *  @return
 */ 
double getCPUCurrProc();

int main(int argc, char* argv[]){
	struct sysinfo memInfo;

/* Virtual memory Usage */
	sysinfo (&memInfo);
	long long totalVirtualMem = memInfo.totalram;
	//Add other values in next statement to avoid int overflow on right hand side...
	totalVirtualMem += memInfo.totalswap;
	totalVirtualMem *= memInfo.mem_unit;
	long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
	//Add other values in next statement to avoid int overflow on right hand side...
	virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
	virtualMemUsed *= memInfo.mem_unit;
	
/* Physical memory Usage */
	long long totalPhysMem = memInfo.totalram;
	//Multiply in next statement to avoid int overflow on right hand side...
	totalPhysMem *= memInfo.mem_unit;
	long long physMemUsed = memInfo.totalram - memInfo.freeram;
	//Multiply in next statement to avoid int overflow on right hand side...
	physMemUsed *= memInfo.mem_unit;

/* CPU Usage */
	initCPU2();

/* Print info */
	std::cout << std::endl;
	std::cout << "###### VIRTUAL MEMORY USAGE #####################" << std::endl;
	std::cout << "totalVirtualMem:       " << totalVirtualMem/1e9 << " GB"  << std::endl;
	std::cout << "virtualMemUsed:        " << totalVirtualMem/1e9 << " GB"  << std::endl;
	std::cout << "getVirtualMemCurrProc: " << totalVirtualMem/1e9 << " GB"  << std::endl;
	std::cout << "###### PHYSICAL MEMORY USAGE ####################" << std::endl;
	std::cout << "totalPhysMem:          " << totalPhysMem/1e9 << " GB"  << std::endl;
	std::cout << "physMemUsed:           " << physMemUsed/1e9 << " GB"  << std::endl;
	std::cout << "getPhysMemCurrProc:    " << getPhysMemCurrProc()/1e6 << " GB"  << std::endl;

	std::cout << "getPhysMemCurrProc:    " << getPhysMemCurrProc()/1e6 << " GB"  << std::endl;
	std::cout << "###### CPU USAGE ################################" << std::endl;
	std::cout << "getPhysMemCurrProc:    " << (double) getCPUCurrUsed() << " %"  << std::endl;
	std::cout << "getCPUCurrProc:        " << getCPUCurrProc() << " %"  << std::endl;
	std::cout << std::endl;
	
	
	return 0;
}


int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int getVirtualMemCurrProc(){ //Note: this value is in KB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmSize:", 7) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

int getPhysMemCurrProc(){ //Note: this value is in KB!
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

/* Read /proc/stat file for the first time 
 * http://www.linuxhowtos.org/System/procstat.htm
 * The meanings of the columns are as follows, from left to right:
 *   user: normal processes executing in user mode
 *   nice: niced processes executing in user mode
 *   system: processes executing in kernel mode
 *   idle: twiddling thumbs
 *   iowait: waiting for I/O to complete
 *   irq: servicing interrupts
 *   softirq: servicing softirqs 
 */
double getCPUCurrUsed(){
  FILE * file;
  /* Open /proc/stat for the first time and read parameters */
  unsigned long long firstVal01, firstVal02, firstVal03, firstVal04, firstVal05, firstVal06, firstVal07;
  file = fopen("/proc/stat", "r");
  fscanf(file, "cpu %llu %llu %llu %llu %llu %llu %llu", &firstVal01, &firstVal02, &firstVal03, &firstVal04, &firstVal05, &firstVal06, &firstVal07);
  fclose(file);

  /* Open /proc/stat for the second time and read parameters */
  unsigned int microseconds = 1e5;
  usleep(microseconds);
  unsigned long long secondVal01, secondVal02, secondVal03, secondVal04, secondVal05, secondVal06, secondVal07;
  file = fopen("/proc/stat", "r");
  fscanf(file, "cpu %llu %llu %llu %llu %llu %llu %llu", &secondVal01, &secondVal02, &secondVal03, &secondVal04, &secondVal05, &secondVal06, &secondVal07);
  fclose(file);

	/* Compute difference
   * https://stackoverflow.com/questions/3017162/how-to-get-total-cpu-usage-in-linux-c */
  double percent;
  unsigned long long count_total, count_partial;

  if (secondVal01 < firstVal01 || secondVal02 < firstVal02 || secondVal03 < firstVal03 || secondVal04 < firstVal04 
    || secondVal05 < firstVal05 || secondVal06 < firstVal06 || secondVal07 < firstVal07 ){
    //Overflow detection. Just skip this value.
    std::cout << "Overflow detection" << std::endl;
    percent = -1.0;
  }
  else{
	count_partial = (secondVal01 - firstVal01) + (secondVal02 - firstVal02) + (secondVal03 - firstVal03);
    count_total = (secondVal01 - firstVal01) + (secondVal02 - firstVal02) + (secondVal03 - firstVal03) + 
      (secondVal04 - firstVal04) + (secondVal05 - firstVal05) + (secondVal06 - firstVal06) + (secondVal07 - firstVal07);
    std::cout << "count_partial | count_total: " << count_partial << " | " << count_total << std::endl;
    percent = (double) count_partial/count_total;
    percent *= 100;
  }

  return percent;
}

void initCPU2(){
    FILE* file;
    struct tms timeSample;
    char line[128];

    lastCPU = times(&timeSample);
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 0;
    while(fgets(line, 128, file) != NULL){
        if (strncmp(line, "processor", 9) == 0) numProcessors++;
    }
    fclose(file);
}

double getCPUCurrProc(){
    struct tms timeSample;
    clock_t now;
    double percent;

    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
        timeSample.tms_utime < lastUserCPU){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        percent = (timeSample.tms_stime - lastSysCPU) +
            (timeSample.tms_utime - lastUserCPU);
        percent /= (now - lastCPU);
        percent /= numProcessors;
        percent *= 100;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    return percent;
}