//util.h
#ifndef PCBROUTER_UTIL_H
#define PCBROUTER_UTIL_H

#include <sys/resource.h>
#include <sys/stat.h>  // Unix-only. Create Folder, change to <filesystem> when C++17 is ready
#include <sys/time.h>
#include <sys/types.h>  // Unix-only. Create Folder, change to <filesystem> when C++17 is ready
#include <cassert>      // assert
#include <cmath>        // fabs
#include <cstdlib>      // atof
#include <cstring>      // strerror
#include <fstream>      // ifstream
#include <iostream>     // cout, ostream
#include <sstream>      // stringstream
#include <string>
#include <vector>

using namespace std;

namespace util {
// =====================================================
// show system info. -----------------------------------
// =====================================================
inline void showSysInfoComdLine(int argc, char *argv[]) {
    cout << "Command line: ";
    for (int i = 0; i < argc; ++i) {
        cout << argv[i] << " ";
    }
    cout << endl;
    int systemret = 0;
    cout << "=================== SYSTEM INFORMATION ==================" << endl;
    systemret *= system("echo 'User:       '`whoami`@`hostname`");
    systemret *= system("echo 'Date:       '`date`");
    systemret *= system("echo 'System:     '`uname -a`");
    cout << "=========================================================" << endl;
    assert(!systemret);
}

// =====================================================
// filename --------------------------------------------
// =====================================================
inline string getFileDirName(const string filePathName) {
    string retStr = filePathName;
    string::size_type pos = retStr.find_last_of("/\\");
    if (pos != string::npos)
        retStr = retStr.substr(0, pos);
    return retStr;
}
inline string getFileName(const string filePathName) {
    string retStr = filePathName;
    string::size_type pos = retStr.find_last_of("/\\");
    if (pos != string::npos)
        retStr = retStr.substr(pos + 1);
    return retStr;
}
inline string getFileNameWoExtension(const string filePathName) {
    string retStr = filePathName;
    string::size_type pos = retStr.find_last_of("/\\");
    if (pos != string::npos)
        retStr = retStr.substr(pos + 1);
    pos = retStr.find_last_of(".");
    if (pos != string::npos)
        retStr = retStr.substr(0, pos);
    return retStr;
}
inline string getFileExtension(const string filePathName) {
    string retStr = filePathName;
    string::size_type pos = retStr.rfind(".");
    if (pos != string::npos)
        retStr = retStr.substr(pos + 1);
    return retStr;
}

// =====================================================
// Directory Unix-only (TODO: std::filesystem in C++17)-
// =====================================================
inline bool createDirectory(const string dirName) {
    // Creating a directory
    struct stat info;

    if (stat(dirName.c_str(), &info) != 0) {
        cerr << "Cannot access directory " << dirName << endl;
    } else if (info.st_mode & S_IFDIR) {
        cout << dirName << " dirctory already exists" << endl;
    } else {
        if (mkdir(dirName.c_str(), 0777) == -1) {
            cerr << "Error creating directory " << dirName << " :  " << strerror(errno) << endl;
            return false;
        }
        cout << "Directory created: " << dirName << endl;
    }
    return true;
}
inline string appendDirectory(const string dirName, const string fileName) {
    string retStr = dirName + "/" + fileName;
    return retStr;
}

// =====================================================
// CPU time & memory usage -----------------------------
// =====================================================
#define TIME_SCALE 1000000.0
#define MEMORY_SCALE 1024.0
class TimeUsage {
   public:
    TimeUsage() {
        start(FULL);
        start(PARTIAL);
    }
    struct TimeState {
        TimeState(long r = 0, long u = 0, long s = 0) : rTime(r), uTime(u), sTime(s) {}
        long rTime, uTime, sTime;  //real, user, system
    };

    enum TimeType {
        FULL,
        PARTIAL
    };
    void start(TimeType type) { (type == FULL) ? checkUsage(tStart_) : checkUsage(pStart_); }

    void showUsage(const string comment, TimeType type) {
        TimeState curSt;
        checkUsage(curSt);
        TimeState dur = (type == FULL) ? diff(tStart_, curSt) : diff(pStart_, curSt);
        if (type == FULL) {
            cout << "---------- " << comment << " total time usage -----------" << endl;
            cout << " Final Real:" << dur.rTime << "s;";
        } else {
            cout << "---------- " << comment << " period time usage -----------" << endl;
            cout << " Real:" << dur.rTime << "s;";
        }
        cout << " User:" << dur.uTime << "s;";
        cout << " System:" << dur.sTime << "s." << endl
             << endl;
    }

   private:
    TimeState diff(TimeState &start, TimeState &end) {
        return TimeState(end.rTime - start.rTime, end.uTime - start.uTime, end.sTime - start.sTime);
    }

    void checkUsage(TimeState &st) const {
        rusage tUsg;
        getrusage(RUSAGE_SELF, &tUsg);
        timeval tReal;
        gettimeofday(&tReal, NULL);
        st.uTime = tUsg.ru_utime.tv_sec + tUsg.ru_utime.tv_usec / TIME_SCALE;
        st.sTime = tUsg.ru_stime.tv_sec + tUsg.ru_stime.tv_usec / TIME_SCALE;
        st.rTime = tReal.tv_sec + tReal.tv_usec / TIME_SCALE;
    }
    TimeState tStart_, pStart_;  //total, period
};
// memory
inline double getPeakMemoryUsage() {
#ifdef __linux__
    char buf[1000];
    ifstream ifs("/proc/self/stat");
    for (int i = 0; i != 23; ++i)
        ifs >> buf;
    return (1.0 / (MEMORY_SCALE * MEMORY_SCALE) * atof(buf));  // GB
#else
    return -1;
#endif
}

// =====================================================
// math-related auxiliary ------------------------------
// =====================================================
inline bool is_equal(double a, double b) {
    return (a - b) < 0.0001;
}
inline double interpolate(const double &a, const double &b, const double &ratio) {
    return a + ((b - a) * ratio);
}

}  // namespace util

#endif  // UTIL_H
