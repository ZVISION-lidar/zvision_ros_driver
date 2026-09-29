#ifndef UTILITIES_TIC_TOC_HPP_
#define UTILITIES_TIC_TOC_HPP_

#include <chrono>
#include <cstdlib>
#include <ctime>
#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED "\033[31m"     /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE "\033[34m"    /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */
class TicToc {
public:
  TicToc() { tic(); }

  void tic() { start = std::chrono::system_clock::now(); }

  double toc() {
    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    start = std::chrono::system_clock::now();
    return elapsed_seconds.count() * 1000;
  }

private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};
#endif