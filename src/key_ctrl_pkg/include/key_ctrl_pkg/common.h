#ifndef COMMON_H
#define COMMON_H
#include <math.h>
#include <string>
#include <vector>
using namespace std;
 
namespace KEY_CTRL {
enum class KB {CLOSE=1 , START , STOP, UP, DOWN, LEFT, RIGHT };
enum class KEY_CLASS { START = 1, DIR, CLOSE};
 
enum class GEER { N = 0, D, R, P };
}  // namespace KEY_CTRL
 
#endif  // COMMON_H