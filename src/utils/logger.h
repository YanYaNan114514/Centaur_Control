#ifndef LOGGER_H
#define LOGGER_H

#include <string>

class Logger
{
public:
    static void initialize();
    static void info(const std::string& message);
    static void warning(const std::string& message);
    static void error(const std::string& message);
    
private:
    static bool initialized_;
};

#endif // LOGGER_H