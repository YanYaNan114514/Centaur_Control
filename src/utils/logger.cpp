#include "logger.h"
#include <iostream>
#include <ctime>
#include <iomanip>
#include <sstream>

bool Logger::initialized_ = false;

void Logger::initialize()
{
    initialized_ = true;
}

void Logger::info(const std::string& message)
{
    if (!initialized_) return;
    
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::cout << "[" << oss.str() << "] [INFO] " << message << std::endl;
}

void Logger::warning(const std::string& message)
{
    if (!initialized_) return;
    
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::cout << "[" << oss.str() << "] [WARN] " << message << std::endl;
}

void Logger::error(const std::string& message)
{
    if (!initialized_) return;
    
    auto now = std::time(nullptr);
    auto tm = *std::localtime(&now);
    
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    
    std::cerr << "[" << oss.str() << "] [ERROR] " << message << std::endl;
}