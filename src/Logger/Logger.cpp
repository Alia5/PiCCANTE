#include "Logger.hpp"

Log::Level Log::current_level = Log::LEVEL_INFO;
std::map<Log::Level, std::string> Log::level_names = {
    {Log::LEVEL_DEBUG, "DEBUG"},
    {Log::LEVEL_INFO, "INFO"},
    {Log::LEVEL_WARNING, "WARNING"},
    {Log::LEVEL_ERROR, "ERROR"},
};
void Log::set_log_level(Level level) { Log::current_level = level; }
