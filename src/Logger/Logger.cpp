#include "Logger.hpp"

Log::Level Log::current_level = Log::LEVEL_INFO;

std::reference_wrapper<std::ostream> Log::out = std::cout;
std::reference_wrapper<std::ostream> Log::err = std::cerr;

std::map<Log::Level, std::string> Log::level_names = {
    {Log::LEVEL_DEBUG, "DEBUG"},
    {Log::LEVEL_INFO, "INFO"},
    {Log::LEVEL_WARNING, "WARNING"},
    {Log::LEVEL_ERROR, "ERROR"},
};
void Log::init(Level level, std::ostream& out_stream, std::ostream& err_stream) {
    Log::current_level = level;
    Log::out = out_stream;
    Log::err = err_stream;
}
void Log::set_log_level(Level level) { Log::current_level = level; }

