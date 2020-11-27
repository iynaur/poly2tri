#pragma once

#include <string>

namespace jsonfileopt
{

void file2string(std::string config_file, std::string &content);
void string2file(const std::string &content, std::string config_file);

}
