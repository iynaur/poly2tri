#include "jsonfileopt.h"
#include <fstream>
#include <sstream>

void jsonfileopt::file2string(std::string config_file, std::string &content)
{
  std::ifstream t(config_file);
  std::stringstream buffer;
  buffer << t.rdbuf();
  content = buffer.str();
}

void jsonfileopt::string2file(const std::string &content, std::string config_file)
{
  std::ofstream fout;
  fout.open(config_file);
  fout << content << std::endl;
  fout.close();
}
