#include "jsonfileopt.h"
#include <fstream>
#include <sstream>
#include <jsoncpp/json/reader.h>
#include <jsoncpp/json/writer.h>


void jsonfileopt::file2json(std::string config_file, Json::Value &offsetOption)
{
  std::ifstream ifs(config_file.c_str());
  Json::Reader reader;
  reader.parse(ifs, offsetOption);
  ifs.close();
}

void jsonfileopt::json2file(const Json::Value &offsetOption, std::string config_file)
{
  Json::StreamWriterBuilder builder;
  const std::string output = Json::writeString(builder, offsetOption);
  std::ofstream fout;
  fout.open(config_file);

  std::string out = offsetOption.toStyledString();
  fout << out << std::endl;

  fout.close();
}

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

void jsonfileopt::json2string(const Json::Value &offsetOption, std::string &content)
{
  content = offsetOption.toStyledString();
}

void jsonfileopt::string2json(const std::string &content, Json::Value &offsetOption)
{
  Json::Reader reader;
  reader.parse(content, offsetOption);
}
