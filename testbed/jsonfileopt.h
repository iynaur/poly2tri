#pragma once

#include <value.h>

namespace jsonfileopt
{
void json2file(const Json::Value &offsetOption, std::string config_file);
void file2json(std::string config_file, Json::Value &offsetOption);
void file2string(std::string config_file, std::string &content);
void string2file(const std::string &content, std::string config_file);
void json2string(const Json::Value &offsetOption, std::string &content);
void json2ShortString(const Json::Value& json, std::string& content);
void string2json(const std::string &content, Json::Value &offsetOption);
}
