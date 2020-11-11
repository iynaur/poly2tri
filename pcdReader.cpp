#include <iostream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "pcdReader.h"
#include <testbed/jsonfileopt.h>
#include <locale> 
#include <codecvt>

std::wstring StringToWstring(const std::string& str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

std::string WstringToString(const std::wstring& wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

using namespace std;

void saveString2File(string content, string filename) {
	if (1) {
		
		std::ofstream fout;
		fout.open(filename);
		fout << content << std::endl;
		fout.close();
	}
}

string readPcdFile(string filename) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(filename, *cloud);
	Json::Value root;
	for (auto& p : cloud->points) {
		Json::Value jp;
		jp["x"] = p.x;
		jp["y"] = p.y;
		jp["z"] = p.z;
		jp["r"] = p.r;
		jp["g"] = p.g;
		jp["b"] = p.b;
		root.append(std::move(jp));
	}
	string ans;
	jsonfileopt::json2string(root, ans);
	return ans;
	
}

int readPCD(char* filename, int slen, char** index) {
	saveString2File("", "into c++ readPCD.txt");
	wstring files = wstring(slen, ' ');
	memcpy((void*)files.data(), filename, slen*sizeof(wchar_t));
	saveString2File("", "before WstringToString.txt");
	saveString2File(WstringToString(files), "pcdfilename.txt");
	saveString2File("", "after WstringToString.txt");
	string ans = readPcdFile(WstringToString(files));
	saveString2File("", "after load pcd.txt");
	wstring wans = StringToWstring(ans);
	saveString2File("", "after StringToWstring.txt");
	char* res = new char[wans.size()*sizeof(wchar_t)];
	memcpy(res, wans.data(), wans.size() * sizeof(wchar_t));
	*index = res;
	saveString2File(ans, "ansjson.txt");
	return ans.size();
}
