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
#include <pcl/common/centroid.h>
#include <pcl/common/centroid.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/time.h>

#include "pcdReader.h"
#include <testbed/jsonfileopt.h>
#include <locale> 
#include <codecvt>
#define __attribute__() //
#include <nlohmann/json.hpp>

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
	string ans;
	if (0) {
		pcl::ScopeTime t("all");
		Json::Value root;
		{
			pcl::ScopeTime t("once");
			int n = cloud->size();
			for (int i = 0; i < n; ++i) {
				root.operator[](i);
			}
#pragma omp parallel for
			for (int i = 0; i < n; ++i) {
				auto& p = cloud->points[i];
				Json::Value& jp = root[i];
				jp["x"] = p.x;
				jp["y"] = p.y;
				jp["z"] = p.z;
				jp["r"] = p.r;
				jp["g"] = p.g;
				jp["b"] = p.b;
			}

		}
		{
			pcl::ScopeTime t("two");
			jsonfileopt::json2ShortString(root, ans);
		}
	}
	else {
		pcl::ScopeTime t("all");
		int n = cloud->size();
		using json = nlohmann::json;
		json root;
		{
			//pcl::ScopeTime t("once");

			if (n) root.operator[](n - 1);
#pragma omp parallel for
			for (int i = 0; i < n; ++i) {
				auto& p = cloud->points[i];
				json& jp = root[i];
				jp.operator[](5);
				jp[0] = p.x;
				jp[1] = p.y;
				jp[2] = p.z;
				jp[3] = p.r;
				jp[4] = p.g;
				jp[5] = p.b;
			}
		}
		{
			//pcl::ScopeTime t("twice");
			ans = root.dump();
		}
#pragma omp parallel for
		for (int i = 0; i < n; ++i) {
			root[i].clear();
			
		}
	}
	return ans;
}

#define saveString2File() //

int readPCD(char* filename, int slen, char** index) {
	auto dbg = sizeof(wchar_t);
	saveString2File("", "into c++ readPCD.txt");
	wstring files = wstring(slen, ' ');
	memcpy((void*)files.data(), filename, slen*sizeof(wchar_t));
	assert(files.substr(slen - 4) == L".pcd");
	saveString2File("", "before WstringToString.txt");
	saveString2File(WstringToString(files), "pcdfilename.txt");
	saveString2File("", "after WstringToString.txt");
	string ans = readPcdFile(WstringToString(files));
	saveString2File("", "after load pcd.txt");
	//wstring wans = StringToWstring(ans);
	saveString2File("", "after StringToWstring.txt");
	char* res = new char[ans.size()];
	memcpy(res, ans.data(), ans.size());
	*index = res;
	saveString2File(ans, "ansjson.txt");
	return ans.size();
}

int axis(void* verts_, int len, void* ans_) {
	float* verts = (float*)verts_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->resize(len);
	for (int i = 0; i < len; ++i) {
		memcpy(cloud->points[i].data, verts + 3 * i, 3 * sizeof(float));
	}

	using namespace pcl;

	// Placeholder for the 3x3 covariance matrix at each surface patch
	Eigen::Matrix3f covariance_matrix;

	// 16-bytes aligned placeholder for the XYZ centroid of a surface patch
	Eigen::Vector4f plan;
	float dummy;
	computePointNormal(*cloud, plan, dummy);

	float* ans = (float*)ans_;
	ans[0] = plan.x();
	ans[1] = plan.y();
	ans[2] = plan.z();
	return 0;
}