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
//#define PCL_NO_PRECOMPILE
#include <pcl/surface/poisson.h>
#include "../pcdReader.h"
#include <locale> 
#include <codecvt>

using namespace pcl;
using namespace std;

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


int
main (int argc, char** argv)
{
    string filename = "C:/Users/admin/PointCloudExporter/Assets/StreamingAssets\\Simon2.pcd";
	wstring wf = StringToWstring(filename);
    char* cp;
    readPCD((char*)wf.data(), filename.size(), &cp);
    
}
