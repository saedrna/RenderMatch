/*
 * @Author:  
 * @Date:  
 * Convert original images into tiled TIFF for faster reading
 */
#include <iostream>
#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <math.h>
#include <stdlib.h>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <nlohmann/json.hpp>
#include <xml2json.hpp>

#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <sstream>

#include <math.h>
#include <string.h>

#include <QtCore/qfile.h>
#include <QtCore/qlist.h>
#include <QtCore/qshareddata.h>
#include <QtCore/qmetatype.h>
#include <cxxopts.hpp>

#include <opencv2/imgcodecs.hpp>
#include <osg/PagedLOD>
#include <osgDB/DatabasePager>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgUtil/SmoothingVisitor>
#include <osgViewer/Viewer>

#include <RenderMatch/block.h>
#include <base/base.h>

#include <gdal.h>


using namespace std;

void ReadMyTrack(std::vector<std::vector<double>>& myTrack, const std::string& trackPath2Save)
{
	ifstream inputfile;
	string tempfile;
	std::vector<std::string> trackTemp;
	inputfile.open(trackPath2Save);
	while (getline(inputfile, tempfile))
	{
		trackTemp.push_back(tempfile);
	}
	inputfile.close();
	for (int i = 0; i < trackTemp.size(); i++)
	{
		stringstream ss(trackTemp[i]);

		std::vector<double> temp3;
		for (int j = 0; j < 10; j++)
		{
			if (getline(ss, tempfile, ' '))
			{
				temp3.push_back(stod(tempfile));
			}
		}
		myTrack.push_back(temp3);
	}
}
void ReadRegularTrack(std::vector<std::vector<double>>& myTrack, const std::string& trackPath2Save)
{
	ifstream inputfile;
	string tempfile;
	std::vector<std::string> trackTemp;
	inputfile.open(trackPath2Save);
	while (getline(inputfile, tempfile))
	{
		trackTemp.push_back(tempfile);
	}
	inputfile.close();
	for (int i = 0; i < trackTemp.size(); i++)
	{
		stringstream ss(trackTemp[i]);

		std::vector<double> temp3;
		for (int j = 0; j < trackTemp.size(); j++)
		{
			if (getline(ss, tempfile, ' '))
			{
				if (!tempfile.empty())
				{
					temp3.push_back(stod(tempfile));
				}

			}
		}
		myTrack.push_back(temp3);
	}
}
void SaveTrack(std::vector<std::vector<double>> TracktoSave, const std::string& filepath2save)
{

	ofstream inputFile;
	inputFile.open(filepath2save);

	for (int i = 0; i != TracktoSave.size(); i++)
	{
		for (int j = 0; j < TracktoSave[i].size(); j++)
		{
			inputFile << TracktoSave[i][j];
			inputFile << " ";
		}
		inputFile << "\n";
	}
	inputFile.close();
}

void ReadTrackTemp(std::vector<std::vector<double>> TrackNew, const std::string& trackPath2Save)
{
	ifstream inputfile;
	string tempfile;
	std::vector<std::string> trackTemp;
	inputfile.open(trackPath2Save);
	while (getline(inputfile, tempfile))
	{
		trackTemp.push_back(tempfile);
	}
	inputfile.close();
	for (int i = 0; i < trackTemp.size(); i++)
	{
		stringstream ss(trackTemp[i]);

		std::vector<double> temp3;
		for (int j = 0; j < trackTemp[i].size(); j++)
		{
			if (getline(ss, tempfile, ' '))
			{
				temp3.push_back(stod(tempfile));
			}
		}
		TrackNew.push_back(temp3);
	}
}

std::string get_filename_noext(const std::string& path) {
	return QFileInfo(QString::fromStdString(path)).baseName().toStdString();
}

using namespace h2o;
using namespace nlohmann;

int main()
{


	/*IN photoscan导出的连接点路径*/
	std::string TiePath = "F:\\experiment\\dortmund\\files\\merge.xml";

	std::string trackFilePath = "F:\\experiment\\dortmund\\output\\track4.txt";

	std::string savePath = "F:\\experiment\\dortmund\\files\\7track2json.json";
	//TiePoint.SaveMyTrack(myTrack,trackFilePath);

	std::vector<std::vector<double>> myTrackinit;
	std::vector<std::vector<double>>& TrackNew = myTrackinit;
	ReadRegularTrack(TrackNew, trackFilePath);

	nlohmann::json jFile;
	QFile file(QString::fromStdString(TiePath));
	file.open(QFile::ReadWrite);
	std::string xml_data = QTextStream(&file).readAll().toStdString();
	std::istringstream ss(xml2json(xml_data.c_str()));
	ss >> jFile;



	json& block = jFile["BlocksExchange"]["Block"];
	auto& TiePoints = jFile["BlocksExchange"]["Block"]["TiePoints"]["TiePoint"];
	auto gp = jFile["BlocksExchange"]["Block"]["Photogroups"]["Photogroup"][0]["Photo"];
	for (auto &p : gp)
	{
		const std::string imgpath = p["ImagePath"];
		std::string ipath = get_directory(imgpath);
		//auto ipath2 = get_filename_noext(imgpath);
		//std::cout << imgpath << std::endl;
		//cv::Mat mmat = cv::imread("D:/image data/uav-ter/UAV and Terrestrial/RELEASE_FOLDER/UAV/DSC07022.JPG");
		if (cv::imread(imgpath, 1).empty()) {
			std::cout << p["Id"] << std::endl;
		}
	}

	json tieTemplate = TiePoints[0];

	//std::cout << std::setw(2)<< TiePoints[0] << std::endl;
	tieTemplate.clear();
	//std::cout << std::setw(2) << tieTemplate << std::endl;

	int trackId = 0, measureId = 0;

#ifdef REGULAR_TRACK2
	for (int i = 0; i < TrackNew.size(); i++)
	{
		tieTemplate["Name"] = std::to_string(i);
		tieTemplate["Type"] = "User";
		measureId = (TrackNew[i].size() - 4) / 3;//减去三个xyz坐标点，除以图像索引id和点位xy
		/*std::cout << tieTemplate["Measurement"][0]["PhotoId"] << std::endl;
		std::cout << tieTemplate["Measurement"][0]["x"] << std::endl;
		std::cout << tieTemplate["Measurement"][0]["y"] << std::endl;*/
		for (int j = 0; j < measureId; j++)
		{

			int PhotoId = TrackNew[i][j * 3 + 3];
			float phoX = TrackNew[i][j * 3 + 3 + 1], phoY = TrackNew[i][j * 3 + 3 + 2];
			tieTemplate["Measurement"][j]["PhotoId"] = std::to_string(PhotoId);
			tieTemplate["Measurement"][j]["x"] = std::to_string(phoX);
			tieTemplate["Measurement"][j]["y"] = std::to_string(phoY);
			std::cout << tieTemplate["Measurement"][j]["PhotoId"] << std::endl;
			std::cout << tieTemplate["Measurement"][j]["x"] << std::endl;
			std::cout << tieTemplate["Measurement"][j]["y"] << std::endl;
		}
		std::cout << std::setw(2) << tieTemplate << std::endl;
		TiePoints.push_back(tieTemplate);
		//tieTemplate = TiePoints[0];
		tieTemplate.clear();

	}
#endif
#ifdef REGULAR_TRACK
	for (int i = 0; i < TrackNew.size(); i++)
	{
		tieTemplate["Name"] = std::to_string(i);
		tieTemplate["Type"] = "User";
		measureId = (TrackNew[i].size() - 4) / 3;//减去三个xyz坐标点，除以图像索引id和点位xy
		/*std::cout << tieTemplate["Measurement"][0]["PhotoId"] << std::endl;
		std::cout << tieTemplate["Measurement"][0]["x"] << std::endl;
		std::cout << tieTemplate["Measurement"][0]["y"] << std::endl;*/
		for (int j = 0; j < measureId; j++)
		{

			int PhotoId = TrackNew[i][j * 3 + 4];
			float phoX = TrackNew[i][j * 3 + 4 + 1], phoY = TrackNew[i][j * 3 + 4 + 2];
			tieTemplate["Measurement"][j]["PhotoId"] = std::to_string(PhotoId);
			tieTemplate["Measurement"][j]["x"] = std::to_string(phoX);
			tieTemplate["Measurement"][j]["y"] = std::to_string(phoY);
			/*std::cout << tieTemplate["Measurement"][j]["PhotoId"] << std::endl;
			std::cout << tieTemplate["Measurement"][j]["x"] << std::endl;
			std::cout << tieTemplate["Measurement"][j]["y"] << std::endl;*/
		}
		//std::cout << std::setw(2) << tieTemplate << std::endl;
		TiePoints.push_back(tieTemplate);
		//tieTemplate = TiePoints[0];
		tieTemplate.clear();

	}
#endif


	//std::ofstream savej("E:\\temp file\\keysAndtrack\\tiepointsFromAgisoft\\tiesRectify\\test2.json");//"E:\temp file\makeTrack\pointsCloud\track2json.json"
	std::ofstream savej(savePath);
	savej << std::setw(2) << jFile;


	savej.close();


	return 0;
}