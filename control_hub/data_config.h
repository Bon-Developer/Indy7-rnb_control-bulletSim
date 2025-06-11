/**
 * data_config.h
 *
 *  Created on: 2021. 3. 7.
 *      Author: RNB_CAD
 */

#ifndef CONTROL_HUB_DATA_CONFIG_H_
#define CONTROL_HUB_DATA_CONFIG_H_




#include <string.h>
#include <cstdio>
#include <vector>
#include <map>
#include "../3rd_party/Eigen/Eigen"

struct LogData
{
	double time;
	std::map<std::string, Eigen::VectorXd> data_map;

	LogData() :time(0.0) {}

	LogData(double time, std::vector<std::string> titles, std::vector<Eigen::VectorXd>& vectors)
	: time(time)
	{
		for (int i=0; i<titles.size(); i++){
			data_map.insert(std::make_pair(titles[i], vectors[i]));
		}
	}

	LogData(LogData const & data)
	: time(data.time)
	{
		for(auto itor=data.data_map.begin(); itor!=data.data_map.end(); itor++){
			data_map.insert(std::make_pair(itor->first, itor->second));
		}
	}

	LogData & operator=(const LogData & data)
	{
		for(auto itor=data.data_map.begin(); itor!=data.data_map.end(); itor++){
			data_map.insert(std::make_pair(itor->first, itor->second));
		}
		return (*this);
	}
};


#endif /* CONTROL_HUB_DATA_CONFIG_H_ */
