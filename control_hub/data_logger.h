/**
 * datalogger.h
 *
 *  Created on: 2021. 3. 7.
 *      Author: RNB_CAD
 */

#ifndef CONTROL_HUB_DATA_LOGGER_H_
#define CONTROL_HUB_DATA_LOGGER_H_

#include "data_config.h"
#include <deque>

#define LOG_BUFF_LEN_SEC 30

class DataLogger {
public:
	int BUFF_SIZE;
    bool log_paused;

    std::vector<std::string> titles;
	std::deque<LogData> data_queue;

    DataLogger(int BUFF_SIZE) : BUFF_SIZE(BUFF_SIZE), log_paused(true) {
		for(auto itor=titles.begin(); itor!=titles.end(); itor++){
			this->titles.push_back(*itor);
		}
    }
	~DataLogger() {}

	void add_title(std::string title){
		this->titles.push_back(title);
	}


	void pause_logging(bool pause){
		log_paused = pause;
	}

    void push_log(const double time, std::vector<Eigen::VectorXd> &vectors){
    	// Update circular buffer
    	if (!log_paused)
    	{
    		data_queue.push_back(LogData(time, titles, vectors));
    		while(data_queue.size()>BUFF_SIZE){
    			data_queue.pop_front();
    		}
    	}
    }
};

#endif /* CONTROL_HUB_DATA_LOGGER_H_ */
