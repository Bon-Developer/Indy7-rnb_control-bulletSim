//
// Created by rnb on 21. 3. 2..
//

#ifndef WEBTEST_HTML_CONTENTS_H
#define WEBTEST_HTML_CONTENTS_H

#include <errno.h>
#ifdef _WIN32
#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#define mkdir(path, flag) _mkdir(path)
#else
#include <unistd.h>
#endif
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include "data_logger.h"
#include "utils_rnb.h"

#define MAXBUF 1024

#define HTTP_FILE_HEADER \
    "HTTP/1.0 200 OK\n" \
    "Content-type: application/octet-stream\n" \
	"Content-Disposition: attachment; filename=\"log.csv\" \n" \
    "\n"

#define HTML_HEADER \
    "HTTP/1.0 200 OK\n" \
    "Content-type: text/html\n" \
    "\n"

#define HTML_CONTENT_HEAD \
    "<!DOCTYPE html>\n" \
    "<html lang = \"ja\">\n" \
    "<head>\n" \
    "<meta charset = \"utf-8\">\n" \
	"<link rel=\"icon\" href=\"data:,\">" \
    "</head>\n" \
	"<body>\n"

#define HTML_CONTENT_TAIL \
        "</body>"\
		"</html>"

#define CHART_MIN_JS "Chart.min.js"


/**
 * @class Dropdown
 * @brief html tag generator for a dropdown
 */
class Dropdown{
public:
    /**
     * @class DropItem
     * @brief dropdown item holder
     */
    struct DropItem{
        std::string name;
        std::string text;
        /**
         * @param name name of the item, recommend all letter in lowercase
         * @param text text to display in the page, all character available
         */
        DropItem(const char* name, const char* text): name(name), text(text){}
    };
    std::string endpoint; /**< @brief the dropdown action endpoint in from of "/ENDPOINT" */
    std::string name; /**< @brief name of the list */
    std::string head_text; /**< @brief guide text to display ex) "select item" */
    std::vector<DropItem> items; /**< @brief list of dropdown items*/
    std::string selected; /**< @brief name of selected item*/
    std::string endpoint_default; /**< @brief the endpoint for "Set Default" button*/
    std::string save_path;
    std::string save_file;

    /**
     * @param endpoint the page endpoint in from of "/ENDPOINT"
     * @param name name of the list
     * @param head_text guide text to display ex) "select item"
     */
	Dropdown(const char* endpoint, const char* name, const char* head_text):
    	endpoint(endpoint), name(name), head_text(head_text){
        selected = "";
		endpoint_default = this->endpoint+"_default";
        save_path = "./formdata";
        save_file = save_path+"/"+name+"-default";
        clear_items();
        load_default();
    }

	/**
	 * @brief	set current selected item as default (saved as file)
	 */
	void set_default(){
        mkdir(save_path.c_str(), 0755);
        std::ofstream wfile(save_file);
        if(wfile.is_open()){
            wfile<<selected;
        }
        wfile.close();
	}

	/**
	 * @brief	load default item (saved as file)
	 */
	void load_default(){
        std::ifstream rfile(save_file);
        std::string rline;
        if(rfile.is_open()){
            getline(rfile, rline);
            select(rline.c_str());
        }
        rfile.close();
	}

    /**
     * @brief select item in the list
     */
    void select(const char* name){
        selected = name;
    }

    /**
     * @brief clear item list
     */
    void clear_items(){
        items.clear();
    }

    /**
     * @brief add item
     * @param name name of the item, used as id
     * @param text text to show in the dropdown list
     */
    void add_item(const char* name, const char* text){
        items.push_back(DropItem(name, text));
    }

    /**
     * @brief get full html tag for the drobdown list
     */
    std::string get_html(){
        std::string html_begin =
                "<form action=\""+endpoint+"\" style=\"float: left\">\n"
                                           "  <label for=\""+name+"\">"+head_text+"</label>\n"
                                                                                  "  <select name=\""+name+"\" id=\""+name+"\">\n";
        std::string html_content = "";
        for (auto itor=items.begin(); itor!=items.end(); itor ++){
            if(selected == itor->name){
                html_content += "    <option value=\"" + itor->name + "\" selected>" + itor->text + "</option>\n";
            }
            else {
                html_content += "    <option value=\"" + itor->name + "\">" + itor->text + "</option>\n";
            }
        }
        std::string html_end =
                "  </select>\n"
                "  <input type=\"submit\" value=\"Select\">\n"
                "</form>\n"
        		"<form action=\"" + endpoint_default + "\" style=\"float: left; margin-left: 0.2em;\">\n"\
                "<input type=\"submit\" value=\"Set Default\" style=\"width: 8em\">\n"\
                "</form>\n"\
                "<br>\n<br>\n";
        return html_begin+html_content+html_end;
    }

    /**
     * @brief update item with input uri
     */
    bool update(const char* uri){
        std::string uri_string(uri);
        if (uri_string.rfind(endpoint_default) == 0){
			set_default();
			return false;
		}
        else if (uri_string.rfind(endpoint) == 0) {
            int ctl_idx = uri_string.rfind(name+"=");
            if (ctl_idx>0){
                ctl_idx+=name.size()+1;
                std::string selected_item = uri_string.substr(ctl_idx);
                int end_idx = selected_item.find("&");
                if(end_idx>0){
                    selected_item = selected_item.substr(0, end_idx);
                }
                select(selected_item.c_str());
                return true;
            }
        }
        return false;
    }
};

/**
 * @class MultipleInputForm
 * @brief html form generator
 */
class MultipleInputForm{
public:
    std::string title; /**< @brief title of the form */
    std::string endpoint; /**< @brief the page endpoint in from of "/ENDPOINT" */
    std::map<std::string, std::vector<double>> value_map; /**< @brief map for name and value of each input */

    std::string endpoint_save;
    std::string endpoint_load;
    std::string save_file;
    std::string save_path;
    /**
     * @param endpoint the page endpoint in from of "/ENDPOINT"
     */
    MultipleInputForm(const char* endpoint): endpoint(endpoint){
        title="";
        endpoint_save = this->endpoint+"_save";
        endpoint_load = this->endpoint+"_load";
        save_path = "./formdata";
    }

    /**
     * @brief set title of the parameter setting panel
     */
    void set_title(const char* title = NULL){
    	if(title != NULL){
            this->title = title;
    	}
        save_file = save_path+"/"+this->title;
    }

    /**
     * @brief clear parameters
     */
    void clear(){
        value_map.clear();
    }

    /**
     * @param name name of the input
     * @param value current value of the input
     */
	void add_input(const char* name, std::vector<double> value){
        value_map.insert(std::make_pair(name, value));
    }

    /**
     * @brief get full html tag for the input form
     */
    std::string get_html(){
        std::string html_begin =
                "<form action=\""+endpoint+"\">\n"\
                "<b>"+title+"</b>\n";

        std::string html_content = "<hr>\n"
                                   "\n"
                                   "<table>\n"
                                   "<th>name</th>\n"
                                   "<th>value</th>\n";

        for (auto itor=value_map.begin(); itor!=value_map.end(); itor++){
            html_content += \
                "<tr>\n"
                "<td>\n"
                "<label for=\""+itor->first+"\">"+itor->first+  \
                ": </label>\n"
                "</td>\n";
            for (int i_val=0; i_val<itor->second.size(); i_val++){
            	std::string value_name = itor->first+std::to_string(i_val);
            	html_content += \
					"<td>\n"
					"<input type=\"number\" step=\"any\"  name=\""+value_name+"\" id=\""+value_name+"\""\
					" value=\"" + RNB::to_string_with_precision(itor->second[i_val], 3) + "\""\
					" style=\"width: 7em\">\n"
					"</td>\n";
            }
			html_content += "</tr>\n";

        }
        html_content +=
                "  </table>\n"
                "  <input type=\"submit\" value=\"Apply\" style=\"width: 10em\">\n"
                "</form>\n";
        std::string html_end =
                "<form action=\"" + endpoint_save + "\" style=\"float: left\">\n"\
                "<input type=\"submit\" value=\"Save\" style=\"width: 5em\">\n"\
                "</form>\n"\
                "<form action=\"" + endpoint_load + "\" style=\"float: left\">\n"\
                "<input type=\"submit\" value=\"Load\" style=\"width: 5em\">\n"\
                "</form>\n"\
                "<br>\n"\
                "<hr>\n";
        return html_begin+html_content+html_end;
    }

    /**
     * @brief load params
     */
    void load_params(){
        std::ifstream rfile(save_file);
        std::string rline;
        std::string uri_string;
        if(rfile.is_open()){
            getline(rfile, rline);
            uri_string = endpoint+"?"+rline;
        	update_params(uri_string);
        }
        rfile.close();
    }

    /**
     * @brief save current params
     */
    void save_params(){
        std::string save_string = "";
        for(auto itor=value_map.begin(); itor!=value_map.end();itor++){
        	for(int i_val=0;i_val<itor->second.size();i_val++){
                save_string += "&"+itor->first+std::to_string(i_val)+"="+std::to_string(itor->second[i_val]);
        	}
        }
        mkdir(save_path.c_str(), 0755);
        std::ofstream wfile(save_file);
        if(wfile.is_open()){
            wfile<<save_string;
        }
        wfile.close();
    }

    void update_params(std::string& uri_string){
    	printf("==============================\n");
    	printf("======== update param ========\n");
    	printf("==============================\n");
        for(auto itor=value_map.begin(); itor!=value_map.end();itor++){
        	for(int i_val=0;i_val<itor->second.size();i_val++){
        		std::string val_name = itor->first+std::to_string(i_val);
        		// parameters are divided by "&" or "?"
                int vidx = uri_string.rfind("&"+val_name+"=");
                if (vidx < 0){
                	vidx = uri_string.rfind("?"+val_name+"=");
                }
                if (vidx>0){
                    vidx+=val_name.size()+2;
                    std::string val_str = uri_string.substr(vidx);
                    int end_idx = val_str.find("&");
                    // if not last, end_idx for current param string can be found with "&"
                    if(end_idx>0){
                        val_str = val_str.substr(0, end_idx);
                    }

                    double val = std::stod(val_str);
                    value_map[itor->first][i_val] = val;
                	std::cout << val_name << " : " << val << std::endl;
                }
        	}
        }
    }

    /**
     * @brief update setting with input uri
     */
    bool update(const char* uri){
        std::string uri_string(uri);
        if (uri_string.rfind(endpoint_load) == 0) {
        	load_params();
            return true;
        }
        if (uri_string.rfind(endpoint_save) == 0) {
        	save_params();
            return false;
        }
        if (uri_string.rfind(endpoint) == 0) {
        	update_params(uri_string);
            return true;
        }
        return false;
    }
};

/**
 * @class HtmlChart
 * @brief html form generator
 */
class HtmlChart{
public:
	std::string chart_js_str;
	std::vector<std::string> color_codes;

	DataLogger* data_logger_p;


	std::string endpoint_step;
	std::string endpoint_pause;
	std::string endpoint_down;
	int plot_step;
	HtmlChart(DataLogger* data_logger_p, const char* endpoint_step, const char* endpoint_pause, const char* endpoint_down) :
		data_logger_p(data_logger_p), endpoint_step(endpoint_step), endpoint_pause(endpoint_pause), endpoint_down(endpoint_down) {

		plot_step = (int)(data_logger_p->BUFF_SIZE/LOG_BUFF_LEN_SEC/10);

		chart_js_str = "";
        std::ifstream rfile("./assets/" CHART_MIN_JS);
        std::string rline;
        if(rfile.is_open()){
            while (getline(rfile, rline))
            {
            	chart_js_str += (rline + "\r\n");
            }
        }
        rfile.close();
        color_codes.push_back("royalblue");
        color_codes.push_back("orangered");
        color_codes.push_back("orange");
        color_codes.push_back("darkorchid");
        color_codes.push_back("mediumseagreen");
        color_codes.push_back("cornflowerblue");
        color_codes.push_back("deeppink");
        color_codes.push_back("palegreen");
	}

	std::string get_articlehead(std::string &article, std::string &timeline){
		return "<canvas id=\""+article+"\"> </canvas> \n"
				"<script> \n"
				"var ctx = document.getElementById('"+article+"').getContext('2d'); \n"
				"var chart = new Chart(ctx, { type: 'line', \n"
				"data: { labels: [" + timeline + "], \n"
				"datasets: [ ";
	}

	void get_full_log_text(std::string& full_log){
		bool log_pause_bak = data_logger_p->log_paused;
		data_logger_p->pause_logging(true);

		full_log = "";

		if(data_logger_p->data_queue.size()==0){
			return;
		}
		LogData data0 = data_logger_p->data_queue[0];
		for(auto itor = data_logger_p->titles.begin(); itor!=data_logger_p->titles.end(); itor++){
			std::string title = *itor;
			Eigen::VectorXd datvec = data0.data_map[title];
			int veclen = datvec.size();
			for(int i=0; i<veclen; i++){
				full_log+=(title+std::to_string(i)+",");
			}
		}
		full_log+="\n";

		for(auto itor_d=data_logger_p->data_queue.begin(); itor_d!=data_logger_p->data_queue.end(); itor_d++){
			for(auto itor_t = data_logger_p->titles.begin(); itor_t!=data_logger_p->titles.end(); itor_t++){
				Eigen::VectorXd datavec = itor_d->data_map[*itor_t];
				for(int i_vec=0; i_vec<datavec.size(); i_vec++){
					full_log+=(RNB::to_string_with_precision(datavec[i_vec], 6)+",");
				}
			}
			full_log += "\n";
		}
		full_log+="\n";
		data_logger_p->pause_logging(log_pause_bak);
	}

	/**
	 * @brief get full html tag for the chart
	 */
	std::string get_html() {
		bool log_pause_bak = data_logger_p->log_paused;
		data_logger_p->pause_logging(true);
		std::string html = "<br> \n <b> Control Log </b> &nbsp; <a href=\"/refresh\"> refresh </a> \n <hr> \n";
		html += "<table>\n"
				"<tr>\n"
				"<td>\n"
				"<form action=\""+endpoint_step+"\">\n"\
				"<b> plotting step </b>\n"
				"<input type=\"number\" name=\"stepsize\" id=\"stepsize\" "
				"min=\""+std::to_string((int)(data_logger_p->BUFF_SIZE/LOG_BUFF_LEN_SEC/100))+"\" "
				"max=\""+std::to_string((int)(data_logger_p->BUFF_SIZE/LOG_BUFF_LEN_SEC))+"\" "\
				"value=\"" + std::to_string(plot_step) + "\""\
				"style=\"width: 7em\">\n"
				"<input type=\"submit\" value=\"Apply\" style=\"width: 5em\">\n"
				"</form>\n"
				"</td>\n"
				"<td>\n"
				"<form action=\"" + endpoint_pause+ "\" style=\"float: left\">\n"
				"<input type=\"submit\" value=\"" + (log_pause_bak ? std::string("Restart Log") : std::string("Pause Log")) + "\" style=\"width: 7em\">\n"
				"</form>\n"
				"</td>\n"
				"<td>\n"
				"<form action=\"" + endpoint_down + "\" style=\"float: left\">\n"
				"<input type=\"submit\" value=\"Download Full\" style=\"width: 10em\">\n"
				"</form>\n"
				"</td>\n"
				"</tr>\n"
				"<table>\n";

		html += "<script src=\"" CHART_MIN_JS "\"> </script> \n";
		std::string article;
		std::string timeline="";

		for(int i = 0;i<data_logger_p->data_queue.size();i+=plot_step){
			timeline += "'"+RNB::to_string_with_precision(data_logger_p->data_queue[i].time, 2)+"', ";
		}

		if(data_logger_p->data_queue.size()>0){
			LogData data0 = data_logger_p->data_queue[0];

			for(auto itor_t=data_logger_p->titles.begin(); itor_t!=data_logger_p->titles.end(); itor_t++){
				article = *itor_t;
				html += get_articlehead(article, timeline);
				for(int j=0; j<data0.data_map[article].size(); j++){
					html += "{ label: '"+article+std::to_string(j);
					html += "', backgroundColor: 'transparent', borderColor: '"+color_codes[j]+"', ";
					html += "data: [";
					for(int i = 0;i<data_logger_p->data_queue.size();i+=plot_step){
						html += RNB::to_string_with_precision(data_logger_p->data_queue[i].data_map[article][j], 5)+", ";
					}
					html += "] },\n";
				}
				html += "] },\n"
						"options: {} }) \n"
						"</script> \n"
						"<br>\n"
						"<hr>\n";
			}
		}
		data_logger_p->pause_logging(log_pause_bak);
		return html;
	}

	bool update_down(std::string uri_string, std::string& html){
        if (uri_string.rfind(endpoint_down) == 0) {
        	get_full_log_text(html);
        	return true;
        }
        if (uri_string.rfind(endpoint_pause) == 0) {
        	data_logger_p->pause_logging(!data_logger_p->log_paused);
        }
        if (uri_string.rfind(endpoint_step) == 0) {
        	int idx_step = uri_string.rfind("stepsize=")+strlen("stepsize=");
        	plot_step = std::stod(uri_string.substr(idx_step));
        }
    	return false;
	}
};

#endif //WEBTEST_HTML_CONTENTS_H
