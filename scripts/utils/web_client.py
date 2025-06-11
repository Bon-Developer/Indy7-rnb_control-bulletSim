import requests
from bs4 import BeautifulSoup

def decode(item):
    if hasattr(item, "decode"):
        return item.decode()
    else:
        return item

##
# @class WebClient
# @brief Python client for ControlHub WebUI
class WebClient:
    ##
    # @param ip IP address of the controller PC, in which the ControlHub and WebUI is running
    # @param port Port number for the UI. Typically 9990 (JointControl) or 9991 (TaskControl)
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.uri = "http://{IP_ADDR}:{UI_PORT}".format(IP_ADDR=ip, UI_PORT=port)
        try:
            self.read_ui()
        except:
            print("Error: Initial Web UI reading failed - Web UI page is not ready")
        
    ##
    # @brief Manually read the ui. Call this function when the gain values are changed by other interface.
    def read_ui(self):
        resp = requests.get(self.uri)
        self.__interpret_resp(resp)
        
    ##
    # @brief Change gain value. Call this function in from of change_gain(gain_id1=gain_value1, gain_id2=gain_value2, ...)
    def change_gain(self, **kwargs):
        param_list = []
        for gain_id, gain_val in kwargs.items():
            if gain_id not in self.gain_ids_all:
                print("INVALID GAIN ID: {} is not in gain_ids_all. Ignoring {}={}".format(gain_id, gain_id, gain_val))
                continue
            param_list.append("{}={}".format(gain_id, gain_val))
        uri_sent = self.uri+"/param_setting?"+"&".join(param_list)
        print("URI sent: {}".format(uri_sent))
        resp = requests.get(uri_sent)
        self.__interpret_resp(resp)
        
    ##
    # @brief Get current gain value by its id
    def get_gain_by_id(self, gain_id):
        if not hasattr(self, "gain_full_dict"):
            print("ERROR: Web Ui is not read yet. call read_ui() first")
            return None
        if not gain_id in self.gain_full_dict:
            print("ERROR: {} is invalid gain id".format(gain_id))
            return None
        return self.gain_full_dict[gain_id]
        
    def __interpret_resp(self, resp):
        soup = BeautifulSoup(resp.text, 'html.parser')
        for doc in soup.children:
            pass
        for body in doc.children:
            pass
        breaknow = False
        hub_title = ""
        gain_ids_all = []
        gain_full_dict = {}
        gain_listed_dict = {}
        for child in body.children:
            try:
                if "h2" in decode(child):
                    hub_title = str(child.contents[0])
                if hasattr(child, "children"):
                    for attr in child.children:
                        id_attr = get_named_val(attr, "id")
                        options = get_options(attr)
                        if "_control" in id_attr and len(options)>0:
                            controllers = options
                        if "table" in decode(attr):
                            for item in attr.children:
                                if "label for" in decode(item):
                                    gain_vals = []
                                    gain_ids = []
                                    for item_row in item.children:
                                        if "label for" in decode(item_row):
                                            gain_name = get_named_val(item_row, "for")
                                        if "input id" in decode(item_row):
                                            gain_id = get_named_val(item_row, "id")
                                            gain_ids.append(gain_id)
                                            gain_ids_all.append(gain_id)
                                            gain_vals.append(float(get_named_val(item_row, "value")))
                                    gain_listed_dict[gain_name] = gain_vals
                                    gain_full_dict.update({k: v for k, v in zip(gain_ids,gain_vals)})
                                if breaknow:
                                    break
                            if breaknow:
                                break
                    if breaknow:
                        break
            except:
                pass
        self.hub_title = hub_title
        self.gain_ids_all = gain_ids_all
        self.gain_full_dict = gain_full_dict
        self.gain_listed_dict = gain_listed_dict


##
# @brief get value of specific option in a html attribute
# @param attr a BeautifulSoup instance for the attribute 
# @param name name of the option
def get_named_val(attr, name):
    decoded = decode(attr)
    id_leftfit = decoded[decoded.find(name+'="')+len(name)+2:]
    id_str = str(id_leftfit[:id_leftfit.find('"')])
    return id_str


##
# @brief get options of a select attribute
# @param attr a BeautifulSoup instance for the attribute 
def get_options(attr):
    if "option" in decode(attr):
        options = [
            str(content[content.find('value="')+7:content.rfind('">')]) 
            for content in map(lambda x:decode(x), attr.contents) if "option" in content
        ]
    else:
        options = []
    return options