import time
import numpy as np
import inspect
import json
import pickle
from threading import Thread, Event
from enum import Enum
import socket

def divide_kwargs(kwargs, func1, func2):
    keys1 = inspect.getargspec(func1).args
    keys2 = inspect.getargspec(func2).args
    kwargs1 = {k:v for k,v in kwargs.items() if k in keys1}
    kwargs2 = {k:v for k,v in kwargs.items() if k in keys2}
    return kwargs1, kwargs2

def list2dict(item_list, item_names):
    return {jname: jval for jname, jval in zip(item_names, item_list)}

def dict2list(item_dict, item_names):
    return [item_dict[jname] for jname in item_names]

def save_pickle(filename, data):
    with open(filename, 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)

def load_pickle(filename):
    with open(filename, 'rb') as f:
        data = pickle.load(f)
        return data

def save_json(filename, data):
    with open(filename, "w") as json_file:
        json.dump(data, json_file, cls=NumpyEncoder,indent=2)

def load_json(filename):
    with open(filename, "r") as st_json:
        st_python = json.load(st_json)
    return st_python
        
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)
    
def send_recv(sdict, host, port):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    client_socket.connect((host, port))

    rdict = {}
    try:
        sjson = json.dumps(sdict, cls=NumpyEncoder)
        sbuff = sjson.encode()
        client_socket.sendall(sbuff)

        rjson = client_socket.recv(1024)
        # rjson = "".join(map(chr, rjson))
        rdict = json.loads(rjson)
        if rdict is None:
            rdict = {}
        rdict = {str(k): v for k,v in rdict.items()}
    finally:
        client_socket.close()
    return rdict
from shutil import copyfile

##
# @brief copy file and replace substring
# @param line_callback function to be called when a line with string_from appeared
def copyfile_replace(file_from, file_to, string_from, string_to, line_callback=None):
    fin = open(file_from, "rt")
    #output file to write the result to
    fout = open(file_to, "wt")
    #for each line in the input file
    for line in fin:
        #read replace the string and write to output file
        if line_callback is not None and string_from in line:
            line_callback(line, string_from, string_to)
        fout.write(line.replace(string_from, string_to))
    #close input and output files
    fin.close()
    fout.close()
    
##
# @class TextColors
# @brief color codes for terminal. use println to simply print colored message
class TextColors(Enum):
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

    def println(self, msg):
        print(self.value + str(msg) + self.ENDC.value)


##
# @class PeriodicTimer
# @brief    Creates a timer that can wait for periodic events.
# @remark   It is recommended to stop timer thread when its use is expired.
class PeriodicTimer:
    ##
    # @param period     period of the timer event, in secondes
    def __init__(self, period):
        self.period = period
        self.__tic = Event()
        self.__stop = Event()
        self.thread_periodic = Thread(target=self.__tic_loop)
        self.thread_periodic.daemon = True
        self.thread_periodic.start()

    def __tic_loop(self):
        while not self.__stop.wait(timeout=self.period):
            self.__tic.set()

    ##
    # @brief    wait for next timer event
    def wait(self):  # Just waiting full period makes too much threading delay - make shorter loop
        while not self.__tic.wait(self.period / 100):
            pass
        self.__tic.clear()

    ##
    # @brief    stop the timer thread
    def stop(self):
        self.__stop.set()

    def __del__(self):
        self.stop()

    def call_periodic(self, fun, N=None, timeout=None, args=[], kwargs={}):
        i_call = 0
        time_start = time.time()
        while True:
            self.wait()
            fun(*args, **kwargs)
            i_call += 1
            if N is not None and i_call > N:
                break
            if timeout is not None and time.time() - time_start > timeout:
                break

    def call_in_thread(self, fun, N=None, timeout=None, args=[], kwargs={}):
        kwargs_new = dict(fun=fun, N=N, timeout=timeout, **kwargs)
        t = Thread(target=self.call_periodic, args=args, kwargs=kwargs_new)
        t.daemon = True
        t.start()

##
# @class StrictTimer
# @brief realtime timer for non-realtime os
class StrictTimer:
    def __init__(self, DT):
        self.DT = DT
        self.t_s = time.time()
        
    def sleep(self):
        while True:
            if time.time()-self.t_s>=self.DT:
                self.t_s += self.DT
                break
                
##
# @class LowPassFilter
class LowPassFilter:
    ##
    # @param dT sampling period
    # @param fc cutoff frequency
    def __init__(self, dT, fc, X0=None):
        self.dT, self.fc = dT, fc
        if X0 is not None:
            self.reset(X0)
        
    def reset(self, X):
        self.Xpre = np.copy(X)
        self.Ypre = np.copy(X)
        
    def update(self, X):
        self.Ypre = (1-self.fc*self.dT/2) / (1+self.fc*self.dT/2) * self.Ypre + (self.fc*self.dT/2) / (1+self.fc*self.dT/2)*(X+self.Xpre)
        self.Xpre = np.copy(X)
        return np.copy(self.Ypre)
