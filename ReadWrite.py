from func_timeout import func_timeout, FunctionTimedOut
import time
import ast
import numpy as np

class ReadWrite:
    
    def __init__(self, ser=None, divider=':', NMS = 1):
        self.ser = ser or None
        self._divider = divider
        
        self._conversions = {'readConvert': (3.3)/(4095.), #convert int read value into volts
                            'writeConvert': (255.0)/ (0.445), #convert volt write value into ints
                            'nodeGain': 6.0, #op amp gain for node readings
                            'gateGain': 1./2., #voltage divider gain for gate readings
                            'eta_writeConvert': (128.0)/1., #convert eta (0-1.) into ints
                            'Gmin_writeConvert': (128.0)/(5.0)} #convert Gmin value into ints
        
        self._NMS = NMS
        
        self.gateconversion = self._conversions['readConvert']*self._conversions['gateGain']/self._NMS
        self.nodeconversion = self._conversions['readConvert']*self._conversions['nodeGain']/self._NMS
        
        
    @property
    def NMS(self):
        return self._NMS
        
    @NMS.setter
    def NMS(self, value):
        self._NMS = value
        self.gateconversion = self._conversions['readConvert']*self._conversions['gateGain']/self._NMS
        self.nodeconversion = self._conversions['readConvert']*self._conversions['nodeGain']/self._NMS
  

    def read_safe(self, timeoutval = 0.1):
        try:
            line = func_timeout(timeoutval, self.ser.readline)
            # print(line)
            return(line)
        except FunctionTimedOut:
            # print("Exception: No line to read")
            return(None)
        
        
    def clear_output(self):
        data = 1
        while data is not None:
            data = self.read_safe()
            
            
    def send_message(self, message):
        self.ser.write((message + "\n").encode())
        time.sleep(0.2)

            
    def interpret(self, byteval, suppress = False):
        if byteval is None:
            return
        strval = byteval.decode("utf-8")
        messages = strval.split("\r")[0].split(self._divider)
        try:
            msgtype = messages[1]
            if msgtype == 'msg':
                msg = messages[2]
                if not suppress:
                    print(msg)
                return msg, None
            elif msgtype == 'msgvar':
                varname = messages[2]
                val = messages[3]
                if not suppress:
                    print("{}, {}".format(varname,val))
                return varname, val
            elif msgtype == '1Darr':
                varname = messages[2]
                array = ast.literal_eval(messages[3])
                if not suppress:
                    print("{}, {}".format(varname,array))
                return varname, array
            elif msgtype == 'arr':
                varname = messages[2]
                arrshape = ast.literal_eval(messages[4])
                array = ast.literal_eval(messages[3])
                array = np.array(array)
                array = array.reshape(arrshape)
                return varname, array
        except: 
            print("exception:")
            print(messages)
            
    def send_recieve(self, message, timeoutval=1000., suppress = False):
        self.clear_output()
        time.sleep(1)
        self.send_message(message)
        data = 1
        while data is not None:
            data = self.read_safe(timeoutval = timeoutval)
            if not data:
                break    
            varname, val = self.interpret(data, suppress)

            if "finished" in varname:
                break

            if varname == "finalize":
                break
                
                
                
    def measure(self, timeoutval = 1000., suppress = False):
#         self.clear_output()
        time.sleep(1)
#         self.send_message("random;")
        data = 1
        savedict = {}
        while data is not None:
            data = self.read_safe(timeoutval = timeoutval)
            if not data:
                break    
            varname, val = self.interpret(data, suppress)

            if varname == "sending data":
                savedict = {}

            elif "finished" in varname:
                break

            elif varname == "finalize":
                break
            else:
                savedict[varname] = val
        return savedict
    
    
    def train_measure(self, timeoutval=1000.):
        time.sleep(1)
#         self.clear_output()
#         self.send_message("random;")
        data = 1
        savedict = {}
        listodicts = []
        reconstructing = False
        reconstructing_det = False
        idx = 0
        Vtest = []
        Vrecon = []
        while data is not None:
            data = self.read_safe(timeoutval = timeoutval)
            if not data:
                break    
            varname, val = self.interpret(data, suppress=True)
            
            if varname == "epoch":
                print(varname, val, end='\r')
            
            if reconstructing:
                if varname == "testidx":
                    idx = val
                
                elif varname == "finalizereconstruct":
                    reconstructing = False
                    idx=0
                    savedict['Vtest'] = Vtest
                    savedict['Vrecon'] = Vrecon
                    Vtest = []
                    Vrecon = []
                    
                elif varname == "Vtest":
                    Vtest.append(val)
                elif varname == "Vrecon":
                    Vrecon.append(val)
                else:
#                     newvarname = varname + "_{}".format(trial)
                    savedict[varname] = val

            if reconstructing_det:
                if varname == "testidx":
                    idx = val
                
                elif varname == "finalizereconstructdet":
                    #print("Finished Reconstructing det")
                    reconstructing_det = False
                    idx=0
                    savedict['Vtest_det'] = Vtest
                    savedict['Vrecon_det'] = Vrecon
                    #print(savedict)
                    Vtest = []
                    Vrecon = []
                    
                elif varname == "Vtest":
                    Vtest.append(val)
                elif varname == "Vrecon":
                    Vrecon.append(val)
                else:
#                     newvarname = varname + "_{}".format(trial)
                    savedict[varname] = val
            
           
            elif "finished" in varname:
                break
                
            elif varname == "reconstruction":
                reconstructing = True

            elif varname == "reconstruction_deterministic":
               # print("Reconstructing det")
                reconstructing_det = True
                

            elif varname == "finalize":
               # print('finalize epoch')
                #print(savedict)
                listodicts.append(savedict)
                savedict={}
            else:
                savedict[varname] = val
        return listodicts
    
    def convert(self, savedict):
        newdict = {}
        for key, val in savedict.items():
            val = val.astype(np.float32)
            if "Gate" in key:
                val *= self.gateconversion
            else:
                val *= self.nodeconversion
            newdict[key] = val
        return newdict