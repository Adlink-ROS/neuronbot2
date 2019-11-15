#!/usr/bin/env python

import yaml
import sys
import os



local_path=os.path.dirname(__file__)
if (len(sys.argv) != 2):
    print 'please specify buad'
    sys.exit()

f = open(local_path+'/../params/base_params.yaml')  
params = yaml.load(f)  
params['buadrate']=int(sys.argv[1])
f.close()
f = open(local_path+'/../params/base_params.yaml', "w")  
yaml.dump(params, f)  
f.close() 

f = open(local_path+'/../params/base_params_with_imu.yaml')  
params = yaml.load(f)  
params['buadrate']=int(sys.argv[1])
f.close()
f = open(local_path+'/../params/base_params_with_imu.yaml', "w")  
yaml.dump(params, f)  
f.close() 

f = open(local_path+'/../params/base_params_without_odom.yaml')  
params = yaml.load(f)  
params['buadrate']=int(sys.argv[1])
f.close()
f = open(local_path+'/../params/base_params_without_odom.yaml', "w")  
yaml.dump(params, f)  
f.close() 
