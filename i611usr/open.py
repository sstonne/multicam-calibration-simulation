# -*- coding: utf-8 -*-

import time
import datetime
from i611_io import *

IOinit()

def check_gripper():                                         
        a = din(48)                                                   
        b = din(49)                                                   
        c = din(50)                                                   
        d = din(51)                                                   
        result = [d,c,b,a]                                                   
        return result                                                        
                                                                                 
def gripper(onoff):                                                              
        dout(48,'0000')                                                          
        if onoff == 'open':                                                      
                while check_gripper() !=  ['0','1','0','0']:                     
                        dout(48,'0100')                                          
                                                                                    
        elif onoff == 'close':                                                      
                while check_gripper() !=  ['0','0','0','1']:                                                      
                        dout(48,'0001')                                                                             
        else:                                                                                                       
                exit(0)     

#gripper('close')
gripper('open')





