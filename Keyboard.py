
import threading
from getkey import getkey, keys


class keyboardMonitoring():
    def __init__(self):
                
        self.MAX_STEP_X      = 0.15
        self.MAX_STEP_Y      = 0.1
        self.MAX_STEP_Theta  = 30
        self.MAX_STEP_TIME   = 0.5
        self.MIN_STEP_TIME   = 0.1
        
        self.StepTime= 0.4
        self.new_command = 0
        self.new_step_x = 0
        self.new_step_y = 0
        self.new_step_theta = 0
        self.new_step_time = 0.3
        self.stop = 1
        self.WriteData =0
        self.F = [0 , 0, 0]
        self.Duration = 0

    def read_kbd_input(self,name):
        #print('Ready for keyboard input:')
        self.exit = 0
        while (self.exit == 0):
            key = getkey()
            if (key == 'z' or key == 'Z'):
                self.exit =1
                print ("EXIT")
            elif (key == keys.UP):
                if (self.new_step_x<self.MAX_STEP_X):               
                    self.new_step_x = self.new_step_x + 0.01
                    print ("Speed ", str(self.new_step_x))
                    self.new_command = 1
            elif (key == keys.DOWN):
                if (self.new_step_x>-self.MAX_STEP_X):               
                    self.new_step_x = self.new_step_x - 0.01
                    print ("Speed ", str(self.new_step_x))
                    self.new_command = 1
            elif (key == keys.RIGHT):
                if (self.new_step_theta < self.MAX_STEP_Theta):               
                    self.new_step_theta = self.new_step_theta + 1
                    print ("Theta ", str(self.new_step_theta))
                    self.new_command = 1
            elif (key == keys.LEFT):
                if (self.new_step_theta >- self.MAX_STEP_Theta):               
                    self.new_step_theta = self.new_step_theta - 1
                    print ("Theta ", str(self.new_step_theta))
                    self.new_command = 1
            elif (key == 'a' or key == 'A' ):
                if (self.new_step_y >- self.MAX_STEP_Y):               
                    self.new_step_y = self.new_step_y - 0.005
                    print ("StepY ", str(self.new_step_y))
                    self.new_command = 1
            elif (key == 'd' or key == 'D' ):
                if (self.new_step_y < self.MAX_STEP_Y):               
                    self.new_step_y = self.new_step_y + 0.005
                    print ("StepY ", str(self.new_step_y))
                    self.new_command = 1
            elif (key == 't' or key == 'T' ):
                if (self.new_step_time < self.MAX_STEP_TIME):               
                    self.new_step_time = self.new_step_time + 0.005
                    print ("StepTime ", str(self.new_step_time))
                    self.new_command = 1
            elif (key == 'y' or key == 'Y' ):
                if (self.new_step_time > -self.MAX_STEP_TIME):               
                    self.new_step_time = self.new_step_time - 0.005
                    print ("StepTime ", str(self.new_step_time))
                    self.new_command = 1
                    
            elif (key == ' '):
                if(self.stop == 1):
                    self.new_step_x = 0
                    self.new_step_y = 0
                    self.new_step_theta = 0
                    self.new_command = 1
                    self.stop = 0
                else:
                    self.stop = 1

            elif (key == 's' or key == 'S'):
                if (WriteData == 1):
                    WriteData = 0
                    file.close() 
                    print ("file is closed")
                else:
                    file = open('log.csv','w') 
                    WriteData = 1
                    print ("file is opened")
            elif (key == 'f' or key == 'F'):
                self.F = [500,0,0]
                self.Duration = 0.050
                print ("Forward Push has been applied")
                

            elif (key == 'g' or key == 'g'):
                self.F = [0,500,0]
                self.Duration = 0.05
                print ("Sideward Push has been applied")

            elif (key == 'r' or key == 'R'):
                print ("Reset Model Pose")
                self.stop = 100
    
    def start(self):        
        self._inputThread = threading.Thread(target=self.read_kbd_input, args=(1,))
        self._inputThread.start()
    
    def stop(self):
        self._inputThread.stop()
    
    