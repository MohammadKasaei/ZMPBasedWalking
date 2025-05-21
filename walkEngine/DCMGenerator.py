import numpy as np

class DCMGenerator:
    def __init__(self, g=9.81, z_0=0.8, dt=0.005):
        self.omega_0 = np.sqrt(g / z_0)
        self.dt = dt

    def generate(self, com_x, com_y):
        dcm_x = []
        dcm_y = []
        dxp,dyp = 0.0, 0.0 
        for i in range(len(com_x)-1):
            dx = (com_x[i+1] - com_x[i]) / self.dt
            dy = (com_y[i+1] - com_y[i]) / self.dt
            if abs(dx-dxp)<0.03:
                dcm_x.append(com_x[i] + dx / self.omega_0)
                dxp = dx
            else:
                dcm_x.append(dcm_x[-1])
            
            if abs(dy-dyp)<0.03:
                dcm_y.append(com_y[i] + dy / self.omega_0)
                dyp = dy    
            else:
                dcm_y.append(dcm_y[-1])
            
            
        # Repeat last value to match size
        dcm_x.append(dcm_x[-1])
        dcm_y.append(dcm_y[-1])
        self.dcm_x = dcm_x
        self.dcm_y = dcm_y
        return dcm_x, dcm_y