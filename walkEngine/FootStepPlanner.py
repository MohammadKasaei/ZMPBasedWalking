
import matplotlib.pyplot as plt


class FootStepPlanner:
   
    

    def __init__(self,FR0X=0,FR0Y=0,FL0X=0,FL0Y=0.1,distance_between_feet=0.2):
        self.FR0X = FR0X
        self.FR0Y = FR0Y
        self.FL0X = FL0X
        self.FL0Y = FL0Y
        self.distance_between_feet = distance_between_feet
        self.support_pos_x  = []
        self.support_pos_y  = []
        self.right_foot_x =  []
        self.right_foot_y =  []
        
        self.left_foot_x =  []
        self.left_foot_y =  []
        

    def plan_steps(self,number_of_step,step_x,step_y,first_step_is_right):
        support_pos_X = []
        support_pos_y = []

        right_foot_x =  [self.FR0X]
        right_foot_y =  [self.FR0Y]

        left_foot_x =  [self.FL0X]
        left_foot_y =  [self.FL0Y]

        if (first_step_is_right):
            right_is_support = 0       
            left_is_support = 1        
        else:
            right_is_support = 1       
            left_is_support = 0        
        

        for i in range(1,number_of_step+1):
            if (right_is_support == 1):
                    support_pos_X.append(right_foot_x[len(right_foot_x)-1])
                    support_pos_y.append(right_foot_y[-1])
                    left_foot_x.append(left_foot_x[len(left_foot_x)-1]+step_x)
                    left_foot_y.append(left_foot_y[len(left_foot_y)-1]+step_y)    
                    
                    right_is_support = 0
                    left_is_support = 1

            elif (left_is_support == 1):
                    #print LFootx[len(LFootx)-1]
                    support_pos_X.append(left_foot_x[len(left_foot_x)-1])
                    support_pos_y.append(left_foot_y[len(left_foot_y)-1])
                    right_foot_x.append(right_foot_x[len(right_foot_x)-1]+step_x)
                    right_foot_y.append(right_foot_y[len(right_foot_y)-1]+step_y)        

                    right_is_support = 1
                    left_is_support = 0


        if (right_is_support == 1):
                support_pos_X.append(right_foot_x[len(right_foot_x)-1])
                support_pos_y.append(right_foot_y[len(right_foot_y)-1])    

                left_foot_x.append(left_foot_x[len(left_foot_x)-1]+step_x/2)
                left_foot_y.append(left_foot_y[len(left_foot_y)-1]+step_y/2)    

                support_pos_X.append(left_foot_x[len(left_foot_x)-1])
                support_pos_y.append(left_foot_y[len(left_foot_y)-1])
                        
                right_is_support = 1
                left_is_support = 1

        elif (left_is_support == 1):
                support_pos_X.append(left_foot_x[len(left_foot_x)-1])
                support_pos_y.append(left_foot_x[len(left_foot_x)-1])
                right_foot_x.append(right_foot_x[len(right_foot_x)-1]+step_x/2)
                right_foot_y.append(right_foot_y[len(right_foot_y)-1]+step_y/2)        
                
                support_pos_X.append( right_foot_x[len(right_foot_x)-1])
                support_pos_y.append(right_foot_y[len(right_foot_y)-1])   

                right_is_support = 1
                left_is_support = 1
                
        self.support_pos_x = support_pos_X
        self.support_pos_y = support_pos_y
        self.right_foot_x =  right_foot_x
        self.right_foot_y =  right_foot_y    
        self.left_foot_x =  left_foot_x
        self.left_foot_y =  left_foot_y

