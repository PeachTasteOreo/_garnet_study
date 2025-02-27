# ***** DYNAMIXEL Control Table Addresses *****
ADDR_TORQUE_ENABLE = 64           
ADDR_GOAL_POSITION = 116          
LEN_GOAL_POSITION = 4             
ADDR_PRESENT_POSITION = 132       
LEN_PRESENT_POSITION = 4          
ADDR_GOAL_VELOCITY = 104          
ADDR_OPERATE_MODE = 11            

# ***** Torque Values *****
TORQUE_ENABLE = 1                 
TORQUE_DISABLE = 0                

# ***** Position and Motion Values *****
DXL_MINIMUM_POSITION_VALUE = 0    
DXL_MAXIMUM_POSITION_VALUE = 4095 
DXL_MOVING_STATUS_THRESHOLD = 10  

# ***** Communication Settings *****
BAUDRATE = 57600                  
PROTOCOL_VERSION = 2.0            

# ***** Device and ID Information *****
DEVICENAME = "/dev/ttyACM0"               
DXL_ID = 1                       

# ***** Operating Modes *****
current_control = 0              
velocity_control = 1           
position_control = 3              
extended_position_control = 4     