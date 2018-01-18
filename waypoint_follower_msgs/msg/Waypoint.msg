# The type of information provided about a given waypoint is represented by
# each bit. type = 4*ID + 2*completion + pose. Alternatively, this table:
# Type  |  ID   |  Completion status |  Pose 
#  0       no            no              no
#  1       no            no              yes
#  2       no            yes             no
#  3       no            yes             yes
#  4       yes           no              no
#  5       yes           no              yes
#  6       yes           yes             no
#  7       yes           yes             yes

# Declare the type of information provided.
uint8 INFO_ID         = 4
uint8 INFO_COMPLETION = 2
uint8 INFO_POSE       = 1

uint8               type

# These hold the information.
uint8               id
bool                completed
geometry_msgs/Pose  pose

