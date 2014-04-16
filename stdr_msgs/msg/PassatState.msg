Header header

# the state of the passat state machine. One of the values listed below.
# this can be depended upon by other packages.
int32 fsm_state

# the state of the state machine, for debugging purpose
string internal_state

int32 DEFAULT=0 #this means that the state is none of the one below
int32 FORWARD_GO=1
