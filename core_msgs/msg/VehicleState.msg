Header header
#auto(true) or manual(false)
bool is_auto

#true if estop
bool estop

int16 gear
int16 brake

#the speed is calculated using the encoder value, m/s
float32 speed

#in degree
float32 steer

int64 encoder
int16 alive
