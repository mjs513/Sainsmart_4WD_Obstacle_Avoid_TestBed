
#define telem Serial

//sets up servo angles
const int head_fwd = 90; 
const int head_lft = 0;
const int head_rt = 180;
const int head_ldiag = 45;
const int head_rdiag = 135;

#define SONAR_NUM     4    			// Number or sensors.
#define MAX_DISTANCE 200   			// Maximum distance (in cm) to ping.
#define PING_INTERVAL 33   			// Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

// the interval in mS 
#define interval 7500
