Header header

bool lock

float32 roll
float32 pitch
float32 yaw

int32 data_skips

float32 velocity_x
float32 velocity_y

float32 altitude

# Channel (beam) data

# Gain in dB from 6 to 66
int8 gain_a  # Port
int8 gain_b  # Stern
int8 gain_c  # Starboard
int8 gain_d  # Bow

bool lock_a
bool lock_b
bool lock_c
bool lock_d

float32 velocity_a
float32 velocity_b
float32 velocity_c
float32 velocity_d

float32 range_a
float32 range_b
float32 range_c
float32 range_d

