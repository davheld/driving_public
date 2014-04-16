uint8 BLOCK_UPPER = 0
uint8 BLOCK_LOWER = 1
uint8 NUM_BEAMS   = 32

time stamp                       # Time this measurement was received by driver
uint8 block_id # Upper or lower block
uint16 encoder # Encoder angle in 1/100th degree units.  0-35999 
uint16[32] range # range information in ticks (multiply by 0.002 to get meters)
uint8[32] intensity