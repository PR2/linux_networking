int8 STATE_NO_INTERFACE = -1 # The interface does not exist.
int8 STATE_PLUGGED = 0       # The interface exists.
int8 STATE_UP = 1            # The interface is up.
int8 STATE_LINK = 2          # The interface has a link.
int8 STATE_LINK_ADDR = 3     # The interface has a link, and the link has an address.
int8 STATE_ADDR = 4          # The interface has an address.
int8 STATE_PINGING = 5       # Ping packets are making it through.

int8 state                   # The state (defined above).
float32 goodness             # How well this interface is sending packets.
float32 reliability          # How reliable this interface is expected to be.
float32 score                # The score use to rank interfaces.
float32 latency              # Latency of ping packets.
float32 loss                 # Loss of ping packets.
wpa_supplicant_node/Bss bss  # Current BSS (for a wireless network)
