
## 
+ On the touchscreen interface
  1. Go to "A" 
  2. Set **AUTO CONTROL**
     - allows external control via CAN

```

sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0

roslaunch amiga_control control.launch

```

## NOTE: 
+ if the right buttons are triggered
   + the `/joy_teleop/joy` joy message should be changing
   + check the `/cmd_vel` twist message should be changing 
+ if not working, check `$(find amiga_control)/config/teleop_logitech.yaml` 
  to confirm that the correct buttons are configured


## TODO
1. find way to set **AUTO CONTROL** from the PC
   - should involve scanning messages sent when the touch screen
2. do all the `sudo` commands from `udev` rules
