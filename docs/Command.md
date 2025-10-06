# Commands: 



## Set to position: 

Available positions: 
- Park
- Ready
- Straight-Up

**ABNF:**<br>

```
set-position-command = ros2 <WS> service <WS> call <WS> service std_srvs/srv/Trigger"
service = "/" position
position = "park" / "ready" / "straight-up"
```

**Example:**<br>

- Park
`ros2 service call /park std_srvs/srv/Trigger`

- Ready
  

## Move Single Servo:

Servo numbers: 
0. Base
1. Shoulder
2. Elbow
3. Wrist 
4. Wrist Rotation
5. Gripper

**ABNF:**<br>

```
move-servo-command = "ros2 <WS> service <WS> call <WS> /move_servo <WS> arminator_driver/srv/MoveServo <WS> servo-args
servo-args         = "{" wsp "servo:" wsp servo-num "," wsp "angle:" wsp angle "," wsp "time:" wsp time wsp "}"
servo-num          = "0" / "1" / "2" / "3" / "4" / "5"  ; 0=Base, 1=Shoulder, etc.
angle              = int                               ; <angle in degrees>
time               = int                               ; <time in ms>
int                = 1*DIGIT
wsp                = *WSP                              ; zero or more whitespace
```

**Example:**<br>
`ros2 service call /move_servo arminator_driver/srv/MoveServo "{servo: 3, angle: 0, time: 1000}"`


## Multi Servo Command: 

Servo numbers: 
0. Base
1. Shoulder
2. Elbow
3. Wrist 
4. Wrist Rotation
5. Gripper
   

**ABNF:**<br>


**Example:**<br>