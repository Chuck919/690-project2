from controller import Robot, DistanceSensor, Motor, Camera

TIME_STEP = 64
MAX_SPEED = 6.28
THRESHOLD = 300.0

# ------------------ create robot ------------------
robot = Robot()

# ------------------ distance sensors ------------------
psNames = ['ps0','ps1','ps2','ps3','ps4','ps5','ps6','ps7']
ps = []
for name in psNames:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# ------------------ motors ------------------
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# ------------------ camera ------------------
camera = robot.getDevice('camera')
camera.enable(TIME_STEP)

# ------------------ helper functions ------------------
def stop():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    robot.step(TIME_STEP)

def turn_left_90():
    stop()
    leftMotor.setVelocity(-0.5 * MAX_SPEED)
    rightMotor.setVelocity(0.5 * MAX_SPEED)
    for _ in range(10):  # tune if needed
        robot.step(TIME_STEP)
    print("Turning Left")
    stop()

def turn_right_90():
    stop()
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    for _ in range(10):
        robot.step(TIME_STEP)
    print("Turning Right")
    stop()

def turn_180():
    stop()
    leftMotor.setVelocity(0.5 * MAX_SPEED)
    rightMotor.setVelocity(-0.5 * MAX_SPEED)
    for _ in range(22):
        robot.step(TIME_STEP)
    print("Turning Around")
    stop()

def sees_green():
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()

    r_total = g_total = b_total = 0
    count = width * height

    for x in range(width):
        for y in range(height):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)

            r_total += r
            g_total += g
            b_total += b

    r_avg = r_total / count
    g_avg = g_total / count
    b_avg = b_total / count

    # green dominates
    return g_avg > r_avg * 1.2 and g_avg > b_avg * 1.2

# ------------------ main loop ------------------
while robot.step(TIME_STEP) != -1:

    psValues = [sensor.getValue() for sensor in ps]
    
    # openness (lower = more open)
    left_space  = psValues[5]   # left = ps5
    right_space = psValues[2]   # right = ps2

    # front uses ps7 and ps0
    front_obstacle = psValues[7] > THRESHOLD or psValues[0] > THRESHOLD
    right_obstacle = right_space > THRESHOLD
    
    if front_obstacle:
        # Corner ahead or dead end. Turning left forces the obstacle to our right.
        # If dead end, turn left twice over two loops (180 degrees).
        turn_left_90()

    elif not right_obstacle:
        for _ in range(5):  
            robot.step(TIME_STEP)
            
        # The wall on the right ended; follow it around the corner
        turn_right_90()
        
        # Drive forward far enough to enter new corridor
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
        
        for _ in range(20):  
            robot.step(TIME_STEP)

    else:
        # Right wall exists, front is clear -> keep tracking the wall
        leftMotor.setVelocity(0.5 * MAX_SPEED)
        rightMotor.setVelocity(0.5 * MAX_SPEED)
