import math
import time

import pygame

CW = 1
CCW = -1

class Robot:

    def __init__(self, debug=False):

        self.debug = debug

        self.deg2rad = (math.pi/180.)
        self.rad2deg = (180./math.pi)

        self.velocity = 1.
        self.angularVelocity = 2.*self.deg2rad
        self.angularDir = CW
        self.angle = 0

        # as measured as center of wheel axle
        self.x = 0
        self.y = 0  

        self.distTol = 2.
        self.angleTol = 1.*self.deg2rad

        # box robot dims, pixels
        self.width = 100
        self.height = 100

    def __str__(self):

        msg = "Position (pixels): %5.2f, %5.2f\n" % (self.x, self.y)
        msg += "Angle (rads): %5.2f" % self.angle
        return msg
        
    def isAtPosition(self, x, y):
        return self.getDistance(x, y) < self.distTol

    def isPointedAtPosition(self, x, y, update=True):
        pAngle = self.getAngleToPosition(x, y)
        if self.debug:
            print("robot vs. pointing angle:", self.angle, pAngle)
        isPointed = abs(pAngle - self.angle) < self.angleTol
        if isPointed and update:
            # might as well get us pointed exactly as we should
            self.angle = pAngle
        return isPointed
            
    def getDistance(self, x, y):

        return math.sqrt((self.x - x)**2 + (self.y - y)**2)    

    def isAtPosition(self, x, y):
        dist = self.getDistance(x, y)
        if self.debug:
            print("distance from ", x, y, dist)
        return dist < self.distTol

    def pointToPosition(self, x, y):

        angleToPos = self.getAngleToPosition()
        print("angleToPos:", angleToPos)

    def getAngleToPosition(self, x, y):

        # what is the angle between the two points?
        dx = self.x - x
        dy = self.y -y
        return math.tanh(dy/dx)

    def setAngularDir(self, x, y):

        pAngle = self.getAngleToPosition(x, y)
        if pAngle > self.angle:
            self.angularDir = CW
        else:
            self.angularDir = CCW

    def rotate(self, deltaSecs):
        deltaAngle = self.angularVelocity * deltaSecs * self.angularDir
        print("deltaAngle", deltaAngle)
        self.angle += deltaAngle

    def translate(self, deltaSecs):
        dx = self.velocity * math.cos(self.angle) * deltaSecs               
        dy = self.velocity * math.sin(self.angle) * deltaSecs
        print("delta pos", dx, dy) 
        self.x += dx
        self.y += dy             

    def move(self, x, y, deltaSecs):
        "Move the robot to given position - first rotate, then move"

        # are we there already?  If so, nothing to do
        if self.isAtPosition(x, y):
            if self.debug:
                print("Already at position; not moving")
            return

        # if self.debug:
        #     print("translating towards position")

        # self.translate(deltaSecs)

        # return
 
        # are we pointed in the right dir?
        # if not, that's what we do
        if self.isPointedAtPosition(x, y):
            # move along the straight line to the point
            if self.debug:
                print("translating towards position")
            self.translate(deltaSecs)
        else:
            # rotate so that we're pointing straight at it
            if self.debug:
                print("rotating to point to position")
            self.setAngularDir(x, y)
            self.rotate(deltaSecs)

def drawPaths(display, color, paths):

    if len(paths) < 2:
        return

    for i in range(len(paths) - 1):
        start = paths[i]
        end = paths[i+1]
        pygame.draw.line(display, color, start, end)
        print("line", start, end)

def rotCenter(image, rect, angle):
    """rotate an image while keeping its center"""
    rot_image = pygame.transform.rotate(image, angle)
    rot_rect = rot_image.get_rect(center=rect.center)
    return rot_image,rot_rect

def runGame(paths):

    print("runGame with paths:")
    for p in paths:
        print(p)

    deg2rad = (math.pi/180.)
    rad2deg = (180./math.pi)

    # table dimensions, cms
    tableWidth = 10*20
    tableHeight = 6*20
    tableRatio = tableHeight/tableWidth

    # game display in pixels
    displayWidth = 800
    # display_height = 600
    displayHeight = int(displayWidth * tableRatio) 
    print("display pixels", displayWidth, displayHeight)

    gameDisplay = pygame.display.set_mode((displayWidth,displayHeight))
    pygame.display.set_caption('EV3 python simulation')

    # colors
    black = (0,0,0)
    white = (255,255,255)
    # setup display with matt
    fn = "fllReplayMat2.jpg"
    matImg = pygame.image.load(fn)
    matImg = pygame.transform.scale(matImg, (displayWidth, displayHeight))


    # setup robot
    robot = Robot(debug=True)
    # place it in starting corner
    initX = robot.width / 2.
    initY = displayHeight - (robot.height / 2)
    robot.x = initX
    robot.y = initY

    # setup robot image
    fnRobot = "robotPic1.jpg"
    # robotW = robotH = 50 # pixels
    robotImg = pygame.image.load(fnRobot)
    robotImg = pygame.transform.scale(robotImg, (robot.width, robot.height))
    robotImgOrg = pygame.transform.scale(robotImg, (robot.width, robot.height))


    # paths = [
    #     (initX, initY),
    #     (displayWidth/4, int((3/4.)*displayHeight)),
    #     (displayWidth/2, int((3/4.)*displayHeight))
    # ]

    # for p in paths:
    #     print(p)

    currentPathIndex = 0

    # start game loop
    clock = pygame.time.Clock()
    tickTime = 1000. # msecs?
    running = True
    while running:

        # quit?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # base image is the matt
        gameDisplay.fill(white)
        gameDisplay.blit(matImg, (0, 0))
   
        # then draw paths
        drawPaths(gameDisplay, black, paths)

        # pygame.display.update()
        # continue

        # then draw robot:
        # update the robot's position
        # but what's it's next gaol?  time to go to the next?
        x, y = paths[currentPathIndex]
        if robot.isAtPosition(x, y):
            # it's there already, advace to next path point
            if currentPathIndex < len(paths) - 1:
                currentPathIndex += 1
                x, y = paths[currentPathIndex]

        robot.move(x, y, 1.)

        # get the robot image's rotation right
        # rotCenter(robotImg, robotImg.get_rect(), robot.angle)
        # robotImg = pygame.transform.rotate(robotImgOrg, 45.0)
        rotDegs = -1.0*rad2deg*robot.angle
        robotImg = pygame.transform.rotate(robotImgOrg, rotDegs)

        # robot's position is center of image,
        # so we must offset the robot image
        robotImgX = robot.x - (robot.width/2)
        robotImgY = robot.y - (robot.height/2)
        gameDisplay.blit(robotImg, (robotImgX, robotImgY))

        pygame.display.update()

        # TBF: need to understand this
        clock.tick(tickTime)
        time.sleep(.10)

def testRobot():
    # moveRobot()
    r = Robot(debug=True)
    x = 20
    y = 20
    rad2deg = (180./math.pi)
    print(r.getDistance(x, y))
    rads = r.getAngleToPosition(x, y)
    print(rads, rads*rad2deg)
    print("at pos?", r.isAtPosition(x, y))
    print("pointed?", r.isPointedAtPosition(x, y))

    print("Original Robot", r)
    # r.rotate(1.)

    while not r.isAtPosition(x, y):
        r.move(x, y, 1.)
        print("Now Robot", r)
        time.sleep(1)

    print("Robot arrived at position", x, y)
    print(r)    

def main():

    # Construct the paths for the robot
    # based off the robot and the table:

    # table dimensions, cms
    tableWidth = 10*20
    tableHeight = 6*20
    tableRatio = tableHeight/tableWidth
    # game display in pixels
    displayWidth = 800
    # display_height = 600
    displayHeight = int(displayWidth * tableRatio) 

    # setup robot
    robot = Robot(debug=True)
    # place it in starting corner
    initX = robot.width / 2.
    initY = displayHeight - (robot.height / 2)

    # Now we can setup our paths the robot will follow
    paths = [
        (initX, initY),
        (displayWidth/4, int((3/4.)*displayHeight)),
        (displayWidth/2, int((3/4.)*displayHeight))
    ]

    runGame(paths)

if __name__ == '__main__':
    main()
