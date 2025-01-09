/*
 * Instrument cluster simulator
 *
 * (c) 2014 Open Garages - Craig Smith <craig@theialabs.com>
 *
 * Forked version :
 *  - new GUI
 *  - Flexible CLI/Config Structure
 *
 */

#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include "lib.h"
#include "data.h"

// Define the data directory if not defined
#ifndef DATA_DIR
#define DATA_DIR "./data/"  // Needs trailing slash
#endif

// ICSim constants
#define SCREEN_WIDTH 862
#define SCREEN_HEIGHT 669
#define ROAD_VIEW_HEIGHT 334   // ~half of 669
#define ROAD_LEFT_BOUNDARY 450 // Adjust this value based on the road's left edge in pixels
#define ROAD_RIGHT_BOUNDARY 650 // Adjust this value based on the road's right edge in pixels



// Define other necessary macros if not defined
#ifndef CAN_LEFT_SIGNAL
#define CAN_LEFT_SIGNAL 0x01
#endif

#ifndef CAN_RIGHT_SIGNAL
#define CAN_RIGHT_SIGNAL 0x02
#endif

#ifndef CAN_DOOR1_LOCK
#define CAN_DOOR1_LOCK 0x01
#endif

#ifndef CAN_DOOR2_LOCK
#define CAN_DOOR2_LOCK 0x02
#endif

#ifndef CAN_DOOR3_LOCK
#define CAN_DOOR3_LOCK 0x04
#endif

#ifndef CAN_DOOR4_LOCK
#define CAN_DOOR4_LOCK 0x08
#endif

#ifndef DOOR_LOCKED
#define DOOR_LOCKED 0
#endif

#ifndef DOOR_UNLOCKED
#define DOOR_UNLOCKED 1
#endif

#ifndef OFF
#define OFF 0
#endif

#ifndef ON
#define ON 1
#endif

#ifndef LIGHT_LEVEL
#define LIGHT_LEVEL 50
#endif

#ifndef VIN
#define VIN "1HGCM82633A004352"
#endif

#include <time.h> // For random seed and spawning intervals

// Traffic car constants
#define TRAFFIC_CAR_WIDTH 70
#define TRAFFIC_CAR_HEIGHT 80
#define MAX_TRAFFIC_CARS 5
#define TRAFFIC_CAR_SPEED 3 // Speed of traffic cars
#define TRAFFIC_SPAWN_INTERVAL 15000 // Set to 15 seconds
// Traffic car structure
typedef struct {
    int x, y;
    SDL_Rect rect;
} TrafficCar;

// Traffic car global variables
TrafficCar trafficCars[MAX_TRAFFIC_CARS];
int trafficCarCount = 0;
Uint32 lastTrafficSpawnTime = 0; // Tracks the last spawn time for traffic





// ICSim globals
int can_socket; // socket
struct canfd_frame cf;
const int canfd_on = 1;
int debug = 0;
int randomize_flag = 0;
int seed = 0;
int currentTime;

int doorPos = DEFAULT_DOOR_POS;
int signalPos = DEFAULT_SIGNAL_POS;
int speedPos = DEFAULT_SPEED_POS;
int warningPos = DEFAULT_WARNING_POS;
int luminosityPos = DEFAULT_LUMINOSITY_POS;
int lightPos = DEFAULT_LIGHT_POS;

long currentSpeed = 0;
int doorStatus[4];
int turnStatus[2];
int luminosityLevel = 0;
char lightStatus = 0;
char *model = NULL;
char dataFile[256];

char warningState = 0;
char diagActive = 0;

char diagSession = 1;
int lastDiagTesterPresent = 0;
int diagSeed[2];
int sessionKey[2] = {0x35, 0x30}; // Based on the 2 last character of the VIN, ie 50
char seedGenerated = 0;
char secretSessionFound = 0;

int shareSeed = -1;
char controlLightOn = 0;
char controlIsNight = 0;
char controlWarningActive = 0;
char controlDiagOn = 0;
char controlDiagActive = 0;
char controlTurnValue = 0;
char controlLuminosity = 0;
char controlCurrentSpeed = 0;

char pristine = 1;
int lastUpdate = 0;
char isoTpRequest = 0;
int isoTpRemainingBytes = 0;
int isoTpFirstFlowTime = 0;

char score = 0;

SDL_Renderer *renderer = NULL;
SDL_Texture *baseTexture = NULL;
SDL_Texture *needleTex = NULL;
SDL_Texture *spriteTex = NULL;
SDL_Texture *spriteAltTex = NULL;
SDL_Texture *scoreboardTex = NULL;
SDL_Texture *trafficCarTexture = NULL; // Texture for traffic cars
int running_flag = 0;
static unsigned char doorState = 0x00;
FILE *fptr;

SDL_Texture *roadTexture = NULL;  // For the scrolling road background
SDL_Texture *carTexture = NULL;   // For the car sprite

// Variables to track car position and road scrolling
int carX = 390;         // Where we draw the car horizontally on the screen
int carY = 380;         // Where we draw the car vertically on the screen
int trackOffset = 0;    // How far the road is scrolled (vertical offset)
int turnSpeed = 2;      // How fast the car shifts horizontally
int trackScrollSpeed = 2; // Base rate for background scrolling

/* SimConfig Structure */
typedef struct {
    int multipleECUs;
    int gatewayNode;
    int messageAuth;
    int firmwareUpdate;
    int canFDSupport;
    int intrusionDetection;
} SimConfig;

/* Function Prototypes */
void print_pkt(struct canfd_frame);
void print_bin(unsigned char *, int);

// Simple map function
long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Function to spawn a traffic car
// Function to spawn a traffic car
void spawnTrafficCar() {
    if (trafficCarCount >= MAX_TRAFFIC_CARS) return;

    // Define boundaries for the left lane
    int leftLaneStart = ROAD_LEFT_BOUNDARY;  // Left lane start
    int leftLaneEnd = leftLaneStart + TRAFFIC_CAR_WIDTH; // Left lane width

    int minimumDistance = 200; // Minimum vertical distance between cars
    TrafficCar newCar;

    // Spawn above the visible area
    newCar.x = leftLaneStart;
    newCar.y = -TRAFFIC_CAR_HEIGHT;

    // Ensure the new car does not violate the minimum distance from the nearest car
    for (int i = 0; i < trafficCarCount; i++) {
        if (trafficCars[i].x == leftLaneStart && 
            abs(newCar.y - trafficCars[i].y) < minimumDistance) {
            return; // Do not spawn if it violates the minimum distance
        }
    }

    // Add the new car to the array
    newCar.rect = (SDL_Rect){newCar.x, newCar.y, TRAFFIC_CAR_WIDTH, TRAFFIC_CAR_HEIGHT};
    trafficCars[trafficCarCount++] = newCar;
}



// Function to update traffic car positions
void updateTrafficCars() {
    // Define boundaries for the left lane
    int leftLaneStart = ROAD_LEFT_BOUNDARY;  // Left lane start
    int leftLaneEnd = ROAD_LEFT_BOUNDARY - TRAFFIC_CAR_WIDTH; // Left lane end

    for (int i = 0; i < trafficCarCount; i++) {
        trafficCars[i].y += TRAFFIC_CAR_SPEED;
        trafficCars[i].rect.y = trafficCars[i].y;

        // If a car moves off-screen (below the visible road area), respawn it in the left lane
        if (trafficCars[i].y > SCREEN_HEIGHT) {
            trafficCars[i].x = leftLaneStart - 50 + rand() % (leftLaneEnd - leftLaneStart);
            trafficCars[i].y = -TRAFFIC_CAR_HEIGHT;
            trafficCars[i].rect.x = trafficCars[i].x;
            trafficCars[i].rect.y = trafficCars[i].y;
        }
    }
}

// Function to draw traffic cars
void drawTrafficCars() {
    for (int i = 0; i < trafficCarCount; i++) {
        SDL_RenderCopy(renderer, trafficCarTexture, NULL, &trafficCars[i].rect); // Use trafficCarTexture for traffic cars
    }
}

// Function to detect collisions between the player car and traffic
int checkCollision(SDL_Rect *playerCar, SDL_Rect *trafficCar) {
    return SDL_HasIntersection(playerCar, trafficCar);
}



// Adds data dir to file name
// Uses a single pointer so not to have a memory leak
// returns point to dataFiles or NULL if append is too large
char *getData(char *fname) {
  if(strlen(DATA_DIR) + strlen(fname) > 255) return NULL;
  strncpy(dataFile, DATA_DIR, 255);
  strncat(dataFile, fname, 255-strlen(dataFile));
  return dataFile;
}

void drawRoadAndCar() {
    // Get the dimensions of the road texture
    int roadTexWidth, roadTexHeight;
    SDL_QueryTexture(roadTexture, NULL, NULL, &roadTexWidth, &roadTexHeight);

    // Calculate the vertical offset for the road texture
    int offset = trackOffset % roadTexHeight;

    SDL_Rect src, dst;

    // First part of the road (from offset to the bottom of the texture)
    src.x = 0;
    src.y = roadTexHeight - offset;
    src.w = SCREEN_WIDTH;
    src.h = offset;

    dst.x = 0;
    dst.y = 0;
    dst.w = SCREEN_WIDTH;
    dst.h = offset;

    if (offset > 0) {
        SDL_RenderCopy(renderer, roadTexture, &src, &dst);
    }

    // Second part of the road (wrapping from the top of the texture)
    src.x = 0;
    src.y = 0;
    src.w = SCREEN_WIDTH;
    src.h = ROAD_VIEW_HEIGHT - offset;

    dst.x = 0;
    dst.y = offset;
    dst.w = SCREEN_WIDTH;
    dst.h = ROAD_VIEW_HEIGHT - offset;

    if (ROAD_VIEW_HEIGHT - offset > 0) {
        SDL_RenderCopy(renderer, roadTexture, &src, &dst);
    }

    // Draw the car sprite in the upper half
    SDL_Rect carRect;
    carRect.w = 70; // Car width
    carRect.h = 80; // Car height

    // Restrict the car's position within the road boundaries (horizontal)
    if (carX < ROAD_LEFT_BOUNDARY) carX = ROAD_LEFT_BOUNDARY;
    if (carX > ROAD_RIGHT_BOUNDARY - carRect.w) carX = ROAD_RIGHT_BOUNDARY - carRect.w;

    // Restrict the car's position within the upper half (vertical)
    int upperMargin = 20; // Add some margin for appearance
    if (carY < upperMargin) carY = upperMargin;
    if (carY > ROAD_VIEW_HEIGHT - carRect.h - upperMargin) carY = ROAD_VIEW_HEIGHT - carRect.h - upperMargin;

    // Place the car within the updated boundaries
    carRect.x = carX;
    carRect.y = carY;

    SDL_RenderCopy(renderer, carTexture, NULL, &carRect);

    // Ensure the lower half of the screen has a black background
    SDL_Rect lowerHalf;
    lowerHalf.x = 0;
    lowerHalf.y = ROAD_VIEW_HEIGHT;
    lowerHalf.w = SCREEN_WIDTH;
    lowerHalf.h = SCREEN_HEIGHT - ROAD_VIEW_HEIGHT;

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
    SDL_RenderFillRect(renderer, &lowerHalf);
}

void sendPkt(int mtu) {
  if(write(can_socket, &cf, mtu) != mtu) {
      perror("write");
  }
}

/* Default vehicle state */
void initCarState() {
  doorStatus[0] = DOOR_LOCKED;
  doorStatus[1] = DOOR_LOCKED;
  doorStatus[2] = DOOR_LOCKED;
  doorStatus[3] = DOOR_LOCKED;
  turnStatus[0] = OFF;
  turnStatus[1] = OFF;
  diagActive = 0;
  diagSession = 1;
  seedGenerated = 0;
  secretSessionFound = 0;
  warningState = 0;
}

/* Empty IC */
void blankIC() {
  SDL_RenderCopy(renderer, baseTexture, NULL, NULL);
}

void validateChallenge(int challenge) {
  if (challenges[challenge] == 0) {
    challenges[challenge] = 1;
    score += challengeValue[challenge];
  }
}

void updateScore(int chall) {
  if (chall >= sizeof(challenges) || chall < 0) {
    printf("Error : challenge ID is out of range !");
    exit(42);
  }

  if (challenges[chall] == 0) {
    score += challengeValue[chall];
    challenges[chall] = 1;
    if (score > 100) score = 100;
  }
}

/* Draw functions */
void drawScore() {
  SDL_Rect scoreRect, scoreSrc;
  scoreSrc.x = 0;
  scoreSrc.y = (score/5) * 40;
  scoreSrc.h = 40;
  scoreSrc.w = 68;

  scoreRect.x = 50;
  scoreRect.y = 389;
  scoreRect.h = 40;
  scoreRect.w = 68;

  SDL_RenderCopy(renderer, scoreboardTex, &scoreSrc, &scoreRect);
}

void drawDashboardAndSpeedometer() {
    // Render the lower dashboard region
    SDL_Rect dashboardRect;
    dashboardRect.x = 0;
    dashboardRect.y = ROAD_VIEW_HEIGHT;  // Start rendering from the bottom half
    dashboardRect.w = SCREEN_WIDTH;
    dashboardRect.h = SCREEN_HEIGHT - ROAD_VIEW_HEIGHT; // Height of the dashboard

    // Draw the dashboard
    SDL_RenderCopy(renderer, baseTexture, NULL, &dashboardRect);

    // Speedometer setup
    SDL_Rect dialRect;
    SDL_Point center;
    double angle = 0;

    dialRect.x = 281;  // Center x-coordinate of speedometer
    dialRect.y = ROAD_VIEW_HEIGHT + 50;  // Adjust y-coordinate relative to ROAD_VIEW_HEIGHT
    dialRect.h = 250;
    dialRect.w = 250;

    center.x = 125;  // Needle rotation center
    center.y = 125;
    angle = map_long(currentSpeed, 0, 230, -50, 200); // Map speed to dial angle
    if (angle > 200) angle = 200;

    // Render speedometer background
    SDL_RenderCopy(renderer, baseTexture, NULL, &dashboardRect);

    // Render speedometer needle
    SDL_RenderCopyEx(renderer, needleTex, NULL, &dialRect, angle, &center, SDL_FLIP_NONE);
}

void drawDiag() {
    SDL_Rect diagScreen, diagStatus, diagFeedback;
    diagScreen.x = 610;
    diagScreen.y = 551;
    diagScreen.w = 162;
    diagScreen.h = 102;

    diagStatus.x = 51;
    diagStatus.y = 461;
    diagStatus.w = 161;
    diagStatus.h = 19;

    diagFeedback.x = 33;
    diagFeedback.y = 483;
    diagFeedback.w = 204;
    diagFeedback.h = 152;

    if (controlDiagOn == 0) {
      SDL_RenderCopy(renderer, baseTexture, &diagScreen, &diagScreen);
    } else {
      if (controlDiagActive == 0) {
        SDL_RenderCopy(renderer, spriteTex, &diagScreen, &diagScreen);
      } else {
        SDL_RenderCopy(renderer, spriteAltTex, &diagScreen, &diagScreen);
      }
    }

    if (diagSession == 2 || diagSession == 3)
      SDL_RenderCopy(renderer, spriteTex, &diagStatus, &diagStatus);
    else
      SDL_RenderCopy(renderer, baseTexture, &diagStatus, &diagStatus);

    if (diagActive == 2 || secretSessionFound == 1) {
      SDL_RenderCopy(renderer, spriteAltTex, &diagFeedback, &diagFeedback);

    } else if (diagActive == 1)
      SDL_RenderCopy(renderer, spriteTex, &diagFeedback, &diagFeedback);
    else
      SDL_RenderCopy(renderer, baseTexture, &diagFeedback, &diagFeedback);
}

void drawRoadAndLights() {
  SDL_Rect sky, road, light, autoIndicator, lightDebug;
  sky.x = 0;
  sky.y = 0;
  sky.w = 862;
  sky.h = 162;

  road.x = 0;
  road.y = 162;
  road.w = 862;
  road.h = 160;

  light.x = 171;
  light.y = 382;
  light.w = 40;
  light.h = 24;

  autoIndicator.x = 160;
  autoIndicator.y = 410;
  autoIndicator.w = 57;
  autoIndicator.h = 17;

  lightDebug.x = 685;
  lightDebug.y = 396;
  lightDebug.w = 61;
  lightDebug.h = 35;

  if (controlIsNight == 0) {
    SDL_RenderCopy(renderer, baseTexture, &sky, &sky);
    SDL_RenderCopy(renderer, baseTexture, &road, &road);
  } else {
    SDL_RenderCopy(renderer, spriteTex, &sky, &sky);
    if (controlLightOn == 1 || luminosityLevel < LIGHT_LEVEL) {
      SDL_RenderCopy(renderer, spriteAltTex, &road, &road);
    } else {
      SDL_RenderCopy(renderer, spriteTex, &road, &road);
    }
  }

  if (controlLightOn == 1 ||  luminosityLevel < LIGHT_LEVEL) {
    SDL_RenderCopy(renderer, spriteTex, &light, &light);
    SDL_RenderCopy(renderer, spriteAltTex, &lightDebug, &lightDebug);
    if (luminosityLevel < LIGHT_LEVEL)
      SDL_RenderCopy(renderer, spriteTex, &autoIndicator, &autoIndicator);
    else
      SDL_RenderCopy(renderer, baseTexture, &autoIndicator, &autoIndicator);
  } else {
    SDL_RenderCopy(renderer, baseTexture, &lightDebug, &lightDebug);
    SDL_RenderCopy(renderer, baseTexture, &light, &light);
    SDL_RenderCopy(renderer, baseTexture, &autoIndicator, &autoIndicator);
  }
}

/* Updates door unlocks simulated by door open icons */
void drawDoors() {
    SDL_Rect door_area, update;
    door_area.x = 674;
    door_area.y = 432;
    door_area.w = 81;
    door_area.h = 78;
    SDL_RenderCopy(renderer, baseTexture, &door_area, &door_area);

    // If any door is unlocked, update the base with the red body sprite
    if (doorStatus[0] == DOOR_UNLOCKED || doorStatus[1] == DOOR_UNLOCKED ||
        doorStatus[2] == DOOR_UNLOCKED || doorStatus[3] == DOOR_UNLOCKED) {
        update.x = 693;
        update.y = 432;
        update.w = 43;
        update.h = 78;
        SDL_RenderCopy(renderer, spriteTex, &update, &update);
    }

    // Draw individual door lock/unlock icons
    if (doorStatus[0] == DOOR_UNLOCKED) {
        update.x = 678;
        update.y = 456;
        update.w = 18;
        update.h = 17;
        SDL_RenderCopy(renderer, spriteTex, &update, &update);
    }
    if (doorStatus[1] == DOOR_UNLOCKED) {
        update.x = 738;
        update.y = 456;
        update.w = 18;
        update.h = 18;
        SDL_RenderCopy(renderer, spriteTex, &update, &update);
    }
    if (doorStatus[2] == DOOR_UNLOCKED) {
        update.x = 678;
        update.y = 481;
        update.w = 18;
        update.h = 18;
        SDL_RenderCopy(renderer, spriteTex, &update, &update);
    }
    if (doorStatus[3] == DOOR_UNLOCKED) {
        update.x = 738;
        update.y = 481;
        update.w = 18;
        update.h = 18;
        SDL_RenderCopy(renderer, spriteTex, &update, &update);
    }

}

/* Updates turn signals */
void drawTurnSignals() {
  SDL_Rect left, right, leftTU, leftTD, rightTU, rightTD, warning;
  left.x = 242;
  left.y = 378;
  left.w = 50;
  left.h = 31;

  right.x = 528;
  right.y = 378;
  right.w = 50;
  right.h = 31;

  leftTU.x = 678;
  leftTU.y = 422;
  leftTU.w = 16;
  leftTU.h = 16;

  leftTD.x = 678;
  leftTD.y = 506;
  leftTD.w = 16;
  leftTD.h = 16;

  rightTU.x = 734;
  rightTU.y = 422;
  rightTU.w = 16;
  rightTU.h = 16;

  rightTD.x = 739;
  rightTD.y = 506;
  rightTD.w = 16;
  rightTD.h = 16;

  warning.x = 545;
  warning.y = 418;
  warning.h = 39;
  warning.w = 52;

  if (diagActive == 2) {
    if (currentTime % 1000 >= 500) {
      turnStatus[0] = OFF;
      turnStatus[1] = OFF;
      controlTurnValue = 0;
    } else {
      turnStatus[0] = ON;
      turnStatus[1] = ON;
      controlTurnValue = 3;
    }
  }

  if (turnStatus[0] == OFF) {
      SDL_RenderCopy(renderer, baseTexture, &left, &left);
  } else {
      SDL_RenderCopy(renderer, spriteTex, &left, &left);
  }

  if(turnStatus[1] == OFF) {
      SDL_RenderCopy(renderer, baseTexture, &right, &right);
  } else {
      SDL_RenderCopy(renderer, spriteTex, &right, &right);
  }

  if (controlTurnValue == 1 || controlTurnValue == 3) {
    SDL_RenderCopy(renderer, spriteTex, &leftTU, &leftTU);
    SDL_RenderCopy(renderer, spriteTex, &leftTD, &leftTD);
  }
  else {
    SDL_RenderCopy(renderer, baseTexture, &leftTU, &leftTU);
    SDL_RenderCopy(renderer, baseTexture, &leftTD, &leftTD);
  }

  if (controlTurnValue == 2 || controlTurnValue == 3) {
    SDL_RenderCopy(renderer, spriteTex, &rightTU, &rightTU);
    SDL_RenderCopy(renderer, spriteTex, &rightTD, &rightTD);
  }
  else {
    SDL_RenderCopy(renderer, baseTexture, &rightTU, &rightTU);
    SDL_RenderCopy(renderer, baseTexture, &rightTD, &rightTD);
  }

  if (warningState == 1)
    SDL_RenderCopy(renderer, spriteTex, &warning, &warning);
  else
    SDL_RenderCopy(renderer, baseTexture, &warning, &warning);
}

/* Redraws the IC updating everything
 * Slowest way to go.  Should only use on init
 */
void updateIC() {
    // Create a target texture for double buffering
    SDL_Texture *offscreenTex = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Set the renderer target to the offscreen texture
    SDL_SetRenderTarget(renderer, offscreenTex);

    // Clear the entire offscreen render target
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); // Black background
    SDL_RenderClear(renderer);

    // Draw all components onto the offscreen texture
    drawRoadAndCar();
    drawTrafficCars(); // Draw traffic cars
    drawDashboardAndSpeedometer();
    drawDoors();
    drawTurnSignals();
    drawDiag();

    drawScore();

    // Reset the target back to the default render target
    SDL_SetRenderTarget(renderer, NULL);

    // Copy the offscreen texture to the screen renderer
    SDL_RenderCopy(renderer, offscreenTex, NULL, NULL);

    // Present the final frame to the screen
    SDL_RenderPresent(renderer);

    // Clean up the offscreen texture
    SDL_DestroyTexture(offscreenTex);
}


void redrawIC() {
  blankIC();
  updateIC();
}

/* Parse can frames and update variables */
void updateSharedData(struct canfd_frame *cf, int maxdlen) {
    int seed_val = (cf->data[5] << 8) | cf->data[6];

    if (shareSeed == -1 || seed_val == ((shareSeed +1)%65536)) {
        char key1 = cf->data[5];
        char key2 = cf->data[6];

        for (char i = 0; i <= 4; i++) {
            cf->data[i] ^= key2;
        }
        int crc = cf->data[3];
        int check = cf->data[4];

        if (check == (crc ^ key1)) {
            if (((cf->data[0] + cf->data[1] + cf->data[2]) % 256) == crc) {
                shareSeed = seed_val;

                controlLightOn = cf->data[0] >> 6;
                controlIsNight = (cf->data[0] >> 5) & 0x1;
                controlWarningActive = (cf->data[0] >> 4) & 0x1;
                controlDiagOn = (cf->data[0] >> 3) & 0x1;
                controlDiagActive = (cf->data[0] >> 2) & 0x1;
                controlTurnValue = cf->data[0] & 0x3;
                controlLuminosity = cf->data[1];
                controlCurrentSpeed = cf->data[2];

                // Debug statements
                if (debug) {
                    printf("[Debug] Shared Data Updated:\n");
                    printf("  Light On: %d\n", controlLightOn);
                    printf("  Is Night: %d\n", controlIsNight);
                    printf("  Warning Active: %d\n", controlWarningActive);
                    printf("  Diagnostic On: %d\n", controlDiagOn);
                    printf("  Diagnostic Active: %d\n", controlDiagActive);
                    printf("  Turn Value: %d\n", controlTurnValue);
                    printf("  Luminosity: %d\n", controlLuminosity);
                    printf("  Current Speed: %d\n", controlCurrentSpeed);
                }
            }
        }
    }
}


void updateLuminosityStatus(struct canfd_frame *cf, int maxdlen) {
  int len = (cf->len > maxdlen) ? maxdlen : cf->len;
  if (len < luminosityPos + 1) return;
  pristine = 0;
  luminosityLevel = cf->data[luminosityPos];
  // CHALLENGE CHECK : cut light by night
  if (controlIsNight == 1 && luminosityLevel > LIGHT_LEVEL)
    validateChallenge(CHALLENGE_SPOOF_LIGHT);
}

void drawSpeedStatus(struct canfd_frame *cf, int maxdlen) {
    int len = (cf->len > maxdlen) ? maxdlen : cf->len;
    if(len < speedPos + 2) return; // Ensures data[speedPos +1] is valid
    pristine = 0;

    int speed = (cf->data[speedPos] << 8) | cf->data[speedPos + 1];
    speed = speed / 100; // Assuming the CAN frame sends speed * 100
    currentSpeed = speed;

    // Debug statement
    if (debug) {
        printf("[Debug] Updated Speed: %ld km/h\n", currentSpeed);
    }

    // CHALLENGE CHECK :  spoof speed on IC
    if (currentSpeed >= MAX_SPEED)
        validateChallenge(CHALLENGE_SPOOF_SPEED);
}


void updateWarningStatus(struct canfd_frame *cf, int maxdlen) {
  int len = (cf->len > maxdlen) ? maxdlen : cf->len;
  if (len < warningPos + 1) return;
  pristine = 0;
  warningState = cf->data[warningPos] & 0x01;
}

void updateLightStatus(struct canfd_frame *cf, int maxdlen) {
  int len = (cf->len > maxdlen) ? maxdlen : cf->len;
  if (len < lightPos + 1) return;
  pristine = 0;
  lightStatus = cf->data[lightPos] & 0x01;
}

/* Parses CAN frame and updates turn signal status */
void updateSignalStatus(struct canfd_frame *cf, int maxdlen) {
    int len = (cf->len > maxdlen) ? maxdlen : cf->len;
    if(len <= signalPos) return;  // Changed from < to <=
    pristine = 0;
    if (cf->data[signalPos] & CAN_LEFT_SIGNAL) {
        turnStatus[0] = ON;
    } else {
        turnStatus[0] = OFF;
    }
    if(cf->data[signalPos] & CAN_RIGHT_SIGNAL) {
        turnStatus[1] = ON;
    } else {
        turnStatus[1] = OFF;
    }
    // CHALLENGE CHECK :  spoof turn signals on IC
    if ((turnStatus[1] == ON || turnStatus[0] == ON) && (controlWarningActive ==0 && controlTurnValue == 0))
        validateChallenge(CHALLENGE_TURN_SIGNALS);
}

/**
 * ICSim now uses this helper to send door lock/unlock commands to ID=0x123.
 * The actual lock/unlock logic is done by an external BCM.
 */
void sendDoorCommand(int lockOrUnlock) {
    struct can_frame tx;
    memset(&tx, 0, sizeof(tx));
    tx.can_id  = 0x123;  // Must match the BCM_CMD_ID in bcm.c
    tx.can_dlc = 1;
    tx.data[0] = (lockOrUnlock) ? 1 : 0; // 1=Lock, 0=Unlock

    if (write(can_socket, &tx, sizeof(tx)) < 0) {
        perror("[ICSim] sendDoorCommand");
    } else {
        printf("[ICSim] Sent door command (0x123) => %s\n",
               lockOrUnlock ? "LOCK" : "UNLOCK");
    }
}


void sendLock(char doorBit)
{
    // doorBit = CAN_DOOR1_LOCK (0x01), or CAN_DOOR2_LOCK (0x02), etc.
    doorState |= doorBit;  // set that bit => locked

    struct can_frame tx;
    memset(&tx, 0, sizeof(tx));
    tx.can_id  = 0x123;   // BCM command
    tx.can_dlc = 1;
    tx.data[0] = doorState;

    if (write(can_socket, &tx, sizeof(tx)) < 0) {
        perror("[ICSim] sendLock");
    } else {
        printf("[ICSim] Sent LOCK for bitmask=0x%02X to ID=0x123\n", doorState);
    }
}

void sendUnlock(char doorBit)
{
    doorState &= ~doorBit; // clear that bit => unlocked

    struct can_frame tx;
    memset(&tx, 0, sizeof(tx));
    tx.can_id  = 0x123;
    tx.can_dlc = 1;
    tx.data[0] = doorState;

    if (write(can_socket, &tx, sizeof(tx)) < 0) {
        perror("[ICSim] sendUnlock");
    } else {
        printf("[ICSim] Sent UNLOCK for bitmask=0x%02X to ID=0x123\n", doorState);
    }
}

/* Parses CAN frame and updates door status */
/* Parses CAN frame and updates door status */
void updateDoorStatus(struct canfd_frame *cf, int maxdlen) {
    // Ensure valid data length
    if (cf->len < 1) {  // BCM sends only 1 byte
        printf("[ICSim Debug] Invalid CAN frame length: %d\n", cf->len);
        return;
    }

    // Get the door state bitmask
    unsigned char bits = cf->data[0];  // Only one byte is sent
    printf("[ICSim Debug] Received door bitmask: 0x%02X\n", bits);

    // Update door states based on bitmask
    doorStatus[0] = (bits & CAN_DOOR1_LOCK) ? DOOR_UNLOCKED : DOOR_LOCKED;
    doorStatus[1] = (bits & CAN_DOOR2_LOCK) ? DOOR_UNLOCKED : DOOR_LOCKED;
    doorStatus[2] = (bits & CAN_DOOR3_LOCK) ? DOOR_UNLOCKED : DOOR_LOCKED;
    doorStatus[3] = (bits & CAN_DOOR4_LOCK) ? DOOR_UNLOCKED : DOOR_LOCKED;

    printf("[ICSim Debug] Door States Updated: %d %d %d %d\n",
           doorStatus[0], doorStatus[1], doorStatus[2], doorStatus[3]);

    // Update the GUI
    updateIC();
}





void sendFrameError(int func, int errorCode) {
  memset(&cf, 0, sizeof(cf));
  cf.can_id = 0x7E8;
  cf.len = 4;
  cf.data[0] = 0x03;
  cf.data[1] = 0x7F;
  cf.data[2] = func;
  cf.data[3] = errorCode;
  sendPkt(CAN_MTU);
}

void sendFrameFeedback(int *data, int frameLen) {
  memset(&cf, 0, sizeof(cf));
  cf.can_id = 0x7E8;
  cf.len = frameLen;
  for (char i = 0; i < frameLen; i++) {
    cf.data[i] = data[i];
  }
  sendPkt(CAN_MTU);
}

void sendIsoTpData() {
  int frameFeedback[8];
  char i = 1;
  frameFeedback[0] = 0x20 + ((isoTpRequest - 1)%16);

  for (i; i < 8 && isoTpRemainingBytes > 0; i++) {
    frameFeedback[i] = VIN[sizeof(VIN) - isoTpRemainingBytes];
    isoTpRemainingBytes --;
  }

  if (isoTpRemainingBytes <= 0) {
    // CHALLENGE CHECK :  VIN request
    validateChallenge(CHALLENGE_REQUEST_VIN);
    isoTpRequest = 0;
  } else
    isoTpRequest += 1;
  sendFrameFeedback(frameFeedback, i);
}

void analyseDiagRequest(struct canfd_frame *frame, int maxdlen) {
  int frameFeedback[8];
  int len = (frame->len > maxdlen) ? maxdlen : frame->len;

  if (frame->data[0] < 0x09 && frame->data[0] > 0) {
    int subf = frame->data[2];

    switch (frame->data[1]) {
      case PID_INFO:
        if (frame->data[0] != 2) {
          sendFrameError(PID_INFO, UDS_ERROR_INCORRECT_LENGTH);
          return;
        }
        if (frame->data[2] == PID_INFO_VEHICLE_SPEED) {
          frameFeedback[0] = 0x03;
          frameFeedback[1] = PID_INFO + 0x40;
          frameFeedback[2] = PID_INFO_VEHICLE_SPEED;
          frameFeedback[3] = controlCurrentSpeed;
          sendFrameFeedback(frameFeedback, 4);
        }
        else
          sendFrameError(PID_INFO, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
        break;
      case PID_VEHICLE_INFO:
        if (frame->data[0] != 2) {
          sendFrameError(PID_INFO, UDS_ERROR_INCORRECT_LENGTH);
          return;
        }
        switch (frame->data[2]) {
          case PID_VEHICLE_ECU_NAME:
            frameFeedback[0] = 0x03;
            frameFeedback[1] = PID_VEHICLE_INFO + 0x40;
            frameFeedback[2] = PID_VEHICLE_ECU_NAME;
            frameFeedback[3] = 69;
            frameFeedback[4] = 67;
            frameFeedback[5] = 85;
            sendFrameFeedback(frameFeedback, 6);
            break;

          case PID_VEHICLE_VIN:
            isoTpRemainingBytes = sizeof(VIN);

            if (isoTpRemainingBytes > 6) {
              frameFeedback[0] = 0x10 + ((isoTpRemainingBytes & 0x0F00) >> 8);
              frameFeedback[1] = isoTpRemainingBytes & 0xFF;
              isoTpFirstFlowTime = currentTime;
            }
            else {
              frameFeedback[0] = isoTpRemainingBytes + 1;
              frameFeedback[1] = PID_VEHICLE_INFO + 0x40;
            }
            char i = 2;
            for (i; i < 8 && isoTpRemainingBytes > 0; i++) {
              frameFeedback[i] = VIN[sizeof(VIN) - isoTpRemainingBytes];
              isoTpRemainingBytes --;
            }

          sendFrameFeedback(frameFeedback, i);

            break;
          default:
            sendFrameError(PID_INFO, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
            break;
        }
        break;

      case UDS_SID_TESTER_PRESENT:
        if (frame->data[0] == 0x02) {
          lastDiagTesterPresent = currentTime;
          frameFeedback[0] = 0x02;
          frameFeedback[1] = UDS_SID_TESTER_PRESENT + 0x40;
          frameFeedback[2] = subf;
          sendFrameFeedback(frameFeedback, 3);
        } else {
          sendFrameError(UDS_SID_TESTER_PRESENT, UDS_ERROR_INCORRECT_LENGTH);
        }
        break;

        case UDS_SID_DIAGNOSTIC_CONTROL:
          if (frame->data[0] == 0x02) {
            switch (frame->data[2]) {
              case 0x01:
                diagSession = 1;
                diagActive = 0;
                secretSessionFound = 0;
                lastDiagTesterPresent = currentTime;
                frameFeedback[0] = 0x02;
                frameFeedback[1] = UDS_SID_DIAGNOSTIC_CONTROL + 0x40;
                frameFeedback[2] = subf;
                sendFrameFeedback(frameFeedback, 3);
                break;
              case 0x02:
                diagSession = 2;
                lastDiagTesterPresent = currentTime;
                secretSessionFound = 0;
                frameFeedback[0] = 0x02;
                frameFeedback[1] = UDS_SID_DIAGNOSTIC_CONTROL + 0x40;
                frameFeedback[2] = subf;
                sendFrameFeedback(frameFeedback, 3);
                break;
              case 0x03:
                diagSession = 3;
                lastDiagTesterPresent = currentTime;
                secretSessionFound = 0;
                frameFeedback[0] = 0x02;
                frameFeedback[1] = UDS_SID_DIAGNOSTIC_CONTROL + 0x40;
                frameFeedback[2] = subf;
                sendFrameFeedback(frameFeedback, 3);
                break;
              default:
                sendFrameError(UDS_SID_DIAGNOSTIC_CONTROL, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
                return;
                break;

              frameFeedback[0] = 0x02;
              frameFeedback[1] = UDS_SID_DIAGNOSTIC_CONTROL + 0x40;
              frameFeedback[2] = subf;
              sendFrameFeedback(frameFeedback, 3);
            }
          } else {
            sendFrameError(UDS_SID_DIAGNOSTIC_CONTROL, UDS_ERROR_INCORRECT_LENGTH);
          }
          break;

        case UDS_SID_ECU_RESET:
          if (frame->data[0] == 2) {
            if (frame->data[2] > 0 && frame->data[2] <= 3) {
              diagSession = 1;
              seedGenerated = 0;
              diagActive = 0;
              secretSessionFound = 0;
              frameFeedback[0] = 0x02;
              frameFeedback[1] = UDS_SID_ECU_RESET + 0x40;
              frameFeedback[2] = subf;
              sendFrameFeedback(frameFeedback, 3);
            }
            else
              sendFrameError(UDS_SID_ECU_RESET, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
          } else
            sendFrameError(UDS_SID_ECU_RESET, UDS_ERROR_INCORRECT_LENGTH);
          break;

        case UDS_SID_ROUTINE_CONTROL:
          if (frame->data[0] == 4) {
            if (diagSession == 2) {
              if (frame->data[2] == 0x41) {
                if (frame->data[3] == 0x10 || frame->data[3] == 0x22) {
                  if (frame->data[4] == 0x00 || frame->data[4] == 0x01) {
                    switch (frame->data[3]) {
                      case 0x10:
                        diagActive = frame->data[4];
                        break;
                      case 0x22:
                        diagActive = frame->data[4] * 2;
                        validateChallenge(CHALLENGE_FIND_ROUTINE_CONTROL);
                        break;
                    }
                    frameFeedback[0] = 0x04;
                    frameFeedback[1] = UDS_SID_ROUTINE_CONTROL + 0x40;
                    frameFeedback[2] = subf;
                    frameFeedback[3] = frame->data[3];
                    frameFeedback[4] = frame->data[4];
                    sendFrameFeedback(frameFeedback, 5);
                  } else
                      sendFrameError(UDS_SID_ROUTINE_CONTROL, UDS_ERROR_REQUEST_OUT_RANGE);
                } else
                    sendFrameError(UDS_SID_ROUTINE_CONTROL, UDS_ERROR_REQUEST_OUT_RANGE);
              } else
                  sendFrameError(UDS_SID_ROUTINE_CONTROL, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
            } else
                sendFrameError(UDS_SID_ROUTINE_CONTROL, UDS_ERROR_FUNC_INCORRECT_SESSION);
          } else
              sendFrameError(UDS_SID_ROUTINE_CONTROL, UDS_ERROR_INCORRECT_LENGTH);
          break;

        case UDS_SID_SECURITY_ACCESS:
          if (diagSession != 0x03) {
            sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_FUNC_INCORRECT_SESSION);
            return;
          }

          switch (frame->data[2]) {
            case 0x01:
              if (frame->data[0] != 2) {
                sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_INCORRECT_LENGTH);
                return;
              }
              frameFeedback[0] = 0x04;
              frameFeedback[1] = UDS_SID_SECURITY_ACCESS + 0x40;
              frameFeedback[2] = subf;
              diagSeed[0] = rand()%255;
              diagSeed[1] = rand()%255;
              frameFeedback[3] = diagSeed[0];
              frameFeedback[4] = diagSeed[1];
              seedGenerated = 1;
              sendFrameFeedback(frameFeedback, 5);
              break;
            case 0x02:
              if (frame->data[0] != 4) {
                sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_INCORRECT_LENGTH);
                return;
              }
              if (seedGenerated == 0)
                sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_SEQUENCE_ERROR);

              else {
                if (frame->data[3] == (diagSeed[0] ^ sessionKey[0]) && frame->data[4] == (diagSeed[1] ^ sessionKey[1])) {
                  diagSession = 0x02;
                  frameFeedback[0] = 0x02;
                  frameFeedback[1] = UDS_SID_SECURITY_ACCESS + 0x40;
                  frameFeedback[2] = subf;
                  seedGenerated = 0;
                  secretSessionFound = 1;
                  validateChallenge(CHALLENGE_SECURITY_ACCESS);
                  sendFrameFeedback(frameFeedback, 3);
                } else {
                  sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_INVALID_KEY);
                }
              }
              break;
            default:
              sendFrameError(UDS_SID_SECURITY_ACCESS, UDS_ERROR_SUBFUNC_NOT_SUPPORTED);
              return;
              break;
          }
          break;
        default:
          sendFrameError(frame->data[1], UDS_ERROR_SERVICE_NOT_SUPPORTED);
          break;
    }
  }
  else if ((frame->data[0] & 0xF0) == 0x30 && isoTpRemainingBytes > 0) {
    if ((isoTpFirstFlowTime + ISOTP_TIMEOUT) >= currentTime)
      isoTpRequest = 1;
    else
      isoTpRemainingBytes;
  }
}

void Usage(char *msg) {
  if(msg) printf("%s\n", msg);
  printf("Usage: icsim [options] <can>\n");
  printf("\t-m, --multi-ecus         Enable multiple ECU simulation\n");
  printf("\t-g, --gateway            Enable gateway/IDS module\n");
  printf("\t-a, --message-auth       Enable message authentication\n");
  printf("\t-f, --firmware-update    Enable firmware update simulation\n");
  printf("\t-c, --can-fd-support     Enable CAN FD support\n");
  printf("\t-i, --intrusion-detection Enable intrusion detection\n");
  printf("\t-r\t-randomize IDs\n");
  printf("\t-d\tdebug mode\n");
  printf("\t-h, --help               Display this help message\n");
  exit(1);
}

/* Stub functions for new features */

void initializeMultipleECUs() {
    // TODO: Implement multiple ECUs initialization
    printf("[Feature] Initializing Multiple ECUs...\n");
}

void startGatewayModule() {
    // TODO: Implement gateway/IDS module start
    printf("[Feature] Starting Gateway/IDS Module...\n");
}

void initializeMessageAuth() {
    // TODO: Implement message authentication
    printf("[Feature] Initializing Message Authentication...\n");
}

void initializeFirmwareUpdate() {
    // TODO: Implement firmware update simulation
    printf("[Feature] Initializing Firmware Update Simulation...\n");
}

void enableCanFDSupport() {
    // TODO: Implement CAN FD support features
    printf("[Feature] Enabling CAN FD Support...\n");
}

void initializeIntrusionDetection() {
    // TODO: Implement intrusion detection
    printf("[Feature] Initializing Intrusion Detection...\n");
}

/* Main Function */
int main(int argc, char *argv[]) {
    int opt;
    int option_index = 0;
    Uint32 lastSpawnTime = SDL_GetTicks(); // Track the last car spawn time
    /* Define long options */
    static struct option long_options[] = {
        {"multi-ecus",        no_argument,       0, 'm'},
        {"gateway",           no_argument,       0, 'g'},
        {"message-auth",      no_argument,       0, 'a'},
        {"firmware-update",   no_argument,       0, 'f'},
        {"can-fd-support",    no_argument,       0, 'c'},
        {"intrusion-detection", no_argument,     0, 'i'},
        {"help",              no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    /* Initialize SimConfig with default values */
    SimConfig simConfig = {0, 0, 0, 0, 0, 0};

    /* Parse command-line options */
    while ((opt = getopt_long(argc, argv, "mgafcih?", long_options, &option_index)) != -1) {
        switch(opt) {
            case 'm':
                simConfig.multipleECUs = 1;
                break;
            case 'g':
                simConfig.gatewayNode = 1;
                break;
            case 'a':
                simConfig.messageAuth = 1;
                break;
            case 'f':
                simConfig.firmwareUpdate = 1;
                break;
            case 'c':
                simConfig.canFDSupport = 1;
                break;
            case 'i':
                simConfig.intrusionDetection = 1;
                break;
            case 'r':
                randomize_flag = 1;
                break;
            case 'd':
                debug = 1;
                break;
            case 'h':
            case '?':
            default:
                Usage(NULL);
                break;
        }
    }

    if (optind >= argc) Usage("You must specify at least one CAN device");

    /* Verify data directory exists */
    struct stat dirstat;
    if(stat(DATA_DIR, &dirstat) == -1) {
        printf("ERROR: DATA_DIR not found.  Define in make file or run in src dir\n");
        exit(34);
    }

    /* Create a new raw CAN socket */
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) Usage("Couldn't create raw socket");

    struct ifreq ifr;
    struct sockaddr_can addr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, argv[optind], sizeof(ifr.ifr_name)-1);
    printf("Using CAN interface %s\n", ifr.ifr_name);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        exit(1);
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    /* CAN FD Mode */
    setsockopt(can_socket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

    /* Set up CAN frame structures */
    struct canfd_frame frame;
    struct iovec iov;
    struct msghdr msg;
    struct cmsghdr *cmsg;
    struct timeval tv, timeoutConfig = { 0, 0 };
    fd_set rdfs;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval)) + CMSG_SPACE(sizeof(__u32))];
    int nbytes, maxdlen;
    struct msghdr received_msg;
    struct iovec received_iov;

    /* Bind the socket */
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    /* Initialize Car State */
    initCarState();

    /* Handle Randomization */
    int doorId = DEFAULT_DOOR_ID;
    int signalId = DEFAULT_SIGNAL_ID;
    int speedId = DEFAULT_SPEED_ID;
    int warningId = DEFAULT_WARNING_ID;
    int diagId = DEFAULT_ECU_ID;
    int lightId = DEFAULT_LIGHT_ID;
    int luminosityId = DEFAULT_LUMINOSITY_ID;
    int controlId = DEFAULT_CONTROL_ID;

    if (randomize_flag) {
        seed = time(NULL);
        srand(seed);
        doorId = (rand() % 2046) + 1;
        signalId = (rand() % 2046) + 1;
        speedId = (rand() % 2046) + 1;
        
        doorPos = rand() % 8;       // 0 to 7
        signalPos = rand() % 8;     // 0 to 7
        speedPos = rand() % 7;      // 0 to 6, ensuring speedPos + 1 <= 7
        
        printf("Seed: %d\n", seed);
        FILE *fdseed = fopen("/tmp/icsim_seed.txt", "w");
        if (fdseed) {
            fprintf(fdseed, "%d\n", seed);
            fclose(fdseed);
        }
    }


    /* Initialize SDL */
    SDL_Window *window = NULL;
    SDL_Surface *screenSurface = NULL;
    if(SDL_Init ( SDL_INIT_VIDEO ) < 0 ) {
        printf("SDL Could not initialize\n");
        exit(40);
    }

    window = SDL_CreateWindow("IC Simulator", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if(window == NULL) {
        printf("Window could not be created\n");
        exit(41);
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if(renderer == NULL) {
        printf("Renderer could not be created\n");
        exit(42);
    }

    /* Load Textures */
    SDL_Surface *image = IMG_Load(getData("dashboard.png"));
    if (!image) {
        printf("Error loading dashboard.png: %s\n", IMG_GetError());
        exit(43);
    }
    baseTexture = SDL_CreateTextureFromSurface(renderer, image);
    SDL_FreeSurface(image);

    SDL_Surface *roadSurface = IMG_Load(getData("road_bg.png"));
    if (!roadSurface) {
        printf("Error loading road_bg.png: %s\n", IMG_GetError());
        exit(44);
    }
    roadTexture = SDL_CreateTextureFromSurface(renderer, roadSurface);
    SDL_FreeSurface(roadSurface);

    SDL_Surface *needle = IMG_Load(getData("needle.png"));
    if (!needle) {
        printf("Error loading needle.png: %s\n", IMG_GetError());
        exit(45);
    }
    needleTex = SDL_CreateTextureFromSurface(renderer, needle);
    SDL_FreeSurface(needle);

    SDL_Surface *sprites = IMG_Load(getData("spritesheet.png"));
    if (!sprites) {
        printf("Error loading spritesheet.png: %s\n", IMG_GetError());
        exit(46);
    }
    spriteTex = SDL_CreateTextureFromSurface(renderer, sprites);
    SDL_FreeSurface(sprites);

    SDL_Surface *spritesAlt = IMG_Load(getData("spritesheet-alt.png"));
    if (!spritesAlt) {
        printf("Error loading spritesheet-alt.png: %s\n", IMG_GetError());
        exit(47);
    }
    spriteAltTex = SDL_CreateTextureFromSurface(renderer, spritesAlt);
    SDL_FreeSurface(spritesAlt);

    SDL_Surface *scoreboard = IMG_Load(getData("scoreboard.png"));
    if (!scoreboard) {
        printf("Error loading scoreboard.png: %s\n", IMG_GetError());
        exit(48);
    }
    scoreboardTex = SDL_CreateTextureFromSurface(renderer, scoreboard);
    SDL_FreeSurface(scoreboard);

    SDL_Surface *carSurfaceLoaded = IMG_Load(getData("car_sprite.png"));
    if (!carSurfaceLoaded) {
        printf("Error loading car_sprite.png: %s\n", IMG_GetError());
        exit(49);
    }
    carTexture = SDL_CreateTextureFromSurface(renderer, carSurfaceLoaded);
    SDL_FreeSurface(carSurfaceLoaded);

    SDL_Surface *trafficCarSurface = IMG_Load(getData("car_traffic1.png"));
    if (!trafficCarSurface) {
        printf("Error loading car_traffic1.png: %s\n", IMG_GetError());
        exit(50);
    }
    trafficCarTexture = SDL_CreateTextureFromSurface(renderer, trafficCarSurface);
    SDL_FreeSurface(trafficCarSurface);

    /* Draw the Initial IC */
    redrawIC();

    /* Initialize Features Based on Flags */
    if (simConfig.multipleECUs) {
        initializeMultipleECUs();
    }

    if (simConfig.gatewayNode) {
        startGatewayModule();
    }

    if (simConfig.messageAuth) {
        initializeMessageAuth();
    }

    if (simConfig.firmwareUpdate) {
        initializeFirmwareUpdate();
    }

    if (simConfig.canFDSupport) {
        enableCanFDSupport();
    }

    if (simConfig.intrusionDetection) {
        initializeIntrusionDetection();
    }

    /* Main Loop */
    running_flag = 1;
    SDL_Event event;

    while(running_flag) {
        while(SDL_PollEvent(&event) != 0) {
            switch(event.type) {
              case SDL_QUIT:
                  running_flag = 0;
                  break;
              case SDL_WINDOWEVENT:
                switch(event.window.event) {
                    case SDL_WINDOWEVENT_ENTER:
                    case SDL_WINDOWEVENT_RESIZED:
                        redrawIC();
                        break;
                }
                break;
           }
           SDL_Delay(3);
        }

        /* Receive CAN Frame */
        memset(&frame, 0, sizeof(frame));
        received_iov.iov_base = &frame;
        received_iov.iov_len = sizeof(frame);
        memset(&received_msg, 0, sizeof(received_msg));
        received_msg.msg_name = &addr;
        received_msg.msg_namelen = sizeof(addr);
        received_msg.msg_iov = &received_iov;
        received_msg.msg_iovlen = 1;
        received_msg.msg_control = &ctrlmsg;
        received_msg.msg_controllen = sizeof(ctrlmsg);

        nbytes = recvmsg(can_socket, &received_msg, 0);
        if (nbytes < 0) {
          perror("read");
          return 1;
        }
        if ((size_t)nbytes == CAN_MTU)
          maxdlen = CAN_MAX_DLEN;
        else if ((size_t)nbytes == CANFD_MTU)
          maxdlen = CANFD_MAX_DLEN;
        else {
          fprintf(stderr, "read: incomplete CAN frame\n");
          return 1;
        }

        /* Handle Control Messages */
        for (cmsg = CMSG_FIRSTHDR(&received_msg);
             cmsg && (cmsg->cmsg_level == SOL_SOCKET);
             cmsg = CMSG_NXTHDR(&received_msg,cmsg)) {
          if (cmsg->cmsg_type == SO_TIMESTAMP)
              tv = *(struct timeval *)CMSG_DATA(cmsg);
          else if (cmsg->cmsg_type == SO_RXQ_OVFL)
                 //dropcnt[i] = *(__u32 *)CMSG_DATA(cmsg);
                   fprintf(stderr, "Dropped packet\n");
        }

        currentTime = SDL_GetTicks();
        pristine = 1;

        /* Process CAN Frames Conditionally Based on SimConfig */
        if (frame.can_id == controlId) {
            if (simConfig.messageAuth) {
                // Perform message authentication
                // authenticateMessage(&frame); // Implement this function as needed
                // For now, just a placeholder
                printf("[Feature] Authenticating message...\n");
            }
            updateSharedData(&frame, maxdlen);
        }
        //if (frame.can_id == doorId) updateDoorStatus(&frame, maxdlen);
        if (frame.can_id == 0x124) {
            // data[0] = 1 => locked, 0 => unlocked
          printf("[ICSim Debug] Received CAN ID 0x124, Data=0x%02X\n", frame.data[0]);
          pristine = 0;  // Mark as updated to force redraw
          updateDoorStatus(&frame, maxdlen);
        }
        if (frame.can_id == signalId) updateSignalStatus(&frame, maxdlen);
        if (frame.can_id == speedId) drawSpeedStatus(&frame, maxdlen);
        if (frame.can_id == warningId) updateWarningStatus(&frame, maxdlen);
        if (frame.can_id == lightId) updateLightStatus(&frame, maxdlen);
        if (frame.can_id == luminosityId) updateLuminosityStatus(&frame, maxdlen);
        if (frame.can_id == diagId  || frame.can_id == diagId - 1) analyseDiagRequest(&frame, maxdlen);
        if (diagSession > 1 && ((lastDiagTesterPresent + 3500) < currentTime)) {
          diagSession = 1;
          seedGenerated = 0;
          diagActive = 0;
          secretSessionFound = 0;
        }
        if (isoTpRequest > 0 && isoTpRemainingBytes > 0) {
          sendIsoTpData();
        }

        if (pristine == 0) {
          int roadTexWidth, roadTexHeight;
          SDL_QueryTexture(roadTexture, NULL, NULL, &roadTexWidth, &roadTexHeight);

          // 1) Scroll the road based on currentSpeed
          trackOffset += (currentSpeed / 10) * trackScrollSpeed;
          if (trackOffset < 0) {
              // optional if you want reverse scrolling
              trackOffset = 0;
          }
          // If you want an infinite loop:
          if (trackOffset >= roadTexHeight) {
              trackOffset -= roadTexHeight;
          }

          // 2) Move the car horizontally if turn signals are set
          // turnStatus[0] = left, turnStatus[1] = right
          if (currentSpeed > 0) {
              if (turnStatus[0] == ON && turnStatus[1] == OFF) {
                  carX -= turnSpeed;
              } else if (turnStatus[1] == ON && turnStatus[0] == OFF) {
                  carX += turnSpeed;
              }
          }

          // Spawn traffic cars at regular intervals
          if (currentTime - lastTrafficSpawnTime >= TRAFFIC_SPAWN_INTERVAL) {
              spawnTrafficCar();
              lastTrafficSpawnTime = currentTime; // Reset the spawn timer
          }

          // Update traffic car positions
          updateTrafficCars();

          // Collision detection with player car
          SDL_Rect playerCarRect = {carX, carY, TRAFFIC_CAR_WIDTH, TRAFFIC_CAR_HEIGHT};
          for (int i = 0; i < trafficCarCount; i++) {
              if (checkCollision(&playerCarRect, &trafficCars[i].rect)) {
                  printf("Collision detected! Game Over!\n");
                  running_flag = 0; // End the simulation
                  break;
              }
          }

          // 3) Enforce screen boundaries
          if (carX < ROAD_LEFT_BOUNDARY) carX = ROAD_LEFT_BOUNDARY;
          if (carX > ROAD_RIGHT_BOUNDARY - 50) carX = ROAD_RIGHT_BOUNDARY - 50;

          // Finally, redraw
          updateIC();
      }
    }

    /* Cleanup */
    SDL_DestroyTexture(baseTexture);
    SDL_DestroyTexture(needleTex);
    SDL_DestroyTexture(spriteTex);
    SDL_DestroyTexture(spriteAltTex);
    SDL_DestroyTexture(scoreboardTex);
    SDL_DestroyTexture(roadTexture);
    SDL_DestroyTexture(carTexture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    close(can_socket);

    return 0;
}
