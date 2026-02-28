#include <Arduino.h>
#include "EPD_Painter.h"
#include "EPD_Painter_devices.h"

#define XPOWERS_CHIP_BQ25896
#include <XPowersLib.h>

EPD_Painter epd(EPD_PAPER_DEVICE::LILYGO_T5_S3_GPS);
XPowersPPM PPM;

// --- Game constants ---
#define BALL_SIZE    25
#define BALL_SPEED   24.0f
#define NUM_BALLS    3

#define BRICK_COLS   8
#define BRICK_ROWS   6
#define BRICK_PAD    3
#define BRICK_TOP    10
#define BRICK_H      28

// --- Game state ---
struct Ball {
  float x, y;
  float vx, vy;
  uint8_t color;
};

struct Brick {
  bool alive;
  uint8_t color;
};

Ball balls[NUM_BALLS];
Brick bricks[BRICK_ROWS][BRICK_COLS];

int screenW, screenH;
int brickW;

void initBricks() {
  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      bricks[r][c].alive = true;
      bricks[r][c].color = (r % 3) + 1;
    }
  }
}

void initBalls() {
  // Each ball starts from a different position with a different angle
  balls[0] = { screenW * 0.25f, screenH - 40.0f, BALL_SPEED * 0.7f,  -BALL_SPEED,        3 };
  balls[1] = { screenW * 0.50f, screenH - 60.0f, -BALL_SPEED * 0.5f, -BALL_SPEED * 0.9f, 2 };
  balls[2] = { screenW * 0.75f, screenH - 50.0f, BALL_SPEED * 0.6f,  -BALL_SPEED * 0.8f, 1 };
}

void initGame() {
  screenW = epd.width();
  screenH = epd.height();
  brickW = (screenW - BRICK_PAD) / BRICK_COLS - BRICK_PAD;
  initBricks();
  initBalls();
}

void brickRect(int r, int c, int &bx, int &by, int &bw, int &bh) {
  bw = brickW;
  bh = BRICK_H;
  bx = BRICK_PAD + c * (brickW + BRICK_PAD);
  by = BRICK_TOP + r * (BRICK_H + BRICK_PAD);
}

void updateBall(Ball &ball) {
  ball.x += ball.vx;
  ball.y += ball.vy;

  if (ball.x <= 0) {
    ball.x = 0;
    ball.vx = -ball.vx;
  } else if (ball.x + BALL_SIZE >= screenW) {
    ball.x = screenW - BALL_SIZE;
    ball.vx = -ball.vx;
  }

  if (ball.y <= 0) {
    ball.y = 0;
    ball.vy = -ball.vy;
  }

  if (ball.y + BALL_SIZE >= screenH) {
    ball.y = screenH - BALL_SIZE;
    ball.vy = -ball.vy;
  }

  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!bricks[r][c].alive) continue;

      int bx, by, bw, bh;
      brickRect(r, c, bx, by, bw, bh);

      if (ball.x + BALL_SIZE > bx && ball.x < bx + bw &&
          ball.y + BALL_SIZE > by && ball.y < by + bh) {

        bricks[r][c].alive = false;

        float overlapLeft   = (ball.x + BALL_SIZE) - bx;
        float overlapRight  = (bx + bw) - ball.x;
        float overlapTop    = (ball.y + BALL_SIZE) - by;
        float overlapBottom = (by + bh) - ball.y;

        float minOverlapX = min(overlapLeft, overlapRight);
        float minOverlapY = min(overlapTop, overlapBottom);

        if (minOverlapX < minOverlapY) {
          ball.vx = -ball.vx;
        } else {
          ball.vy = -ball.vy;
        }

        return; // one brick per ball per frame
      }
    }
  }
}

void drawBricks() {
  for (int r = 0; r < BRICK_ROWS; r++) {
    for (int c = 0; c < BRICK_COLS; c++) {
      if (!bricks[r][c].alive) continue;
      int bx, by, bw, bh;
      brickRect(r, c, bx, by, bw, bh);
      epd.fillRect(bx, by, bw, bh, bricks[r][c].color);
    }
  }
}

void drawBall(const Ball &ball) {
  epd.fillCircle((int)ball.x + BALL_SIZE / 2, (int)ball.y + BALL_SIZE / 2, BALL_SIZE / 2, 3);
}

void setup() {
  Serial.begin(115200);
  if (!epd.begin()) {
    Serial.println("EPD init failed");
    while (1);
  }

  epd.setQuality(EPD_Painter::Quality::QUALITY_NORMAL);
  epd.clear();
  epd.clear();

  const auto& cfg = epd.getConfig();
  bool result = PPM.init(*cfg.i2c.wire, cfg.i2c.sda, cfg.i2c.scl, BQ25896_SLAVE_ADDRESS);
  if (!result) {
    while (1) {
      Serial.println("PPM is not online...");
      delay(1000);
    }
  }

  pinMode(0, INPUT);
  initGame();
}

void loop() {
  epd.fillScreen(0);

  // Antighosting

  for (int i=0; i<300; i++){
    epd.getBuffer()[random(192000)]=3;
  }

  for (int i = 0; i < NUM_BALLS; i++) {
    updateBall(balls[i]);
  }

  drawBricks();

  for (int i = 0; i < NUM_BALLS; i++) {
    drawBall(balls[i]);
  }

  // Reset bricks when all destroyed
  bool anyAlive = false;
  for (int r = 0; r < BRICK_ROWS && !anyAlive; r++)
    for (int c = 0; c < BRICK_COLS && !anyAlive; c++)
      if (bricks[r][c].alive) anyAlive = true;
  if (!anyAlive) initBricks();

  epd.paint(1);

  if (digitalRead(0) == 0) {
    epd.clear();
    epd.clear();
    PPM.shutdown();
  }
}