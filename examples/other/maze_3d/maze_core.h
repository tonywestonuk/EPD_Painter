// =============================================================================
//  maze_core.h — platform-independent heart of the 3D maze game.
//
//  Everything here is templated on a GFX type providing the Adafruit_GFX
//  drawing subset (fillRect, drawRect, drawLine, fillTriangle, fillCircle,
//  drawCircle, setCursor, setTextSize, setTextColor, print).  The sketch
//  instantiates it with EPD_PainterAdafruit; a desktop harness can
//  instantiate it with a stub canvas to preview renders as images.
//
//  Colours are the panel's four levels: 0 white, 1 light grey, 2 dark grey,
//  3 black.
// =============================================================================
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdio.h>

// ---- board -----------------------------------------------------------------
#define MZ_W      15          // maze cells
#define MZ_H      15
#define MZ_DEPTH  5           // corridor draw distance (cells)
#define MZ_DOORS  3
#define MZ_VW     960         // panel
#define MZ_VH     540
#define MZ_HUD_H  48          // bottom bar
#define MZ_VIEW_H (MZ_VH - MZ_HUD_H)

// colours
#define C_WHITE 0
#define C_LT    1
#define C_DK    2
#define C_BLACK 3

enum MzDir { N = 0, E = 1, S = 2, W = 3 };
static const int8_t mz_dx[4] = { 0, 1, 0, -1 };
static const int8_t mz_dy[4] = { -1, 0, 1, 0 };

enum MzState { MZ_PLAYING, MZ_KEYPAD, MZ_WON };

enum MzAction {
  ACT_NONE, ACT_TURN_L, ACT_TURN_R, ACT_FWD, ACT_BACK,
  ACT_DIGIT_0, /* ..9 map onto ACT_DIGIT_0+n */
  ACT_KP_CANCEL = ACT_DIGIT_0 + 10,
  ACT_NEW_GAME
};

struct MzDoor  { int8_t x, y, dir; bool locked; };
struct MzThing { int8_t x, y; bool taken; };

struct MzGame {
  uint8_t  walls[MZ_H][MZ_W];       // bit per MzDir: wall present
  uint8_t  seen[MZ_H][MZ_W];        // automap fog
  MzDoor   doors[MZ_DOORS];
  MzThing  keys[MZ_DOORS];
  MzThing  tablets[MZ_DOORS];
  uint8_t  code[MZ_DOORS];          // exit code digits
  bool     tablet_read[MZ_DOORS];
  int8_t   exit_x, exit_y, exit_dir;
  int8_t   px, py, pdir;
  uint8_t  keys_held;
  uint16_t steps;
  MzState  state;
  uint8_t  kp_entry[MZ_DOORS];      // keypad digits entered
  uint8_t  kp_len;
  char     msg[48];                 // transient message line
  uint32_t rng;
};

// ---- tiny deterministic rng ------------------------------------------------
static inline uint32_t mzRand(MzGame &g) {
  g.rng ^= g.rng << 13; g.rng ^= g.rng >> 17; g.rng ^= g.rng << 5;
  return g.rng;
}

// ---- maze helpers ----------------------------------------------------------
static inline bool mzIn(int x, int y) { return x >= 0 && x < MZ_W && y >= 0 && y < MZ_H; }

static inline bool mzWall(const MzGame &g, int x, int y, int d) {
  if (!mzIn(x, y)) return true;
  return (g.walls[y][x] >> d) & 1;
}

static inline int mzDoorAt(const MzGame &g, int x, int y, int d) {
  int nx = x + mz_dx[d], ny = y + mz_dy[d];
  for (int i = 0; i < MZ_DOORS; i++) {
    const MzDoor &dr = g.doors[i];
    if ((dr.x == x && dr.y == y && dr.dir == d) ||
        (dr.x == nx && dr.y == ny && dr.dir == (d + 2) % 4))
      return i;
  }
  return -1;
}

static inline bool mzExitAt(const MzGame &g, int x, int y, int d) {
  return x == g.exit_x && y == g.exit_y && d == g.exit_dir;
}

// passable = no wall, or wall-slot occupied by an unlocked door
static inline bool mzOpen(const MzGame &g, int x, int y, int d) {
  int i = mzDoorAt(g, x, y, d);
  if (i >= 0) return !g.doors[i].locked;
  return !mzWall(g, x, y, d);
}

// ---- generation ------------------------------------------------------------
// Recursive backtracker (iterative), then doors along the solution path and
// keys/tablets in the region each door newly closes off — always solvable.

static inline void mzCarve(MzGame &g, int x, int y, int d) {
  g.walls[y][x] &= ~(1 << d);
  g.walls[y + mz_dy[d]][x + mz_dx[d]] &= ~(1 << ((d + 2) % 4));
}

// BFS with doors >= firstClosed treated as walls; fills dist[], returns cell
// count. dist is int16[MZ_H][MZ_W], -1 = unreached.
static inline int mzBfs(const MzGame &g, int sx, int sy, int firstClosed,
                        int16_t dist[MZ_H][MZ_W], int8_t par[MZ_H][MZ_W]) {
  static int16_t qx[MZ_W * MZ_H], qy[MZ_W * MZ_H];
  for (int y = 0; y < MZ_H; y++)
    for (int x = 0; x < MZ_W; x++) { dist[y][x] = -1; if (par) par[y][x] = -1; }
  int head = 0, tail = 0, n = 0;
  qx[tail] = sx; qy[tail] = sy; tail++; dist[sy][sx] = 0;
  while (head < tail) {
    int x = qx[head], y = qy[head]; head++; n++;
    for (int d = 0; d < 4; d++) {
      if (mzWall(g, x, y, d)) continue;
      int di = mzDoorAt(g, x, y, d);
      if (di >= 0 && di >= firstClosed) continue;
      int nx = x + mz_dx[d], ny = y + mz_dy[d];
      if (!mzIn(nx, ny) || dist[ny][nx] >= 0) continue;
      dist[ny][nx] = dist[y][x] + 1;
      if (par) par[ny][nx] = (d + 2) % 4;    // direction back to parent
      qx[tail] = nx; qy[tail] = ny; tail++;
    }
  }
  return n;
}

static inline void mzNewGame(MzGame &g, uint32_t seed) {
  memset(&g, 0, sizeof(g));
  g.rng = seed ? seed : 0xBADA55;

  // all walls up
  for (int y = 0; y < MZ_H; y++)
    for (int x = 0; x < MZ_W; x++) g.walls[y][x] = 0x0F;

  // backtracker
  static int16_t stx[MZ_W * MZ_H], sty[MZ_W * MZ_H];
  static uint8_t vis[MZ_H][MZ_W];
  memset(vis, 0, sizeof(vis));
  int sp = 0;
  stx[0] = 0; sty[0] = 0; vis[0][0] = 1;
  while (sp >= 0) {
    int x = stx[sp], y = sty[sp];
    int opts[4], no = 0;
    for (int d = 0; d < 4; d++) {
      int nx = x + mz_dx[d], ny = y + mz_dy[d];
      if (mzIn(nx, ny) && !vis[ny][nx]) opts[no++] = d;
    }
    if (!no) { sp--; continue; }
    int d = opts[mzRand(g) % no];
    mzCarve(g, x, y, d);
    int nx = x + mz_dx[d], ny = y + mz_dy[d];
    vis[ny][nx] = 1;
    sp++; stx[sp] = nx; sty[sp] = ny;
  }

  // a few extra links so the maze has loops (less back-tracking tedium)
  for (int i = 0; i < MZ_W * MZ_H / 12; i++) {
    int x = mzRand(g) % (MZ_W - 2) + 1, y = mzRand(g) % (MZ_H - 2) + 1;
    int d = mzRand(g) % 4;
    if (mzWall(g, x, y, d)) mzCarve(g, x, y, d);
  }

  // exit = farthest cell from start; path via parents
  static int16_t dist[MZ_H][MZ_W];
  static int8_t  par[MZ_H][MZ_W];
  mzBfs(g, 0, 0, 0, dist, par);
  int fx = 0, fy = 0;
  for (int y = 0; y < MZ_H; y++)
    for (int x = 0; x < MZ_W; x++)
      if (dist[y][x] > dist[fy][fx]) { fx = x; fy = y; }
  g.exit_x = fx; g.exit_y = fy;
  // exit door sits on a solid wall of that cell (prefer the maze boundary)
  g.exit_dir = -1;
  for (int d = 0; d < 4; d++) {
    int nx = fx + mz_dx[d], ny = fy + mz_dy[d];
    if (!mzIn(nx, ny)) { g.exit_dir = d; break; }
  }
  if (g.exit_dir < 0)
    for (int d = 0; d < 4; d++)
      if (mzWall(g, fx, fy, d)) { g.exit_dir = d; break; }
  if (g.exit_dir < 0) g.exit_dir = N;

  // solution path start->exit, as a list of (cell, entry dir)
  static int16_t path_x[MZ_W * MZ_H], path_y[MZ_W * MZ_H];
  int plen = 0;
  { int x = fx, y = fy;
    while (!(x == 0 && y == 0) && plen < MZ_W * MZ_H) {
      path_x[plen] = x; path_y[plen] = y; plen++;
      int d = par[y][x];
      x += mz_dx[d]; y += mz_dy[d];
    }
    path_x[plen] = 0; path_y[plen] = 0; plen++;
    // reverse into start->exit order
    for (int i = 0; i < plen / 2; i++) {
      int16_t t;
      t = path_x[i]; path_x[i] = path_x[plen-1-i]; path_x[plen-1-i] = t;
      t = path_y[i]; path_y[i] = path_y[plen-1-i]; path_y[plen-1-i] = t;
    }
  }

  // doors at ~35/60/85% along the path (edge entered at that step)
  for (int i = 0; i < MZ_DOORS; i++) {
    int at = plen * (35 + 25 * i) / 100;
    if (at < 1) at = 1;
    if (at >= plen) at = plen - 1;
    int x0 = path_x[at - 1], y0 = path_y[at - 1];
    int x1 = path_x[at],     y1 = path_y[at];
    int d;
    for (d = 0; d < 4; d++)
      if (x0 + mz_dx[d] == x1 && y0 + mz_dy[d] == y1) break;
    g.doors[i] = { (int8_t)x0, (int8_t)y0, (int8_t)d, true };
  }

  // key i and tablet i live in the region reachable with doors i.. closed,
  // as far from the start as possible (dead ends feel earned)
  for (int i = 0; i < MZ_DOORS; i++) {
    mzBfs(g, 0, 0, i, dist, nullptr);
    int kx = -1, ky = -1, tx = -1, ty = -1;
    for (int y = 0; y < MZ_H; y++)
      for (int x = 0; x < MZ_W; x++) {
        if (dist[y][x] < 0) continue;
        if (x == g.exit_x && y == g.exit_y) continue;
        bool used = (x == 0 && y == 0);
        for (int j = 0; j < i; j++) {
          if (g.keys[j].x == x && g.keys[j].y == y) used = true;
          if (g.tablets[j].x == x && g.tablets[j].y == y) used = true;
        }
        if (used) continue;
        if (kx < 0 || dist[y][x] > dist[ky][kx]) { tx = kx; ty = ky; kx = x; ky = y; }
        else if (tx < 0 || dist[y][x] > dist[ty][tx]) { tx = x; ty = y; }
      }
    g.keys[i]    = { (int8_t)kx, (int8_t)ky, false };
    g.tablets[i] = { (int8_t)(tx < 0 ? kx : tx), (int8_t)(ty < 0 ? ky : ty), false };
    g.code[i]    = mzRand(g) % 10;
  }

  g.px = 0; g.py = 0;
  g.pdir = mzOpen(g, 0, 0, E) ? E : S;
  g.seen[0][0] = 1;
  g.state = MZ_PLAYING;
  snprintf(g.msg, sizeof(g.msg), "find 3 keys and 3 tablets");
}

// ---- game actions ----------------------------------------------------------
static inline void mzEnterCell(MzGame &g) {
  g.seen[g.py][g.px] = 1;
  for (int i = 0; i < MZ_DOORS; i++) {
    MzThing &k = g.keys[i];
    if (!k.taken && k.x == g.px && k.y == g.py) {
      k.taken = true; g.keys_held++;
      snprintf(g.msg, sizeof(g.msg), "picked up a key");
    }
    MzThing &t = g.tablets[i];
    if (!t.taken && t.x == g.px && t.y == g.py) {
      t.taken = true; g.tablet_read[i] = true;
      static const char *rom[3] = { "I", "II", "III" };
      snprintf(g.msg, sizeof(g.msg), "tablet %s reads: %d", rom[i], g.code[i]);
    }
  }
}

// returns true if the screen needs re-rendering
static inline bool mzAction(MzGame &g, int act) {
  g.msg[0] = 0;

  if (g.state == MZ_WON) {
    if (act == ACT_NEW_GAME || act == ACT_FWD) { mzNewGame(g, g.rng); return true; }
    return false;
  }

  if (g.state == MZ_KEYPAD) {
    if (act >= ACT_DIGIT_0 && act < ACT_DIGIT_0 + 10) {
      if (g.kp_len < MZ_DOORS) g.kp_entry[g.kp_len++] = act - ACT_DIGIT_0;
      if (g.kp_len == MZ_DOORS) {
        bool ok = true;
        for (int i = 0; i < MZ_DOORS; i++) ok &= (g.kp_entry[i] == g.code[i]);
        if (ok) { g.state = MZ_WON; }
        else {
          g.kp_len = 0;
          snprintf(g.msg, sizeof(g.msg), "the door stays shut");
        }
      }
      return true;
    }
    if (act == ACT_KP_CANCEL || act == ACT_BACK) { g.state = MZ_PLAYING; g.kp_len = 0; return true; }
    return false;
  }

  switch (act) {
    case ACT_TURN_L: g.pdir = (g.pdir + 3) % 4; return true;
    case ACT_TURN_R: g.pdir = (g.pdir + 1) % 4; return true;
    case ACT_BACK: {
      int d = (g.pdir + 2) % 4;
      if (mzOpen(g, g.px, g.py, d)) {
        g.px += mz_dx[d]; g.py += mz_dy[d]; g.steps++;
        mzEnterCell(g);
      }
      return true;
    }
    case ACT_FWD: {
      int d = g.pdir;
      if (mzExitAt(g, g.px, g.py, d)) { g.state = MZ_KEYPAD; g.kp_len = 0; return true; }
      int di = mzDoorAt(g, g.px, g.py, d);
      if (di >= 0 && g.doors[di].locked) {
        if (g.keys_held) {
          g.keys_held--; g.doors[di].locked = false;
          snprintf(g.msg, sizeof(g.msg), "the key turns - the door opens");
        } else
          snprintf(g.msg, sizeof(g.msg), "locked - you need a key");
        return true;
      }
      if (mzOpen(g, g.px, g.py, d)) {
        g.px += mz_dx[d]; g.py += mz_dy[d]; g.steps++;
        mzEnterCell(g);
      } else
        snprintf(g.msg, sizeof(g.msg), "a wall");
      return true;
    }
  }
  return false;
}

// =============================================================================
// Rendering
// =============================================================================

// perspective frame per depth: half-extents as thousandths of the viewport
static const uint16_t mz_t[MZ_DEPTH + 1] = { 1000, 620, 400, 270, 190, 140 };

struct MzFrame { int x0, y0, x1, y1; };

// 1px hairlines all but vanish on e-paper: structural edges are drawn 3px
// (2px on the small automap) by offsetting along the minor axis.
static inline int mzAbs(int v) { return v < 0 ? -v : v; }

template <typename GFX>
static void mzLineT(GFX &gfx, int x0, int y0, int x1, int y1, int color, int t) {
  gfx.drawLine(x0, y0, x1, y1, color);
  bool horiz = mzAbs(x1 - x0) >= mzAbs(y1 - y0);
  for (int i = 1; i < t; i++) {
    int o = (i + 1) / 2 * ((i & 1) ? 1 : -1);   // +1, -1, +2, -2...
    if (horiz) gfx.drawLine(x0, y0 + o, x1, y1 + o, color);
    else       gfx.drawLine(x0 + o, y0, x1 + o, y1, color);
  }
}

template <typename GFX>
static void mzLineT2(GFX &gfx, int x0, int y0, int x1, int y1, int color) {
  mzLineT(gfx, x0, y0, x1, y1, color, 2);
}

template <typename GFX>
static void mzLine3(GFX &gfx, int x0, int y0, int x1, int y1, int color) {
  mzLineT(gfx, x0, y0, x1, y1, color, 3);
}

template <typename GFX>
static void mzRect3(GFX &gfx, int x, int y, int w, int h, int color) {
  for (int i = 0; i < 3; i++)
    gfx.drawRect(x + i, y + i, w - 2 * i, h - 2 * i, color);
}

static inline MzFrame mzFrame(int d) {
  int cx = MZ_VW / 2, cy = MZ_VIEW_H / 2;
  int hw = (int)((int32_t)(MZ_VW / 2) * mz_t[d] / 1000);
  int hh = (int)((int32_t)(MZ_VIEW_H / 2) * mz_t[d] / 1000);
  return { cx - hw, cy - hh, cx + hw, cy + hh };
}

template <typename GFX>
static void mzText(GFX &gfx, int x, int y, int size, int color, const char *s) {
  gfx.setTextSize(size);
  gfx.setTextColor(color);
  gfx.setCursor(x, y);
  gfx.print(s);
}

// door graphic on the front-wall rectangle f (which spans one cell width)
template <typename GFX>
static void mzDrawDoor(GFX &gfx, const MzFrame &f, bool locked, bool isExit) {
  int w = f.x1 - f.x0, h = f.y1 - f.y0;
  int dw = w * 44 / 100, dh = h * 78 / 100;
  int x = f.x0 + (w - dw) / 2, y = f.y1 - dh;
  gfx.fillRect(x, y, dw, dh, isExit ? C_DK : C_LT);
  mzRect3(gfx, x, y, dw, dh, C_BLACK);
  mzRect3(gfx, x + 1, y + 1, dw - 2, dh - 2, C_BLACK);
  if (isExit) {
    // keypad plate
    int pw = dw / 3, ph = dh / 4;
    int px = x + (dw - pw) / 2, py = y + dh / 3;
    gfx.fillRect(px, py, pw, ph, C_WHITE);
    mzRect3(gfx, px, py, pw, ph, C_BLACK);
    for (int r = 0; r < 3; r++)
      for (int c = 0; c < 3; c++)
        gfx.fillRect(px + pw / 6 + c * pw / 3 - 1, py + ph / 6 + r * ph / 3 - 1,
                     pw / 8 + 1, ph / 8 + 1, C_BLACK);
  } else if (locked && dw > 12) {
    // keyhole
    int cx = x + dw / 2, cy = y + dh / 2;
    int r = dw / 12 + 2;
    gfx.fillCircle(cx, cy, r, C_BLACK);
    gfx.fillTriangle(cx - r / 2, cy, cx + r / 2, cy, cx, cy + 2 * r, C_BLACK);
  } else if (dw > 12) {
    gfx.fillCircle(x + dw * 4 / 5, y + dh / 2, dw / 16 + 1, C_BLACK);
  }
}

// free-standing door frame you walk through (an unlocked door in side view
// or overhead as you pass) is drawn only as its front-facing case; passing
// through an open door we draw a lintel at that frame
template <typename GFX>
static void mzDrawLintel(GFX &gfx, const MzFrame &f) {
  int w = f.x1 - f.x0;
  gfx.fillRect(f.x0, f.y0, w, 6, C_DK);
  gfx.fillRect(f.x0, f.y0, 5, f.y1 - f.y0, C_DK);
  gfx.fillRect(f.x1 - 5, f.y0, 5, f.y1 - f.y0, C_DK);
}

template <typename GFX>
static void mzDrawKey(GFX &gfx, int d) {
  MzFrame fn = mzFrame(d + 1), fd = mzFrame(d);
  int cx = MZ_VW / 2;
  int fy = (fd.y1 + fn.y1) / 2;                     // on the floor of cell d
  int s = (fd.y1 - fn.y1) / 2;                      // size with distance
  if (s < 6) s = 6;
  // bow (ring) + shaft + teeth, black silhouette
  gfx.drawCircle(cx - s, fy - s / 2, s / 2 + 1, C_BLACK);
  gfx.drawCircle(cx - s, fy - s / 2, s / 2, C_BLACK);
  gfx.fillRect(cx - s / 2, fy - s / 2 - 1, s * 3 / 2, s / 4 + 2, C_BLACK);
  gfx.fillRect(cx + s / 2, fy - s / 2 - 1, s / 5 + 2, s / 2, C_BLACK);
  gfx.fillRect(cx + s - s / 5, fy - s / 2 - 1, s / 5 + 2, s * 2 / 5, C_BLACK);
}

template <typename GFX>
static void mzDrawTablet(GFX &gfx, int d) {
  MzFrame fn = mzFrame(d + 1), fd = mzFrame(d);
  int cx = MZ_VW / 2;
  int base = (fd.y1 + fn.y1) / 2;
  int h = (fd.y1 - fn.y1) / 2;
  if (h < 6) h = 6;
  int w = h * 3 / 4;
  // standing stone with a chiselled top
  gfx.fillTriangle(cx - w / 2, base - h * 2 / 3, cx, base - h, cx + w / 2, base - h * 2 / 3, C_DK);
  gfx.fillRect(cx - w / 2, base - h * 2 / 3, w, h * 2 / 3, C_DK);
  mzRect3(gfx, cx - w / 2, base - h * 2 / 3, w, h * 2 / 3, C_BLACK);
  if (h > 12) {
    mzLine3(gfx, cx - w / 4, base - h / 2, cx + w / 4, base - h / 2, C_BLACK);
    mzLine3(gfx, cx - w / 4, base - h / 3, cx + w / 4, base - h / 3, C_BLACK);
  }
}

// ---- the corridor ----------------------------------------------------------
template <typename GFX>
static void mzRenderView(GFX &gfx, MzGame &g) {
  gfx.fillRect(0, 0, MZ_VW, MZ_VIEW_H, C_WHITE);

  int x = g.px, y = g.py, dir = g.pdir;
  int dl = (dir + 3) % 4, dr = (dir + 1) % 4;

  for (int d = 0; d <= MZ_DEPTH; d++) {
    MzFrame fd = mzFrame(d);
    MzFrame fn = mzFrame(d + 1 > MZ_DEPTH ? MZ_DEPTH : d + 1);
    int cx4 = x + mz_dx[dir] * d, cy4 = y + mz_dy[dir] * d;

    // mark corridor cells on the automap as glimpsed
    if (mzIn(cx4, cy4)) g.seen[cy4][cx4] |= 1;

    bool wl = !mzOpen(g, cx4, cy4, dl);
    bool wr = !mzOpen(g, cx4, cy4, dr);
    int  dli = mzDoorAt(g, cx4, cy4, dl);
    int  dri = mzDoorAt(g, cx4, cy4, dr);

    // LEFT
    if (wl) {
      gfx.fillTriangle(fd.x0, fd.y0, fn.x0, fn.y0, fd.x0, fd.y1, C_LT);
      gfx.fillTriangle(fn.x0, fn.y0, fn.x0, fn.y1, fd.x0, fd.y1, C_LT);
      mzLine3(gfx, fd.x0, fd.y0, fn.x0, fn.y0, C_BLACK);
      mzLine3(gfx, fd.x0, fd.y1, fn.x0, fn.y1, C_BLACK);
      mzLine3(gfx, fn.x0, fn.y0, fn.x0, fn.y1, C_BLACK);
      if (dli >= 0 && g.doors[dli].locked && fn.x0 - fd.x0 > 30) {
        // a locked door in the side wall, drawn foreshortened
        int mx0 = fd.x0 + (fn.x0 - fd.x0) / 4, mx1 = fn.x0 - (fn.x0 - fd.x0) / 4;
        int my0 = fd.y1 - (fd.y1 - fn.y1) * 3 / 4, my1 = fd.y1 - (fd.y1 - fn.y1) / 8;
        gfx.fillTriangle(mx0, my0 - 20, mx1, my0, mx0, my1, C_DK);
        gfx.fillTriangle(mx1, my0, mx1, my1 - 8, mx0, my1, C_DK);
      }
    } else {
      // opening: the branch corridor's far wall, flat, one cell in
      gfx.fillRect(fd.x0, fn.y0, fn.x0 - fd.x0, fn.y1 - fn.y0, C_WHITE);
      mzLine3(gfx, fd.x0, fn.y0, fn.x0, fn.y0, C_BLACK);
      mzLine3(gfx, fd.x0, fn.y1, fn.x0, fn.y1, C_BLACK);
      mzLine3(gfx, fn.x0, fn.y0, fn.x0, fn.y1, C_BLACK);
      mzLine3(gfx, fd.x0, fn.y0, fd.x0, fd.y0 + 1, C_LT);
    }

    // RIGHT (mirror)
    if (wr) {
      gfx.fillTriangle(fd.x1, fd.y0, fn.x1, fn.y0, fd.x1, fd.y1, C_LT);
      gfx.fillTriangle(fn.x1, fn.y0, fn.x1, fn.y1, fd.x1, fd.y1, C_LT);
      mzLine3(gfx, fd.x1, fd.y0, fn.x1, fn.y0, C_BLACK);
      mzLine3(gfx, fd.x1, fd.y1, fn.x1, fn.y1, C_BLACK);
      mzLine3(gfx, fn.x1, fn.y0, fn.x1, fn.y1, C_BLACK);
      if (dri >= 0 && g.doors[dri].locked && fd.x1 - fn.x1 > 30) {
        int mx0 = fn.x1 + (fd.x1 - fn.x1) / 4, mx1 = fd.x1 - (fd.x1 - fn.x1) / 4;
        int my0 = fd.y1 - (fd.y1 - fn.y1) * 3 / 4, my1 = fd.y1 - (fd.y1 - fn.y1) / 8;
        gfx.fillTriangle(mx1, my0 - 20, mx0, my0, mx1, my1, C_DK);
        gfx.fillTriangle(mx0, my0, mx0, my1 - 8, mx1, my1, C_DK);
      }
    } else {
      gfx.fillRect(fn.x1, fn.y0, fd.x1 - fn.x1, fn.y1 - fn.y0, C_WHITE);
      mzLine3(gfx, fn.x1, fn.y0, fd.x1, fn.y0, C_BLACK);
      mzLine3(gfx, fn.x1, fn.y1, fd.x1, fn.y1, C_BLACK);
      mzLine3(gfx, fn.x1, fn.y0, fn.x1, fn.y1, C_BLACK);
    }

    // objects on the floor of this cell (behind any nearer wall, in front of
    // the next frame) — draw before a possible front wall at this depth
    if (d > 0) {
      for (int i = 0; i < MZ_DOORS; i++) {
        if (!g.keys[i].taken && g.keys[i].x == cx4 && g.keys[i].y == cy4)
          mzDrawKey(gfx, d);
        if (!g.tablets[i].taken && g.tablets[i].x == cx4 && g.tablets[i].y == cy4)
          mzDrawTablet(gfx, d);
      }
    }

    // walked-through open door: draw its frame as we look down the corridor
    { int di0 = mzDoorAt(g, cx4, cy4, dir);
      bool exit0 = mzExitAt(g, cx4, cy4, dir);
      bool front = !mzOpen(g, cx4, cy4, dir) || exit0;
      if (di0 >= 0 && !g.doors[di0].locked && !front)
        mzDrawLintel(gfx, mzFrame(d + 1 > MZ_DEPTH ? MZ_DEPTH : d + 1));

      // FRONT wall?
      if (front) {
        MzFrame ff = mzFrame(d + 1 > MZ_DEPTH ? MZ_DEPTH : d + 1);
        gfx.fillRect(ff.x0, ff.y0, ff.x1 - ff.x0, ff.y1 - ff.y0, C_WHITE);
        mzRect3(gfx, ff.x0, ff.y0, ff.x1 - ff.x0, ff.y1 - ff.y0, C_BLACK);
        if (exit0)             mzDrawDoor(gfx, ff, false, true);
        else if (di0 >= 0)     mzDrawDoor(gfx, ff, g.doors[di0].locked, false);
        break;
      }
    }

    if (d == MZ_DEPTH) {   // depth exhausted: distance haze
      MzFrame ff = mzFrame(MZ_DEPTH);
      gfx.fillRect(ff.x0, ff.y0, ff.x1 - ff.x0, ff.y1 - ff.y0, C_DK);
    }
  }
}

// ---- automap ---------------------------------------------------------------
#define MAP_CELL 7
#define MAP_PAD  6

template <typename GFX>
static void mzRenderMap(GFX &gfx, const MzGame &g) {
  int mw = MZ_W * MAP_CELL, mh = MZ_H * MAP_CELL;
  int ox = MZ_VW - mw - MAP_PAD - 8, oy = MAP_PAD;
  gfx.fillRect(ox - 4, oy - 4, mw + 8, mh + 8, C_WHITE);
  gfx.drawRect(ox - 4, oy - 4, mw + 8, mh + 8, C_BLACK);
  for (int y = 0; y < MZ_H; y++)
    for (int x = 0; x < MZ_W; x++) {
      if (!g.seen[y][x]) { gfx.fillRect(ox + x * MAP_CELL, oy + y * MAP_CELL, MAP_CELL, MAP_CELL, C_LT); continue; }
      int cx = ox + x * MAP_CELL, cy = oy + y * MAP_CELL;
      if (mzWall(g, x, y, N)) mzLineT2(gfx, cx, cy, cx + MAP_CELL, cy, C_BLACK);
      if (mzWall(g, x, y, W)) mzLineT2(gfx, cx, cy, cx, cy + MAP_CELL, C_BLACK);
      if (mzWall(g, x, y, S)) mzLineT2(gfx, cx, cy + MAP_CELL, cx + MAP_CELL, cy + MAP_CELL, C_BLACK);
      if (mzWall(g, x, y, E)) mzLineT2(gfx, cx + MAP_CELL, cy, cx + MAP_CELL, cy + MAP_CELL, C_BLACK);
      for (int i = 0; i < MZ_DOORS; i++)
        if (g.doors[i].locked && ((g.doors[i].x == x && g.doors[i].y == y)))
          gfx.fillRect(cx + 2, cy + 2, MAP_CELL - 3, MAP_CELL - 3, C_DK);
      if (x == g.exit_x && y == g.exit_y)
        gfx.fillRect(cx + 1, cy + 1, MAP_CELL - 1, MAP_CELL - 1, C_BLACK);
    }
  // player arrow
  int px = ox + g.px * MAP_CELL + MAP_CELL / 2, py = oy + g.py * MAP_CELL + MAP_CELL / 2;
  int fx = mz_dx[g.pdir], fy = mz_dy[g.pdir];
  gfx.fillTriangle(px + fx * 3, py + fy * 3,
                   px - fx * 3 + fy * 2, py - fy * 3 + fx * 2,
                   px - fx * 3 - fy * 2, py - fy * 3 - fx * 2, C_BLACK);
}

// ---- HUD -------------------------------------------------------------------
template <typename GFX>
static void mzRenderHud(GFX &gfx, const MzGame &g) {
  int y0 = MZ_VIEW_H;
  gfx.fillRect(0, y0, MZ_VW, MZ_HUD_H, C_WHITE);
  gfx.fillRect(0, y0, MZ_VW, 3, C_BLACK);

  // keys held, drawn as glyphs
  mzText(gfx, 16, y0 + 14, 2, C_BLACK, "KEYS");
  for (int i = 0; i < MZ_DOORS; i++) {
    int kx = 90 + i * 40, ky = y0 + 24;
    bool have = false;
    { int held = g.keys_held, unlocked = 0;
      for (int j = 0; j < MZ_DOORS; j++) if (!g.doors[j].locked) unlocked++;
      have = i < held; (void)unlocked; }
    if (have) {
      gfx.fillCircle(kx, ky, 7, C_BLACK);
      gfx.fillRect(kx + 4, ky - 2, 18, 5, C_BLACK);
      gfx.fillRect(kx + 14, ky + 2, 4, 6, C_BLACK);
      gfx.fillRect(kx + 20, ky + 2, 4, 5, C_BLACK);
    } else {
      gfx.drawCircle(kx, ky, 7, C_LT);
      mzRect3(gfx, kx + 4, ky - 2, 18, 5, C_LT);
    }
  }

  // tablet digits
  { char buf[40];
    static const char *rom[3] = { "I", "II", "III" };
    char *p = buf;
    for (int i = 0; i < MZ_DOORS; i++) {
      if (g.tablet_read[i]) p += snprintf(p, 8, "%s:%d  ", rom[i], g.code[i]);
      else                  p += snprintf(p, 8, "%s:?  ", rom[i]);
    }
    mzText(gfx, 250, y0 + 14, 2, C_BLACK, buf);
  }

  // message line
  if (g.msg[0]) mzText(gfx, 480, y0 + 14, 2, C_DK, g.msg);

  // compass + steps
  { char buf[24];
    static const char cd[4] = { 'N', 'E', 'S', 'W' };
    snprintf(buf, sizeof(buf), "%c  %u", cd[g.pdir], g.steps);
    mzText(gfx, MZ_VW - 130, y0 + 14, 2, C_BLACK, buf);
  }
}

// ---- keypad modal ----------------------------------------------------------
#define KP_CELL  76
#define KP_GAP   6
#define KP_W     (3 * KP_CELL + 4 * KP_GAP)
#define KP_H     (96 + 4 * KP_CELL + 3 * KP_GAP + 10)
#define KP_X     ((MZ_VW - KP_W) / 2)
#define KP_Y     ((MZ_VIEW_H - KP_H) / 2)
#define KP_GRID_Y (KP_Y + 96)

// which digit (0-9), 10 = cancel, -1 = none, for a panel tap
static inline int mzKeypadHit(int x, int y) {
  if (x < KP_X || x >= KP_X + KP_W || y < KP_Y || y >= KP_Y + KP_H) return 10;
  if (y < KP_GRID_Y) return -1;
  int col = (x - KP_X - KP_GAP) / (KP_CELL + KP_GAP);
  int row = (y - KP_GRID_Y) / (KP_CELL + KP_GAP);
  if (col < 0 || col > 2 || row < 0 || row > 3) return -1;
  if (row == 3) return col == 1 ? 0 : -1;         // bottom row: only "0"
  return row * 3 + col + 1;
}

template <typename GFX>
static void mzRenderKeypad(GFX &gfx, const MzGame &g) {
  gfx.fillRect(KP_X - 6, KP_Y - 6, KP_W + 12, KP_H + 12, C_WHITE);
  mzRect3(gfx, KP_X - 6, KP_Y - 6, KP_W + 12, KP_H + 12, C_BLACK);
  mzRect3(gfx, KP_X - 5, KP_Y - 5, KP_W + 10, KP_H + 10, C_BLACK);
  mzText(gfx, KP_X + (KP_W - 10 * 12) / 2, KP_Y + 10, 2, C_BLACK, "ENTER CODE");

  // entry boxes
  for (int i = 0; i < MZ_DOORS; i++) {
    int bx = KP_X + (KP_W - (3 * 48 + 2 * 18)) / 2 + i * (48 + 18), by = KP_Y + 40;
    mzRect3(gfx, bx, by, 48, 44, C_BLACK);
    if (i < g.kp_len) {
      char d[2] = { (char)('0' + g.kp_entry[i]), 0 };
      mzText(gfx, bx + 12, by + 8, 4, C_BLACK, d);
    }
  }

  for (int n = 1; n <= 9; n++) {
    int row = (n - 1) / 3, col = (n - 1) % 3;
    int bx = KP_X + KP_GAP + col * (KP_CELL + KP_GAP);
    int by = KP_GRID_Y + row * (KP_CELL + KP_GAP);
    mzRect3(gfx, bx, by, KP_CELL, KP_CELL, C_BLACK);
    char d[2] = { (char)('0' + n), 0 };
    mzText(gfx, bx + KP_CELL / 2 - 12, by + KP_CELL / 2 - 16, 4, C_BLACK, d);
  }
  int bx = KP_X + KP_GAP + 1 * (KP_CELL + KP_GAP);
  int by = KP_GRID_Y + 3 * (KP_CELL + KP_GAP);
  mzRect3(gfx, bx, by, KP_CELL, KP_CELL, C_BLACK);
  mzText(gfx, bx + KP_CELL / 2 - 12, by + KP_CELL / 2 - 16, 4, C_BLACK, "0");
  mzText(gfx, KP_X, KP_Y + KP_H + 10, 2, C_DK, "tap outside to step back");
}

// ---- win screen ------------------------------------------------------------
template <typename GFX>
static void mzRenderWon(GFX &gfx, const MzGame &g) {
  gfx.fillRect(0, 0, MZ_VW, MZ_VH, C_WHITE);
  for (int i = 0; i < 4; i++)
    mzRect3(gfx, 60 + i, 100 + i, MZ_VW - 120 - 2 * i, 260 - 2 * i, C_BLACK);
  mzText(gfx, 300, 160, 8, C_BLACK, "ESCAPED");
  { char buf[48];
    snprintf(buf, sizeof(buf), "the maze took you %u steps", g.steps);
    mzText(gfx, 330, 250, 3, C_DK, buf);
  }
  mzText(gfx, 340, 420, 3, C_BLACK, "tap for a new maze");
}

// faint chevrons marking the touch zones: sides turn, centre-high steps
// forward, centre-low steps back
template <typename GFX>
static void mzRenderHints(GFX &gfx) {
  int my = MZ_VIEW_H / 2;
  gfx.fillTriangle(18, my, 46, my - 26, 46, my + 26, C_LT);
  gfx.fillTriangle(MZ_VW - 18, my, MZ_VW - 46, my - 26, MZ_VW - 46, my + 26, C_LT);
  gfx.fillTriangle(MZ_VW / 2, MZ_VIEW_H - 84, MZ_VW / 2 - 26, MZ_VIEW_H - 56,
                   MZ_VW / 2 + 26, MZ_VIEW_H - 56, C_LT);
  gfx.fillTriangle(MZ_VW / 2, MZ_VIEW_H - 12, MZ_VW / 2 - 26, MZ_VIEW_H - 40,
                   MZ_VW / 2 + 26, MZ_VIEW_H - 40, C_LT);
}

// which action a tap at panel coordinates means, given the game state
static inline int mzTouchAction(const MzGame &g, int x, int y) {
  if (g.state == MZ_WON) return ACT_NEW_GAME;
  if (g.state == MZ_KEYPAD) {
    int k = mzKeypadHit(x, y);
    if (k >= 0 && k <= 9) return ACT_DIGIT_0 + k;
    if (k == 10) return ACT_KP_CANCEL;
    return ACT_NONE;
  }
  if (x < MZ_VW * 3 / 10) return ACT_TURN_L;
  if (x > MZ_VW * 7 / 10) return ACT_TURN_R;
  if (y > MZ_VIEW_H - 110 && y < MZ_VH) return ACT_BACK;
  return ACT_FWD;
}

// ---- top-level render ------------------------------------------------------
template <typename GFX>
static void mzRender(GFX &gfx, MzGame &g) {
  if (g.state == MZ_WON) { mzRenderWon(gfx, g); return; }
  mzRenderView(gfx, g);
  mzRenderHints(gfx);
  mzRenderMap(gfx, g);
  mzRenderHud(gfx, g);
  if (g.state == MZ_KEYPAD) mzRenderKeypad(gfx, g);
}
