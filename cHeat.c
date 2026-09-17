/* cHeat — 3D Wi-Fi heatmap on /dev/fb0. Channel × bearing × RSSI. */
#include <fcntl.h>
#include <linux/fb.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAXAP 48

typedef struct {
    char ssid[28], bssid[20];
    int sig, chan;
    double ang;
} Ap;

static int fb = -1;
static unsigned char *map;
static size_t maplen;
static unsigned W, H, BPP, LINE;
static struct termios oldt;
static int raw_on;
static Ap aps[MAXAP];
static int nap;
static double yaw = 0.7, pitch = 0.55, zoom = 88;
static uint16_t C_BG, C_AXIS, C_TXT, C_DIM, C_HI;

static uint16_t rgb565(int r, int g, int b)
{
    if (r < 0) r = 0;
    if (g < 0) g = 0;
    if (b < 0) b = 0;
    if (r > 255) r = 255;
    if (g > 255) g = 255;
    if (b > 255) b = 255;
    return (uint16_t)(((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3));
}

static uint16_t heat(int sig)
{
    /* -90..-30 → blue..red */
    double t = (sig + 90) / 60.0;
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return rgb565((int)(40 + 210 * t), (int)(200 - 140 * t), (int)(200 - 180 * t));
}

static void px(int x, int y, uint16_t c)
{
    unsigned char *p;
    if ((unsigned)x >= W || (unsigned)y >= H)
        return;
    p = map + (size_t)y * LINE + (size_t)x * (BPP / 8);
    if (BPP == 16)
        ((uint16_t *)p)[0] = c;
    else if (BPP == 32) {
        p[0] = (unsigned char)((c & 0x1f) << 3);
        p[1] = (unsigned char)(((c >> 5) & 0x3f) << 2);
        p[2] = (unsigned char)(((c >> 11) & 0x1f) << 3);
        p[3] = 0;
    }
}

static void clear_fb(uint16_t c)
{
    unsigned y, x;
    for (y = 0; y < H; y++)
        for (x = 0; x < W; x++)
            px(x, y, c);
}

static void fill_rect(int x, int y, int w, int h, uint16_t c)
{
    int i, j;
    for (j = 0; j < h; j++)
        for (i = 0; i < w; i++)
            px(x + i, y + j, c);
}

static void hline(int x0, int x1, int y, uint16_t c)
{
    if (x0 > x1) {
        int t = x0;
        x0 = x1;
        x1 = t;
    }
    for (; x0 <= x1; x0++)
        px(x0, y, c);
}

static void tri(int x0, int y0, int x1, int y1, int x2, int y2, uint16_t c)
{
    int xs[3] = {x0, x1, x2}, ys[3] = {y0, y1, y2}, i, j;
    for (i = 0; i < 2; i++)
        for (j = i + 1; j < 3; j++)
            if (ys[j] < ys[i]) {
                int t = ys[i];
                ys[i] = ys[j];
                ys[j] = t;
                t = xs[i];
                xs[i] = xs[j];
                xs[j] = t;
            }
    if (ys[2] == ys[0])
        return;
    for (i = ys[0]; i <= ys[2]; i++) {
        int xa, xb;
        if (i <= ys[1] && ys[1] != ys[0])
            xa = xs[0] + (xs[1] - xs[0]) * (i - ys[0]) / (ys[1] - ys[0]);
        else if (ys[2] != ys[1])
            xa = xs[1] + (xs[2] - xs[1]) * (i - ys[1]) / (ys[2] - ys[1]);
        else
            xa = xs[1];
        xb = xs[0] + (xs[2] - xs[0]) * (i - ys[0]) / (ys[2] - ys[0]);
        hline(xa, xb, i, c);
    }
}

static uint16_t shade(uint16_t c, int pct)
{
    int r = ((c >> 11) & 0x1f) << 3;
    int g = ((c >> 5) & 0x3f) << 2;
    int b = (c & 0x1f) << 3;
    r = r * pct / 100;
    g = g * pct / 100;
    b = b * pct / 100;
    return rgb565(r, g, b);
}

static void line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        px(x0, y0, c);
        if (x0 == x1 && y0 == y1)
            break;
        {
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

static const unsigned char FONT[96][5] = {
    {0,0,0,0,0},{0,0,0x5f,0,0},{0,7,0,7,0},{0x14,0x7f,0x14,0x7f,0x14},
    {0x24,0x2a,0x7f,0x2a,0x12},{0x23,0x13,8,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},
    {0,5,3,0,0},{0,0x1c,0x22,0x41,0},{0,0x41,0x22,0x1c,0},{0x14,8,0x3e,8,0x14},
    {8,8,0x3e,8,8},{0,0x50,0x30,0,0},{8,8,8,8,8},{0,0x60,0x60,0,0},
    {0x20,0x10,8,4,2},{0x3e,0x51,0x49,0x45,0x3e},{0,0x42,0x7f,0x40,0},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4b,0x31},{0x18,0x14,0x12,0x7f,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3c,0x4a,0x49,0x49,0x30},{1,0x71,9,5,3},
    {0x36,0x49,0x49,0x49,0x36},{6,0x49,0x49,0x29,0x1e},{0,0x36,0x36,0,0},
    {0,0x56,0x36,0,0},{8,0x14,0x22,0x41,0},{0x14,0x14,0x14,0x14,0x14},
    {0,0x41,0x22,0x14,8},{2,1,0x51,9,6},{0x32,0x49,0x79,0x41,0x3e},
    {0x7e,0x11,0x11,0x11,0x7e},{0x7f,0x49,0x49,0x49,0x36},{0x3e,0x41,0x41,0x41,0x22},
    {0x7f,0x41,0x41,0x22,0x1c},{0x7f,0x49,0x49,0x49,0x41},{0x7f,9,9,9,1},
    {0x3e,0x41,0x49,0x49,0x7a},{0x7f,8,8,8,0x7f},{0,0x41,0x7f,0x41,0},
    {0x20,0x40,0x41,0x3f,1},{0x7f,8,0x14,0x22,0x41},{0x7f,0x40,0x40,0x40,0x40},
    {0x7f,2,0x0c,2,0x7f},{0x7f,4,8,0x10,0x7f},{0x3e,0x41,0x41,0x41,0x3e},
    {0x7f,9,9,9,6},{0x3e,0x41,0x51,0x21,0x5e},{0x7f,9,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{1,1,0x7f,1,1},{0x3f,0x40,0x40,0x40,0x3f},
    {0x1f,0x20,0x40,0x20,0x1f},{0x3f,0x40,0x38,0x40,0x3f},{0x63,0x14,8,0x14,0x63},
    {7,8,0x70,8,7},{0x61,0x51,0x49,0x45,0x43},{0,0x7f,0x41,0x41,0},
    {2,4,8,0x10,0x20},{0,0x41,0x41,0x7f,0},{4,2,1,2,4},{0x40,0x40,0x40,0x40,0x40},
    {0,1,2,4,0},{0x20,0x54,0x54,0x54,0x78},{0x7f,0x48,0x44,0x44,0x38},
    {0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7f},{0x38,0x54,0x54,0x54,0x18},
    {8,0x7e,9,1,2},{0x0c,0x52,0x52,0x52,0x3e},{0x7f,8,4,4,0x78},
    {0,0x44,0x7d,0x40,0},{0x20,0x40,0x44,0x3d,0},{0x7f,0x10,0x28,0x44,0},
    {0,0x41,0x7f,0x40,0},{0x7c,4,0x18,4,0x78},{0x7c,8,4,4,0x78},
    {0x38,0x44,0x44,0x44,0x38},{0x7c,0x14,0x14,0x14,8},{8,0x14,0x14,0x18,0x7c},
    {0x7c,8,4,4,8},{0x48,0x54,0x54,0x54,0x20},{4,0x3f,0x44,0x40,0x20},
    {0x3c,0x40,0x40,0x20,0x7c},{0x1c,0x20,0x40,0x20,0x1c},{0x3c,0x40,0x30,0x40,0x3c},
    {0x44,0x28,0x10,0x28,0x44},{0x0c,0x50,0x50,0x50,0x3c},{0x44,0x64,0x54,0x4c,0x44},
};

static void text(int x, int y, const char *s, uint16_t c)
{
    while (*s) {
        unsigned char ch = (unsigned char)*s++;
        int gx, gy;
        unsigned char col;
        if (ch < 32 || ch > 126)
            ch = '?';
        for (gx = 0; gx < 5; gx++) {
            col = FONT[ch - 32][gx];
            for (gy = 0; gy < 7; gy++)
                if (col & (1 << gy))
                    px(x + gx, y + gy, c);
        }
        x += 6;
    }
}

static int project(double x, double y, double z, int *sx, int *sy, double *dep)
{
    double cy = cos(yaw), syw = sin(yaw), cp = cos(pitch), sp = sin(pitch);
    double x1 = x * cy - y * syw;
    double y1 = x * syw + y * cy;
    double y2 = y1 * cp - z * sp;
    double z2 = y1 * sp + z * cp;
    double f;
    if (z2 < -2.4)
        return 0;
    f = 3.4 + z2;
    if (f < 0.5)
        return 0;
    f = zoom / f;
    *sx = (int)(W * 0.50 + x1 * f);
    *sy = (int)(H * 0.52 - y2 * f);
    if (dep)
        *dep = z2;
    return 1;
}

static void edge(double x0, double y0, double z0, double x1, double y1, double z1, uint16_t c)
{
    int a, b, d, e;
    if (project(x0, y0, z0, &a, &b, NULL) && project(x1, y1, z1, &d, &e, NULL))
        line(a, b, d, e, c);
}

static int split_g(char *line, char **out, int max)
{
    int n = 0;
    char *r = line, *w = line;
    out[0] = line;
    while (*r && n < max) {
        if (*r == '\\' && r[1]) {
            *w++ = r[1];
            r += 2;
            continue;
        }
        if (*r == ':') {
            *w++ = 0;
            r++;
            if (++n < max)
                out[n] = w;
            continue;
        }
        *w++ = *r++;
    }
    *w = 0;
    return n + 1;
}

static int freq_to_chan(int mhz)
{
    if (mhz >= 2412 && mhz <= 2484)
        return 1 + (mhz - 2412) / 5;
    if (mhz >= 5000 && mhz <= 5900)
        return 36 + (mhz - 5180) / 20;
    if (mhz > 100 && mhz < 200)
        return mhz; /* already a channel? */
    return 0;
}

static double bssid_ang(const char *b)
{
    unsigned h = 2166136261u;
    while (*b)
        h = (h ^ (unsigned char)*b++) * 16777619u;
    return (h % 360) * M_PI / 180.0;
}

static int cmp_sig(const void *a, const void *b)
{
    return ((const Ap *)b)->sig - ((const Ap *)a)->sig;
}

static void scan_aps(void)
{
    FILE *p;
    char line[512];
    nap = 0;
    p = popen("nmcli -g SSID,BSSID,SIGNAL,FREQ device wifi list 2>/dev/null", "r");
    if (!p)
        return;
    while (fgets(line, sizeof line, p) && nap < MAXAP) {
        char *f[5];
        int nf, mhz, ch;
        line[strcspn(line, "\n")] = 0;
        if (!line[0] || !strncmp(line, "Error", 5))
            continue;
        nf = split_g(line, f, 4);
        if (nf < 3)
            continue;
        mhz = nf >= 4 ? atoi(f[3]) : 0;
        ch = freq_to_chan(mhz);
        if (!ch)
            ch = 6;
        snprintf(aps[nap].ssid, sizeof aps[nap].ssid, "%s", f[0][0] ? f[0] : "*");
        snprintf(aps[nap].bssid, sizeof aps[nap].bssid, "%s", f[1]);
        aps[nap].sig = atoi(f[2]);
        aps[nap].chan = ch;
        aps[nap].ang = bssid_ang(f[1]);
        nap++;
    }
    pclose(p);
    qsort(aps, (size_t)nap, sizeof(Ap), cmp_sig);
}

static double chan_x(int ch)
{
    if (ch <= 14)
        return (ch - 7) * 0.32;
    return 2.6 + ((ch - 36) / 20.0) * 0.15;
}

static int proj4(double X[4], double Y[4], double Z[4], int sx[4], int sy[4], double *dep)
{
    int k;
    double d, sum = 0;
    for (k = 0; k < 4; k++) {
        if (!project(X[k], Y[k], Z[k], &sx[k], &sy[k], &d))
            return 0;
        sum += d;
    }
    if (dep)
        *dep = sum * 0.25;
    return 1;
}

static void quadf(int sx[4], int sy[4], uint16_t c)
{
    tri(sx[0], sy[0], sx[1], sy[1], sx[2], sy[2], c);
    tri(sx[0], sy[0], sx[2], sy[2], sx[3], sy[3], c);
}

static void bar3(double x, double y, double h, uint16_t c)
{
    double r = 0.13;
    double X[4], Y[4], Z[4];
    int sx[4], sy[4];
    /* top */
    X[0] = x - r; Y[0] = y - r; Z[0] = h;
    X[1] = x + r; Y[1] = y - r; Z[1] = h;
    X[2] = x + r; Y[2] = y + r; Z[2] = h;
    X[3] = x - r; Y[3] = y + r; Z[3] = h;
    if (proj4(X, Y, Z, sx, sy, NULL))
        quadf(sx, sy, shade(c, 100));
    /* +Y face */
    X[0] = x - r; Y[0] = y + r; Z[0] = 0;
    X[1] = x + r; Y[1] = y + r; Z[1] = 0;
    X[2] = x + r; Y[2] = y + r; Z[2] = h;
    X[3] = x - r; Y[3] = y + r; Z[3] = h;
    if (proj4(X, Y, Z, sx, sy, NULL))
        quadf(sx, sy, shade(c, 70));
    /* +X face */
    X[0] = x + r; Y[0] = y - r; Z[0] = 0;
    X[1] = x + r; Y[1] = y + r; Z[1] = 0;
    X[2] = x + r; Y[2] = y + r; Z[2] = h;
    X[3] = x + r; Y[3] = y - r; Z[3] = h;
    if (proj4(X, Y, Z, sx, sy, NULL))
        quadf(sx, sy, shade(c, 55));
    edge(x - r, y - r, h, x + r, y - r, h, shade(c, 40));
    edge(x + r, y - r, h, x + r, y + r, h, shade(c, 40));
}

typedef struct {
    int i;
    double d;
} Ord;

static int cmp_ord(const void *a, const void *b)
{
    const Ord *x = a, *y = b;
    if (x->d < y->d)
        return -1;
    if (x->d > y->d)
        return 1;
    return 0;
}

static void render(const char *note)
{
    int i, k;
    char line[80];
    Ord ord[MAXAP];
    clear_fb(C_BG);
    fill_rect(0, 0, (int)W, 14, C_DIM);
    fill_rect(0, (int)H - 30, (int)W, 30, C_DIM);

    for (i = -3; i <= 3; i++)
        edge(-2.8, i * 0.45, 0, 3.2, i * 0.45, 0, C_AXIS);
    for (k = 1; k <= 13; k += 2) {
        double x = chan_x(k);
        int sx, sy;
        edge(x, -1.8, 0, x, 1.8, 0, C_AXIS);
        if (project(x, -1.95, 0, &sx, &sy, NULL)) {
            char n[4];
            snprintf(n, sizeof n, "%d", k);
            text(sx - 3, sy, n, C_AXIS);
        }
    }
    edge(-2.8, 0, 0, 3.2, 0, 0, C_HI);
    edge(0, -1.8, 0, 0, 1.8, 0, C_HI);

    for (i = 0; i < nap; i++) {
        double x = chan_x(aps[i].chan);
        double y = sin(aps[i].ang) * 1.5;
        double z = (aps[i].sig + 95) / 42.0;
        int sx, sy;
        if (z < 0.10)
            z = 0.10;
        if (z > 2.2)
            z = 2.2;
        if (!project(x, y, z, &sx, &sy, &ord[i].d))
            ord[i].d = -99;
        ord[i].i = i;
    }
    qsort(ord, (size_t)nap, sizeof(Ord), cmp_ord);
    for (k = 0; k < nap; k++) {
        i = ord[k].i;
        {
            double x = chan_x(aps[i].chan);
            double y = sin(aps[i].ang) * 1.5;
            double z = (aps[i].sig + 95) / 42.0;
            if (z < 0.10)
                z = 0.10;
            if (z > 2.2)
                z = 2.2;
            bar3(x, y, z, heat(aps[i].sig));
        }
    }

    text(4, 4, "cHeat", C_HI);
    snprintf(line, sizeof line, "%d AP", nap);
    text((int)W - 6 * (int)strlen(line) - 6, 4, line, C_TXT);
    /* legend */
    for (i = 0; i < 8; i++)
        fill_rect(80 + i * 10, 4, 9, 6, heat(-90 + i * 8));
    text(80, 4, "", C_TXT);

    text(4, (int)H - 26, note, C_TXT);
    for (i = 0; i < nap && i < 3; i++) {
        snprintf(line, sizeof line, "%d %-10.10s %+ddB ch%d", i + 1, aps[i].ssid, aps[i].sig, aps[i].chan);
        text(4 + (i % 1) * 0, (int)H - 18 + i * 8, line, heat(aps[i].sig));
        if (i >= 2)
            break;
    }
    if (nap > 3)
        ; /* top 3 only in footer */
}

static void raw(int on)
{
    struct termios t;
    if (on) {
        tcgetattr(0, &oldt);
        t = oldt;
        t.c_lflag &= ~(ICANON | ECHO);
        t.c_cc[VMIN] = 0;
        t.c_cc[VTIME] = 0;
        tcsetattr(0, TCSANOW, &t);
        raw_on = 1;
    } else if (raw_on) {
        tcsetattr(0, TCSANOW, &oldt);
        raw_on = 0;
    }
}

static int fb_open(void)
{
    struct fb_var_screeninfo v;
    struct fb_fix_screeninfo f;
    fb = open("/dev/fb0", O_RDWR);
    if (fb < 0)
        return -1;
    if (ioctl(fb, FBIOGET_VSCREENINFO, &v) < 0)
        return -1;
    if (ioctl(fb, FBIOGET_FSCREENINFO, &f) < 0)
        return -1;
    W = v.xres;
    H = v.yres;
    BPP = v.bits_per_pixel;
    LINE = f.line_length;
    maplen = f.smem_len ? f.smem_len : (size_t)LINE * H;
    map = mmap(NULL, maplen, PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
    return map == MAP_FAILED ? -1 : 0;
}

int main(void)
{
    char note[64] = "s scan  arrows  q";
    int run = 1, auto_on = 0;
    if (fb_open() < 0) {
        fprintf(stderr, "cHeat: /dev/fb0: need the deck screen\n");
        return 1;
    }
    C_BG = rgb565(8, 10, 14);
    C_AXIS = rgb565(40, 48, 58);
    C_TXT = rgb565(200, 210, 220);
    C_DIM = rgb565(20, 24, 30);
    C_HI = rgb565(80, 220, 140);
    raw(1);
    scan_aps();
    render(note);
    while (run) {
        unsigned char ch = 0;
        fd_set rf;
        struct timeval tv = {0, auto_on ? 8000000 : 40000};
        FD_ZERO(&rf);
        FD_SET(0, &rf);
        if (select(1, &rf, NULL, NULL, &tv) > 0)
            if (read(0, &ch, 1) != 1)
                ch = 0;
        if (!ch) {
            if (auto_on) {
                scan_aps();
                snprintf(note, sizeof note, "auto %d ap", nap);
                render(note);
            }
            continue;
        }
        if (ch == 0x1b) {
            unsigned char seq[8] = {0};
            struct timeval t2 = {0, 80000};
            FD_ZERO(&rf);
            FD_SET(0, &rf);
            if (select(1, &rf, NULL, NULL, &t2) > 0)
                read(0, seq, 6);
            if (seq[0] == '[' && seq[1] == 'A')
                pitch -= 0.10;
            else if (seq[0] == '[' && seq[1] == 'B')
                pitch += 0.10;
            else if (seq[0] == '[' && seq[1] == 'C')
                yaw += 0.12;
            else if (seq[0] == '[' && seq[1] == 'D')
                yaw -= 0.12;
            if (pitch > 1.2)
                pitch = 1.2;
            if (pitch < 0.15)
                pitch = 0.15;
            render(note);
            continue;
        }
        if (ch == 'q')
            run = 0;
        else if (ch == 's' || ch == 'S') {
            scan_aps();
            snprintf(note, sizeof note, "scan %d", nap);
            render(note);
        } else if (ch == 'a') {
            auto_on ^= 1;
            snprintf(note, sizeof note, auto_on ? "auto on" : "auto off");
            render(note);
        } else if (ch == '+' || ch == '=') {
            zoom *= 1.12;
            render(note);
        } else if (ch == '-') {
            zoom /= 1.12;
            render(note);
        }
    }
    raw(0);
    munmap(map, maplen);
    close(fb);
    return 0;
}
