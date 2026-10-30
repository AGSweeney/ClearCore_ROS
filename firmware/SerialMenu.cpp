/* MIT License. Copyright (c) 2026 Adam G. Sweeney <agsweeney@gmail.com> */

#include "SerialMenu.h"

#include "ClearCore.h"
#include "MotionCore.h"
#include "RosConfig.h"
#include "Transport.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum MenuScreen {
    SCREEN_MAIN,
    SCREEN_NETWORK,
    SCREEN_AXES,
    SCREEN_AXIS,
    SCREEN_MOTION,
    SCREEN_SAFETY,
};

/* A pending prompt. The next line is its answer. */
enum MenuField {
    FIELD_NONE,
    FIELD_NET_IP,
    FIELD_NET_MASK,
    FIELD_NET_GW,
    FIELD_AXIS_NAME,
    FIELD_AXIS_STEPS,
    FIELD_AXIS_PITCH,
    FIELD_AXIS_GEAR,
    FIELD_AXIS_OFFSET,
    FIELD_AXIS_MIN,
    FIELD_AXIS_MAX,
    FIELD_AXIS_POS_DI,
    FIELD_AXIS_NEG_DI,
    FIELD_VEL,
    FIELD_ACCEL,
    FIELD_DECEL,
    FIELD_WATCHDOG,
    FIELD_ESTOP,
    FIELD_CONFIRM_RESET,
    FIELD_CONFIRM_RESTART,
};

static bool g_active = false;
static MenuScreen g_screen = SCREEN_MAIN;
static MenuField g_field = FIELD_NONE;
static uint8_t g_axis = 0;
static char g_pendIp[16];
static char g_pendMask[16];

/* Static: each is a few hundred bytes. */
static MotionConfigPatch g_cur;
static MotionConfigPatch g_patch;
static uint8_t g_limitFlags = 0;

static const char *const kMotorNames[CCROS_AXIS_COUNT] = {"M0", "M1", "M2", "M3"};

static void Out(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    TransportUsbSendLine(buf);
}

static void Prompt(const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    TransportUsbSend(buf);
}

static void Refresh() {
    MotionSnapshotConfig(&g_cur, &g_limitFlags);
}

static void BeginPatch() {
    memset(&g_patch, 0, sizeof(g_patch));
}

static void Report(const char *err, const char *note) {
    if (err) {
        Out("  Error: %s", err);
    } else {
        Out("  Saved.%s", note ? note : "");
    }
}

static bool ParseNumber(const char *s, double *out) {
    char *end = nullptr;
    *out = strtod(s, &end);
    return end != s && *end == '\0';
}

static bool ParseUint(const char *s, uint32_t *out) {
    double d = 0;
    if (!ParseNumber(s, &d) || d < 0.0 || d > 4294967295.0 || d != (double)(uint32_t)d) {
        return false;
    }
    *out = (uint32_t)d;
    return true;
}

static bool IpOk(const char *s) {
    uint8_t parts = 0;
    uint16_t acc = 0;
    bool any = false;
    for (;; s++) {
        const char c = *s;
        if (c >= '0' && c <= '9') {
            acc = (uint16_t)(acc * 10u + (uint16_t)(c - '0'));
            any = true;
            if (acc > 255) {
                return false;
            }
        } else if (c == '.' || c == '\0') {
            if (!any || parts >= 4) {
                return false;
            }
            parts++;
            acc = 0;
            any = false;
            if (c == '\0') {
                break;
            }
        } else {
            return false;
        }
    }
    return parts == 4;
}

static void FormatIp(char *buf, size_t len, const uint8_t ip[4]) {
    snprintf(buf, len, "%u.%u.%u.%u", (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2],
             (unsigned)ip[3]);
}

static bool IpZero(const uint8_t ip[4]) {
    return ip[0] == 0 && ip[1] == 0 && ip[2] == 0 && ip[3] == 0;
}

/* Limit input codes 1..12 are connectors IO-0..IO-5, DI-6..DI-8, A-9..A-12. */
static const char *PinName(uint8_t di, char *buf, size_t len) {
    if (di == 0 || di == 255 || di > 12) {
        return "none";
    }
    const unsigned connector = (unsigned)di - 1u;
    const char *prefix = di <= 6 ? "IO-" : (di <= 9 ? "DI-" : "A-");
    snprintf(buf, len, "%u (%s%u)", (unsigned)di, prefix, connector);
    return buf;
}

static bool LimitOn(uint8_t axis, bool max) {
    return (g_limitFlags & (1u << (axis * 2u + (max ? 1u : 0u)))) != 0;
}

static const char *EstopText(uint8_t mode) {
    switch (mode) {
        case 0: return "ignored";
        case 1: return "fault when DI-6 is low";
        case 2: return "fault when DI-6 is high";
        default: return "?";
    }
}

static const char *Unit(uint8_t axis) {
    return g_cur.rotary[axis] ? "rad" : "m";
}

static void ShowMain() {
    Out("");
    Out("=== %s setup (protocol %s) ===", CCROS_FIRMWARE_NAME, CCROS_PROTOCOL_VERSION);
    Out("IP now: %s   Motors: %s%s", TransportIpString(),
        MotionIsEnabled() ? "ENABLED" : "disabled", MotionTestMode() ? "   TEST MODE" : "");
    Out("  1) Network");
    Out("  2) Axes: enable, scaling, joint map, limits");
    Out("  3) Motion: velocity and acceleration");
    Out("  4) Safety: watchdog, estop, test mode");
    Out("  5) Show config (JSON)");
    Out("  6) Reset all settings to defaults");
    Out("  7) Restart board");
    Out("  q) Exit menu");
    Prompt("> ");
}

static void ShowNetwork() {
    uint8_t mode = 0;
    uint8_t ip[4], nm[4], gw[4];
    char a[16], b[16], c[16];
    MotionGetNetworkConfig(&mode, ip, nm, gw);
    FormatIp(a, sizeof(a), ip);
    FormatIp(b, sizeof(b), nm);
    FormatIp(c, sizeof(c), gw);
    Out("");
    Out("=== Network ===");
    if (mode == 1) {
        Out("Saved:   static %s  netmask %s  gateway %s", a, b, IpZero(gw) ? "none" : c);
    } else {
        Out("Saved:   DHCP (falls back to 192.168.0.109)");
    }
    Out("Running: %s", TransportIpString());
    Out("  1) Use DHCP");
    Out("  2) Set static address");
    Out("  b) Back");
    Out("Saved settings take effect after a restart (main menu 7).");
    Prompt("> ");
}

static void ShowAxes() {
    Out("");
    Out("=== Axes ===");
    Out("     Motor  Joint            On   Type    Steps/rev");
    for (uint8_t a = 0; a < CCROS_AXIS_COUNT; a++) {
        Out("  %u) %-5s  %-15s  %-3s  %-6s  %lu", (unsigned)(a + 1u), kMotorNames[a], g_cur.name[a],
            (g_cur.axisMask & (1u << a)) ? "yes" : "no", g_cur.rotary[a] ? "rotary" : "linear",
            (unsigned long)g_cur.stepsPerRev[a]);
    }
    Out("  b) Back");
    Prompt("Axis> ");
}

static void ShowAxis() {
    const uint8_t a = g_axis;
    char p1[16], p2[16];
    Out("");
    Out("=== Axis %s (%s) ===", kMotorNames[a], g_cur.name[a]);
    Out("   1) Enabled          %s", (g_cur.axisMask & (1u << a)) ? "yes" : "no");
    Out("   2) Joint name       %s", g_cur.name[a]);
    Out("   3) Steps per rev    %lu", (unsigned long)g_cur.stepsPerRev[a]);
    if (a == CCROS_AXIS_A) {
        Out("   4) Lead (mm/rev)    n/a on A");
    } else {
        Out("   4) Lead (mm/rev)    %.4f", g_cur.pitchMm[a]);
    }
    Out("   5) Type             %s", g_cur.rotary[a] ? "rotary (rad)" : "linear (m)");
    Out("   6) Direction        %+d", (int)g_cur.direction[a]);
    Out("   7) Gear             %.6f", g_cur.gear[a]);
    Out("   8) Offset           %.6f %s", g_cur.offset[a], Unit(a));
    if (LimitOn(a, false)) {
        Out("   9) Soft min         %.6f %s", g_cur.limitMin[a], Unit(a));
    } else {
        Out("   9) Soft min         off");
    }
    if (LimitOn(a, true)) {
        Out("  10) Soft max         %.6f %s", g_cur.limitMax[a], Unit(a));
    } else {
        Out("  10) Soft max         off");
    }
    Out("  11) + limit switch   %s", PinName(g_cur.posLim[a], p1, sizeof(p1)));
    Out("  12) - limit switch   %s", PinName(g_cur.negLim[a], p2, sizeof(p2)));
    Out("   b) Back");
    Out("Items 1, 3-8 need the motors disabled.");
    Prompt("%s> ", kMotorNames[a]);
}

static void ShowMotion() {
    Out("");
    Out("=== Motion (all axes) ===");
    Out("  1) Velocity       %lu steps/s", (unsigned long)g_cur.vel);
    Out("  2) Acceleration   %lu steps/s^2", (unsigned long)g_cur.accel);
    Out("  3) Deceleration   %lu steps/s^2", (unsigned long)g_cur.decel);
    Out("  b) Back");
    Prompt("> ");
}

static void ShowSafety() {
    Out("");
    Out("=== Safety ===");
    if (g_cur.watchdogMs == 0) {
        Out("  1) Host watchdog   off");
    } else {
        Out("  1) Host watchdog   %lu ms", (unsigned long)g_cur.watchdogMs);
    }
    Out("  2) Estop input     %s", EstopText(g_cur.estopDi6));
    Out("  3) Test mode       %s", MotionTestMode() ? "ON (DI-6 and HLFB checks skipped)" : "off");
    Out("  b) Back");
    Prompt("> ");
}

static void ShowScreen() {
    Refresh();
    switch (g_screen) {
        case SCREEN_MAIN: ShowMain(); break;
        case SCREEN_NETWORK: ShowNetwork(); break;
        case SCREEN_AXES: ShowAxes(); break;
        case SCREEN_AXIS: ShowAxis(); break;
        case SCREEN_MOTION: ShowMotion(); break;
        case SCREEN_SAFETY: ShowSafety(); break;
    }
}

static void Ask(MenuField field, const char *fmt, ...) {
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_field = field;
    TransportUsbSend(buf);
}

static void Enter() {
    g_active = true;
    g_screen = SCREEN_MAIN;
    g_field = FIELD_NONE;
    TransportSetUsbInteractive(true);
}

static void Exit(const char *why) {
    g_active = false;
    g_field = FIELD_NONE;
    TransportSetUsbInteractive(false);
    Out("%s", why);
}

static void ShowConfigJson() {
    static char body[CCROS_MAX_REPLY];
    MotionFillConfigJson(body, sizeof(body));
    TransportUsbSendLine(body);
}

static void StartStaticWizard() {
    uint8_t mode = 0;
    uint8_t ip[4], nm[4], gw[4];
    char cur[16];
    MotionGetNetworkConfig(&mode, ip, nm, gw);
    FormatIp(cur, sizeof(cur), ip);
    if (IpZero(ip)) {
        Ask(FIELD_NET_IP, "IP address: ");
    } else {
        Ask(FIELD_NET_IP, "IP address [%s]: ", cur);
    }
}

static void ChooseMain(uint32_t n) {
    switch (n) {
        case 1: g_screen = SCREEN_NETWORK; break;
        case 2: g_screen = SCREEN_AXES; break;
        case 3: g_screen = SCREEN_MOTION; break;
        case 4: g_screen = SCREEN_SAFETY; break;
        case 5: ShowConfigJson(); break;
        case 6:
            Ask(FIELD_CONFIRM_RESET,
                "Erase saved settings (mechanics, limits, joint map, network)? Type YES: ");
            break;
        case 7: Ask(FIELD_CONFIRM_RESTART, "Restart the board now? (y/n): "); break;
        default: Out("  Pick 1-7 or q."); break;
    }
}

static void ChooseNetwork(uint32_t n) {
    static char body[160];
    if (n == 1) {
        Report(MotionConfigureNetwork("dhcp", true, nullptr, false, nullptr, false, nullptr, false, body,
                                      sizeof(body)),
               " DHCP takes effect after a restart.");
    } else if (n == 2) {
        StartStaticWizard();
    } else {
        Out("  Pick 1, 2, or b.");
    }
}

static void ChooseAxis(uint32_t n) {
    const uint8_t a = g_axis;
    switch (n) {
        case 1:
            BeginPatch();
            g_patch.hasAxisMask = true;
            g_patch.axisMask = g_cur.axisMask ^ (1u << a);
            Report(MotionConfigure(&g_patch), nullptr);
            break;
        case 2: Ask(FIELD_AXIS_NAME, "Joint name (letters, digits, _) [%s]: ", g_cur.name[a]); break;
        case 3:
            Ask(FIELD_AXIS_STEPS, "Steps per motor rev [%lu]: ", (unsigned long)g_cur.stepsPerRev[a]);
            break;
        case 4:
            if (a == CCROS_AXIS_A) {
                Out("  A has no lead. Use gear for rotary scaling.");
            } else {
                Ask(FIELD_AXIS_PITCH, "Lead in mm per motor rev [%.4f]: ", g_cur.pitchMm[a]);
            }
            break;
        case 5:
            BeginPatch();
            g_patch.hasRotary[a] = true;
            g_patch.rotary[a] = !g_cur.rotary[a];
            Report(MotionConfigure(&g_patch), nullptr);
            break;
        case 6:
            BeginPatch();
            g_patch.hasDirection[a] = true;
            g_patch.direction[a] = (int8_t)-g_cur.direction[a];
            Report(MotionConfigure(&g_patch), nullptr);
            break;
        case 7: Ask(FIELD_AXIS_GEAR, "Gear, joint units per motor unit [%.6f]: ", g_cur.gear[a]); break;
        case 8: Ask(FIELD_AXIS_OFFSET, "Offset in %s [%.6f]: ", Unit(a), g_cur.offset[a]); break;
        case 9: Ask(FIELD_AXIS_MIN, "Soft min in %s, or 'off': ", Unit(a)); break;
        case 10: Ask(FIELD_AXIS_MAX, "Soft max in %s, or 'off': ", Unit(a)); break;
        case 11:
        case 12:
            Out("  Inputs: 1-6 = IO-0..IO-5, 7-9 = DI-6..DI-8, 10-12 = A-9..A-12, 0 = none.");
            Out("  A switch is active when the input reads high.");
            Ask(n == 11 ? FIELD_AXIS_POS_DI : FIELD_AXIS_NEG_DI, "%s limit input: ", n == 11 ? "+" : "-");
            break;
        default: Out("  Pick 1-12 or b."); break;
    }
}

static void ChooseMotion(uint32_t n) {
    switch (n) {
        case 1: Ask(FIELD_VEL, "Velocity, steps/s [%lu]: ", (unsigned long)g_cur.vel); break;
        case 2: Ask(FIELD_ACCEL, "Acceleration, steps/s^2 [%lu]: ", (unsigned long)g_cur.accel); break;
        case 3: Ask(FIELD_DECEL, "Deceleration, steps/s^2 [%lu]: ", (unsigned long)g_cur.decel); break;
        default: Out("  Pick 1-3 or b."); break;
    }
}

static void ChooseSafety(uint32_t n) {
    switch (n) {
        case 1:
            Ask(FIELD_WATCHDOG, "Host watchdog in ms, 0 = off [%lu]: ", (unsigned long)g_cur.watchdogMs);
            break;
        case 2:
            Out("  0 = ignored, 1 = fault when DI-6 is low, 2 = fault when DI-6 is high");
            Ask(FIELD_ESTOP, "Estop mode [%u]: ", (unsigned)g_cur.estopDi6);
            break;
        case 3: {
            const bool on = !MotionTestMode();
            Report(MotionSetTestMode(on), nullptr);
            if (on) {
                Out("  Test mode skips the DI-6 estop and HLFB checks. Bench use only.");
            }
            break;
        }
        default: Out("  Pick 1-3 or b."); break;
    }
}

static void HandleChoice(const char *lc) {
    if (lc[0] == '\0') {
        return;
    }
    if (strcmp(lc, "q") == 0) {
        Exit("Menu closed. Type 'menu' to reopen.");
        return;
    }
    if (strcmp(lc, "b") == 0) {
        g_screen = g_screen == SCREEN_AXIS ? SCREEN_AXES : SCREEN_MAIN;
        return;
    }
    uint32_t n = 0;
    if (!ParseUint(lc, &n)) {
        Out("  Enter a number, b to go back, or q to exit.");
        return;
    }
    switch (g_screen) {
        case SCREEN_MAIN: ChooseMain(n); break;
        case SCREEN_NETWORK: ChooseNetwork(n); break;
        case SCREEN_AXES:
            if (n >= 1 && n <= CCROS_AXIS_COUNT) {
                g_axis = (uint8_t)(n - 1u);
                g_screen = SCREEN_AXIS;
            } else {
                Out("  Pick 1-4 or b.");
            }
            break;
        case SCREEN_AXIS: ChooseAxis(n); break;
        case SCREEN_MOTION: ChooseMotion(n); break;
        case SCREEN_SAFETY: ChooseSafety(n); break;
    }
}

/* Applies one uint32 field from the Motion and Safety screens. */
static void ApplyUintField(MenuField field, uint32_t v) {
    BeginPatch();
    switch (field) {
        case FIELD_VEL: g_patch.hasVel = true; g_patch.vel = v; break;
        case FIELD_ACCEL: g_patch.hasAccel = true; g_patch.accel = v; break;
        case FIELD_DECEL: g_patch.hasDecel = true; g_patch.decel = v; break;
        case FIELD_WATCHDOG: g_patch.hasWatchdog = true; g_patch.watchdogMs = v; break;
        case FIELD_ESTOP:
            if (v > 2) {
                Out("  Error: estop mode must be 0, 1, or 2");
                return;
            }
            g_patch.hasEstop = true;
            g_patch.estopDi6 = (uint8_t)v;
            break;
        case FIELD_AXIS_STEPS:
            g_patch.hasSteps = true;
            memcpy(g_patch.stepsPerRev, g_cur.stepsPerRev, sizeof(g_patch.stepsPerRev));
            g_patch.stepsPerRev[g_axis] = v;
            break;
        case FIELD_AXIS_POS_DI:
        case FIELD_AXIS_NEG_DI:
            if (v > 12) {
                Out("  Error: input must be 0-12");
                return;
            }
            if (field == FIELD_AXIS_POS_DI) {
                g_patch.hasPosLim[g_axis] = true;
                g_patch.posLim[g_axis] = (uint8_t)v;
            } else {
                g_patch.hasNegLim[g_axis] = true;
                g_patch.negLim[g_axis] = (uint8_t)v;
            }
            break;
        default: return;
    }
    Report(MotionConfigure(&g_patch), nullptr);
}

static void ApplyNumberField(MenuField field, double v) {
    const uint8_t a = g_axis;
    BeginPatch();
    switch (field) {
        case FIELD_AXIS_PITCH:
            g_patch.hasPitch = true;
            memcpy(g_patch.pitchMm, g_cur.pitchMm, sizeof(g_patch.pitchMm));
            g_patch.pitchMm[a] = v;
            break;
        case FIELD_AXIS_GEAR: g_patch.hasGear[a] = true; g_patch.gear[a] = v; break;
        case FIELD_AXIS_OFFSET: g_patch.hasOffset[a] = true; g_patch.offset[a] = v; break;
        case FIELD_AXIS_MIN: g_patch.hasLimitMin[a] = true; g_patch.limitMin[a] = v; break;
        case FIELD_AXIS_MAX: g_patch.hasLimitMax[a] = true; g_patch.limitMax[a] = v; break;
        default: return;
    }
    Report(MotionConfigure(&g_patch), nullptr);
}

static void HandleNetworkField(const char *in) {
    uint8_t mode = 0;
    uint8_t ip[4], nm[4], gw[4];
    MotionGetNetworkConfig(&mode, ip, nm, gw);
    if (g_field == FIELD_NET_IP) {
        if (in[0] == '\0') {
            if (IpZero(ip)) {
                Out("  Cancelled. An IP address is required.");
                g_field = FIELD_NONE;
                return;
            }
            FormatIp(g_pendIp, sizeof(g_pendIp), ip);
        } else if (IpOk(in)) {
            strncpy(g_pendIp, in, sizeof(g_pendIp) - 1u);
            g_pendIp[sizeof(g_pendIp) - 1u] = '\0';
        } else {
            Out("  Not an IPv4 address. Example: 192.168.0.50");
            StartStaticWizard();
            return;
        }
        char cur[16];
        if (IpZero(nm)) {
            strcpy(cur, "255.255.255.0");
        } else {
            FormatIp(cur, sizeof(cur), nm);
        }
        Ask(FIELD_NET_MASK, "Netmask [%s]: ", cur);
        strcpy(g_pendMask, cur);
        return;
    }
    if (g_field == FIELD_NET_MASK) {
        if (in[0] != '\0') {
            if (!IpOk(in)) {
                Out("  Not a netmask. Example: 255.255.255.0");
                Ask(FIELD_NET_MASK, "Netmask [%s]: ", g_pendMask);
                return;
            }
            strncpy(g_pendMask, in, sizeof(g_pendMask) - 1u);
            g_pendMask[sizeof(g_pendMask) - 1u] = '\0';
        }
        char cur[16];
        FormatIp(cur, sizeof(cur), gw);
        Ask(FIELD_NET_GW, "Gateway, or 'none' [%s]: ", IpZero(gw) ? "none" : cur);
        return;
    }
    /* FIELD_NET_GW */
    const char *gateway = nullptr;
    if (strcmp(in, "none") == 0) {
        gateway = "0.0.0.0";
    } else if (in[0] != '\0') {
        if (!IpOk(in)) {
            Out("  Not an IPv4 address. Example: 192.168.0.1");
            Ask(FIELD_NET_GW, "Gateway, or 'none': ");
            return;
        }
        gateway = in;
    }
    g_field = FIELD_NONE;
    static char body[160];
    Report(MotionConfigureNetwork("static", true, g_pendIp, true, g_pendMask, true, gateway,
                                  gateway != nullptr, body, sizeof(body)),
           " The static address takes effect after a restart.");
}

static void HandleField(const char *in, const char *lc) {
    const MenuField field = g_field;
    if (field == FIELD_NET_IP || field == FIELD_NET_MASK || field == FIELD_NET_GW) {
        HandleNetworkField(lc);
        return;
    }
    g_field = FIELD_NONE;
    if (field == FIELD_CONFIRM_RESET) {
        if (strcmp(in, "YES") == 0) {
            Report(MotionResetConfig(), " Defaults are active. Network returns to DHCP after a restart.");
        } else {
            Out("  Not reset.");
        }
        return;
    }
    if (field == FIELD_CONFIRM_RESTART) {
        if (strcmp(lc, "y") == 0 || strcmp(lc, "yes") == 0) {
            Out("  Restarting. Reopen this port in a few seconds.");
            Delay_ms(100);
            MotionRestart();
        } else {
            Out("  Not restarted.");
        }
        return;
    }
    if (in[0] == '\0') {
        Out("  Unchanged.");
        return;
    }
    switch (field) {
        case FIELD_AXIS_NAME:
            if (strlen(in) >= sizeof(g_patch.name[0])) {
                Out("  Error: joint name must be 1..15 letters, digits, or _");
                return;
            }
            BeginPatch();
            g_patch.hasName[g_axis] = true;
            strcpy(g_patch.name[g_axis], in);
            Report(MotionConfigure(&g_patch), nullptr);
            return;
        case FIELD_AXIS_MIN:
        case FIELD_AXIS_MAX:
            if (strcmp(lc, "off") == 0) {
                BeginPatch();
                if (field == FIELD_AXIS_MIN) {
                    g_patch.hasClearMin[g_axis] = true;
                } else {
                    g_patch.hasClearMax[g_axis] = true;
                }
                Report(MotionConfigure(&g_patch), nullptr);
                return;
            }
            break;
        default: break;
    }
    switch (field) {
        case FIELD_AXIS_PITCH:
        case FIELD_AXIS_GEAR:
        case FIELD_AXIS_OFFSET:
        case FIELD_AXIS_MIN:
        case FIELD_AXIS_MAX: {
            double v = 0;
            if (!ParseNumber(in, &v)) {
                Out("  Error: expected a number");
                return;
            }
            ApplyNumberField(field, v);
            return;
        }
        default: {
            uint32_t v = 0;
            if (!ParseUint(in, &v)) {
                Out("  Error: expected a whole number");
                return;
            }
            ApplyUintField(field, v);
            return;
        }
    }
}

bool SerialMenuHandleLine(const char *line) {
    /* Trimmed copy, and a lowercase copy for commands. Joint names keep their case. */
    char in[64];
    char lc[64];
    while (*line == ' ' || *line == '\t') {
        line++;
    }
    strncpy(in, line, sizeof(in) - 1u);
    in[sizeof(in) - 1u] = '\0';
    size_t n = strlen(in);
    while (n > 0 && (in[n - 1u] == ' ' || in[n - 1u] == '\t')) {
        in[--n] = '\0';
    }
    for (size_t i = 0; i <= n; i++) {
        const char c = in[i];
        lc[i] = (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
    }

    if (!g_active) {
        if (in[0] == '\0') {
            Out("Type 'menu' and press Enter for setup.");
            return true;
        }
        if (strcmp(lc, "menu") == 0 || strcmp(lc, "setup") == 0) {
            Enter();
            ShowScreen();
            return true;
        }
        return false;
    }
    if (in[0] == '{') {
        Exit("Menu closed: JSON-RPC request received.");
        return false;
    }
    if (g_field != FIELD_NONE) {
        HandleField(in, lc);
    } else {
        HandleChoice(lc);
    }
    if (g_active && g_field == FIELD_NONE) {
        ShowScreen();
    }
    return true;
}
