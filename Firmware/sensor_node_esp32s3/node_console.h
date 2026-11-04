/*
 * node_console.h — line editor and command parser for the USB serial console (no Arduino dependency)
 *
 * The sketch feeds every character it receives on the USB serial port into
 * ConsoleLine::feed(); when a line is complete the parsed ConsoleCommand comes
 * back and the sketch acts on it. Keeping the parsing here lets the host tests
 * cover it (Firmware/tests/host/test_node_logic.cpp).
 *
 * Commands (case-insensitive, '?' or 'help' prints this list on the device):
 *   ?  help              list commands
 *   s  stat              print the statistics line now
 *   d  diag              self-diagnosis of radar, lidar, Wi-Fi and memory
 *   raw radar [N]        capture the next N raw radar UART bytes (default 600) and print them as hex
 *   raw lidar [N]        same for the LIDAR (default 4700 = 100 frames)
 *   subs                 list the receivers currently subscribed
 *   wifi                 access point status and the joined stations with RSSI
 *   reconfig             re-send the RD-03D multi-target configuration now
 *   log on|off           periodic [stat] lines on or off (off while capturing by hand)
 *   reboot               restart the node
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef NODE_CONSOLE_H
#define NODE_CONSOLE_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

enum ConsoleCmd : uint8_t {
    CON_NONE = 0,     /* empty line                                   */
    CON_UNKNOWN,      /* not a command; `raw` holds what was typed    */
    CON_HELP,
    CON_STAT,
    CON_DIAG,
    CON_RAW,          /* arg = "radar" | "lidar" | "", num = bytes or -1 */
    CON_SUBS,
    CON_WIFI,
    CON_RECONFIG,
    CON_LOG,          /* arg = "on" | "off" | ""                       */
    CON_REBOOT,
};

#define CONSOLE_LINE_MAX 48
#define CONSOLE_ARG_MAX  16

struct ConsoleCommand {
    ConsoleCmd cmd;
    char       arg[CONSOLE_ARG_MAX];     /* first argument, lower-cased, "" if none */
    long       num;                      /* numeric second argument, -1 if none     */
    char       raw[CONSOLE_LINE_MAX + 1];
};

static const char CONSOLE_HELP[] =
    "commands: ? help | s stat | d diag | raw radar [N] | raw lidar [N] | subs | wifi | reconfig | log on|off | reboot";

static inline char consoleLower(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

static inline bool consoleIsSpace(char c) { return c == ' ' || c == '\t'; }

/* Copies the next whitespace-delimited word of `s` into `out` (lower-cased), advances `s`. */
static inline bool consoleWord(const char *&s, char *out, size_t max) {
    while (consoleIsSpace(*s)) s++;
    if (*s == '\0') { out[0] = '\0'; return false; }
    size_t n = 0;
    while (*s && !consoleIsSpace(*s)) {
        if (n + 1 < max) out[n++] = consoleLower(*s);
        s++;
    }
    out[n] = '\0';
    return true;
}

static inline void consoleParse(const char *line, ConsoleCommand &out) {
    out.cmd = CON_NONE;
    out.arg[0] = '\0';
    out.num = -1;
    strncpy(out.raw, line, CONSOLE_LINE_MAX);
    out.raw[CONSOLE_LINE_MAX] = '\0';

    const char *s = line;
    char w[CONSOLE_ARG_MAX];
    if (!consoleWord(s, w, sizeof(w))) return;              /* empty line */

    if (!strcmp(w, "?") || !strcmp(w, "h") || !strcmp(w, "help"))                 out.cmd = CON_HELP;
    else if (!strcmp(w, "s") || !strcmp(w, "stat") || !strcmp(w, "status"))      out.cmd = CON_STAT;
    else if (!strcmp(w, "d") || !strcmp(w, "diag"))                              out.cmd = CON_DIAG;
    else if (!strcmp(w, "raw"))                                                  out.cmd = CON_RAW;
    else if (!strcmp(w, "subs") || !strcmp(w, "subscribers"))                    out.cmd = CON_SUBS;
    else if (!strcmp(w, "w") || !strcmp(w, "wifi"))                              out.cmd = CON_WIFI;
    else if (!strcmp(w, "reconfig") || !strcmp(w, "cfg"))                        out.cmd = CON_RECONFIG;
    else if (!strcmp(w, "log"))                                                  out.cmd = CON_LOG;
    else if (!strcmp(w, "reboot") || !strcmp(w, "reset") || !strcmp(w, "restart")) out.cmd = CON_REBOOT;
    else { out.cmd = CON_UNKNOWN; return; }

    if (consoleWord(s, w, sizeof(w))) {
        /* a numeric first argument (e.g. "raw 600") is taken as the number */
        char *end = nullptr;
        long v = strtol(w, &end, 10);
        if (end && *end == '\0' && end != w) out.num = v;
        else strncpy(out.arg, w, sizeof(out.arg));
        out.arg[sizeof(out.arg) - 1] = '\0';
        if (consoleWord(s, w, sizeof(w))) {
            v = strtol(w, &end, 10);
            if (end && *end == '\0' && end != w) out.num = v;
        }
    }
}

/* Accumulates characters until CR or LF; handles backspace. feed() returns
 * true when a complete, parsed command is available in `out`. Lines longer
 * than CONSOLE_LINE_MAX are truncated. */
class ConsoleLine {
public:
    bool feed(char c, ConsoleCommand &out) {
        if (c == '\r' || c == '\n') {
            if (_len == 0 && c == '\n' && _lastWasCr) { _lastWasCr = false; return false; }  /* CRLF: second half */
            _lastWasCr = (c == '\r');
            if (_len == 0) return false;
            _buf[_len] = '\0';
            consoleParse(_buf, out);
            _len = 0;
            return out.cmd != CON_NONE;
        }
        _lastWasCr = false;
        if (c == 0x08 || c == 0x7F) {                        /* backspace / delete */
            if (_len) _len--;
            return false;
        }
        if ((unsigned char)c < 0x20) return false;           /* other control characters */
        if (_len < CONSOLE_LINE_MAX) _buf[_len++] = c;
        return false;
    }

    void reset() { _len = 0; _lastWasCr = false; }

private:
    char   _buf[CONSOLE_LINE_MAX + 1];
    size_t _len = 0;
    bool   _lastWasCr = false;
};

#endif /* NODE_CONSOLE_H */
