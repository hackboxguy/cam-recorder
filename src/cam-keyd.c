/*
 * cam-keyd — reads the local keyboard and drives the video apps.
 *
 * Deliberately separate from both video apps and never opens the camera.
 * camview cannot read the keyboard, so if key handling lived inside the video
 * apps there would be no way to press "-" to get back once "+" had switched
 * away. Keeping input in its own daemon also makes the input layer pluggable:
 * streamdeck-ctrl can replace this program later and talk to exactly the same
 * recorder socket, with nothing else changing.
 *
 *   Enter  -> "REC TOGGLE main" on the recorder's control socket
 *   "+"    -> systemctl start camview            (Conflicts= stops the recorder)
 *   "-"    -> systemctl start cam-recorder    (Conflicts= stops camview)
 *   "/"    -> "REC TOGGLE left"    (dual camera, future)
 *   "*"    -> "REC TOGGLE right"   (dual camera, future)
 *
 * Both keypad and main-keyboard variants of every key are accepted. They are
 * DISTINCT keycodes (KEY_KPENTER != KEY_ENTER), and handling only one looks
 * exactly like "the key does nothing" — an unpleasant thing to debug.
 *
 * Presses are debounced across ALL devices (see DEBOUNCE_MS): because every
 * matching node is watched, one physical press can be delivered more than once.
 *
 * All matching devices are opened at once rather than picking one: the attached
 * dongle presents as FIVE nodes (event5-event9 for one "MOSART Semi. 2.4G
 * Keyboard Mouse") and which of them carries a given key is not knowable in
 * advance — measured here, event5 has the whole keypad while event7 has only
 * Enter and KP+/KP-. A separate numeric keypad is another node again.
 * Devices are rescanned periodically so a 2.4 GHz dongle that re-enumerates
 * does not require a service restart.
 *
 * Copyright (C) 2026, Albert David <albert.david@gmail.com>
 * SPDX-License-Identifier: GPL-2.0-only
 */

#define _GNU_SOURCE
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

/* A single physical press can arrive on more than one event node: this dongle
 * advertises KEY_ENTER on both event5 and event7, and every node is watched
 * because which one carries a given key is not knowable in advance. Two
 * deliveries of one press would toggle recording twice - start then instantly
 * stop, seen as the red dot flashing. Duplicates land within a millisecond or
 * two, so 200 ms is far above the noise and far below any deliberate re-press. */
#define DEBOUNCE_MS      200

#define MAX_KBD          16
#define RESCAN_SECS      3
#define DEFAULT_SOCKET   "/run/cam-recorder.sock"
#define RECORDER_UNIT    "cam-recorder"
#define CAMVIEW_UNIT     "camview"

static volatile sig_atomic_t running = 1;
static const char *sockpath = DEFAULT_SOCKET;
static int verbose = 0;

static void on_sig(int s) { (void)s; running = 0; }

#define LOGV(...) do { if (verbose) { fprintf(stderr, __VA_ARGS__); fflush(stderr); } } while (0)

/* --------------------------------------------------------------- recorder IPC */

/*
 * The recorder is only running when it owns the display. When camview has it,
 * the socket is simply absent — that is a normal state, not an error, so a
 * failed connect is logged at most and never retried or treated as fatal.
 */
static int recorder_send(const char *cmd)
{
	struct sockaddr_un a;
	char buf[512];
	int fd, n;

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0)
		return -1;

	memset(&a, 0, sizeof(a));
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof(a.sun_path), "%s", sockpath);

	if (connect(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		LOGV("keyd: recorder socket unavailable (%s) — ignoring key\n",
		     strerror(errno));
		close(fd);
		return -1;
	}

	n = snprintf(buf, sizeof(buf), "%s\n", cmd);
	if (write(fd, buf, n) != n) {
		close(fd);
		return -1;
	}
	n = read(fd, buf, sizeof(buf) - 1);   /* best-effort; reply is for logs only */
	if (n > 0) {
		buf[n] = '\0';
		LOGV("keyd: %s -> %s", cmd, buf);
	}
	close(fd);
	return 0;
}

/* Fire-and-forget. Conflicts= in the unit files does the stopping, so we never
 * have to sequence a stop before a start (and never race the two). */
static void switch_unit(const char *unit)
{
	pid_t p = fork();

	if (p == 0) {
		/* execlp, not a hardcoded path: systemctl lives in /bin on this
		 * image and /usr/bin on others, and a wrong path here fails
		 * silently in the child — indistinguishable from "the key does
		 * nothing". */
		execlp("systemctl", "systemctl", "start", unit, (char *)NULL);
		_exit(127);
	}
	LOGV("keyd: switching to %s\n", unit);
}

/* ------------------------------------------------------------ device handling */

struct kbd {
	int  fd;
	char path[288];   /* d_name can be 255 bytes */
};

/* Every keycode this daemon acts on. Used both to decide which devices are
 * worth opening and (in handle_key) to decide what to do with a press. */
static const int action_keys[] = {
	KEY_ENTER, KEY_KPENTER,
	KEY_KPPLUS, KEY_EQUAL,
	KEY_KPMINUS, KEY_MINUS,
	KEY_KPSLASH, KEY_SLASH,
	KEY_KPASTERISK,
};

/*
 * A device is interesting if it can emit at least one key we act on.
 *
 * This is deliberately a CAPABILITY test, not "does it look like a keyboard".
 * An earlier version required KEY_A && KEY_Z && KEY_ENTER, which silently
 * rejected a standalone USB numeric keypad — a device with no letter keys at
 * all, and exactly the hardware in use here. The keypad would simply never be
 * watched, which is indistinguishable from "the keys do nothing".
 *
 * It still excludes the noise: gpio-keys (KEY_POWER only) and the HDMI/DP
 * audio jacks that also appear under /dev/input on this board report none of
 * these codes, so they are not opened. The power button in particular must
 * never reach handle_key().
 */
static int has_action_keys(int fd)
{
	unsigned long evbit = 0, keybit[(KEY_MAX / (8 * sizeof(long))) + 1];
	size_t i;

	if (ioctl(fd, EVIOCGBIT(0, sizeof(evbit)), &evbit) < 0)
		return 0;
	if (!(evbit & (1 << EV_KEY)))
		return 0;

	memset(keybit, 0, sizeof(keybit));
	if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybit)), keybit) < 0)
		return 0;

#define HAS(k) (keybit[(k) / (8 * sizeof(long))] & (1UL << ((k) % (8 * sizeof(long)))))
	for (i = 0; i < sizeof(action_keys) / sizeof(action_keys[0]); i++)
		if (HAS(action_keys[i]))
			return 1;
#undef HAS
	return 0;
}

static void close_all(struct kbd *k, int *n)
{
	for (int i = 0; i < *n; i++)
		if (k[i].fd >= 0)
			close(k[i].fd);
	*n = 0;
}

static int scan_keyboards(struct kbd *k, int max)
{
	struct dirent *de;
	DIR *d = opendir("/dev/input");
	int n = 0;

	if (!d)
		return 0;

	while ((de = readdir(d)) && n < max) {
		char path[288];   /* d_name can be 255 bytes */
		int fd;

		if (strncmp(de->d_name, "event", 5) != 0)
			continue;
		snprintf(path, sizeof(path), "/dev/input/%s", de->d_name);

		fd = open(path, O_RDONLY | O_NONBLOCK);
		if (fd < 0)
			continue;
		if (!has_action_keys(fd)) {
			close(fd);
			continue;
		}
		k[n].fd = fd;
		snprintf(k[n].path, sizeof(k[n].path), "%s", path);
		LOGV("keyd: watching %s\n", path);
		n++;
	}
	closedir(d);
	return n;
}

/* ----------------------------------------------------------------- key actions */

static long long now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void handle_key(int code)
{
	/* Monotonic, not the event's own timeval: that is CLOCK_REALTIME by
	 * default and an NTP step could make the delta negative or huge. */
	static int last_code = -1;
	static long long last_ms = 0;
	long long t = now_ms();

	/* Same key again within the window == the same physical press arriving
	 * on a second node. A DIFFERENT key is always let through, so pressing
	 * "-" and Enter in quick succession still works. */
	if (code == last_code && t - last_ms < DEBOUNCE_MS) {
		LOGV("keyd: ignoring duplicate keycode %d (%lldms)\n", code, t - last_ms);
		return;
	}
	last_code = code;
	last_ms = t;

	switch (code) {
	case KEY_ENTER:
	case KEY_KPENTER:
		recorder_send("REC TOGGLE main");
		break;

	case KEY_KPPLUS:
	case KEY_EQUAL:          /* "+" on a main keyboard is shift+= */
		switch_unit(CAMVIEW_UNIT);
		break;

	case KEY_KPMINUS:
	case KEY_MINUS:
		switch_unit(RECORDER_UNIT);
		break;

	/* Dual-camera channels. The verbs exist in the recorder's protocol
	 * already, so these keys work the moment the second camera lands and
	 * neither the key map nor the socket API has to change. Today the
	 * recorder answers "ERR channel not implemented". */
	case KEY_KPSLASH:
	case KEY_SLASH:
		recorder_send("REC TOGGLE left");
		break;

	case KEY_KPASTERISK:
		recorder_send("REC TOGGLE right");
		break;

	default:
		break;
	}
}

int main(int argc, char **argv)
{
	struct kbd kbd[MAX_KBD];
	int nkbd = 0;
	time_t last_scan = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "-v") || !strcmp(argv[i], "--verbose"))
			verbose = 1;
		else if (!strncmp(argv[i], "--socket=", 9))
			sockpath = argv[i] + 9;
		else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
			printf("usage: %s [-v] [--socket=PATH]\n", argv[0]);
			return 0;
		}
	}

	signal(SIGINT, on_sig);
	signal(SIGTERM, on_sig);
	signal(SIGCHLD, SIG_IGN);     /* reap the systemctl forks */

	memset(kbd, 0, sizeof(kbd));
	fprintf(stderr, "cam-keyd: socket=%s\n", sockpath);

	while (running) {
		struct pollfd pfd[MAX_KBD];
		time_t now = time(NULL);
		int rc;

		/* Periodic rescan. The 2.4 GHz dongle can re-enumerate on its own,
		 * and a keyboard may be plugged in long after boot; neither should
		 * need a service restart. */
		if (now - last_scan >= RESCAN_SECS) {
			int want = (nkbd == 0);
			for (int i = 0; i < nkbd; i++)
				if (fcntl(kbd[i].fd, F_GETFD) < 0)
					want = 1;
			if (want) {
				close_all(kbd, &nkbd);
				nkbd = scan_keyboards(kbd, MAX_KBD);
				if (nkbd == 0)
					LOGV("keyd: no keyboard present\n");
			}
			last_scan = now;
		}

		if (nkbd == 0) {
			sleep(1);
			continue;
		}

		for (int i = 0; i < nkbd; i++) {
			pfd[i].fd = kbd[i].fd;
			pfd[i].events = POLLIN;
			pfd[i].revents = 0;
		}

		rc = poll(pfd, nkbd, 1000);
		if (rc <= 0)
			continue;

		for (int i = 0; i < nkbd; i++) {
			struct input_event ev;
			ssize_t n;

			if (!(pfd[i].revents & (POLLIN | POLLERR | POLLHUP)))
				continue;
			if (pfd[i].revents & (POLLERR | POLLHUP)) {
				/* Device vanished — force a rescan next tick. */
				LOGV("keyd: %s went away\n", kbd[i].path);
				close_all(kbd, &nkbd);
				last_scan = 0;
				break;
			}

			while ((n = read(kbd[i].fd, &ev, sizeof(ev))) == sizeof(ev)) {
				/* value 1 = press. Ignore release (0) and autorepeat
				 * (2) so holding Enter cannot toggle recording
				 * dozens of times per second. */
				if (ev.type == EV_KEY && ev.value == 1)
					handle_key(ev.code);
			}
			if (n < 0 && errno != EAGAIN) {
				close_all(kbd, &nkbd);
				last_scan = 0;
				break;
			}
		}
	}

	close_all(kbd, &nkbd);
	return 0;
}
