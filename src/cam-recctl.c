/*
 * cam-recctl — command-line client for the cam-recorder control socket.
 *
 * Exists because the image has NO unix-socket client at all: socat and python
 * are not installed and busybox nc has no -U. Without this the control socket
 * is unreachable from a shell, so recording could only ever be driven by a
 * physically attached keyboard — untestable over SSH and unscriptable.
 *
 *   cam-recctl status
 *   cam-recctl start | stop | toggle [main|left|right]
 *   cam-recctl watch          # stream STATE pushes until interrupted
 *
 * Exit status is 0 when the recorder answered OK/STATE, 1 on ERR, 2 if the
 * recorder is not running — so shell scripts can branch on it.
 *
 * Copyright (C) 2026, Albert David <albert.david@gmail.com>
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

#define DEFAULT_SOCKET "/run/cam-recorder.sock"

static void usage(const char *me)
{
	fprintf(stderr,
		"usage: %s [--socket=PATH] <command>\n"
		"  status                    show recorder state\n"
		"  start|stop|toggle [chan]  control recording (chan: main|left|right)\n"
		"  watch                     print STATE updates as they happen\n",
		me);
	exit(2);
}

int main(int argc, char **argv)
{
	const char *sockpath = DEFAULT_SOCKET;
	const char *cmd = NULL, *chan = NULL;
	struct sockaddr_un a;
	char buf[1024];
	int fd, i, n, rc = 0;
	int watch = 0;

	for (i = 1; i < argc; i++) {
		if (!strncmp(argv[i], "--socket=", 9))
			sockpath = argv[i] + 9;
		else if (!cmd)
			cmd = argv[i];
		else if (!chan)
			chan = argv[i];
		else
			usage(argv[0]);
	}
	if (!cmd)
		usage(argv[0]);

	fd = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		perror("socket");
		return 2;
	}
	memset(&a, 0, sizeof(a));
	a.sun_family = AF_UNIX;
	snprintf(a.sun_path, sizeof(a.sun_path), "%s", sockpath);

	if (connect(fd, (struct sockaddr *)&a, sizeof(a)) != 0) {
		/* Not an error worth a stack trace: when camview owns the display
		 * the recorder is stopped and its socket simply does not exist. */
		fprintf(stderr, "cam-recctl: recorder not running (%s: %s)\n",
			sockpath, strerror(errno));
		close(fd);
		return 2;
	}

	if (!strcmp(cmd, "watch")) {
		watch = 1;
		n = snprintf(buf, sizeof(buf), "STATUS\n");
	} else if (!strcmp(cmd, "status")) {
		n = snprintf(buf, sizeof(buf), "STATUS\n");
	} else if (!strcmp(cmd, "start") || !strcmp(cmd, "stop") || !strcmp(cmd, "toggle")) {
		char verb[16];
		size_t k;

		for (k = 0; k < sizeof(verb) - 1 && cmd[k]; k++)
			verb[k] = (char)toupper((unsigned char)cmd[k]);
		verb[k] = '\0';
		n = snprintf(buf, sizeof(buf), "REC %s %s\n", verb, chan ? chan : "main");
	} else {
		close(fd);
		usage(argv[0]);
		return 2;
	}

	if (write(fd, buf, n) != n) {
		perror("write");
		close(fd);
		return 2;
	}

	/* The recorder pushes a STATE line to every client BEFORE sending the
	 * command's own reply, so the first read is usually STATE and the OK/ERR
	 * arrives after it. Keep reading until an OK/ERR line is seen (bounded,
	 * so a recorder that answers nothing cannot hang a script) - otherwise
	 * "start" with no stick would exit 0 while printing ERR. */
	{
		int reads = 0;
		int done = 0;
		/* Hard bound on every blocking read. STATUS is answered with a
		 * single STATE line and no OK/ERR terminator, so a loop that waits
		 * for one blocks forever; and a wedged recorder must never hang a
		 * script either. Applies to watch too - it just re-arms. */
		struct timeval tv = { .tv_sec = 3, .tv_usec = 0 };

		setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

		/* STATUS is one line and done - do not go looking for a reply. */
		if (!strcmp(cmd, "status"))
			reads = 3;

		while (!done && (watch || reads < 4)) {
			char *ln;

			n = read(fd, buf, sizeof(buf) - 1);
			if (n <= 0)          /* EOF, or the 3 s timeout expired */
				break;
			buf[n] = '\0';
			fputs(buf, stdout);
			fflush(stdout);
			reads++;

			for (ln = buf; ln && *ln; ) {
				if (!strncmp(ln, "ERR", 3)) {
					rc = 1;
					done = 1;
				} else if (!strncmp(ln, "OK", 2)) {
					done = 1;
				}
				ln = strchr(ln, '\n');
				if (ln)
					ln++;
			}
			if (watch)
				done = 0;
		}
		/* STATUS only ever gets a STATE line, never OK/ERR, so its lack of
		 * a terminator is not a failure. For the action verbs it is. */
		if (!done && !watch && strcmp(cmd, "status"))
			rc = 1;
	}

	close(fd);
	return rc;
}
