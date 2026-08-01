# cam-recorder — build for both consumers of this repo:
#   * Yocto (jetson-edge-platform) via a bbclass-driven `make` + `make install`
#   * Raspberry Pi OS via the misc-tools pi5-gmsl-vd56g4 image hook
#
# Deliberately plain: no autotools, no cmake. Both callers just need
#   make && make install DESTDIR=<staging>
# and cross-compilation works by passing CC/CFLAGS/LDFLAGS in, which is exactly
# what OpenEmbedded already exports.

PKGCONFIG ?= pkg-config
prefix    ?= /usr
bindir    ?= $(prefix)/bin
datadir   ?= $(prefix)/share
unitdir   ?= $(prefix)/lib/systemd/system
sysconfdir ?= /etc

GST_PKGS  := gstreamer-1.0 gstreamer-video-1.0 gio-unix-2.0 glib-2.0
GST_CFLAGS := $(shell $(PKGCONFIG) --cflags $(GST_PKGS))
GST_LIBS   := $(shell $(PKGCONFIG) --libs   $(GST_PKGS))

WARN := -Wall -Wextra

BINS := cam-recorder cam-keyd cam-recctl

all: $(BINS)

# The only binary needing GStreamer; keyd and recctl are plain POSIX so they
# build even on a system with no GStreamer dev packages.
cam-recorder: src/cam-recorder.c
	$(CC) $(CFLAGS) $(WARN) $< -o $@ $(GST_CFLAGS) $(LDFLAGS) $(GST_LIBS)

cam-keyd: src/cam-keyd.c
	$(CC) $(CFLAGS) $(WARN) $< -o $@ $(LDFLAGS)

cam-recctl: src/cam-recctl.c
	$(CC) $(CFLAGS) $(WARN) $< -o $@ $(LDFLAGS)

install: all
	install -d $(DESTDIR)$(bindir) $(DESTDIR)$(datadir)/cam-recorder $(DESTDIR)$(unitdir)
	install -m 0755 $(BINS) $(DESTDIR)$(bindir)/
	install -m 0755 scripts/camview $(DESTDIR)$(bindir)/camview
	install -m 0644 share/rec-dot.png $(DESTDIR)$(datadir)/cam-recorder/
	install -m 0644 units/*.service $(DESTDIR)$(unitdir)/
	install -d $(DESTDIR)/lib/udev/rules.d
	install -m 0644 udev/99-usb-media.rules $(DESTDIR)/lib/udev/rules.d/
	install -m 0755 scripts/cam-media-setup $(DESTDIR)$(bindir)/cam-media-setup
	install -d $(DESTDIR)$(sysconfdir)
	echo cam-recorder > $(DESTDIR)$(sysconfdir)/cam-display-app

clean:
	rm -f $(BINS)

.PHONY: all install clean
