/*
 * cam-recorder — live camera view on the local display, plus on-demand
 * recording to a USB stick, with a red REC dot shown while recording.
 *
 * Camera-agnostic: it consumes whatever /dev/video0 offers and forces no caps
 * of its own (--width/--height/--fps/--format override if you need to pin
 * them). It is however TEGRA-specific, not portable: the flip/scale is
 * nvvidconv and the display is nvdrmvideosink, both NVIDIA elements, and the
 * CPU x264 encode is a consequence of the Orin Nano having no NVENC.
 *
 * Why this is a C program and not a gst-launch line: the record branch is
 * added and removed WHILE the pipeline runs, so the live view never stops and
 * each recording gets a properly finalised MP4 (EOS through qtmux). gst-launch
 * cannot restructure a running pipeline.
 *
 * Pipeline (shape validated on hardware, see docs/video-record-plan.md §3):
 *
 *   v4l2src <device's own caps, or --width/--height/--fps/--format>
 *     ! tee                                   <- SYSTEM memory; see note below
 *     +-- queue ! videoconvert RGBA
 *     |     ! gdkpixbufoverlay (red dot)      <- display only, never in the file
 *     |     ! nvvidconv flip-method=2 -> NVMM RGBA <view-width>x<view-height>
 *     |     ! nvdrmvideosink offset-x=...
 *     +-- queue(leaky) ! videoconvert RGBA
 *           ! nvvidconv flip-method=2 -> NVMM NV12
 *           ! nvvidconv -> system I420        <- CPU encoder needs system memory
 *           ! x264enc ultrafast ! h264parse ! qtmux ! filesink
 *
 * Two constraints that are NOT obvious and were measured, not assumed:
 *
 *   1. videoflip must not be used. A CPU rotate-180 on 1124x1364 nearly halves
 *      the frame rate (300 frames: 21 s vs 11 s) and was the entire
 *      bottleneck — x264enc costs almost nothing here. nvvidconv does the same
 *      rotation inside the scale pass it already performs, for free.
 *   2. The tee must carry SYSTEM memory. Teeing video/x-raw(memory:NVMM)
 *      fails with not-negotiated(-4), so each branch does its own hardware
 *      flip. Both are free, so nothing is lost.
 *
 * The overlay must precede nvvidconv: it is a CPU element and cannot accept
 * memory:NVMM buffers. That puts it before the 180 rotation too, so the dot is
 * positioned bottom-left in order to appear top-right on screen.
 *
 * The Orin Nano has no NVENC, so H.264 is necessarily CPU-side via x264enc.
 *
 * Control: newline-delimited text on a Unix socket (default
 * /run/cam-recorder.sock), deliberately shaped like streamdeck-ctrl's push
 * model so that daemon can replace cam-keyd later without this program
 * changing. socat-friendly, so everything is testable over SSH with no
 * keyboard attached.
 *
 *   REC START|STOP|TOGGLE [main]     -> OK RECORDING <path> | OK IDLE | ERR <why>
 *   STATUS                           -> STATE main IDLE|RECORDING|NO_MEDIA ...
 *   (STATE lines are also pushed asynchronously to every connected client)
 *
 * Copyright (C) 2026, Albert David <albert.david@gmail.com>
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <gst/gst.h>
#include <glib-unix.h>
#include <gio/gio.h>
#include <gio/gunixsocketaddress.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/videodev2.h>

#define DEFAULT_DEVICE      "/dev/video0"
#define DEFAULT_SOCKET      "/run/cam-recorder.sock"
#define DEFAULT_RECDIR      "/media/usb/recording"
#define DEFAULT_DOT         "/usr/share/cam-recorder/rec-dot.png"

/* No sensor geometry is compiled in. The source caps are left entirely
 * unconstrained unless the operator asks for something specific, so the app
 * binds to whatever the attached camera actually offers - camview has always
 * worked this way (it sets no caps at all) and it is what makes this program
 * sensor-agnostic rather than just sensor-agnostically NAMED.
 *
 * The view size is the on-screen rectangle, not a sensor property: it defaults
 * to the 888x1080 camview uses, which fits a portrait sensor on a 1080p screen
 * with room for a second view later. Override for a different sensor aspect or
 * a different panel. */
#define DEFAULT_VIEW_W   888
#define DEFAULT_VIEW_H   1080
#define DEFAULT_FPS_HINT 30      /* only used for the encoder keyframe interval */

/* Poll interval for "can we record?", in seconds. Cheap: one access() call. */
#define MEDIA_POLL_SECS 1

typedef enum {
	REC_IDLE = 0,
	REC_RECORDING,
	REC_NO_MEDIA,
} RecState;

typedef struct {
	GstElement *pipeline;
	GstElement *tee;
	GstElement *dispq;          /* display-branch queue, for level reporting */
	GstElement *overlay;        /* gdkpixbufoverlay on the display branch */

	/* Record branch. Created on start, torn down after EOS on stop. */
	GstElement *rec_bin;
	GstPad     *tee_rec_pad;

	gboolean    recording;
	gboolean    media_ready;
	gchar      *cur_path;
	gint64      started_us;

	/* config */
	gchar      *device;
	gchar      *recdir;
	gchar      *dotfile;
	gchar      *sockpath;
	gint        offset_x;
	gint        offset_y;
	gint        bitrate_kbps;
	gboolean    require_mount;  /* recdir's parent must be a real mount point */
	gint        dq_buffers;     /* display queue depth; 0 = GStreamer defaults */
	gboolean    no_tee;         /* diagnostic: chain src straight to the display */
	gboolean    generic;        /* TRUE = no Tegra elements (Pi 5 etc.) */
	gchar      *platform_arg;   /* "tegra" | "generic" | NULL = autodetect */
	gchar      *render_rect;    /* generic only: kmssink render-rectangle */
	gint        io_mode;        /* v4l2src io-mode; -1 = platform default */
	/* All zero/NULL == "let the device decide". Only what is set is forced. */
	gint        src_w, src_h, src_fps;
	gchar      *src_format;
	gint        view_w, view_h;
	gint        flip_method;
	gint        dot_size;
	guint64     lat_sum;        /* frame age at the sink, for --debug latency */
	guint       lat_n;

	GMainLoop  *loop;
	GList      *clients;        /* GSocketConnection*, for async STATE pushes */
} Recorder;

/* ------------------------------------------------------------------ helpers */

static const char *state_name(Recorder *r)
{
	if (r->recording)
		return "RECORDING";
	return r->media_ready ? "IDLE" : "NO_MEDIA";
}

/* A directory is a mount point if its st_dev differs from its parent's. Used
 * because g_mkdir_with_parents() on an UNMOUNTED /media/usb would happily
 * create /media/usb/recording on the root filesystem, making media_is_ready()
 * answer TRUE with no stick attached — recordings would land on the SD card and
 * then be shadowed the moment a real stick got mounted over them. The writable
 * test alone cannot catch that; only the mount check can. */
static gboolean is_mountpoint(const char *path)
{
	struct stat st, parent;
	gchar *dir = g_path_get_dirname(path);
	gboolean res = (stat(path, &st) == 0) && (stat(dir, &parent) == 0) &&
		       (st.st_dev != parent.st_dev);

	g_free(dir);
	return res;
}

/* "Can we record?" is asked as "is the stick mounted AND the recording
 * directory writable", NOT "is a device present". The writable half stays
 * correct if the stick is mounted by something else, and it correctly says no
 * when a bad unplug has left the filesystem read-only — which a device check
 * would miss. */
static gboolean media_is_ready(Recorder *r)
{
	gchar *mnt;
	gboolean ok = FALSE;

	if (r->require_mount) {
		mnt = g_path_get_dirname(r->recdir);
		ok = is_mountpoint(mnt);
		g_free(mnt);
		if (!ok)
			return FALSE;
	}
	if (g_mkdir_with_parents(r->recdir, 0755) != 0)
		return FALSE;
	return access(r->recdir, W_OK) == 0;
}

static gchar *make_path(Recorder *r)
{
	time_t now = time(NULL);
	struct tm tm;
	char stamp[32];

	localtime_r(&now, &tm);
	strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
	return g_strdup_printf("%s/cam-%s.mp4", r->recdir, stamp);
}

static void broadcast(Recorder *r, const char *line);

/* The dot is toggled by alpha rather than by adding/removing the element, so
 * the display branch caps never renegotiate mid-stream. */
static void set_dot(Recorder *r, gboolean on)
{
	if (r->overlay)
		g_object_set(r->overlay, "alpha", on ? 1.0 : 0.0, NULL);
}

static void push_state(Recorder *r)
{
	gchar *line;
	gint64 secs = 0;

	if (r->recording && r->started_us)
		secs = (g_get_monotonic_time() - r->started_us) / G_USEC_PER_SEC;

	line = g_strdup_printf("STATE main %s %s %" G_GINT64_FORMAT,
			       state_name(r),
			       r->cur_path ? r->cur_path : "-",
			       secs);
	broadcast(r, line);
	g_free(line);
}

/* ------------------------------------------------------------ platform probe */

/*
 * Ask the V4L2 node what format it is currently configured for.
 *
 * Needed on the Pi: its RP1 CFE is set up out-of-band by cam-media-setup
 * (media-ctl + v4l2-ctl), and linking v4l2src UNFILTERED then fails with
 * "Buffer pool activation failed / Failed to allocate required memory" - the
 * probe-driven negotiation picks something the CFE cannot allocate. Pinning the
 * caps to what the device already reports keeps the "device decides" behaviour
 * while giving v4l2src a format it can actually use.
 *
 * Tegra does not need this (its tegracam node negotiates fine unfiltered), so
 * it is only applied on generic.
 */
static gboolean query_device_fmt(const char *dev, gint *w, gint *h, const gchar **fmt)
{
	struct v4l2_format f;
	int fd = open(dev, O_RDWR);

	if (fd < 0)
		return FALSE;
	memset(&f, 0, sizeof(f));
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd, VIDIOC_G_FMT, &f) != 0) {
		close(fd);
		return FALSE;
	}
	close(fd);

	*w = f.fmt.pix.width;
	*h = f.fmt.pix.height;
	switch (f.fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_GREY:  *fmt = "GRAY8";  break;
	case V4L2_PIX_FMT_YUYV:  *fmt = "YUY2";   break;
	case V4L2_PIX_FMT_UYVY:  *fmt = "UYVY";   break;
	case V4L2_PIX_FMT_NV12:  *fmt = "NV12";   break;
	case V4L2_PIX_FMT_RGB24: *fmt = "RGB";    break;
	case V4L2_PIX_FMT_BGR24: *fmt = "BGR";    break;
	default:                 *fmt = NULL;     break;   /* pin size only */
	}
	return (*w > 0 && *h > 0);
}

/*
 * Which pipeline shape to build. The two platforms differ only at the ends -
 * the flip/scale element and the display sink - so this is a runtime choice
 * rather than a compile-time one, and a single binary serves both.
 *
 * Autodetection is "does nvdrmvideosink exist", which is true only on a Tegra
 * image with the NVIDIA GStreamer plugins installed. --platform overrides it.
 */
static gboolean platform_is_generic(Recorder *r)
{
	GstElementFactory *f;
	gboolean generic;

	if (r->platform_arg) {
		if (!g_ascii_strcasecmp(r->platform_arg, "generic"))
			return TRUE;
		if (!g_ascii_strcasecmp(r->platform_arg, "tegra"))
			return FALSE;
		g_warning("unknown --platform=%s, autodetecting", r->platform_arg);
	}
	f = gst_element_factory_find("nvdrmvideosink");
	generic = (f == NULL);
	if (f)
		gst_object_unref(f);
	return generic;
}

/*
 * Largest mode of the first connected DRM connector.
 *
 * Needed because kmssink only negotiates at the DISPLAY MODE SIZE - measured on
 * Pi 5, where 888x1080 fails not-negotiated(-4) with and without
 * render-rectangle, while 1920x1080 works. So on generic the display branch
 * must scale to the panel's mode and position afterwards with render-rectangle,
 * unlike Tegra where nvdrmvideosink happily takes an arbitrary-sized buffer and
 * places it with offset-x.
 */
static void detect_display_mode(gint *w, gint *h)
{
	GDir *d = g_dir_open("/sys/class/drm", 0, NULL);
	const char *name;

	*w = 1920; *h = 1080;                 /* sane default if nothing is readable */
	if (!d)
		return;
	while ((name = g_dir_read_name(d))) {
		gchar *st = NULL, *modes = NULL, *pst, *pmo;
		int mw, mh;

		if (!strchr(name, '-'))           /* connectors look like card1-HDMI-A-1 */
			continue;
		pst = g_strdup_printf("/sys/class/drm/%s/status", name);
		pmo = g_strdup_printf("/sys/class/drm/%s/modes", name);
		if (g_file_get_contents(pst, &st, NULL, NULL) &&
		    g_str_has_prefix(st, "connected") &&
		    g_file_get_contents(pmo, &modes, NULL, NULL) &&
		    sscanf(modes, "%dx%d", &mw, &mh) == 2 && mw > 0 && mh > 0) {
			*w = mw; *h = mh;
			g_free(st); g_free(modes); g_free(pst); g_free(pmo);
			break;
		}
		g_free(st); g_free(modes); g_free(pst); g_free(pmo);
	}
	g_dir_close(d);
}

/* ------------------------------------------------------- record branch mgmt */

/* Built as a bin so start/stop is one link and one unlink. Sink pad of the bin
 * is the ghost pad on the leaky queue. */
static GstElement *build_rec_bin(Recorder *r, const char *path, GError **err)
{
	GstElement *bin, *q, *conv, *flip, *tosys, *enc, *parse, *mux, *sink;
	GstCaps *caps_nvmm, *caps_sys, *caps_rgba;
	GstPad *qsink;
	gboolean ok;

	bin   = gst_bin_new("recbin");
	q     = gst_element_factory_make("queue", "recq");
	conv  = gst_element_factory_make("videoconvert", "recconv");
	/* Tegra: nvvidconv does the rotate in hardware and a second one pulls the
	 * frame back out of NVMM for the CPU encoder. Generic: the rotate happens
	 * once before the tee (cheaper than per-branch), so the record branch only
	 * needs the colour conversion that videoconvert above already does. */
	flip  = r->generic ? NULL : gst_element_factory_make("nvvidconv", "recflip");
	tosys = r->generic ? NULL : gst_element_factory_make("nvvidconv", "rectosys");
	enc   = gst_element_factory_make("x264enc", "recenc");
	parse = gst_element_factory_make("h264parse", "recparse");
	mux   = gst_element_factory_make("qtmux", "recmux");
	sink  = gst_element_factory_make("filesink", "recsink");

	if (!bin || !q || !conv || !enc || !parse || !mux || !sink ||
	    (!r->generic && (!flip || !tosys))) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
			    "missing GStreamer element for the record branch");
		return NULL;
	}

	/* Leaky: if the encoder ever falls behind, drop RECORDED frames rather
	 * than back-pressuring through the tee and stalling the live view. The
	 * live view is the thing the user is watching; it must never suffer for
	 * the recording. */
	g_object_set(q, "leaky", 2 /* downstream */, "max-size-buffers", 8,
		     "max-size-bytes", 0, "max-size-time", (guint64)0, NULL);
	if (flip)
		g_object_set(flip, "flip-method", r->flip_method, NULL);
	g_object_set(enc, "tune", 0x4 /* zerolatency */, "speed-preset", 1 /* ultrafast */,
		     "bitrate", r->bitrate_kbps, "key-int-max", (r->src_fps ? r->src_fps : DEFAULT_FPS_HINT) * 2, NULL);
	g_object_set(sink, "location", path, "sync", FALSE, "async", FALSE, NULL);
	/* Let qtmux write a still-playable file if we are killed without EOS. */
	g_object_set(mux, "faststart", FALSE, "fragment-duration", 1000, NULL);

	gst_bin_add_many(GST_BIN(bin), q, conv, enc, parse, mux, sink, NULL);
	if (!r->generic)
		gst_bin_add_many(GST_BIN(bin), flip, tosys, NULL);

	/* link_filtered does NOT take ownership of the caps, so every one of
	 * these needs an explicit unref on both the success and failure path. */
	caps_nvmm = gst_caps_from_string("video/x-raw(memory:NVMM),format=(string)NV12");
	caps_sys  = gst_caps_from_string("video/x-raw,format=(string)I420");
	caps_rgba = gst_caps_from_string("video/x-raw,format=(string)RGBA");

	if (r->generic)
		ok = gst_element_link(q, conv) &&
		     gst_element_link_filtered(conv, enc, caps_sys) &&
		     gst_element_link_many(enc, parse, mux, sink, NULL);
	else
		ok = gst_element_link(q, conv) &&
		     gst_element_link_filtered(conv, flip, caps_rgba) &&
		     gst_element_link_filtered(flip, tosys, caps_nvmm) &&
		     gst_element_link_filtered(tosys, enc, caps_sys) &&
		     gst_element_link_many(enc, parse, mux, sink, NULL);

	gst_caps_unref(caps_nvmm);
	gst_caps_unref(caps_sys);
	gst_caps_unref(caps_rgba);

	if (!ok) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
			    "failed to link the record branch");
		gst_object_unref(bin);
		return NULL;
	}

	qsink = gst_element_get_static_pad(q, "sink");
	gst_element_add_pad(bin, gst_ghost_pad_new("sink", qsink));
	gst_object_unref(qsink);

	return bin;
}

static gboolean rec_start(Recorder *r, gchar **reply)
{
	GError *err = NULL;
	GstPad *sinkpad;

	if (r->recording) {
		*reply = g_strdup_printf("OK RECORDING %s", r->cur_path);
		return TRUE;
	}
	/* No stick -> silently refuse. Per the agreed behaviour the key press is
	 * ignored with no on-screen feedback; the socket still reports why. */
	if (!media_is_ready(r)) {
		r->media_ready = FALSE;
		*reply = g_strdup("ERR NO_MEDIA");
		push_state(r);
		return FALSE;
	}

	g_free(r->cur_path);
	r->cur_path = make_path(r);

	r->rec_bin = build_rec_bin(r, r->cur_path, &err);
	if (!r->rec_bin) {
		*reply = g_strdup_printf("ERR %s", err ? err->message : "recbin");
		g_clear_error(&err);
		g_free(r->cur_path);
		r->cur_path = NULL;
		return FALSE;
	}

	gst_bin_add(GST_BIN(r->pipeline), r->rec_bin);
	/* Sync state BEFORE linking so the new bin is ready for the first buffer. */
	if (!gst_element_sync_state_with_parent(r->rec_bin)) {
		*reply = g_strdup("ERR recbin state");
		gst_bin_remove(GST_BIN(r->pipeline), r->rec_bin);
		r->rec_bin = NULL;
		return FALSE;
	}

	r->tee_rec_pad = gst_element_request_pad_simple(r->tee, "src_%u");

	/* Re-base the branch to zero. Buffers coming off the tee carry the
	 * PIPELINE's running time, which is however long the live view has been
	 * up — so without this qtmux writes that offset as leading duration and
	 * a 12 s recording reports itself as 27 s (measured). Shifting the pad by
	 * -running_time makes the first recorded buffer land at PTS 0. */
	{
		GstClock *clk = gst_element_get_clock(r->pipeline);

		if (clk) {
			GstClockTime base = gst_element_get_base_time(r->pipeline);
			GstClockTime now  = gst_clock_get_time(clk);

			if (now > base)
				gst_pad_set_offset(r->tee_rec_pad, -(gint64)(now - base));
			gst_object_unref(clk);
		}
	}

	sinkpad = gst_element_get_static_pad(r->rec_bin, "sink");
	if (gst_pad_link(r->tee_rec_pad, sinkpad) != GST_PAD_LINK_OK) {
		*reply = g_strdup("ERR tee link");
		gst_object_unref(sinkpad);
		gst_element_release_request_pad(r->tee, r->tee_rec_pad);
		gst_object_unref(r->tee_rec_pad);
		r->tee_rec_pad = NULL;
		gst_bin_remove(GST_BIN(r->pipeline), r->rec_bin);
		r->rec_bin = NULL;
		return FALSE;
	}
	gst_object_unref(sinkpad);

	r->recording  = TRUE;
	r->started_us = g_get_monotonic_time();
	set_dot(r, TRUE);
	g_message("recording -> %s", r->cur_path);
	*reply = g_strdup_printf("OK RECORDING %s", r->cur_path);
	push_state(r);
	return TRUE;
}

/* Disposal is deferred to the main loop: tearing elements down from the
 * streaming thread is not safe. Callers hold a reference for the callback.
 *
 * Reached from two places — the EOS probe (normal) and a backstop timeout (if
 * EOS never arrives because an element errored). Whichever runs first does the
 * work; the other only drops its reference. */
static gboolean drop_rec_bin(gpointer user)
{
	GstElement *bin = user;

	if (!g_object_get_data(G_OBJECT(bin), "dropped")) {
		GstObject *parent = gst_element_get_parent(bin);

		g_object_set_data(G_OBJECT(bin), "dropped", GINT_TO_POINTER(1));
		gst_element_set_state(bin, GST_STATE_NULL);
		if (parent) {
			gst_bin_remove(GST_BIN(parent), bin);
			gst_object_unref(parent);
		}
	}
	gst_object_unref(bin);
	return G_SOURCE_REMOVE;
}

/* Fires when EOS has traversed the whole record branch. qtmux writes the moov
 * atom on EOS and pushes it downstream BEFORE forwarding the event, so by the
 * time EOS reaches filesink's sink pad every byte of the file has been written
 * and it is safe to go to NULL. Waiting for this rather than guessing a delay
 * is what guarantees a valid, seekable MP4. */
static GstPadProbeReturn rec_eos_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user)
{
	(void)pad;
	if (GST_EVENT_TYPE(GST_PAD_PROBE_INFO_EVENT(info)) != GST_EVENT_EOS)
		return GST_PAD_PROBE_OK;

	g_idle_add(drop_rec_bin, gst_object_ref(user));
	/* Swallow it: this EOS belongs to the record branch alone and must not
	 * reach the pipeline bus, where it would look like the whole stream
	 * ended and take the live view down with it. */
	return GST_PAD_PROBE_DROP;
}

/* Idle probe: the tee pad is blocked, so no buffer can enter the record branch
 * while we detach it. EOS is then sent INTO the branch so qtmux finalises the
 * MP4 index — without that the file is unplayable. */
static GstPadProbeReturn unlink_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user)
{
	Recorder *r = user;
	GstElement *bin, *fsink;
	GstPad *sinkpad;

	(void)info;
	if (!r->rec_bin)
		return GST_PAD_PROBE_REMOVE;

	bin = r->rec_bin;
	r->rec_bin = NULL;

	sinkpad = gst_element_get_static_pad(bin, "sink");
	gst_pad_unlink(pad, sinkpad);

	fsink = gst_bin_get_by_name(GST_BIN(bin), "recsink");
	if (fsink) {
		GstPad *fpad = gst_element_get_static_pad(fsink, "sink");

		gst_pad_add_probe(fpad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM,
				  rec_eos_probe, bin,
				  (GDestroyNotify)gst_object_unref);
		gst_object_ref(bin);        /* held by the probe's destroy notify */
		gst_object_unref(fpad);
		gst_object_unref(fsink);
	}

	gst_pad_send_event(sinkpad, gst_event_new_eos());
	gst_object_unref(sinkpad);

	gst_element_release_request_pad(r->tee, pad);
	if (r->tee_rec_pad) {
		gst_object_unref(r->tee_rec_pad);
		r->tee_rec_pad = NULL;
	}

	/* Backstop: if an element errors, EOS never lands and the probe above
	 * never fires. Tear down anyway so a failed recording cannot leak a bin
	 * into the running pipeline for the rest of the session. */
	g_timeout_add(5000, drop_rec_bin, gst_object_ref(bin));
	return GST_PAD_PROBE_REMOVE;
}

static gboolean rec_stop(Recorder *r, gchar **reply)
{
	if (!r->recording) {
		*reply = g_strdup("OK IDLE");
		return TRUE;
	}

	r->recording = FALSE;
	set_dot(r, FALSE);
	g_message("stopping recording -> %s", r->cur_path ? r->cur_path : "?");

	if (r->tee_rec_pad)
		gst_pad_add_probe(r->tee_rec_pad, GST_PAD_PROBE_TYPE_IDLE,
				  unlink_probe, r, NULL);

	*reply = g_strdup("OK IDLE");
	push_state(r);
	return TRUE;
}

/* ------------------------------------------------------------ control socket */

static void broadcast(Recorder *r, const char *line)
{
	gchar *msg = g_strdup_printf("%s\n", line);
	GList *l = r->clients, *next;

	while (l) {
		GOutputStream *os;
		next = l->next;
		os = g_io_stream_get_output_stream(G_IO_STREAM(l->data));
		/* Write errors are IGNORED on purpose. A dead client is reaped by
		 * the read path (on_line), which owns the reference; dropping it
		 * here too would unref the same connection twice. */
		g_output_stream_write_all(os, msg, strlen(msg), NULL, NULL, NULL);
		l = next;
	}
	g_free(msg);
}

static gchar *handle_command(Recorder *r, const char *line)
{
	gchar **tok = g_strsplit_set(g_strstrip((gchar *)line), " \t", -1);
	gchar *reply = NULL;

	if (!tok[0] || !*tok[0]) {
		reply = g_strdup("ERR empty");
	} else if (g_ascii_strcasecmp(tok[0], "STATUS") == 0) {
		gint64 secs = (r->recording && r->started_us)
			? (g_get_monotonic_time() - r->started_us) / G_USEC_PER_SEC : 0;
		reply = g_strdup_printf("STATE main %s %s %" G_GINT64_FORMAT,
					state_name(r),
					r->cur_path ? r->cur_path : "-", secs);
	} else if (g_ascii_strcasecmp(tok[0], "REC") == 0 && tok[1]) {
		/* Channel argument is accepted and currently must be "main";
		 * left/right exist in the protocol for the dual-camera work so
		 * neither the verbs nor the key map have to churn later. */
		const char *chan = tok[2] ? tok[2] : "main";
		if (g_ascii_strcasecmp(chan, "main") != 0) {
			reply = g_strdup("ERR channel not implemented");
		} else if (g_ascii_strcasecmp(tok[1], "START") == 0) {
			rec_start(r, &reply);
		} else if (g_ascii_strcasecmp(tok[1], "STOP") == 0) {
			rec_stop(r, &reply);
		} else if (g_ascii_strcasecmp(tok[1], "TOGGLE") == 0) {
			if (r->recording)
				rec_stop(r, &reply);
			else
				rec_start(r, &reply);
		} else {
			reply = g_strdup("ERR bad REC verb");
		}
	} else {
		reply = g_strdup("ERR unknown command");
	}

	g_strfreev(tok);
	return reply;
}

typedef struct {
	Recorder          *r;
	GSocketConnection *conn;
	GDataInputStream  *in;
} Client;

static void client_read_line(Client *c);

static void on_line(GObject *src, GAsyncResult *res, gpointer user)
{
	Client *c = user;
	gsize len = 0;
	GError *err = NULL;
	gchar *line = g_data_input_stream_read_line_finish(G_DATA_INPUT_STREAM(src),
							   res, &len, &err);
	if (!line) {
		c->r->clients = g_list_remove(c->r->clients, c->conn);
		g_clear_error(&err);
		g_object_unref(c->in);
		g_object_unref(c->conn);
		g_free(c);
		return;
	}

	gchar *reply = handle_command(c->r, line);
	if (reply) {
		gchar *out = g_strdup_printf("%s\n", reply);
		GOutputStream *os = g_io_stream_get_output_stream(G_IO_STREAM(c->conn));
		g_output_stream_write_all(os, out, strlen(out), NULL, NULL, NULL);
		g_free(out);
		g_free(reply);
	}
	g_free(line);
	client_read_line(c);
}

static void client_read_line(Client *c)
{
	g_data_input_stream_read_line_async(c->in, G_PRIORITY_DEFAULT, NULL, on_line, c);
}

static gboolean on_incoming(GSocketService *svc, GSocketConnection *conn,
			    GObject *sobj, gpointer user)
{
	Recorder *r = user;
	Client *c = g_new0(Client, 1);

	(void)svc; (void)sobj;
	c->r = r;
	c->conn = g_object_ref(conn);
	c->in = g_data_input_stream_new(g_io_stream_get_input_stream(G_IO_STREAM(conn)));
	g_data_input_stream_set_newline_type(c->in, G_DATA_STREAM_NEWLINE_TYPE_ANY);

	r->clients = g_list_prepend(r->clients, c->conn);
	client_read_line(c);
	return TRUE;
}

/* ------------------------------------------------------------------- pipeline */

static gboolean on_bus(GstBus *bus, GstMessage *msg, gpointer user)
{
	Recorder *r = user;
	GError *err = NULL;
	gchar *dbg = NULL;

	(void)bus;
	switch (GST_MESSAGE_TYPE(msg)) {
	case GST_MESSAGE_ERROR:
		gst_message_parse_error(msg, &err, &dbg);
		g_printerr("ERROR from %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
		if (dbg)
			g_printerr("  debug: %s\n", dbg);
		g_clear_error(&err);
		g_free(dbg);
		g_main_loop_quit(r->loop);
		break;
	case GST_MESSAGE_WARNING:
		gst_message_parse_warning(msg, &err, &dbg);
		g_warning("%s: %s", GST_OBJECT_NAME(msg->src), err->message);
		g_clear_error(&err);
		g_free(dbg);
		break;
	default:
		break;
	}
	return TRUE;
}

static GstPadProbeReturn sink_lat_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user);

static gboolean build_pipeline(Recorder *r, GError **err)
{
	GstElement *src, *q, *conv, *flip, *sink;
	GstElement *scale = NULL, *conv2 = NULL, *preflip = NULL;
	GstCaps *caps_src, *caps_rgba, *caps_view;
	GstBus *bus;

	r->pipeline = gst_pipeline_new("cam-recorder");
	src   = gst_element_factory_make("v4l2src", "src");
	r->tee = gst_element_factory_make("tee", "t");
	q = r->dispq = gst_element_factory_make("queue", "dispq");
	conv  = gst_element_factory_make("videoconvert", "dispconv");
	r->overlay = gst_element_factory_make("gdkpixbufoverlay", "recdot");

	if (r->generic) {
		/* No hardware converter: videoscale does the fit and kmssink the
		 * presentation. A CPU rotate, when needed, goes BEFORE the tee so
		 * one pass serves display and record - measured at 2 dropped frames
		 * in 300 on a Pi 5, so it is affordable here (unlike Tegra, where a
		 * CPU videoflip halved the frame rate and nvvidconv is free). */
		flip  = NULL;
		scale = gst_element_factory_make("videoscale", "dispscale");
		conv2 = gst_element_factory_make("videoconvert", "dispconv2");
		sink  = gst_element_factory_make("kmssink", "dispsink");
		if (r->flip_method)
			preflip = gst_element_factory_make("videoflip", "preflip");
	} else {
		flip  = gst_element_factory_make("nvvidconv", "dispflip");
		sink  = gst_element_factory_make("nvdrmvideosink", "dispsink");
	}

	if (!r->pipeline || !src || !r->tee || !q || !conv || !sink ||
	    (r->generic ? (!scale || !conv2 || (r->flip_method && !preflip)) : !flip)) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
			    "missing GStreamer element for the display branch (%s)",
			    r->generic ? "generic/kmssink" : "tegra/nvdrmvideosink");
		return FALSE;
	}

	g_object_set(src, "device", r->device, NULL);
	/* io-mode: auto negotiation fails on the Pi's RP1 CFE with
	 * "Buffer pool activation failed / Failed to allocate required memory";
	 * mmap (2) is what the proven Pi pipeline uses. Tegra is left on auto,
	 * where v4l2src negotiates DMABuf/DMA_DRM R8 and that path is wanted. */
	if (r->io_mode >= 0)
		g_object_set(src, "io-mode", r->io_mode, NULL);
	else if (r->generic)
		g_object_set(src, "io-mode", 2 /* mmap */, NULL);
	/* The display queue MUST be small and leaky. With the GStreamer defaults
	 * (non-leaky, 200 buffers / 10 MB / 1 s) the live view ran 1-1.5 s behind
	 * reality; small + leaky made it look live again. Observed on hardware.
	 *
	 * The mechanism is inferred, not measured: a non-leaky queue back-pressures
	 * the tee and hence v4l2src, which then stops dequeuing, so frames pile up
	 * in the KERNEL's V4L2 buffer ring and each one handed over is already
	 * stale. Leaky=downstream never back-pressures, so the ring stays shallow.
	 *
	 * Note this delay is INVISIBLE to the obvious instrumentation: v4l2src
	 * stamps PTS at dequeue, so a probe comparing PTS to the clock reported a
	 * healthy ~70 ms the whole time the view was visibly 1.5 s late, and the
	 * queue itself read 0-2 buffers. Do not "verify" this with a pad probe.
	 *
	 * For a live view the newest frame beats every frame, so dropping is right.
	 * --dq=0 restores the defaults for comparison. */
	if (r->dq_buffers > 0)
		g_object_set(q, "leaky", 2 /* downstream */,
			     "max-size-buffers", r->dq_buffers,
			     "max-size-bytes", 0, "max-size-time", (guint64)0, NULL);
	if (flip)
		g_object_set(flip, "flip-method", r->flip_method, NULL);
	if (preflip)
		g_object_set(preflip, "method", 2 /* rotate-180 */, NULL);
	if (scale)
		g_object_set(scale, "add-borders", TRUE, NULL);
	/* Carried over from the stutter fix: sync=true is REQUIRED (the sink
	 * schedules DRM page-flips off the clock; sync=false blanks the display
	 * entirely), while qos=false + max-lateness=-1 stop it discarding frames
	 * it judges late against an undisciplined clock. See
	 * gmsl-vd56g4/docs/ORIN-WAVESHARE-BRINGUP.md §5. */
	if (r->generic) {
		/* force-modesetting so this works on a console-only image with no
		 * compositor holding the CRTC. render-rectangle is the offset-x
		 * equivalent, but it positions a MODE-SIZED buffer - it cannot be
		 * used to hand kmssink a smaller one (that fails to negotiate), so
		 * it is left unset unless asked for. */
		g_object_set(sink, "sync", TRUE, "force-modesetting", TRUE, NULL);
		if (r->render_rect)
			gst_util_set_object_arg(G_OBJECT(sink), "render-rectangle",
						r->render_rect);
	} else {
		g_object_set(sink, "sync", TRUE, "qos", FALSE,
			     "max-lateness", (gint64)-1,
			     "offset-x", r->offset_x, "offset-y", r->offset_y, NULL);
	}

	gst_bin_add_many(GST_BIN(r->pipeline), src, conv, sink, NULL);
	if (r->generic)
		gst_bin_add_many(GST_BIN(r->pipeline), scale, conv2, NULL);
	else
		gst_bin_add(GST_BIN(r->pipeline), flip);
	if (preflip)
		gst_bin_add(GST_BIN(r->pipeline), preflip);
	if (r->no_tee) {
		gst_object_unref(r->tee);
		gst_object_unref(q);
		r->tee = NULL;
		r->dispq = q = NULL;
	} else {
		gst_bin_add_many(GST_BIN(r->pipeline), r->tee, q, NULL);
	}
	if (r->overlay) {
		/* The overlay sits BEFORE nvvidconv, so the dot is composited into
		 * the un-flipped 1124x1364 frame and nvvidconv then rotates it 180
		 * with everything else. To land TOP-RIGHT on screen it must be
		 * placed BOTTOM-LEFT here: positive offset-x is from the left edge,
		 * negative offset-y from the bottom. (The dot is a symmetric circle,
		 * so being rotated with the frame is invisible.)
		 *
		 * It cannot go after nvvidconv: gdkpixbufoverlay is a CPU element
		 * and does not accept memory:NVMM buffers - linking it there fails
		 * outright with "display link failed (overlay)".
		 *
		 * Scaled up because the source is downscaled 1124->888 afterwards;
		 * 61 px here lands at ~48 px on screen. Alpha 0 hides it without
		 * renegotiating caps mid-stream. */
		/* On generic the rotate happens before the tee, so the overlay
		 * already sees an upright frame and the dot goes top-right
		 * directly: negative offset-x measures from the right edge. */
		g_object_set(r->overlay, "location", r->dotfile,
			     "offset-x", r->generic ? -30 : 30,
			     "offset-y", r->generic ? 30 : -30,
			     "overlay-width", r->dot_size, "overlay-height", r->dot_size,
			     "alpha", 0.0, NULL);
		gst_bin_add(GST_BIN(r->pipeline), r->overlay);
	} else {
		g_warning("gdkpixbufoverlay unavailable — no REC indicator");
	}

	/* Built from the overrides only. If none were given this stays NULL and
	 * the source link is unfiltered, i.e. the device picks. */
	caps_src = NULL;
	if (r->generic && !r->src_w && !r->src_h && !r->src_format) {
		gint dw = 0, dh = 0;
		const gchar *df = NULL;

		if (query_device_fmt(r->device, &dw, &dh, &df)) {
			r->src_w = dw;
			r->src_h = dh;
			if (df && !r->src_format)
				r->src_format = g_strdup(df);
			g_message("source: device reports %dx%d %s", dw, dh, df ? df : "(unmapped)");
		}
	}
	if (r->src_w || r->src_h || r->src_fps || r->src_format) {
		caps_src = gst_caps_new_empty_simple("video/x-raw");
		if (r->src_format)
			gst_caps_set_simple(caps_src, "format", G_TYPE_STRING, r->src_format, NULL);
		if (r->src_w)
			gst_caps_set_simple(caps_src, "width", G_TYPE_INT, r->src_w, NULL);
		if (r->src_h)
			gst_caps_set_simple(caps_src, "height", G_TYPE_INT, r->src_h, NULL);
		if (r->src_fps)
			gst_caps_set_simple(caps_src, "framerate",
					    GST_TYPE_FRACTION, r->src_fps, 1, NULL);
	}
	caps_rgba = gst_caps_from_string("video/x-raw,format=(string)RGBA");
	if (r->generic) {
		/* Plain system memory, and MUST be the display mode size - see
		 * detect_display_mode(). No format is pinned: kmssink and
		 * videoconvert negotiate one between them. */
		caps_view = gst_caps_new_simple("video/x-raw",
				"width",  G_TYPE_INT, r->view_w,
				"height", G_TYPE_INT, r->view_h,
				"pixel-aspect-ratio", GST_TYPE_FRACTION, 1, 1, NULL);
	} else {
		caps_view = gst_caps_new_simple("video/x-raw",
				"format", G_TYPE_STRING, "RGBA",
				"width",  G_TYPE_INT, r->view_w,
				"height", G_TYPE_INT, r->view_h, NULL);
		gst_caps_set_features(caps_view, 0,
				gst_caps_features_new("memory:NVMM", NULL));
	}

	if (r->no_tee) {
		/* Diagnostic only: src -> conv directly, exactly camview's shape.
		 * Recording is impossible in this mode (nothing to branch from). */
		if (!(caps_src ? gst_element_link_filtered(src, conv, caps_src)
			       : gst_element_link(src, conv))) {
			g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED, "display link failed (no-tee)");
			return FALSE;
		}
	} else if (!(preflip
		     ? ((caps_src ? gst_element_link_filtered(src, preflip, caps_src)
				  : gst_element_link(src, preflip)) &&
			gst_element_link(preflip, r->tee))
		     : (caps_src ? gst_element_link_filtered(src, r->tee, caps_src)
				 : gst_element_link(src, r->tee))) ||
		   !gst_element_link(r->tee, q) ||
		   !gst_element_link(q, conv)) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED, "display link failed (front)");
		return FALSE;
	}
	/* conv -> [overlay] -> flip: the overlay works in SYSTEM memory RGBA. */
	if (r->overlay) {
		/* RGBA because gdkpixbufoverlay needs an alpha channel to blend. */
		if (!gst_element_link_filtered(conv, r->overlay, caps_rgba)) {
			g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
				    "display link failed (overlay)");
			return FALSE;
		}
		if (!r->generic && !gst_element_link(r->overlay, flip)) {
			g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
				    "display link failed (overlay->flip)");
			return FALSE;
		}
	} else if (!r->generic && !gst_element_link_filtered(conv, flip, caps_rgba)) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED, "display link failed (conv)");
		return FALSE;
	}
	if (r->generic) {
		/* overlay/conv -> videoconvert -> videoscale -> [mode size] -> kmssink */
		GstElement *head = r->overlay ? r->overlay : conv;

		if (!gst_element_link(head, conv2) ||
		    !gst_element_link(conv2, scale) ||
		    !gst_element_link_filtered(scale, sink, caps_view)) {
			g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED,
				    "display link failed (generic sink)");
			return FALSE;
		}
	} else if (!gst_element_link_filtered(flip, sink, caps_view)) {
		g_set_error(err, G_IO_ERROR, G_IO_ERROR_FAILED, "display link failed (sink)");
		return FALSE;
	}

	if (caps_src)
		gst_caps_unref(caps_src);
	gst_caps_unref(caps_rgba);
	gst_caps_unref(caps_view);

	if (g_getenv("CAM_DEBUG_QUEUE")) {
		GstPad *sp = gst_element_get_static_pad(sink, "sink");

		gst_pad_add_probe(sp, GST_PAD_PROBE_TYPE_BUFFER, sink_lat_probe, r, NULL);
		gst_object_unref(sp);
	}

	bus = gst_element_get_bus(r->pipeline);
	gst_bus_add_watch(bus, on_bus, r);
	gst_object_unref(bus);
	return TRUE;
}

/* ---------------------------------------------------------------- main loop */

/* Frame age at the display sink: pipeline running time now, minus the buffer's
 * PTS (which v4l2src stamps at capture). This is the true end-to-end latency of
 * the live view, and it is measured rather than inferred from queue levels -
 * the queue can read empty while frames are delayed somewhere with no
 * introspectable level at all. */
static GstPadProbeReturn sink_lat_probe(GstPad *pad, GstPadProbeInfo *info, gpointer user)
{
	Recorder *r = user;
	GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
	GstClockTime pts;
	GstClock *clk;

	(void)pad;
	if (!buf)
		return GST_PAD_PROBE_OK;
	pts = GST_BUFFER_PTS(buf);
	if (!GST_CLOCK_TIME_IS_VALID(pts))
		return GST_PAD_PROBE_OK;

	clk = gst_element_get_clock(r->pipeline);
	if (clk) {
		GstClockTime base = gst_element_get_base_time(r->pipeline);
		GstClockTime now  = gst_clock_get_time(clk);

		if (now > base && (now - base) > pts) {
			r->lat_sum += (now - base) - pts;
			r->lat_n++;
		}
		gst_object_unref(clk);
	}
	return GST_PAD_PROBE_OK;
}

/* Reports how many frames the display queue is sitting on. This is the live
 * view's added latency: each buffered frame is 1/30 s of delay. */
static gboolean poll_qlevel(gpointer user)
{
	Recorder *r = user;
	guint bufs = 0, bytes = 0;
	guint64 t = 0;

	if (r->dispq)
		g_object_get(r->dispq, "current-level-buffers", &bufs,
			     "current-level-bytes", &bytes,
			     "current-level-time", &t, NULL);
	{
		/* The CONFIGURED latency is what the sink adds on top of the frame's
		 * age: with sync=true it holds each buffer until
		 * running_time == pts + latency. A probe on the sink pad runs BEFORE
		 * that wait, so this query is the only way to see it. */
		GstQuery *q = gst_query_new_latency();
		gboolean live = FALSE;
		GstClockTime qmin = 0, qmax = 0;

		if (gst_element_query(r->pipeline, q))
			gst_query_parse_latency(q, &live, &qmin, &qmax);
		gst_query_unref(q);

		g_message("dispq: %u buf %.0f ms | frame age at sink: %.0f ms (n=%u) | "
			  "pipeline latency: live=%d min=%.0f ms max=%.0f ms",
			  bufs, (double)t / 1e6,
			  r->lat_n ? (double)r->lat_sum / r->lat_n / 1e6 : -1.0, r->lat_n,
			  live, (double)qmin / 1e6,
			  GST_CLOCK_TIME_IS_VALID(qmax) ? (double)qmax / 1e6 : -1.0);
	}
	(void)bytes;
	r->lat_sum = 0;
	r->lat_n = 0;
	return G_SOURCE_CONTINUE;
}

static gboolean poll_media(gpointer user)
{
	Recorder *r = user;
	gboolean now = media_is_ready(r);

	if (now != r->media_ready) {
		r->media_ready = now;
		g_message("recording media %s", now ? "available" : "gone");
		/* If the stick vanished mid-recording, stop cleanly rather than
		 * writing into a dead mount. The already-written bytes are what
		 * they are; qtmux fragments mean the file is usually playable. */
		if (!now && r->recording) {
			gchar *reply = NULL;
			g_warning("media disappeared while recording — stopping");
			rec_stop(r, &reply);
			g_free(reply);
		}
		push_state(r);
	}
	return G_SOURCE_CONTINUE;
}

static gboolean quit_loop(gpointer user)
{
	g_main_loop_quit(((Recorder *)user)->loop);
	return G_SOURCE_REMOVE;
}

static gboolean on_signal(gpointer user)
{
	Recorder *r = user;
	gchar *reply = NULL;

	if (r->recording) {
		g_message("signal received — finalising recording");
		rec_stop(r, &reply);
		g_free(reply);
		/* Give EOS time to reach filesink, but via a TIMEOUT, not a sleep:
		 * sleeping here would block the main loop that has to run the
		 * teardown idle, so the file would never be finalised. */
		g_timeout_add(2000, quit_loop, r);
		return G_SOURCE_REMOVE;
	}
	g_main_loop_quit(r->loop);
	return G_SOURCE_REMOVE;
}

int main(int argc, char **argv)
{
	Recorder r = { 0 };
	GError *err = NULL;
	GSocketService *svc;
	GSocketAddress *addr;
	GOptionContext *ctx;

	r.device       = g_strdup(DEFAULT_DEVICE);
	r.recdir       = g_strdup(DEFAULT_RECDIR);
	r.dotfile      = g_strdup(DEFAULT_DOT);
	r.sockpath     = g_strdup(DEFAULT_SOCKET);
	r.offset_x     = 516;      /* centred: (1920-888)/2 */
	r.offset_y     = 0;
	r.bitrate_kbps = 8000;
	r.require_mount = TRUE;
	r.dq_buffers    = 2;
	r.view_w        = 0;   /* 0 => platform default, resolved after gst_init */
	r.view_h        = 0;
	r.flip_method   = -1;  /* -1 => platform default */
	r.io_mode       = -1;
	r.dot_size      = 61;

	GOptionEntry opts[] = {
		{ "device",   'd', 0, G_OPTION_ARG_STRING, &r.device,   "V4L2 device", NULL },
		{ "recdir",   'r', 0, G_OPTION_ARG_STRING, &r.recdir,   "recording directory", NULL },
		{ "socket",   's', 0, G_OPTION_ARG_STRING, &r.sockpath, "control socket path", NULL },
		{ "dot",       0,  0, G_OPTION_ARG_STRING, &r.dotfile,  "REC indicator PNG", NULL },
		{ "offset-x",  0,  0, G_OPTION_ARG_INT,    &r.offset_x, "display X offset", NULL },
		{ "offset-y",  0,  0, G_OPTION_ARG_INT,    &r.offset_y, "display Y offset", NULL },
		{ "bitrate",  'b', 0, G_OPTION_ARG_INT,    &r.bitrate_kbps, "x264 bitrate (kbps)", NULL },
		{ "width",   'W', 0, G_OPTION_ARG_INT, &r.src_w,   "force source width (0 = device decides)", NULL },
		{ "height",  'H', 0, G_OPTION_ARG_INT, &r.src_h,   "force source height (0 = device decides)", NULL },
		{ "fps",      0,  0, G_OPTION_ARG_INT, &r.src_fps, "force source framerate (0 = device decides)", NULL },
		{ "format",   0,  0, G_OPTION_ARG_STRING, &r.src_format, "force source pixel format, e.g. GRAY8", NULL },
		{ "view-width",  0, 0, G_OPTION_ARG_INT, &r.view_w, "on-screen width", NULL },
		{ "view-height", 0, 0, G_OPTION_ARG_INT, &r.view_h, "on-screen height", NULL },
		{ "flip",     0,  0, G_OPTION_ARG_INT, &r.flip_method, "nvvidconv flip-method (2 = 180, 0 = none)", NULL },
		{ "dot-size", 0,  0, G_OPTION_ARG_INT, &r.dot_size, "REC dot size in source pixels", NULL },
		{ "platform", 'p', 0, G_OPTION_ARG_STRING, &r.platform_arg,
		  "tegra | generic (default: autodetect by probing nvdrmvideosink)", NULL },
		{ "io-mode", 0, 0, G_OPTION_ARG_INT, &r.io_mode,
		  "v4l2src io-mode (2 = mmap); -1 = platform default", NULL },
		{ "render-rect", 0, 0, G_OPTION_ARG_STRING, &r.render_rect,
		  "generic only: kmssink render-rectangle, e.g. \"<516,0,888,1080>\"", NULL },
		{ "no-tee", 0, 0, G_OPTION_ARG_NONE, &r.no_tee,
		  "diagnostic: no tee/queue, display only (cannot record)", NULL },
		{ "dq", 0, 0, G_OPTION_ARG_INT, &r.dq_buffers,
		  "display queue depth in buffers (0 = GStreamer defaults)", NULL },
		{ "no-mount-check", 0, G_OPTION_FLAG_REVERSE, G_OPTION_ARG_NONE, &r.require_mount,
		  "allow recording to a plain directory (testing)", NULL },
		{ NULL }
	};

	ctx = g_option_context_new("- VD56G4 live view with on-demand recording");
	g_option_context_add_main_entries(ctx, opts, NULL);
	g_option_context_add_group(ctx, gst_init_get_option_group());
	if (!g_option_context_parse(ctx, &argc, &argv, &err)) {
		g_printerr("%s\n", err->message);
		return 1;
	}
	g_option_context_free(ctx);

	gst_init(NULL, NULL);

	/* Platform decides the remaining defaults, so it must be settled first. */
	r.generic = platform_is_generic(&r);
	if (r.flip_method < 0) {
		/* Tegra rig (Waveshare/Orin) is mounted inverted; the EVK/Pi rig is
		 * the right way up - verified from a captured frame. */
		r.flip_method = r.generic ? 0 : 2;
	}
	if (!r.view_w || !r.view_h) {
		if (r.generic)
			detect_display_mode(&r.view_w, &r.view_h);
		else {
			r.view_w = DEFAULT_VIEW_W;
			r.view_h = DEFAULT_VIEW_H;
		}
	}

	if (!build_pipeline(&r, &err)) {
		g_printerr("pipeline: %s\n", err ? err->message : "?");
		return 1;
	}

	r.loop = g_main_loop_new(NULL, FALSE);
	r.media_ready = media_is_ready(&r);

	/* Control socket. Removed first so a stale file from an unclean exit
	 * does not block startup. */
	g_unlink(r.sockpath);
	svc  = g_socket_service_new();
	addr = g_unix_socket_address_new(r.sockpath);
	if (!g_socket_listener_add_address(G_SOCKET_LISTENER(svc), addr,
					   G_SOCKET_TYPE_STREAM, G_SOCKET_PROTOCOL_DEFAULT,
					   NULL, NULL, &err)) {
		g_printerr("socket %s: %s\n", r.sockpath, err ? err->message : "?");
		return 1;
	}
	g_object_unref(addr);
	/* World-writable: the key daemon and any operator shell must be able to
	 * drive it without being root. The commands are not privileged. */
	g_chmod(r.sockpath, 0666);
	g_signal_connect(svc, "incoming", G_CALLBACK(on_incoming), &r);
	g_socket_service_start(svc);

	g_unix_signal_add(SIGINT,  on_signal, &r);
	g_unix_signal_add(SIGTERM, on_signal, &r);
	g_timeout_add_seconds(MEDIA_POLL_SECS, poll_media, &r);
	if (g_getenv("CAM_DEBUG_QUEUE"))
		g_timeout_add_seconds(2, poll_qlevel, &r);

	if (gst_element_set_state(r.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
		g_printerr("failed to start pipeline\n");
		return 1;
	}

	g_message("cam-recorder up: platform=%s device=%s recdir=%s socket=%s "
		  "media=%s src=%s view=%dx%d flip=%d",
		  r.generic ? "generic" : "tegra",
		  r.device, r.recdir, r.sockpath, r.media_ready ? "ready" : "absent",
		  (r.src_w || r.src_h || r.src_fps || r.src_format) ? "forced" : "device-negotiated",
		  r.view_w, r.view_h, r.flip_method);

	g_main_loop_run(r.loop);

	gst_element_set_state(r.pipeline, GST_STATE_NULL);
	gst_object_unref(r.pipeline);
	g_unlink(r.sockpath);
	return 0;
}
