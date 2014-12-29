/*
 * Copyright (c) 2013 Igor Pashev <pashev.igor@gmail.com> (http://osdyson.org)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include <dirent.h>
#include <errno.h>
#include <regex.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <libdevinfo.h>

#include "libusb.h"
#include "libusbi.h"

static const char dev_usb[] = "/dev/usb";
static const char devices[] = "/devices";
static const size_t devices_len = sizeof (devices) - 1;


/*
 * Backend functions
 */
static int solaris_get_device_list (struct libusb_context *,
                                    struct discovered_devs **);
static int solaris_open (struct libusb_device_handle *);
static void solaris_close (struct libusb_device_handle *);

static int solaris_get_device_descriptor (struct libusb_device *,
                                          unsigned char *, int *);
static int solaris_get_active_config_descriptor (struct libusb_device *,
                                                 unsigned char *, size_t,
                                                 int *);
static int solaris_get_config_descriptor (struct libusb_device *, uint8_t,
                                          unsigned char *, size_t, int *);

static int solaris_get_configuration (struct libusb_device_handle *, int *);
static int solaris_set_configuration (struct libusb_device_handle *, int);

static int solaris_claim_interface (struct libusb_device_handle *, int);
static int solaris_release_interface (struct libusb_device_handle *, int);

static int solaris_set_interface_altsetting (struct libusb_device_handle *,
                                             int, int);
static int solaris_clear_halt (struct libusb_device_handle *, unsigned char);
static int solaris_reset_device (struct libusb_device_handle *);
static void solaris_destroy_device (struct libusb_device *);

static int solaris_submit_transfer (struct usbi_transfer *);
static int solaris_cancel_transfer (struct usbi_transfer *);
static void solaris_clear_transfer_priv (struct usbi_transfer *);
static int solaris_handle_events (struct libusb_context *ctx, struct pollfd *,
                                  nfds_t, int);
static int solaris_clock_gettime (int, struct timespec *);


const struct usbi_os_backend solaris_backend = {
  .name = "Synchronous Solaris backend",
  .init = NULL,
  .exit = NULL,
  .get_device_list = solaris_get_device_list,
  .open = solaris_open,
  .close = solaris_close,

  .get_device_descriptor = solaris_get_device_descriptor,
  .get_active_config_descriptor = solaris_get_active_config_descriptor,
  .get_config_descriptor = solaris_get_config_descriptor,

  .get_configuration = solaris_get_configuration,
  .set_configuration = solaris_set_configuration,

  .claim_interface = solaris_claim_interface,
  .release_interface = solaris_release_interface,

  .set_interface_altsetting = solaris_set_interface_altsetting,
  .clear_halt = solaris_clear_halt,
  .reset_device = solaris_reset_device,

  .kernel_driver_active = NULL,
  .detach_kernel_driver = NULL,
  .attach_kernel_driver = NULL,

  .destroy_device = solaris_destroy_device,

  .submit_transfer = solaris_submit_transfer,
  .cancel_transfer = solaris_cancel_transfer,
  .clear_transfer_priv = solaris_clear_transfer_priv,

  .handle_events = solaris_handle_events,

  .clock_gettime = solaris_clock_gettime,

  .device_priv_size = 0,
  .device_handle_priv_size = 0,
  .transfer_priv_size = 0,
  .add_iso_packet_size = 0,
};


static int
di_prop_get_int (di_node_t dn, const char *propname, int *out)
{
  int retval;
  int *intp;

  usbi_dbg ("looking for \"%s\" property", propname);

  retval = di_prop_lookup_ints (DDI_DEV_T_ANY, dn, propname, &intp);
  if (1 == retval)
    {
      if (NULL != out)
        *out = *intp;
      usbi_dbg ("found %s = %d", propname, *intp);
    }
  else if (0 == retval)
    usbi_dbg ("property \"%s\" is empty", propname);
  else if (retval < 0)
    usbi_dbg ("failed to get property \"%s\": %s", propname,
              strerror (errno));
  else
    usbi_dbg ("got %d values of \"%s\"", retval, propname);

  return retval;
}

static void
solaris_add_device (struct libusb_context *ctx,
                    struct discovered_devs **discdevs,
                    const char *device_node_path)
{
  int busnum = 0;
  int devaddr = 0;
  int numconf = 1;
  unsigned long session_id;
  enum libusb_speed speed;

  usbi_info (ctx, "device node \"%s\"", device_node_path);

  di_node_t devnode = di_init (device_node_path, DINFOPROP);
  if (DI_NODE_NIL == devnode)
    {
      usbi_err (ctx, "di_init() failed: %s, skipping", strerror (errno));
      return;
    }

  /* From now work with libdevinfo */

  if (1 != di_prop_get_int (devnode, "assigned-address", &busnum))
    goto cleanup;

  if (1 != di_prop_get_int (devnode, "usb-num-configs", &numconf))
    goto cleanup;

  /* XXX: Super speed is not supported.  */
  /* XXX: if no *-speed property exists, it is full-speed device.  */
  if (0 <= di_prop_get_int (devnode, "low-speed", NULL))
    speed = LIBUSB_SPEED_LOW;
  else if (0 <= di_prop_get_int (devnode, "full-speed", NULL))
    speed = LIBUSB_SPEED_FULL;
  else if (0 <= di_prop_get_int (devnode, "high-speed", NULL))
    speed = LIBUSB_SPEED_HIGH;
  else if (0 <= di_prop_get_int (devnode, "super-speed", NULL))
    speed = LIBUSB_SPEED_SUPER;
  else
    speed = LIBUSB_SPEED_FULL;


  /* get device address - a number after @ in device_node_path:
   * in "pci@0,0/pci106b,3f@6/device@2" device address is 2
   */
  char *at = strrchr (device_node_path, '@');
  if (NULL == at)
    {                           /* can't happen! */
      usbi_err (ctx,
                "failed to parse device node to device address, skipping");
      goto cleanup;
    }
  at++;
  devaddr = atoi (at);

  session_id = busnum << 8 | devaddr;
  usbi_dbg ("busnum %d devaddr %d session_id %u", busnum, devaddr,
            session_id);

cleanup:
  di_fini (devnode);
  return;
}


int
solaris_get_device_list (struct libusb_context *ctx,
                         struct discovered_devs **discdevs)
{
  char vidpid_path[sizeof ("/dev/usb/vvvv.pppp")];
  char devstat_path[sizeof ("/dev/usb/vvvv.pppp/iiii/devstat")];        /* 9999 instances should be enough. */

  /* for realpath():  */
  char
    device_path[sizeof
                ("/devices/pci@NNNN,MMMM/pciXXXX,YYYY@IIII/device@IIII:AAAA.BBBB.devstat")];

  regex_t regex;
  int retval;
  char *device_node_path;

  if (0 != regcomp (&regex, "[0-9a-f]+\\.[0-9a-f]+", REG_EXTENDED))
    {
      usbi_err (ctx, "regcomp() failed");
      return (LIBUSB_ERROR_NO_MEM);
    }

  /* open /dev/usb for browsing.  */
  DIR *dev_usb_dir = opendir (dev_usb);
  if (NULL == dev_usb_dir)
    {
      usbi_err (ctx, "opendir(\"%s\") failed: %s", dev_usb, strerror (errno));
      regfree (&regex);
      return (LIBUSB_ERROR_ACCESS);
    }

  struct dirent *vidpid;
  usbi_dbg ("start browsing %s", dev_usb);
  while ((vidpid = readdir (dev_usb_dir)) != NULL)
    {
      if (0 != regexec (&regex, vidpid->d_name, 0, NULL, 0))
        {
          if ('.' != vidpid->d_name[0])
            usbi_dbg ("skipping %s", vidpid->d_name);
          continue;
        }

      usbi_dbg ("found %s", vidpid->d_name);

      retval =
        snprintf (vidpid_path, sizeof (vidpid_path), "%s/%s", dev_usb,
                  vidpid->d_name);
      if (retval >= sizeof (vidpid_path))
        {
          usbi_err (ctx, "vidpid_path: snprintf() failed, skipping");
          continue;
        }

      /* open /dev/usb/<VID>.<PID> for browsing.  */
      DIR *vidpid_dir = opendir (vidpid_path);
      if (NULL == vidpid_dir)
        {
          usbi_err (ctx, "opendir(\"%s\") failed: %s, skipping", vidpid_path,
                    strerror (errno));
          continue;
        }

      struct dirent *inst;
      usbi_dbg ("start browsing %s", vidpid_path);
      while ((inst = readdir (vidpid_dir)) != NULL)
        {
          if ('.' == inst->d_name[0])
            continue;

          usbi_dbg ("found instance %s", inst->d_name);

          usbi_info (ctx, "found ugen device %s/%s", vidpid_path,
                     inst->d_name);

          /* We need *any* file in the instance subdir
           * only to get the real device path under /devices.
           * E. g. given /dev/usb/a12.1/0/devstat -> /devices/pci@0,0/pci106b,3f@6/device@2:a12.1.devstat
           * we should get /devices/pci@0,0/pci106b,3f@6/device@2
           * devstat always exists, so use it.
           */
          retval =
            snprintf (devstat_path, sizeof (devstat_path), "%s/%s/devstat",
                      vidpid_path, inst->d_name);
          if (retval >= sizeof (devstat_path))
            {
              usbi_err (ctx, "devstat_path: snprintf() failed, skipping");
              continue;
            }

          device_node_path = realpath (devstat_path, device_path);
          if (NULL == device_node_path)
            {
              usbi_err (ctx, "realpath() for \"%s\" failed: %s", devstat_path,
                        strerror (errno));
              continue;
            }

          usbi_dbg ("device path \"%s\"", device_path);

          /* is real path under /devices directory?  */
          if (strncmp (device_path, devices, devices_len) != 0)
            {
              /* No */
              usbi_warn (ctx, "\"%s\" is not under /devices, skipping",
                         device_path);
              continue;
            }

          /* for di_init() we need to:
           *
           * a) strip /devices from the beginning of device_path
           * b) strip everything after the colon in device_path
           *
           * E. g. /devices/pci@0,0/pci106b,3f@6/device@2:a12.1.devstat
           * should become /pci@0,0/pci106b,3f@6/device@2
           */

          /* a) */
          device_node_path += devices_len;

          /* b) */
          char *colon = strrchr (device_node_path, ':');
          if (colon)
            *colon = '\0';
          else
            usbi_warn (ctx, "no colon in device node path");

          solaris_add_device (ctx, discdevs, device_node_path);
        }
      usbi_dbg ("stop browsing %s", vidpid_path);
    }
  usbi_dbg ("stop browsing %s", dev_usb);

  (void) closedir (dev_usb_dir);

  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_open (struct libusb_device_handle *handle)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

void
solaris_close (struct libusb_device_handle *handle)
{
}

int
solaris_get_device_descriptor (struct libusb_device *dev, unsigned char *buf,
                               int *host_endian)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_get_active_config_descriptor (struct libusb_device *dev,
                                      unsigned char *buf, size_t len,
                                      int *host_endian)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_get_config_descriptor (struct libusb_device *dev, uint8_t idx,
                               unsigned char *buf, size_t len,
                               int *host_endian)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_get_configuration (struct libusb_device_handle *handle, int *config)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_set_configuration (struct libusb_device_handle *handle, int config)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_claim_interface (struct libusb_device_handle *handle, int iface)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_release_interface (struct libusb_device_handle *handle, int iface)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_set_interface_altsetting (struct libusb_device_handle *handle,
                                  int iface, int altsetting)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_clear_halt (struct libusb_device_handle *handle,
                    unsigned char endpoint)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_reset_device (struct libusb_device_handle *handle)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

void
solaris_destroy_device (struct libusb_device *dev)
{
}

int
solaris_submit_transfer (struct usbi_transfer *itransfer)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_cancel_transfer (struct usbi_transfer *itransfer)
{
  usbi_dbg ("");

  return (LIBUSB_ERROR_NOT_SUPPORTED);
}

void
solaris_clear_transfer_priv (struct usbi_transfer *itransfer)
{
}

int
solaris_handle_events (struct libusb_context *ctx, struct pollfd *fds,
                       nfds_t nfds, int num_ready)
{
  return (LIBUSB_ERROR_NO_DEVICE);
}

int
solaris_clock_gettime (int clkid, struct timespec *tp)
{
  usbi_dbg ("clock %d", clkid);

  if (clkid == USBI_CLOCK_REALTIME)
    return clock_gettime (CLOCK_REALTIME, tp);

  if (clkid == USBI_CLOCK_MONOTONIC)
    return clock_gettime (CLOCK_MONOTONIC, tp);

  return (LIBUSB_ERROR_INVALID_PARAM);
}
