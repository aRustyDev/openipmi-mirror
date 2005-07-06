/*
 * ipmi.c
 *
 * MontaVista IPMI generic code
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002,2003 MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public License
 *  as published by the Free Software Foundation; either version 2 of
 *  the License, or (at your option) any later version.
 *
 *
 *  THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this program; if not, write to the Free
 *  Software Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>

#include <OpenIPMI/os_handler.h>
#include <OpenIPMI/ipmi_conn.h>
#include <OpenIPMI/ipmi_err.h>
#include <OpenIPMI/ipmi_auth.h>
#include <OpenIPMI/ipmi_lan.h>
#include <OpenIPMI/ipmi_smi.h>
#include <OpenIPMI/ipmi_msgbits.h>
#include <OpenIPMI/mxp.h>

#include <OpenIPMI/internal/ipmi_domain.h>
#include <OpenIPMI/internal/ipmi_mc.h>
#include <OpenIPMI/internal/ipmi_int.h>
#include <OpenIPMI/internal/ipmi_oem.h>

#if defined(DEBUG_MSG) || defined(DEBUG_RAWMSG)
static void
dump_hex(void *vdata, int len)
{
    unsigned char *data = vdata;
    int i;
    for (i=0; i<len; i++) {
	if ((i != 0) && ((i % 16) == 0)) {
	    ipmi_log(IPMI_LOG_DEBUG_CONT, "\n  ");
	}
	ipmi_log(IPMI_LOG_DEBUG_CONT, " %2.2x", data[i]);
    }
}
#endif
static os_handler_t *ipmi_os_handler;

unsigned int __ipmi_log_mask = 0;

os_handler_t *
ipmi_get_global_os_handler(void)
{
    return ipmi_os_handler;
}
    
int
ipmi_create_global_lock(ipmi_lock_t **new_lock)
{
    return ipmi_create_lock_os_hnd(ipmi_os_handler, new_lock);
}

int
ipmi_create_lock(ipmi_domain_t *domain, ipmi_lock_t **new_lock)
{
    return ipmi_create_lock_os_hnd(ipmi_domain_get_os_hnd(domain), new_lock);
}

void
ipmi_log(enum ipmi_log_type_e log_type, char *format, ...)
{
    va_list ap;

    va_start(ap, format);
    if (ipmi_os_handler->vlog)
	ipmi_os_handler->vlog(ipmi_os_handler, log_type, format, ap);
    else
	vfprintf(stderr, format, ap);
    va_end(ap);
}

static unsigned int
ipmi_get_unicode(int len,
		 unsigned char **d, int in_len,
		 char *out, int out_len)
{
    if (out_len < len)
	len = out_len;

    memcpy(out, *d, len);
    *d += len;
    return len;
}

static unsigned int
ipmi_get_bcd_plus(int len,
		  unsigned char **d, int in_len,
		  char *out, int out_len)
{
    static char table[16] = {
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', ' ', '-', '.', ':', ',', '_'
    };
    int pos;
    int bo;
    int val = 0;
    int i;
    int real_length;
    char *out_s = out;

    real_length = (in_len * 8) / 4;
    if (len > real_length)
	len = real_length;
    
    if (len > out_len)
	len = out_len;

    bo = 0;
    pos = 0;
    for (i=0; i<len; i++) {
	switch (bo) {
	case 0:
	    val = **d & 0xf;
	    bo = 4;
	    break;
	case 4:
	    val = (**d >> 4) & 0xf;
	    (*d)++;
	    bo = 0;
	    break;
	}
	*out = table[val];
	out++;
    }

    if (bo != 0)
	(*d)++;

    return out - out_s;
}

static unsigned int
ipmi_get_6_bit_ascii(int len,
		     unsigned char **d, int in_len,
		     char *out, int out_len)
{
    static char table[64] = {
	' ', '!', '"', '#', '$', '%', '&', '\'',
	'(', ')', '*', '+', ',', '-', '.', '/', 
	'0', '1', '2', '3', '4', '5', '6', '7',
	'8', '9', ':', ';', '<', '=', '>', '?',
	'&', 'A', 'B', 'C', 'D', 'E', 'F', 'G',
	'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 
	'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W',
	'X', 'Y', 'Z', '[', '\\', ']', '^', '_' 
    };
    int pos;
    int bo;
    int val = 0;
    int i;
    int real_length;
    char *out_s = out;

    real_length = (in_len * 8) / 6;
    if (len > real_length)
	len = real_length;
    
    if (len > out_len)
	len = out_len;

    bo = 0;
    pos = 0;
    for (i=0; i<len; i++) {
	switch (bo) {
	case 0:
	    val = **d & 0x3f;
	    bo = 6;
	    break;
	case 2:
	    val = (**d >> 2) & 0x3f;
	    (*d)++;
	    bo = 0;
	    break;
	case 4:
	    val = (**d >> 4) & 0xf;
	    (*d)++;
	    val |= (**d & 0x3) << 4;
	    bo = 2;
	    break;
	case 6:
	    val = (**d >> 6) & 0x3;
	    (*d)++;
	    val |= (**d & 0xf) << 2;
	    bo = 4;
	    break;
	}
	*out = table[val];
	out++;
    }

    if (bo != 0)
	(*d)++;

    return out - out_s;
}

static unsigned int
ipmi_get_8_bit_ascii(int len,
		     unsigned char **d, int in_len,
		     char *out, int out_len)
{
    int j;

    
    if (len > in_len)
	len = in_len;

    if (len > out_len)
	len = out_len;

    for (j=0; j<len; j++) {
	*out = **d;
	out++;
	(*d)++;
    }
    return len;
};

unsigned int
ipmi_get_device_string(unsigned char        **pinput,
		       unsigned int         in_len,
		       char                 *output,
		       int                  semantics,
		       int                  force_unicode,
		       enum ipmi_str_type_e *stype,
		       unsigned int         max_out_len)
{
    int           type;
    int           len;
    unsigned int  olen;

    if (max_out_len == 0)
	return 0;

    if (in_len <= 0) {
	*output = '\0';
	return 0;
    }

#if 0
    /* Note that this is technically correct, but commonly ignored.
       0xc1 is invalid, but some FRU and SDR data still uses it.  Grr.
       The FRU stuff has to handle the end-of-area marker c1 itself,
       anyway, so this is relatively safe.  In a "correct" system you
       should never see a 0xc1 here, anyway. */
    if (**pinput == 0xc1) {
	*output = '\0';
	(*pinput)++;
	return 0;
    }
#endif

    type = (**pinput >> 6) & 3;

    /* Special case for FRU data, type 3 is unicode if the language is
       non-english. */
    if ((force_unicode) && (type == 3)) {
	type = 0;
	force_unicode = 0;
    }

    len = **pinput & 0x3f;
    (*pinput)++;
    in_len--;
    *stype = IPMI_ASCII_STR;
    switch (type)
    {
	case 0: /* Unicode */
	    olen = ipmi_get_unicode(len, pinput, in_len, output, max_out_len);
	    if (semantics == IPMI_STR_FRU_SEMANTICS)
		*stype = IPMI_BINARY_STR;
	    else
		*stype = IPMI_UNICODE_STR;
	    break;
	case 1: /* BCD Plus */
	    olen = ipmi_get_bcd_plus(len, pinput, in_len, output, max_out_len);
	    break;
	case 2: /* 6-bit ASCII */
	    olen=ipmi_get_6_bit_ascii(len, pinput, in_len, output, max_out_len);
	    break;
	case 3: /* 8-bit ASCII */
	    olen=ipmi_get_8_bit_ascii(len, pinput, in_len, output, max_out_len);
	    break;
        default:
	    olen = 0;
    }

    return olen;
}

/* Element will be zero if not present, n-1 if present. */
static char table_4_bit[256] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0f, 0x0c, 0x0d, 0x00,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
    0x09, 0x0a, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/* Element will be zero if not present, n-1 if present. */
static char table_6_bit[256] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x21, 0x08,
    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
    0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
    0x00, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30,
    0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void
ipmi_set_bcdplus(char          *input,
		 unsigned int  in_len,
		 unsigned char *output,
		 int           *out_len)
{
    int  len = *out_len;
    char *s = input;
    int  pos = 0;
    int  bit = 0;
    int  count = 0;

    while (in_len > 0) {
	switch(bit) {
	    case 0:
		pos++;
		if (pos >= len)
		    goto out_overflow;
		output[pos] = table_4_bit[(int) *s] - 1;
		bit = 4;
		break;

	    case 4:
		output[pos] |= (table_4_bit[(int) *s] - 1) << 4;
		bit = 0;
		break;
	}
	count++;
	in_len--;
	s++;
    }
    pos++;
 out_overflow:
    output[0] = (0x01 << 6) | count;
    *out_len = pos;
}

static void
ipmi_set_6_bit_ascii(char          *input,
		     unsigned int  in_len,
		     unsigned char *output,
		     int           *out_len)
{
    int  len = *out_len;
    char *s = input;
    int  pos = 0;
    int  bit = 0;
    int  count = 0;
    int  cval;
    int  oval;

    while (in_len > 0) {
	cval = *s;
	s++;
	oval = table_6_bit[cval] - 1;
	switch(bit) {
	case 0:
	    pos++;
	    if (pos >= len) 
		goto out_overflow;
	    output[pos] = oval;
	    bit = 6;
	    break;

	case 2:
	    output[pos] |= oval << 2;
	    bit = 0;
	    break;

	case 4:
	    output[pos] |= oval << 4;
	    pos++;
	    if (pos >= len) 
		goto out_overflow;
	    output[pos] = (oval >> 4) & 0x3;
	    bit = 2;
	    break;

	case 6:
	    output[pos] |= oval << 6;
	    pos++;
	    if (pos >= len) 
		goto out_overflow;
	    output[pos] = (oval >> 2) & 0xf;
	    bit = 4;
	    break;
	}
	count++;
	in_len--;
    }
    pos++;
 out_overflow:
    output[0] = (0x02 << 6) | count;
    *out_len = pos;
}

static void
ipmi_set_8_bit_ascii(char          *input,
		     int           in_len,
		     unsigned char *output,
		     int           *out_len)
{
    char tmp[2];
    /* truncate if necessary. */
    if (in_len > (*out_len - 1))
	in_len = *out_len - 1;

    /* A length of 1 is invalid, make it 2 with a nil char */
    if (in_len == 1) {
	tmp[0] = input[0];
	tmp[1] = '\0';
	input = tmp;
	in_len++;
    }

    *out_len = in_len + 1;

    memcpy(output+1, input, in_len);
    output[0] = (0x03 << 6) | in_len;
}

void
ipmi_set_device_string(char                 *input,
		       enum ipmi_str_type_e type,
		       unsigned int         in_len,
		       unsigned char        *output,
		       int                  force_unicode,
		       int                  *out_len)
{
    char         *s = input;
    int          bsize = 0; /* Start with 4-bit. */
    unsigned int i;

    /* Max size is 64 (63 bytes + the type byte). */
    if (*out_len > 64)
	*out_len = 64;
    /* Truncate */
    if (in_len > 63)
	in_len = 63;

    if (type == IPMI_ASCII_STR) {
	for (i=0; i<in_len; i++) {
	    if (table_4_bit[(int) *s] == 0) {
		bsize |= 1;
		if (table_6_bit[(int) *s] == 0) {
		    bsize |= 2;
		    break;
		}
	    }
	    s++;
	}
	if (bsize == 0) {
	    /* We can encode it in 4-bit BCD+ */
	    ipmi_set_bcdplus(input, in_len, output, out_len);
	} else if (bsize == 1) {
	    /* We can encode it in 6-bit ASCII. */
	    ipmi_set_6_bit_ascii(input, in_len, output, out_len);
	} else {
	    ipmi_set_8_bit_ascii(input, in_len, output, out_len);
	}
    } else {
	/* The input and output are unicode. */
	if (in_len > *out_len-1)
	    in_len = *out_len-1;
	if ((force_unicode) && (type == IPMI_UNICODE_STR))
	    *output = (0x3 << 6) | in_len;
	else
	    *output = in_len;
	memcpy(output+1, input, in_len);
	*out_len = in_len + 1;
    }
}

static long seq = 0;
static os_hnd_lock_t *seq_lock;
long
ipmi_get_seq(void)
{
    long rv;

    if (seq_lock)
	ipmi_os_handler->lock(ipmi_os_handler, seq_lock);
    rv = seq;
    seq++;
    if (seq_lock)
	ipmi_os_handler->unlock(ipmi_os_handler, seq_lock);

    return rv;
}

void
ipmi_event_state_init(ipmi_event_state_t *events)
{
    events->status = 0;
    events->__assertion_events = 0;
    events->__deassertion_events = 0;
}

void
ipmi_threshold_event_clear(ipmi_event_state_t          *events,
			   enum ipmi_thresh_e          type,
			   enum ipmi_event_value_dir_e value_dir,
			   enum ipmi_event_dir_e       dir)
{
    if (dir == IPMI_ASSERTION) {
	events->__assertion_events &= ~(1 << (type*2+value_dir));
    } else {
	events->__deassertion_events &= ~(1 << (type*2+value_dir));
    }
}

void
ipmi_threshold_event_set(ipmi_event_state_t          *events,
			 enum ipmi_thresh_e          type,
			 enum ipmi_event_value_dir_e value_dir,
			 enum ipmi_event_dir_e       dir)
{
    if (dir == IPMI_ASSERTION) {
	events->__assertion_events |= 1 << (type*2+value_dir);
    } else {
	events->__deassertion_events |= 1 << (type*2+value_dir);
    }
}

int
ipmi_is_threshold_event_set(ipmi_event_state_t          *events,
			    enum ipmi_thresh_e          type,
			    enum ipmi_event_value_dir_e value_dir,
			    enum ipmi_event_dir_e       dir)
{
    if (dir == IPMI_ASSERTION) {
	return (events->__assertion_events & (1 << (type*2+value_dir))) != 0;
    } else {
	return (events->__deassertion_events & (1 << (type*2+value_dir))) != 0;
    }
}

void
ipmi_discrete_event_clear(ipmi_event_state_t    *events,
			  int                   event_offset,
			  enum ipmi_event_dir_e dir)
{
    if (dir == IPMI_ASSERTION) {
	events->__assertion_events &= ~(1 << event_offset);
    } else {
	events->__deassertion_events &= ~(1 << event_offset);
    }
}

void
ipmi_discrete_event_set(ipmi_event_state_t    *events,
			int                   event_offset,
			enum ipmi_event_dir_e dir)
{
    if (dir == IPMI_ASSERTION) {
	events->__assertion_events |= 1 << event_offset;
    } else {
	events->__deassertion_events |= 1 << event_offset;
    }
}

int
ipmi_is_discrete_event_set(ipmi_event_state_t    *events,
			   int                   event_offset,
			   enum ipmi_event_dir_e dir)
{
    if (dir == IPMI_ASSERTION) {
	return (events->__assertion_events & (1 << event_offset)) != 0;
    } else {
	return (events->__deassertion_events & (1 << event_offset)) != 0;
    }
}

#define IPMI_SENSOR_EVENTS_ENABLED	0x80
#define IPMI_SENSOR_SCANNING_ENABLED	0x40
#define IPMI_SENSOR_BUSY		0x20

unsigned int ipmi_event_state_size(void)
{
    return sizeof(ipmi_event_state_t);
}

void
ipmi_copy_event_state(ipmi_event_state_t *dest, ipmi_event_state_t *src)
{
    *dest = *src;
}

void
ipmi_event_state_set_events_enabled(ipmi_event_state_t *events, int val)
{
    if (val)
	events->status |= IPMI_SENSOR_EVENTS_ENABLED;
    else
	events->status &= ~IPMI_SENSOR_EVENTS_ENABLED;
}

int
ipmi_event_state_get_events_enabled(ipmi_event_state_t *events)
{
    return (events->status >> 7) & 1;
}

void
ipmi_event_state_set_scanning_enabled(ipmi_event_state_t *events, int val)
{
    if (val)
	events->status |= IPMI_SENSOR_SCANNING_ENABLED;
    else
	events->status &= ~IPMI_SENSOR_SCANNING_ENABLED;
}

int
ipmi_event_state_get_scanning_enabled(ipmi_event_state_t *events)
{
    return (events->status >> 6) & 1;
}

void
ipmi_event_state_set_busy(ipmi_event_state_t *events, int val)
{
    if (val)
	events->status |= IPMI_SENSOR_BUSY;
    else
	events->status &= ~IPMI_SENSOR_BUSY;
}

int
ipmi_event_state_get_busy(ipmi_event_state_t *events)
{
    return (events->status >> 5) & 1;
}

unsigned int ipmi_thresholds_size(void)
{
    return sizeof(ipmi_thresholds_t);
}

void
ipmi_copy_thresholds(ipmi_thresholds_t *dest, ipmi_thresholds_t *src)
{
    *dest = *src;
}

int ipmi_thresholds_init(ipmi_thresholds_t *th)
{
    int i;
    for (i=0; i<6; i++)
	th->vals[i].status = 0;
    return 0;
}

int ipmi_threshold_set(ipmi_thresholds_t  *th,
		       ipmi_sensor_t      *sensor,
		       enum ipmi_thresh_e threshold,
		       double             value)
{
    int rv = 0;

    if (threshold > IPMI_UPPER_NON_RECOVERABLE)
	return EINVAL;

    if (sensor) {
	int val;
	rv = ipmi_sensor_threshold_settable(sensor, threshold, &val);
	if (rv)
	    return rv;
	if (!val)
	    return ENOSYS;
    }

    th->vals[threshold].status = 1;
    th->vals[threshold].val = value;
    return 0;
}

int ipmi_threshold_get(ipmi_thresholds_t  *th,
		       enum ipmi_thresh_e threshold,
		       double             *value)
{
    if (threshold > IPMI_UPPER_NON_RECOVERABLE)
	return EINVAL;

    if (th->vals[threshold].status) {
	*value = th->vals[threshold].val;
	return 0;
    } else {
	return ENOSYS;
    }
}

unsigned int ipmi_states_size(void)
{
    return sizeof(ipmi_states_t);
}

void
ipmi_copy_states(ipmi_states_t *dest, ipmi_states_t *src)
{
    *dest = *src;
}

void
ipmi_init_states(ipmi_states_t *states)
{
    states->__event_messages_enabled = 0;
    states->__sensor_scanning_enabled = 0;
    states->__initial_update_in_progress = 0;
    states->__states = 0;
}

int
ipmi_is_event_messages_enabled(ipmi_states_t *states)
{
    return states->__event_messages_enabled;
}

void
ipmi_set_event_messages_enabled(ipmi_states_t *states, int val)
{
    states->__event_messages_enabled = val;
}

int
ipmi_is_sensor_scanning_enabled(ipmi_states_t *states)
{
    return states->__sensor_scanning_enabled;
}

void
ipmi_set_sensor_scanning_enabled(ipmi_states_t *states, int val)
{
    states->__sensor_scanning_enabled = val;
}

int
ipmi_is_initial_update_in_progress(ipmi_states_t *states)
{
    return states->__initial_update_in_progress;
}

void
ipmi_set_initial_update_in_progress(ipmi_states_t *states, int val)
{
    states->__initial_update_in_progress = val;
}

int
ipmi_is_state_set(ipmi_states_t *states,
		  int           state_num)
{
    return (states->__states & (1 << state_num)) != 0;
}

void
ipmi_set_state(ipmi_states_t *states,
	       int           state_num,
	       int           val)
{
    if (val)
	states->__states |= 1 << state_num;
    else
	states->__states &= ~(1 << state_num);
}

int
ipmi_is_threshold_out_of_range(ipmi_states_t      *states,
			       enum ipmi_thresh_e thresh)
{
    return (states->__states & (1 << thresh)) != 0;
}

void
ipmi_set_threshold_out_of_range(ipmi_states_t      *states,
				enum ipmi_thresh_e thresh,
				int                val)
{
    if (val)
	states->__states |= 1 << thresh;
    else
	states->__states &= ~(1 << thresh);
}

void ipmi_oem_force_conn_init(void);
int ipmi_oem_motorola_mxp_init(void);
int ipmi_oem_intel_init(void);
int ipmi_oem_atca_conn_init(void);
int ipmi_oem_atca_init(void);
int init_oem_test(void);
int _ipmi_smi_init(os_handler_t *os_hnd);
int _ipmi_lan_init(os_handler_t *os_hnd);
int _ipmi_mxp_init(os_handler_t *os_hnd);
int ipmi_malloc_init(os_handler_t *os_hnd);
int _ipmi_fru_init(void);
void _ipmi_fru_shutdown(void);

void ipmi_oem_atca_conn_shutdown(void);
void ipmi_oem_intel_shutdown(void);
void ipmi_oem_atca_shutdown(void);
int _ipmi_smi_shutdown(void);
int _ipmi_lan_shutdown(void);
int _ipmi_mxp_shutdown(void);

int
ipmi_init(os_handler_t *handler)
{
    int rv;

    ipmi_os_handler = handler;

    /* Set up memory allocation first */
    ipmi_malloc_init(handler);

    /* Set up logging in malloc code. */
    ipmi_malloc_log = ipmi_log;

    rv = _ipmi_conn_init(handler);
    if (rv)
	return rv;

    if (handler->create_lock) {
	rv = handler->create_lock(handler, &seq_lock);
	if (rv)
	    goto out_err;
    } else {
	seq_lock = NULL;
    }

    rv = _ipmi_smi_init(handler);
    if (rv)
	goto out_err;

    rv = _ipmi_lan_init(handler);
    if (rv)
	goto out_err;

    rv = _ipmi_mxp_init(handler);
    if (rv)
	goto out_err;

    _ipmi_domain_init();
    _ipmi_mc_init();

    rv = _ipmi_fru_init();
    if (rv)
	goto out_err;

    /* Call the OEM handlers. */
    ipmi_oem_force_conn_init();
    ipmi_oem_motorola_mxp_init();
    ipmi_oem_intel_init();
    ipmi_oem_atca_conn_init();
    ipmi_oem_atca_init();
    init_oem_test();

    return 0;

 out_err:
    _ipmi_mxp_shutdown();
    _ipmi_smi_shutdown();
    _ipmi_lan_shutdown();
    ipmi_oem_intel_shutdown();
    _ipmi_mc_shutdown();
    _ipmi_domain_shutdown();
    _ipmi_fru_shutdown();
    if (seq_lock)
	handler->destroy_lock(ipmi_os_handler, seq_lock);
    ipmi_os_handler = NULL;
    return rv;
}

void
ipmi_shutdown(void)
{
    _ipmi_mxp_shutdown();
    _ipmi_lan_shutdown();
    _ipmi_smi_shutdown();
    ipmi_oem_atca_shutdown();
    ipmi_oem_atca_conn_shutdown();
    ipmi_oem_intel_shutdown();
    _ipmi_mc_shutdown();
    _ipmi_domain_shutdown();
    _ipmi_conn_shutdown();
    _ipmi_fru_shutdown();
    if (seq_lock)
	ipmi_os_handler->destroy_lock(ipmi_os_handler, seq_lock);
}

enum con_type_e { SMI, LAN, MXP };

struct ipmi_args_s
{
    enum con_type_e con_type;

    int             smi_intf;

    char            *str_addr[2];
    char            *str_port[2];
    int             num_addr;
    int             authtype_set;
    int             authtype;
    int             privilege;
    int             privilege_set;
    int             username_set;
    char            username[16];
    unsigned int    username_len;
    int             password_set;
    char            password[16];
    unsigned int    password_len;

    unsigned char   swid;
    struct in_addr  lan_addr[2];
    int             lan_port[2];
};

#define CHECK_ARG \
    do { \
        if (*curr_arg >= arg_count) { \
	    rv = EINVAL; \
	    goto out_err; \
        } \
    } while(0)

int
ipmi_parse_args(int         *curr_arg,
		int         arg_count,
		char        * const *args,
		ipmi_args_t **iargs)
{
    ipmi_args_t *p;
    int         rv;
    int         len;
    
    p = ipmi_mem_alloc(sizeof(*p));
    if (!p)
	return ENOMEM;
    memset(p, 0, sizeof(*p));

    CHECK_ARG;

    if (strcmp(args[*curr_arg], "smi") == 0) {
	(*curr_arg)++; CHECK_ARG;

	p->con_type = SMI;

	p->smi_intf = atoi(args[*curr_arg]);
	(*curr_arg)++;
    } else if (strcmp(args[*curr_arg], "lan") == 0) {
	(*curr_arg)++; CHECK_ARG;

	p->con_type = LAN;
	p->num_addr = 1;

	p->str_addr[0] = ipmi_strdup(args[*curr_arg]);
	if (p->str_addr[0] == NULL) {
	    rv = ENOMEM;
	    goto out_err;
	}
	(*curr_arg)++; CHECK_ARG;
	p->str_port[0] = ipmi_strdup(args[*curr_arg]);
	if (p->str_port[0] == NULL) {
	    rv = ENOMEM;
	    goto out_err;
	}
	(*curr_arg)++; CHECK_ARG;

    doauth:
	if (strcmp(args[*curr_arg], "none") == 0) {
	    p->authtype = IPMI_AUTHTYPE_NONE;
	} else if (strcmp(args[*curr_arg], "md2") == 0) {
	    p->authtype = IPMI_AUTHTYPE_MD2;
	} else if (strcmp(args[*curr_arg], "md5") == 0) {
	    p->authtype = IPMI_AUTHTYPE_MD5;
	} else if (strcmp(args[*curr_arg], "straight") == 0) {
	    p->authtype = IPMI_AUTHTYPE_STRAIGHT;
	} else if (p->num_addr == 1) {
	    p->num_addr++;
	    p->str_addr[1] = ipmi_strdup(args[*curr_arg]);
	    if (p->str_addr[1] == NULL) {
		rv = ENOMEM;
		goto out_err;
	    }
	    (*curr_arg)++; CHECK_ARG;
	    p->str_port[1] = ipmi_strdup(args[*curr_arg]);
	    if (p->str_port[1] == NULL) {
		rv = ENOMEM;
		goto out_err;
	    }
	    (*curr_arg)++; CHECK_ARG;
	    goto doauth;
	} else {
	    rv = EINVAL;
	    goto out_err;
	}
	(*curr_arg)++; CHECK_ARG;
	p->authtype_set = 1;

	if (strcmp(args[*curr_arg], "callback") == 0) {
	    p->privilege = IPMI_PRIVILEGE_CALLBACK;
	} else if (strcmp(args[*curr_arg], "user") == 0) {
	    p->privilege = IPMI_PRIVILEGE_USER;
	} else if (strcmp(args[*curr_arg], "operator") == 0) {
	    p->privilege = IPMI_PRIVILEGE_OPERATOR;
	} else if (strcmp(args[*curr_arg], "admin") == 0) {
	    p->privilege = IPMI_PRIVILEGE_ADMIN;
	} else if (strcmp(args[*curr_arg], "oem") == 0) {
	    p->privilege = IPMI_PRIVILEGE_OEM;
	} else {
	    rv = EINVAL;
	    goto out_err;
	}
	p->privilege_set = 1;
	(*curr_arg)++; CHECK_ARG;

	len = strlen(args[*curr_arg]);
	if (len > 16)
	    len = 16;
	memcpy(p->username, args[*curr_arg], len);
	p->username[16] = '\0';
	p->username_len = len;
	p->username_set = 1;
	(*curr_arg)++; CHECK_ARG;

	len = strlen(args[*curr_arg]);
	if (len > 16)
	    len = 16;
	memcpy(p->password, args[*curr_arg], len);
	p->password_len = len;
	p->password_set = 1;
	(*curr_arg)++;
    } else if (strcmp(args[*curr_arg], "mxp") == 0) {
	struct hostent *ent;

	(*curr_arg)++; CHECK_ARG;

	p->con_type = MXP;
	p->num_addr = 1;

	ent = gethostbyname(args[*curr_arg]);
	if (!ent) {
	    rv = h_errno;
	    goto out_err;
	}
	memcpy(&p->lan_addr[0],
	       ent->h_addr_list[0],
	       ent->h_length);
	(*curr_arg)++; CHECK_ARG;
	p->lan_port[0] = atoi(args[*curr_arg]);
	(*curr_arg)++; CHECK_ARG;

    doauth_mxp:
	if (strcmp(args[*curr_arg], "none") == 0) {
	    p->authtype = IPMI_AUTHTYPE_NONE;
	} else if (strcmp(args[*curr_arg], "md2") == 0) {
	    p->authtype = IPMI_AUTHTYPE_MD2;
	} else if (strcmp(args[*curr_arg], "md5") == 0) {
	    p->authtype = IPMI_AUTHTYPE_MD5;
	} else if (strcmp(args[*curr_arg], "straight") == 0) {
	    p->authtype = IPMI_AUTHTYPE_STRAIGHT;
	} else if (p->num_addr == 1) {
	    p->num_addr++;
	    ent = gethostbyname(args[*curr_arg]);
	    if (!ent) {
		rv = h_errno;
		goto out_err;
	    }
	    memcpy(&p->lan_addr[1],
		   ent->h_addr_list[0],
		   ent->h_length);
	    (*curr_arg)++; CHECK_ARG;
	    p->lan_port[1] = atoi(args[*curr_arg]);
	    (*curr_arg)++; CHECK_ARG;

	    goto doauth_mxp;
	} else {
	    rv = EINVAL;
	    goto out_err;
	}
	(*curr_arg)++; CHECK_ARG;

	if (strcmp(args[*curr_arg], "callback") == 0) {
	    p->privilege = IPMI_PRIVILEGE_CALLBACK;
	} else if (strcmp(args[*curr_arg], "user") == 0) {
	    p->privilege = IPMI_PRIVILEGE_USER;
	} else if (strcmp(args[*curr_arg], "operator") == 0) {
	    p->privilege = IPMI_PRIVILEGE_OPERATOR;
	} else if (strcmp(args[*curr_arg], "admin") == 0) {
	    p->privilege = IPMI_PRIVILEGE_ADMIN;
	} else if (strcmp(args[*curr_arg], "oem") == 0) {
	    p->privilege = IPMI_PRIVILEGE_OEM;
	} else {
	    rv = EINVAL;
	    goto out_err;
	}
	(*curr_arg)++; CHECK_ARG;

	memset(p->username, 0, sizeof(p->username));
	memset(p->password, 0, sizeof(p->password));
	strncpy(p->username, args[*curr_arg], 16);
	p->username[16] = '\0';
	(*curr_arg)++; CHECK_ARG;
	strncpy(p->password, args[*curr_arg], 16);
	p->password[16] = '\0';
	(*curr_arg)++; CHECK_ARG;

	p->swid = strtoul(args[*curr_arg], NULL, 0);
	(*curr_arg)++;
    } else {
	rv = EINVAL;
	goto out_err;
    }

    *iargs = p;
    return 0;

 out_err:
    ipmi_free_args(p);
    return rv;
}

int
ipmi_parse_args2(int         *curr_arg,
		 int         arg_count,
		 char        * const *args,
		 ipmi_args_t **iargs)
{
    ipmi_args_t *p;
    int rv;
    
    p = ipmi_mem_alloc(sizeof(*p));
    if (!p)
	return ENOMEM;
    memset(p, 0, sizeof(*p));

    CHECK_ARG;

    if (strcmp(args[*curr_arg], "smi") == 0) {
	(*curr_arg)++; CHECK_ARG;

	p->con_type = SMI;

	p->smi_intf = atoi(args[*curr_arg]);
	(*curr_arg)++;
    } else if (strcmp(args[*curr_arg], "lan") == 0) {
	int i;
	int len;

	(*curr_arg)++; CHECK_ARG;

	p->con_type = LAN;
	p->num_addr = 1;

	while (*curr_arg < arg_count) {
	    if (args[*curr_arg][0] != '-') {
		break;
	    }

	    if (strcmp(args[*curr_arg], "-U") == 0) {
		(*curr_arg)++; CHECK_ARG;
		len = strlen(args[*curr_arg]);
		if (len > 16)
		    len = 16;
		memcpy(p->username, args[*curr_arg], len);
		p->username_set = 1;
		p->username_len = len;
	    } else if (strcmp(args[*curr_arg], "-P") == 0) {
		(*curr_arg)++; CHECK_ARG;
		len = strlen(args[*curr_arg]);
		if (len > 16)
		    len = 16;
		memcpy(p->password, args[*curr_arg], len);
		p->password_set = 1;
		p->password_len = len;
	    } else if (strcmp(args[*curr_arg], "-s") == 0) {
		p->num_addr = 2;
	    } else if (strcmp(args[*curr_arg], "-A") == 0) {
		(*curr_arg)++; CHECK_ARG;
		if (strcmp(args[*curr_arg], "none") == 0) {
		    p->authtype = IPMI_AUTHTYPE_NONE;
		} else if (strcmp(args[*curr_arg], "md2") == 0) {
		    p->authtype = IPMI_AUTHTYPE_MD2;
		} else if (strcmp(args[*curr_arg], "md5") == 0) {
		    p->authtype = IPMI_AUTHTYPE_MD5;
		} else if (strcmp(args[*curr_arg], "straight") == 0) {
		    p->authtype = IPMI_AUTHTYPE_STRAIGHT;
		} else {
		    rv = EINVAL;
		    goto out_err;
		}
		p->authtype_set = 1;
	    } else if (strcmp(args[*curr_arg], "-L") == 0) {
		(*curr_arg)++; CHECK_ARG;

		if (strcmp(args[*curr_arg], "callback") == 0) {
		    p->privilege = IPMI_PRIVILEGE_CALLBACK;
		} else if (strcmp(args[*curr_arg], "user") == 0) {
		    p->privilege = IPMI_PRIVILEGE_USER;
		} else if (strcmp(args[*curr_arg], "operator") == 0) {
		    p->privilege = IPMI_PRIVILEGE_OPERATOR;
		} else if (strcmp(args[*curr_arg], "admin") == 0) {
		    p->privilege = IPMI_PRIVILEGE_ADMIN;
		} else if (strcmp(args[*curr_arg], "oem") == 0) {
		    p->privilege = IPMI_PRIVILEGE_OEM;
		} else {
		    rv = EINVAL;
		    goto out_err;
		}
		p->privilege_set = 1;
	    } else if (strcmp(args[*curr_arg], "-p") == 0) {
		(*curr_arg)++; CHECK_ARG;
		p->str_port[0] = ipmi_strdup(args[*curr_arg]);
		if (p->str_port[0] == NULL) {
		    rv = ENOMEM;
		    goto out_err;
		}
	    } else if (strcmp(args[*curr_arg], "-p2") == 0) {
		(*curr_arg)++; CHECK_ARG;
		p->str_port[2] = ipmi_strdup(args[*curr_arg]);
		if (p->str_port[2] == NULL) {
		    rv = ENOMEM;
		    goto out_err;
		}
	    }
	    (*curr_arg)++;
	}

	for (i=0; i<p->num_addr; i++) {
	    CHECK_ARG;
	    p->str_addr[i] = ipmi_strdup(args[*curr_arg]);
	    if (p->str_addr[0] == NULL) {
		rv = ENOMEM;
		goto out_err;
	    }
	    (*curr_arg)++;
	    if (! p->str_port[i]) {
		p->str_port[i] = ipmi_strdup("623");
		if (p->str_port[0] == NULL) {
		    rv = ENOMEM;
		    goto out_err;
		}
	    }
	}
    } else {
	rv = EINVAL;
	goto out_err;
    }

    *iargs = p;
    return 0;

 out_err:
    ipmi_free_args(p);
    return rv;
}

void
ipmi_free_args(ipmi_args_t *args)
{
    if (args->str_addr[0])
	ipmi_mem_free(args->str_addr[0]);
    if (args->str_addr[1])
	ipmi_mem_free(args->str_addr[1]);
    if (args->str_port[0])
	ipmi_mem_free(args->str_port[0]);
    if (args->str_port[1])
	ipmi_mem_free(args->str_port[1]);
    ipmi_mem_free(args);
}

int
ipmi_parse_options(ipmi_open_option_t *option,
		   char               *arg)
{
    if (strcmp(arg, "-noall") == 0) {
	option->option = IPMI_OPEN_OPTION_ALL;
	option->ival = 0;
    } else if (strcmp(arg, "-all") == 0) {
	option->option = IPMI_OPEN_OPTION_ALL;
	option->ival = 1;
    } else if (strcmp(arg, "-nosdrs") == 0) {
	option->option = IPMI_OPEN_OPTION_SDRS;
	option->ival = 0;
    } else if (strcmp(arg, "-sdrs") == 0) {
	option->option = IPMI_OPEN_OPTION_SDRS;
	option->ival = 1;
    } else if (strcmp(arg, "-nofrus") == 0) {
	option->option = IPMI_OPEN_OPTION_FRUS;
	option->ival = 0;
    } else if (strcmp(arg, "-frus") == 0) {
	option->option = IPMI_OPEN_OPTION_FRUS;
	option->ival = 1;
    } else if (strcmp(arg, "-nosel") == 0) {
	option->option = IPMI_OPEN_OPTION_SEL;
	option->ival = 0;
    } else if (strcmp(arg, "-sel") == 0) {
	option->option = IPMI_OPEN_OPTION_SEL;
	option->ival = 1;
    } else if (strcmp(arg, "-noipmbscan") == 0) {
	option->option = IPMI_OPEN_OPTION_IPMB_SCAN;
	option->ival = 0;
    } else if (strcmp(arg, "-ipmbscan") == 0) {
	option->option = IPMI_OPEN_OPTION_IPMB_SCAN;
	option->ival = 1;
    } else if (strcmp(arg, "-nooeminit") == 0) {
	option->option = IPMI_OPEN_OPTION_OEM_INIT;
	option->ival = 0;
    } else if (strcmp(arg, "-oeminit") == 0) {
	option->option = IPMI_OPEN_OPTION_OEM_INIT;
	option->ival = 1;
    } else if (strcmp(arg, "-noseteventrcvr") == 0) {
	option->option = IPMI_OPEN_OPTION_SET_EVENT_RCVR;
	option->ival = 0;
    } else if (strcmp(arg, "-seteventrcvr") == 0) {
	option->option = IPMI_OPEN_OPTION_SET_EVENT_RCVR;
	option->ival = 1;
    } else if (strcmp(arg, "-nosetseltime") == 0) {
	option->option = IPMI_OPEN_OPTION_SET_SEL_TIME;
	option->ival = 0;
    } else if (strcmp(arg, "-setseltime") == 0) {
	option->option = IPMI_OPEN_OPTION_SET_SEL_TIME;
	option->ival = 1;
    } else
	return EINVAL;

    return 0;
}

int
ipmi_args_setup_con(ipmi_args_t  *args,
		    os_handler_t *handlers,
		    void         *user_data,
		    ipmi_con_t   **con)
{
    switch(args->con_type) {
    case SMI:
	return ipmi_smi_setup_con(args->smi_intf, handlers, user_data, con);

    case LAN:
    {
	int              i;
	ipmi_lanp_parm_t parms[6];

	i = 0;
	parms[i].parm_id = IPMI_LANP_PARMID_ADDRS;
	parms[i].parm_data = args->str_addr;
	parms[i].parm_data_len = args->num_addr;
	i++;
	parms[i].parm_id = IPMI_LANP_PARMID_PORTS;
	parms[i].parm_data = args->str_port;
	parms[i].parm_data_len = args->num_addr;
	i++;
	if (args->authtype_set) {
	    parms[i].parm_id = IPMI_LANP_PARMID_AUTHTYPE;
	    parms[i].parm_val = args->authtype;
	    i++;
	}
	if (args->privilege_set) {
	    parms[i].parm_id = IPMI_LANP_PARMID_PRIVILEGE;
	    parms[i].parm_val = args->privilege;
	    i++;
	}
	if (args->username_set) {
	    parms[i].parm_id = IPMI_LANP_PARMID_USERNAME;
	    parms[i].parm_data = args->username;
	    parms[i].parm_data_len = args->username_len;
	    i++;
	}
	if (args->password_set) {
	    parms[i].parm_id = IPMI_LANP_PARMID_PASSWORD;
	    parms[i].parm_data = args->password;
	    parms[i].parm_data_len = args->password_len;
	    i++;
	}
	return ipmi_lanp_setup_con(parms, i, handlers, user_data, con);
    }

    case MXP:
	return mxp_lan_setup_con(args->lan_addr,
				 args->lan_port,
				 args->num_addr,
				 args->authtype,
				 args->privilege,
				 args->username,
				 strlen(args->username),
				 args->password,
				 strlen(args->password),
				 handlers, user_data,
				 args->swid,
				 con);

    default:
	return EINVAL;
    }
}

/* This is the number of seconds between 1/1/70 (IPMI event date) and
   1/1/98 (ipmi SNMP trap local timestamp). */
#define IPMI_SNMP_DATE_OFFSET 883612800

int
ipmi_handle_snmp_trap_data(void            *src_addr,
			   unsigned int    src_addr_len,
			   int             src_addr_type,
			   long            specific,
			   unsigned char   *data,
			   unsigned int    data_len)
{
    int           handled = 0;
    unsigned char pet_ack[12];
    ipmi_msg_t    *msg = NULL;

    if (DEBUG_RAWMSG) {
	ipmi_log(IPMI_LOG_DEBUG_START, "Got SNMP trap from:\n  ");
	dump_hex(src_addr, src_addr_len);
	ipmi_log(IPMI_LOG_DEBUG_CONT, "\n data is:\n  ");
	dump_hex(data, data_len);
	ipmi_log(IPMI_LOG_DEBUG_END, " ");
    }

    if (data_len < 46)
	return 0;

    /* I will take this opportunity to note that the SNMP trap format
       from IPMI is insufficient to actually perform the job.  It does
       not have:
       1 A guaranteed way to correlate the timestamp to the SEL.
       2 A guaranteed way to correlate the record id to the SEL.
       3 The channel or the LUN for the event generator.

       Because of these, there is no guaranteed way to correlate the
       data from the SNMP trap to an SEL event.  This can result in
       duplicate events, which is very bad.  So we currently do not
       deliver the events this way, we pass a NULL in the event
       message to tell the domain code to rescan the SEL for this MC.
       In addition, item 3 above means that you cannot determine which
       sensor issued the event, since the channel and the LUN are
       required to find the sensor.
    */

 /* Until we have some way to get full valid data from the trap, we
    just disable it. */
#if 0
    ipmi_msg_t     tmsg;
    unsigned char  edata[17];
    unsigned long  timestamp;
    int16_t        utc_off;
    unsigned short record_id;
    timestamp = ntohl(*((uint32_t *) (data+18)));
    if (data[27] == 0xff)
	/* Can't handle unspecific event generator */
	return 0;
    if ((data[28] == 0xff) || (data[28] == 0x00))
	/* Can't handle unspecific sensor */
	return 0;
    if (timestamp == 0)
	/* Can't handle unspecified timestamp. */
	return 0;
    utc_off = ntohs(*((uint16_t *) (data+22)));
    if (utc_off == -1)
	/* If unspecified (0xffff), we assume zero (UTC). */
	utc_off = 0;
    timestamp -= utc_off; /* Remove timezone offset */
    timestamp += IPMI_SNMP_DATE_OFFSET; /* Convert to 1/1/70 offset */

    /* We assume the record id is in the sequence # field, since that
       makes the most sense. */
    record_id = ntohs(*((uint16_t *) (data+16)));

    tmsg.netfn = IPMI_APP_NETFN;
    tmsg.cmd = IPMI_READ_EVENT_MSG_BUFFER_CMD;
    tmsg.data = edata;
    tmsg.data_len = 17;
    msg = &tmsg;
    edata[0] = 0;
    edata[1] = record_id & 0xff;
    edata[2] = (record_id >> 8) & 0xff;
    edata[3] = 2; /* record type - system event */
    edata[4] = timestamp & 0xff;
    edata[5] = (timestamp >> 8) & 0xff;
    edata[6] = (timestamp >> 16) & 0xff;
    edata[7] = (timestamp >> 24) & 0xff;
    edata[8] = data[27]; /* Event generator */
    /* FIXME - is there a way to get the LUN? */
    edata[9] = 0; /* Assume channel 0, lun 0 */
    edata[10] = 0x04; /* IPMI 1.5 revision */
    edata[11] = (specific >> 16) & 0xff; /* Sensor type */
    edata[12] = data[28]; /* Sensor number */
    edata[13] = (specific >> 8) & 0xff; /* Event dir/type */
    memcpy(edata+14, data+31, 3); /* Event data 1-3 */
#endif

    pet_ack[0] = data[17]; /* Record id */
    pet_ack[1] = data[16];
    pet_ack[2] = data[21]; /* Timestamp */
    pet_ack[3] = data[20];
    pet_ack[4] = data[19];
    pet_ack[5] = data[18];
    pet_ack[6] = data[25]; /* Event source type */
    pet_ack[7] = data[27]; /* Sensor device */
    pet_ack[8] = data[28]; /* Sensor number */
    memcpy(pet_ack+9, data+31, 3); /* Event data 1-3 */

    if (src_addr_type == IPMI_EXTERN_ADDR_IP)
	handled = ipmi_lan_handle_external_event(src_addr, msg, pet_ack);

    return handled;
}

char *
ipmi_openipmi_version(void)
{
    return OPENIPMI_VERSION;
}

void
ipmi_handle_rsp_item_copyall(ipmi_con_t            *ipmi,
			     ipmi_msgi_t           *rspi,
			     ipmi_addr_t           *addr,
			     unsigned int          addr_len,
			     ipmi_msg_t            *msg,
			     ipmi_ll_rsp_handler_t rsp_handler)
{
    int used = IPMI_MSG_ITEM_NOT_USED;

    memcpy(&rspi->addr, addr, addr_len);
    rspi->addr_len = addr_len;
    rspi->msg = *msg;
    memcpy(rspi->data, msg->data, msg->data_len);
    rspi->msg.data = rspi->data;

    /* call the user handler. */
    if (rsp_handler)
	used = rsp_handler(ipmi, rspi);

    if (!used)
	ipmi_mem_free(rspi);
}

void
ipmi_handle_rsp_item_copymsg(ipmi_con_t            *ipmi,
			     ipmi_msgi_t           *rspi,
			     ipmi_msg_t            *msg,
			     ipmi_ll_rsp_handler_t rsp_handler)
{
    int used = IPMI_MSG_ITEM_NOT_USED;

    rspi->msg = *msg;
    memcpy(rspi->data, msg->data, msg->data_len);
    rspi->msg.data = rspi->data;

    /* call the user handler. */
    if (rsp_handler)
	used = rsp_handler(ipmi, rspi);

    if (!used)
	ipmi_mem_free(rspi);
}

void
ipmi_handle_rsp_item(ipmi_con_t            *ipmi,
		     ipmi_msgi_t           *rspi,
		     ipmi_ll_rsp_handler_t rsp_handler)
{
    int used = IPMI_MSG_ITEM_NOT_USED;

    /* call the user handler. */
    if (rsp_handler)
	used = rsp_handler(ipmi, rspi);

    if (!used)
	ipmi_mem_free(rspi);
}

os_handler_t *
ipmi_alloc_os_handler(void)
{
    os_handler_t *rv = ipmi_mem_alloc(sizeof(*rv));
    if (rv)
	memset(rv, 0, sizeof(*rv));
    return rv;
}

void
ipmi_free_os_handler(os_handler_t *handler)
{
    ipmi_mem_free(handler);
}

