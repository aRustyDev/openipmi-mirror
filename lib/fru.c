/*
 * fru.c
 *
 * IPMI code for handling FRUs based on sdr.c
 *
 * Author: MontaVista Software, Inc.
 *         Corey Minyard <minyard@mvista.com>
 *         source@mvista.com
 *
 * Copyright 2002,2003 MontaVista Software Inc.
 *
 * Note that this file was originally written by Thomas Kanngieser
 * <thomas.kanngieser@fci.com> of FORCE Computers, but I've pretty
 * much gutted it and rewritten it, nothing really remained the same.
 * Thomas' code was helpful, though and many thanks go to him.
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
#include <time.h>
#include <stdint.h>
#include <errno.h>

#include <OpenIPMI/ipmiif.h>
#include <OpenIPMI/ipmi_fru.h>
#include <OpenIPMI/ipmi_int.h>
#include <OpenIPMI/ipmi_err.h>
#include <OpenIPMI/ipmi_msgbits.h>
#include <OpenIPMI/ipmi_domain.h>

#define IPMI_LANG_CODE_ENGLISH	25

#define MAX_FRU_DATA_FETCH 16

/* record types */
#define IPMI_FRU_FTR_UNDEFINED        -1
#define IPMI_FRU_FTR_INTERNAL_USE_AREA 0
#define IPMI_FRU_FTR_CHASSIS_INFO_AREA 1
#define IPMI_FRU_FTR_BOARD_INFO_AREA   2
#define IPMI_FRU_FTR_PRODUCT_INFO_AREA 3
#define IPMI_FRU_FTR_MULTI_RECORD_AREA 4
#define IPMI_FRU_FTR_NUMBER            (IPMI_FRU_FTR_MULTI_RECORD_AREA + 1)


typedef struct ipmi_fru_record_s ipmi_fru_record_t;

typedef struct fru_record_handlers_s
{
    void (*free)(ipmi_fru_record_t *item);
} fru_record_handlers_t;

struct ipmi_fru_record_s
{
    int                   type;
    fru_record_handlers_t *handlers;
    void                  *data;
};

struct ipmi_fru_s
{
    int deleted;

    ipmi_lock_t *lock;

    ipmi_domain_t        *domain;
    unsigned char        is_logical;
    unsigned char        device_address;
    unsigned char        device_id;
    unsigned char        lun;
    unsigned char        private_bus;
    unsigned char        channel;

    ipmi_fru_fetched_cb fetched_handler;
    void                *fetched_cb_data;

    int          fetch_in_progress;
    int          fetch_by_words;
    void         *data;
    unsigned int data_len;
    unsigned int curr_pos;

    unsigned char version;

    ipmi_fru_record_t *internal_use;
    ipmi_fru_record_t *chassis_info;
    ipmi_fru_record_t *board_info;
    ipmi_fru_record_t *product_info;
    ipmi_fru_record_t *multi_record;
};

/***********************************************************************
 *
 * general utilities
 *
 **********************************************************************/
static void
fru_lock(ipmi_fru_t *fru)
{
    ipmi_lock(fru->lock);
}

static void
fru_unlock(ipmi_fru_t *fru)
{
    ipmi_unlock(fru->lock);
}

static unsigned char
checksum(unsigned char *data, unsigned int length)
{
    unsigned char sum = 0;

    while (length) {
	sum += *data;
	data++;
	length--;
    }

    return sum;
}

static int
read_fru_time(unsigned char **data,
	      unsigned int  *len,
	      time_t        *time)
{
    time_t        t;
    struct tm     tm;
    unsigned char *d = *data;

    if (*len < 3)
	return ENODATA;

    /* minutes since 1996.01.01 00:00:00 */
    t = *d++ * 256 * 256;
    t += *d++ * 256;
    t += *d++;

    *len -= 3;
    *data += 3;

    /* Convert to seconds. */
    t *= 60;

    /* FIXME - we shouldn't depend on using mktime. */

    /* create date offset */
    tm.tm_sec = 0;
    tm.tm_min = 0;
    tm.tm_hour = 0;
    tm.tm_mday = 1;
    tm.tm_mon = 0;
    tm.tm_year = 96;
    tm.tm_isdst = 0;

    *time = t + mktime(&tm);

    return 0;
}

/***********************************************************************
 *
 * Basic string handling for FRUs.
 *
 **********************************************************************/

typedef struct fru_string_s
{
    enum ipmi_str_type_e type;
    unsigned short       length;
    char                 *str;
} fru_string_t;

static int
fru_decode_string(unsigned char **in,
		  unsigned int  *in_len,
		  int           lang_code,
		  int           force_english,
		  fru_string_t  *out)
{
    unsigned char str[IPMI_MAX_STR_LEN+1];
    int           force_unicode;
    unsigned int  skip = **in & 0x3f;

    if (**in == 0xc1)
	/* The field is not present. */
	return 0;

    if (skip+1 > *in_len)
	return EBADMSG;

    force_unicode = !force_english && (lang_code != IPMI_LANG_CODE_ENGLISH);
    out->length = ipmi_get_device_string(*in, *in_len, str, force_unicode,
					 &out->type, sizeof(str));

    *in += skip+1;
    in_len -= skip+1;

    if (out->length != 0) {
	out->str = ipmi_mem_alloc(out->length);
	if (!out->str)
	    return ENOMEM;
	memcpy(out->str, str, out->length);
    } else {
	out->str = NULL;
    }
    return 0;
}

static int
fru_string_to_out(char *out, unsigned int *length, fru_string_t *in)
{
    int clen;

    if (!in->str)
	return ENOSYS;

    if (in->length > *length)
	clen = *length;
    else
	clen = in->length;
    memcpy(out, in->str, clen);

    if (in->type == IPMI_ASCII_STR) {
	/* NIL terminate the ASCII string. */
	if (clen == *length)
	    clen--;

	out[clen] = '\0';
    }

    *length = clen;

    return 0;
}

static void
fru_free_string(fru_string_t *str)
{
    if (str->str)
	ipmi_mem_free(str->str);
}


/***********************************************************************
 *
 * Custom field handling for FRUs.  This is a variable-length array
 * of strings.
 *
 **********************************************************************/
typedef struct fru_variable_s
{
    unsigned short len;
    unsigned short next;
    fru_string_t   *strings;
} fru_variable_t;

static int
fru_decode_variable_string(unsigned char  **in,
			   unsigned int   *in_len,
			   int            lang_code,
			   fru_variable_t *v)
{
    int err;

    if (v->next == v->len) {
	fru_string_t *n;
	int          n_len = v->len + 5;

	n = ipmi_mem_alloc(sizeof(fru_string_t) * n_len);
	if (!n)
	    return ENOMEM;

	if (v->strings) {
	    memcpy(n, v->strings, sizeof(fru_string_t) * v->len);
	    ipmi_mem_free(v->strings);
	}
	v->strings = n;
	v->len = n_len;
    }

    err = fru_decode_string(in, in_len, lang_code, 0, &v->strings[v->next]);
    if (!err)
	v->next++;
    return err;
}

static int
fru_variable_string_to_out(char *out, unsigned int *length,
			   fru_variable_t *in, unsigned int num)
{
    if (num >= in->next)
	return EINVAL;

    return fru_string_to_out(out, length, &in->strings[num]);
}

static int
fru_variable_string_length(fru_variable_t *in,
			   unsigned int   num,
			   unsigned int   *length)
{
    if (num >= in->next)
	return EINVAL;

    *length = in->strings[num].length;
    return 0;
}

static int
fru_variable_string_type(fru_variable_t       *in,
			 unsigned int         num,
			 enum ipmi_str_type_e *type)
{
    if (num >= in->next)
	return EINVAL;

    *type = in->strings[num].type;
    return 0;
}

static void
fru_free_variable_string(fru_variable_t *v)
{
    int i;

    for (i=0; i<v->next; i++)
	fru_free_string(&v->strings[i]);

    if (v->strings)
	ipmi_mem_free(v->strings);
}


/***********************************************************************
 *
 * Here is the basic FRU handling.
 *
 **********************************************************************/

static ipmi_fru_record_t *
fru_record_alloc(int type, fru_record_handlers_t *handlers, int extra_len)
{
    ipmi_fru_record_t *rec;

    rec = ipmi_mem_alloc(sizeof(ipmi_fru_record_t)+extra_len);
    if (!rec)
	return NULL;

    memset(rec, 0, sizeof(ipmi_fru_record_t)+extra_len);

    rec->type = type;
    rec->handlers = handlers;
    rec->data = ((char *) rec) + sizeof(ipmi_fru_record_t);

    return rec;
}

static void *
fru_record_get_data(ipmi_fru_record_t *rec)
{
    return rec->data;
}

static void
fru_record_free(ipmi_fru_record_t *rec)
{
    ipmi_mem_free(rec);
}


/***********************************************************************
 *
 * Various macros for common handling.
 *
 **********************************************************************/

#define HANDLE_STR_DECODE(fname, force_english) \
    err = fru_decode_string(&data, &data_len, u->lang_code,	\
			    force_english, &u->fname);		\
    if (err)							\
	goto out_err

#define HANDLE_CUSTOM_DECODE() \
do {									\
    while ((data_len > 0) && (*data != 0xc1)) {				\
	err = fru_decode_variable_string(&data, &data_len,		\
					 IPMI_LANG_CODE_ENGLISH,	\
					 &u->custom);			\
	if (err)							\
	    goto out_err;						\
    }									\
} while (0)

#define GET_DATA_PREFIX(lcname, ucname) \
    ipmi_fru_ ## lcname ## _area_t *u;				\
    fru_lock(fru);						\
    if (!fru->lcname) {						\
	fru_unlock(fru);					\
	return ENOSYS;						\
    }								\
    u = fru_record_get_data(fru->lcname);			\
    if (fru->lcname->type != IPMI_FRU_FTR_## ucname ## _AREA) {	\
	fru_unlock(fru);					\
	return EINVAL;						\
    }

#define GET_DATA_STR(lcname, ucname, fname) \
int									\
ipmi_fru_get_ ## lcname ## _ ## fname ## _len(ipmi_fru_t   *fru,	\
					      unsigned int *length)	\
{									\
    GET_DATA_PREFIX(lcname, ucname);					\
    if (!u->fname.str) {						\
	fru_unlock(fru);						\
	return ENOSYS;							\
    }									\
    *length = u->fname.length;						\
    fru_unlock(fru);							\
    return 0;								\
}									\
int									\
ipmi_fru_get_ ## lcname ## _ ## fname ## _type(ipmi_fru_t           *fru,\
					       enum ipmi_str_type_e *type)\
{									\
    GET_DATA_PREFIX(lcname, ucname);					\
    if (!u->fname.str) {						\
	fru_unlock(fru);						\
	return ENOSYS;							\
    }									\
    *type = u->fname.type;						\
    fru_unlock(fru);							\
    return 0;								\
}									\
int									\
ipmi_fru_get_ ## lcname ## _ ## fname(ipmi_fru_t	*fru,		\
				      char              *str,		\
				      unsigned int      *strlen)	\
{									\
    int rv;								\
    GET_DATA_PREFIX(lcname, ucname);					\
    rv = fru_string_to_out(str, strlen, &u->fname);			\
    fru_unlock(fru);							\
    return rv;								\
}

#define GET_CUSTOM_STR(lcname, ucname) \
int									\
ipmi_fru_get_ ## lcname ## _ ## custom ## _len(ipmi_fru_t   *fru,	\
					       unsigned int num,	\
					       unsigned int *length)	\
{									\
    int rv;								\
    GET_DATA_PREFIX(lcname, ucname);					\
    rv = fru_variable_string_length(&u->custom, num, length);		\
    fru_unlock(fru);							\
    return rv;								\
}									\
int									\
ipmi_fru_get_ ## lcname ## _ ## custom ## _type(ipmi_fru_t   *fru,	\
					        unsigned int num,	\
					        enum ipmi_str_type_e *type) \
{									\
    int rv;								\
    GET_DATA_PREFIX(lcname, ucname);					\
    rv = fru_variable_string_type(&u->custom, num, type);		\
    fru_unlock(fru);							\
    return rv;								\
}									\
int									\
ipmi_fru_get_ ## lcname ## _ ## custom(ipmi_fru_t	 *fru,		\
				       unsigned int      num,		\
				       char              *str,		\
				       unsigned int      *strlen)	\
{									\
    int rv;								\
    GET_DATA_PREFIX(lcname, ucname);					\
    rv = fru_variable_string_to_out(str, strlen, &u->custom, num);	\
    fru_unlock(fru);							\
    return rv;								\
}



/***********************************************************************
 *
 * Handling for FRU internal use areas.
 *
 **********************************************************************/

typedef struct ipmi_fru_internal_use_area_s
{
    /* version bit 7-4 reserved (0000), bit 3-0 == 0001 */
    unsigned char  version;
    unsigned short length;
    unsigned char  *data;
} ipmi_fru_internal_use_area_t;


static void
fru_internal_use_area_free(ipmi_fru_record_t *rec)
{
    ipmi_fru_internal_use_area_t *u = fru_record_get_data(rec);

    ipmi_mem_free(u->data);
    fru_record_free(rec);
}

fru_record_handlers_t internal_use_handlers =
{
    .free = fru_internal_use_area_free,
};

static int
fru_decode_internal_use_area(unsigned char     *data,
			     unsigned int      data_len,
			     ipmi_fru_record_t **rrec)
{
    ipmi_fru_internal_use_area_t *u;
    ipmi_fru_record_t            *rec;

    rec = fru_record_alloc(IPMI_FRU_FTR_INTERNAL_USE_AREA,
			   &internal_use_handlers,
			   sizeof(ipmi_fru_internal_use_area_t));
    if (!rec)
	return ENOMEM;

    u = fru_record_get_data(rec);

    u->version = *data;
    u->length = data_len-1;
    u->data = ipmi_mem_alloc(u->length);
    if (!u->data) {
	ipmi_mem_free(rec);
	return ENOMEM;
    }

    memcpy(u->data, data+1, u->length);

    *rrec = rec;

    return 0;
}

int 
ipmi_fru_get_internal_use_version(ipmi_fru_t    *fru,
				  unsigned char *version)
{
    GET_DATA_PREFIX(internal_use, INTERNAL_USE);

    *version = u->version;

    fru_unlock(fru);

    return 0;
}

int 
ipmi_fru_get_internal_use_length(ipmi_fru_t   *fru,
				 unsigned int *length)
{
    GET_DATA_PREFIX(internal_use, INTERNAL_USE);

    *length = u->length;

    fru_unlock(fru);

    return 0;
}


int 
ipmi_fru_get_internal_use_data(ipmi_fru_t    *fru,
			       unsigned char *data,
			       unsigned int  *max_len)
{
    int l;
    GET_DATA_PREFIX(internal_use, INTERNAL_USE);

    l = *max_len;

    if (l > u->length)
	l = u->length;

    memcpy(data, u->data, l);

    *max_len = l;

    fru_unlock(fru);

    return 0;
}



/***********************************************************************
 *
 * Handling for FRU chassis info areas
 *
 **********************************************************************/

typedef struct ipmi_fru_chassis_info_area_s
{
    /* version bit 7-4 reserved (0000), bit 3-0 == 0001 */
    unsigned char  version;
    unsigned char  type;  /* chassis type CT_xxxx */
    unsigned char  lang_code;
    fru_string_t   part_number;
    fru_string_t   serial_number;
    fru_variable_t custom;
} ipmi_fru_chassis_info_area_t;

static void
fru_chassis_info_area_free(ipmi_fru_record_t *rec)
{
    ipmi_fru_chassis_info_area_t *u = fru_record_get_data(rec);

    fru_free_string(&u->part_number);
    fru_free_string(&u->serial_number);
    fru_free_variable_string(&u->custom);
    fru_record_free(rec);
}

fru_record_handlers_t chassis_info_handlers =
{
    .free = fru_chassis_info_area_free,
};

static int
fru_decode_chassis_info_area(unsigned char     *data,
			     unsigned int      data_len,
			     ipmi_fru_record_t **rrec)
{
    ipmi_fru_chassis_info_area_t *u;
    ipmi_fru_record_t            *rec;
    int                          err;
    unsigned char                version;
    unsigned char                length;

    version = *data;
    length = (*(data+1)) * 8;
    if ((length == 0) || (length > data_len))
	return EBADMSG;

    if (checksum(data, length) != 0)
	return EBADMSG;

    data_len--; /* remove the checksum */

    rec = fru_record_alloc(IPMI_FRU_FTR_CHASSIS_INFO_AREA,
			   &chassis_info_handlers,
			   sizeof(*u));
    if (!rec)
	return ENOMEM;

    u = fru_record_get_data(rec);

    u->version = version;
    data += 2; data_len -= 2;
    u->type = *data;
    data++; data_len--;
    u->lang_code = IPMI_LANG_CODE_ENGLISH;
    HANDLE_STR_DECODE(part_number, 1);
    HANDLE_STR_DECODE(serial_number, 1);
    HANDLE_CUSTOM_DECODE();

    *rrec = rec;

    return 0;

 out_err:
    fru_chassis_info_area_free(rec);
    return err;
}

int 
ipmi_fru_get_chassis_info_version(ipmi_fru_t    *fru,
				  unsigned char *version)
{
    GET_DATA_PREFIX(chassis_info, CHASSIS_INFO);
    
    *version = u->version;

    fru_unlock(fru);

    return 0;
}

int 
ipmi_fru_get_chassis_info_type(ipmi_fru_t    *fru,
			       unsigned char *type)
{
    GET_DATA_PREFIX(chassis_info, CHASSIS_INFO);
    
    *type = u->type;

    fru_unlock(fru);

    return 0;
}

GET_DATA_STR(chassis_info, CHASSIS_INFO, part_number)
GET_DATA_STR(chassis_info, CHASSIS_INFO, serial_number)
GET_CUSTOM_STR(chassis_info, CHASSIS_INFO)

/***********************************************************************
 *
 * Handling for FRU board info areas
 *
 **********************************************************************/

typedef struct ipmi_fru_board_info_area_s
{
    /* version bit 7-4 reserved (0000), bit 3-0 == 0001 */
    unsigned char  version;
    unsigned char  lang_code;
    time_t         mfg_time;
    fru_string_t   board_manufacturer;
    fru_string_t   board_product_name;
    fru_string_t   board_serial_number;
    fru_string_t   board_part_number;
    fru_string_t   fru_file_id;
    fru_variable_t custom;
} ipmi_fru_board_info_area_t;

static void
fru_board_info_area_free(ipmi_fru_record_t *rec)
{
    ipmi_fru_board_info_area_t *u = fru_record_get_data(rec);

    fru_free_string(&u->board_manufacturer);
    fru_free_string(&u->board_product_name);
    fru_free_string(&u->board_serial_number);
    fru_free_string(&u->board_part_number);
    fru_free_string(&u->fru_file_id);
    fru_free_variable_string(&u->custom);
    fru_record_free(rec);
}

fru_record_handlers_t board_info_handlers =
{
    .free = fru_board_info_area_free,
};

static int
fru_decode_board_info_area(unsigned char     *data,
			   unsigned int      data_len,
			   ipmi_fru_record_t **rrec)
{
    ipmi_fru_board_info_area_t *u;
    ipmi_fru_record_t            *rec;
    int                          err;
    unsigned char                version;
    unsigned char                length;

    version = *data;
    length = (*(data+1)) * 8;
    if ((length == 0) || (length > data_len))
	return EBADMSG;

    if (checksum(data, length) != 0)
	return EBADMSG;

    data_len--; /* remove the checksum */

    rec = fru_record_alloc(IPMI_FRU_FTR_BOARD_INFO_AREA,
			   &board_info_handlers,
			   sizeof(*u));
    if (!rec)
	return ENOMEM;

    u = fru_record_get_data(rec);

    u->version = version;
    data += 2; data_len -= 2;
    u->lang_code = *data;
    if (u->lang_code == 0)
	u->lang_code = IPMI_LANG_CODE_ENGLISH;
    data++; data_len--;

    err = read_fru_time(&data, &data_len, &u->mfg_time);
    if (err)
	goto out_err;

    HANDLE_STR_DECODE(board_manufacturer, 0);
    HANDLE_STR_DECODE(board_product_name, 0);
    HANDLE_STR_DECODE(board_serial_number, 1);
    HANDLE_STR_DECODE(board_part_number, 1);
    HANDLE_STR_DECODE(fru_file_id, 1);
    HANDLE_CUSTOM_DECODE();

    *rrec = rec;

    return 0;

 out_err:
    fru_board_info_area_free(rec);
    return err;
}

int 
ipmi_fru_get_board_info_version(ipmi_fru_t    *fru,
				unsigned char *version)
{
    GET_DATA_PREFIX(board_info, BOARD_INFO);
    
    *version = u->version;

    fru_unlock(fru);

    return 0;
}

int 
ipmi_fru_get_board_info_lang_code(ipmi_fru_t    *fru,
				  unsigned char *type)
{
    GET_DATA_PREFIX(board_info, BOARD_INFO);
    
    *type = u->lang_code;

    fru_unlock(fru);

    return 0;
}

int 
ipmi_fru_get_board_info_mfg_time(ipmi_fru_t *fru,
				 time_t     *time)
{
    GET_DATA_PREFIX(board_info, BOARD_INFO);
    
    *time = u->mfg_time;

    fru_unlock(fru);

    return 0;
}

GET_DATA_STR(board_info, BOARD_INFO, board_manufacturer)
GET_DATA_STR(board_info, BOARD_INFO, board_product_name)
GET_DATA_STR(board_info, BOARD_INFO, board_serial_number)
GET_DATA_STR(board_info, BOARD_INFO, board_part_number)
GET_DATA_STR(board_info, BOARD_INFO, fru_file_id)
GET_CUSTOM_STR(board_info, BOARD_INFO)

/***********************************************************************
 *
 * Handling for FRU product info areas
 *
 **********************************************************************/

typedef struct ipmi_fru_product_info_area_s
{
    /* version bit 7-4 reserved (0000), bit 3-0 == 0001 */
    unsigned char  version;
    unsigned char  lang_code;
    fru_string_t   manufacturer_name;
    fru_string_t   product_name;
    fru_string_t   product_part_model_number;
    fru_string_t   product_version;
    fru_string_t   product_serial_number;
    fru_string_t   asset_tag;
    fru_string_t   fru_file_id;
    fru_variable_t custom;
} ipmi_fru_product_info_area_t;

static void
fru_product_info_area_free(ipmi_fru_record_t *rec)
{
    ipmi_fru_product_info_area_t *u = fru_record_get_data(rec);

    fru_free_string(&u->manufacturer_name);
    fru_free_string(&u->product_name);
    fru_free_string(&u->product_part_model_number);
    fru_free_string(&u->product_version);
    fru_free_string(&u->product_serial_number);
    fru_free_string(&u->asset_tag);
    fru_free_string(&u->fru_file_id);
    fru_free_variable_string(&u->custom);
    fru_record_free(rec);
}

fru_record_handlers_t product_info_handlers =
{
    .free = fru_product_info_area_free,
};

static int
fru_decode_product_info_area(unsigned char     *data,
			     unsigned int      data_len,
			     ipmi_fru_record_t **rrec)
{
    ipmi_fru_product_info_area_t *u;
    ipmi_fru_record_t            *rec;
    int                          err;
    unsigned char                version;
    unsigned char                length;

    version = *data;
    length = (*(data+1)) * 8;
    if ((length == 0) || (length > data_len))
	return EBADMSG;

    if (checksum(data, length) != 0)
	return EBADMSG;

    data_len--; /* remove the checksum */

    rec = fru_record_alloc(IPMI_FRU_FTR_PRODUCT_INFO_AREA,
			   &product_info_handlers,
			   sizeof(*u));
    if (!rec)
	return ENOMEM;

    u = fru_record_get_data(rec);

    u->version = version;
    data += 2; data_len -= 2;
    u->lang_code = *data;
    if (u->lang_code == 0)
	u->lang_code = IPMI_LANG_CODE_ENGLISH;
    data++; data_len--;
    HANDLE_STR_DECODE(manufacturer_name, 0);
    HANDLE_STR_DECODE(product_name, 0);
    HANDLE_STR_DECODE(product_part_model_number, 0);
    HANDLE_STR_DECODE(product_version, 0);
    HANDLE_STR_DECODE(product_serial_number, 1);
    HANDLE_STR_DECODE(asset_tag, 0);
    HANDLE_STR_DECODE(fru_file_id, 1);
    HANDLE_CUSTOM_DECODE();

    *rrec = rec;

    return 0;

 out_err:
    fru_product_info_area_free(rec);
    return err;
}

int 
ipmi_fru_get_product_info_version(ipmi_fru_t    *fru,
				  unsigned char *version)
{
    GET_DATA_PREFIX(product_info, PRODUCT_INFO);
    
    *version = u->version;

    fru_unlock(fru);

    return 0;
}

int 
ipmi_fru_get_product_info_lang_code(ipmi_fru_t    *fru,
				    unsigned char *type)
{
    GET_DATA_PREFIX(product_info, PRODUCT_INFO);
    
    *type = u->lang_code;

    fru_unlock(fru);

    return 0;
}

GET_DATA_STR(product_info, PRODUCT_INFO, manufacturer_name)
GET_DATA_STR(product_info, PRODUCT_INFO, product_name)
GET_DATA_STR(product_info, PRODUCT_INFO, product_part_model_number)
GET_DATA_STR(product_info, PRODUCT_INFO, product_version)
GET_DATA_STR(product_info, PRODUCT_INFO, product_serial_number)
GET_DATA_STR(product_info, PRODUCT_INFO, asset_tag)
GET_DATA_STR(product_info, PRODUCT_INFO, fru_file_id)
GET_CUSTOM_STR(product_info, PRODUCT_INFO)

typedef struct fru_offset_s
{
    int type;
    int offset;
} fru_offset_t;

static int
process_fru_info(ipmi_fru_t *fru)
{
    unsigned char *data = fru->data;
    unsigned int  data_len = fru->data_len;
    fru_offset_t  foff[IPMI_FRU_FTR_NUMBER];
    int           i, j;
    int           err = 0;

    if (checksum(data, 8) != 0)
	return EBADMSG;

    fru->version = *data;

    for (i=0; i<IPMI_FRU_FTR_NUMBER; i++) {
	foff[i].type = i;
	foff[i].offset = data[i+1] * 8;
	if (foff[i].offset >= data_len)
	    return EBADMSG;
    }

    /* Sort the field by offset.  Not many fields, so we use a bubble
       sort.  We sort these so we can find the start of the next
       area and know the size of the current area. */
    for (i=0; i<IPMI_FRU_FTR_NUMBER; i++) {
	for (j=0; j<IPMI_FRU_FTR_NUMBER-1; j++) {
	    fru_offset_t tmp;
	    if (foff[j].offset > foff[j+1].offset) {
		tmp = foff[j];
		foff[j] = foff[j+1];
		foff[j+1] = tmp;
	    }
	}
    }

    for (i=0; i<IPMI_FRU_FTR_NUMBER; i++) {
	int plen, next_off, offset;

	offset = foff[i].offset;
	if (offset == 0)
	    continue;

	if (i == (IPMI_FRU_FTR_NUMBER - 1))
	    next_off = data_len;
	else
	    next_off = foff[i+1].offset;
	plen = next_off - offset;

	switch (foff[i].type) {
	case IPMI_FRU_FTR_INTERNAL_USE_AREA:
	    err = fru_decode_internal_use_area(data+offset, plen,
					       &fru->internal_use);
	    break;

	case IPMI_FRU_FTR_CHASSIS_INFO_AREA:
	    err = fru_decode_chassis_info_area(data+offset, plen,
					       &fru->chassis_info);
	    break;

	case IPMI_FRU_FTR_BOARD_INFO_AREA:
	    err = fru_decode_board_info_area(data+offset, plen,
					       &fru->board_info);
	    break;

	case IPMI_FRU_FTR_PRODUCT_INFO_AREA:
	    err = fru_decode_product_info_area(data+offset, plen,
					       &fru->product_info);
	    break;

	case IPMI_FRU_FTR_MULTI_RECORD_AREA:
	    /* FIXME */
	    break;
	}

	if (err)
	    goto out_err;
    }

    return 0;

 out_err:
    return err;
}

static void
fru_record_destroy(ipmi_fru_record_t *rec)
{
    if (rec)
	rec->handlers->free(rec);
}

static void
final_fru_destroy(ipmi_fru_t *fru)
{
    fru_record_destroy(fru->internal_use);
    fru_record_destroy(fru->chassis_info);
    fru_record_destroy(fru->board_info);
    fru_record_destroy(fru->product_info);
    fru_record_destroy(fru->multi_record);
    fru_unlock(fru);
    ipmi_destroy_lock(fru->lock);
    ipmi_mem_free(fru);
}

int
ipmi_fru_destroy(ipmi_fru_t            *fru,
		 ipmi_fru_destroyed_cb handler,
		 void                  *cb_data)
{
    /* FIXME - handle callbacks. */
    fru_lock(fru);
    if (fru->fetch_in_progress) {
	fru->deleted = 1;
	fru_unlock(fru);
    } else {
	final_fru_destroy(fru);
    }
    return 0;
}

void
fetch_complete(ipmi_fru_t *fru, int err)
{
    if (!err)
	err = process_fru_info(fru);

    if (fru->fetched_handler)
	fru->fetched_handler(fru, err, fru->fetched_cb_data);
    fru->fetch_in_progress = 0;

    if (fru->data)
	ipmi_mem_free(fru->data);
    fru->data = NULL;

    if (fru->deleted)
      final_fru_destroy(fru);
    else
      fru_unlock(fru);
}

static int request_next_data(ipmi_fru_t   *fru,
			     ipmi_addr_t  *addr,
			     unsigned int addr_len);

static void
fru_data_handler(ipmi_domain_t *domain,
		 ipmi_addr_t   *addr,
		 unsigned int  addr_len,
		 ipmi_msg_t    *msg,
		 void          *rsp_data1,
		 void          *rsp_data2)
{
    ipmi_fru_t    *fru = rsp_data1;
    unsigned char *data = msg->data;
    int           count;
    int           err;

    fru_lock(fru);

    if (fru->deleted) {
	fetch_complete(fru, ECANCELED);
	goto out;
    }

    if (data[0] != 0) {
	if (fru->curr_pos >= 8) {
	    /* Some screwy cards give more size in the info than they
	       really have, if we have enough, try to process it. */
	    ipmi_log(IPMI_LOG_WARNING, "IPMI error getting FRU data: %x",
		     data[0]);
	    fru->data_len = fru->curr_pos;
	    fetch_complete(fru, 0);
	} else {
	    ipmi_log(IPMI_LOG_ERR_INFO, "IPMI error getting FRU data: %x",
		     data[0]);
	    fetch_complete(fru, IPMI_IPMI_ERR_VAL(data[0]));
	}
	goto out;
    }

    if (msg->data_len < 2) {
	ipmi_log(IPMI_LOG_ERR_INFO, "FRU data response too small");
	fetch_complete(fru, EINVAL);
	goto out;
    }

    count = data[1] << fru->fetch_by_words;

    if (count == 0) {
	ipmi_log(IPMI_LOG_ERR_INFO, "FRU got zero-sized data, must make"
		 " progress!");
	fetch_complete(fru, EINVAL);
	goto out;
    }

    if (count > msg->data_len-2) {
	ipmi_log(IPMI_LOG_ERR_INFO, "FRU data count mismatch");
	fetch_complete(fru, EINVAL);
	goto out;
    }

    memcpy(fru->data+fru->curr_pos, data+2, count);
    fru->curr_pos += count;

    if (fru->curr_pos < fru->data_len) {
	/* More to fetch. */
	err = request_next_data(fru, addr, addr_len);
	if (err) {
	    ipmi_log(IPMI_LOG_ERR_INFO, "Error requesting next FRU data");
	    fetch_complete(fru, err);
	    goto out;
	}
    } else {
	fetch_complete(fru, 0);
	goto out;
    }

    fru_unlock(fru);
 out:
    return;
}

static int
request_next_data(ipmi_fru_t   *fru,
		  ipmi_addr_t  *addr,
		  unsigned int addr_len)
{
    unsigned char cmd_data[4];
    ipmi_msg_t    msg;
    int           to_read;

    /* We only request as much as we have to.  Don't always reqeust
       the maximum amount, some machines don't like this. */
    to_read = fru->data_len - fru->curr_pos;
    if (to_read > MAX_FRU_DATA_FETCH)
	to_read = MAX_FRU_DATA_FETCH;

    cmd_data[0] = fru->device_id;
    ipmi_set_uint16(cmd_data+1, fru->curr_pos >> fru->fetch_by_words);
    cmd_data[3] = to_read >> fru->fetch_by_words;
    msg.data = cmd_data;
    msg.netfn = IPMI_STORAGE_NETFN;
    msg.cmd = IPMI_READ_FRU_DATA_CMD;
    msg.data = cmd_data;
    msg.data_len = 4;

    return ipmi_send_command_addr(fru->domain,
				  addr, addr_len,
				  &msg,
				  fru_data_handler,
				  fru,
				  NULL);
}

static void
fru_inventory_area_handler(ipmi_domain_t *domain,
			   ipmi_addr_t   *addr,
			   unsigned int  addr_len,
			   ipmi_msg_t    *msg,
			   void          *rsp_data1,
			   void          *rsp_data2)
{
    ipmi_fru_t    *fru = rsp_data1;
    unsigned char *data = msg->data;
    int           err;

    fru_lock(fru);

    if (fru->deleted) {
	fetch_complete(fru, ECANCELED);
	goto out;
    }

    if (data[0] != 0) {
	ipmi_log(IPMI_LOG_ERR_INFO,
		 "IPMI error getting FRU inventory area: %x",
		 data[0]);
	fetch_complete(fru, IPMI_IPMI_ERR_VAL(data[0]));
	goto out;
    }

    if (msg->data_len < 4) {
	ipmi_log(IPMI_LOG_ERR_INFO, "FRU inventory area too small");
	fetch_complete(fru, EINVAL);
	goto out;
    }

    fru->data_len = ipmi_get_uint16(data+1);
    fru->fetch_by_words = data[3] & 1;

    if (fru->data_len < 8) {
	ipmi_log(IPMI_LOG_ERR_INFO, "FRU space less than the header");
	fetch_complete(fru, EMSGSIZE);
	goto out;
    }

    fru->data = ipmi_mem_alloc(fru->data_len);
    if (!fru->data) {
	ipmi_log(IPMI_LOG_ERR_INFO, "Error allocating FRU data");
	fetch_complete(fru, ENOMEM);
	goto out;
    }

    err = request_next_data(fru, addr, addr_len);
    if (err) {
	ipmi_log(IPMI_LOG_ERR_INFO, "Error requesting next FRU data");
	fetch_complete(fru, err);
	goto out;
    }

    fru_unlock(fru);
 out:
    return;
}

static int
start_logical_fru_fetch(ipmi_fru_t *fru)
{
    unsigned char    cmd_data[1];
    ipmi_ipmb_addr_t ipmb;
    ipmi_msg_t       msg;

    ipmb.addr_type = IPMI_IPMB_ADDR_TYPE;
    ipmb.channel = fru->channel;
    ipmb.slave_addr = fru->device_address;
    ipmb.lun = fru->lun;

    cmd_data[0] = fru->device_id;
    msg.netfn = IPMI_STORAGE_NETFN;
    msg.cmd = IPMI_GET_FRU_INVENTORY_AREA_INFO_CMD;
    msg.data = cmd_data;
    msg.data_len = 1;

    return ipmi_send_command_addr(fru->domain,
				  (ipmi_addr_t *) &ipmb,
				  sizeof(ipmb),
				  &msg,
				  fru_inventory_area_handler,
				  fru,
				  NULL);
}

static int
start_physical_fru_fetch(ipmi_fru_t *fru)
{
    /* FIXME - this is going to suck, but needs to be implemented. */
    return ENOSYS;
}

int
ipmi_fru_alloc(ipmi_domain_t *domain,
	       unsigned char is_logical,
	       unsigned char device_address,
	       unsigned char device_id,
	       unsigned char lun,
	       unsigned char private_bus,
	       unsigned char channel,
	       ipmi_fru_fetched_cb fetched_handler,
	       void                *fetched_cb_data,
	       ipmi_fru_t    **new_fru)
{
    ipmi_fru_t *fru;
    int        err;

    fru = ipmi_mem_alloc(sizeof(*fru));
    if (!fru)
	return ENOMEM;
    memset(fru, 0, sizeof(*fru));

    err = ipmi_create_lock(domain, &fru->lock);
    if (err) {
	ipmi_mem_free(fru);
	return err;
    }

    fru->domain = domain;
    fru->is_logical = is_logical;
    fru->device_address = device_address;
    fru->device_id = device_id;
    fru->lun = lun;
    fru->private_bus = private_bus;
    fru->channel = channel;

    fru->fetched_handler = fetched_handler;
    fru->fetched_cb_data = fetched_cb_data;

    fru->deleted = 0;
    fru->fetch_in_progress = 1;

    if (fru->is_logical)
	err = start_logical_fru_fetch(fru);
    else
	err = start_physical_fru_fetch(fru);
    if (err) {
	ipmi_destroy_lock(fru->lock);
	ipmi_mem_free(fru);
	return err;
    }

    if (new_fru)
	*new_fru = fru;
    return 0;
}
