/*
 * img.c
 *
 * Sector image files for IBM/ISO track formats.
 *
 * Written & released by Keir Fraser <keir.xen@gmail.com>
 *
 * This is free and unencumbered software released into the public domain.
 * See the file COPYING for more details, or visit <http://unlicense.org>.
 */

static void img_extend(struct image *im);
static void img_setup_track(
    struct image *im, uint16_t track, uint32_t *start_pos);
static bool_t img_read_track(struct image *im);
static bool_t img_write_track(struct image *im);
static bool_t mfm_open(struct image *im);
static bool_t mfm_read_track(struct image *im);
static bool_t mfm_write_track(struct image *im);
static bool_t fm_open(struct image *im);
static bool_t fm_read_track(struct image *im);
static bool_t fm_write_track(struct image *im);

static bool_t msx_open(struct image *im);
static bool_t pc_dos_open(struct image *im);
static bool_t ti99_open(struct image *im);

#define LAYOUT_interleaved              0
#define LAYOUT_interleaved_swap_sides   1
#define LAYOUT_sequential_reverse_side1 2

#define sec_sz(im) (128u << (im)->img.sec_no)

#define _IAM 1 /* IAM */
#define _ITN 1 /* inter-track numbering */
#define _C(cyls) ((cyls) / 40)
#define _R(rpm) ((rpm) / 60 - 5)
#define _S(sides) ((sides) - 1)
const static struct img_type {
    uint8_t nr_secs:6;
    uint8_t nr_sides:1;
    uint8_t has_iam:1;
    uint8_t gap3;
    uint8_t interleave:3;
    uint8_t no:3;
    uint8_t base:1;
    uint8_t inter_track_numbering:1;
    uint8_t skew:4;
    uint8_t cyls:2;
    uint8_t rpm:2;
} img_type[] = {
    {  8, _S(1), _IAM, 84, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 160k */
    {  9, _S(1), _IAM, 84, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 180k */
    { 10, _S(1), _IAM, 30, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 200k */
    {  8, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 320k */
    {  9, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 360k (#1) */
    { 10, _S(2), _IAM, 30, 1, 2, 1, 0, 0, _C(40), _R(300) }, /* 400k (#1) */
    { 15, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(360) }, /* 1.2MB */
    {  9, _S(1), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 360k (#2) */
    { 10, _S(1), _IAM, 30, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 400k (#2) */
    { 11, _S(1), _IAM,  3, 2, 2, 1, 0, 0, _C(80), _R(300) }, /* 440k */
    {  8, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 640k */
    {  9, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 720k */
    { 10, _S(2), _IAM, 30, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 800k */
    { 11, _S(2), _IAM,  3, 2, 2, 1, 0, 0, _C(80), _R(300) }, /* 880k */
    { 18, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 1.44M */
    { 19, _S(2), _IAM, 70, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 1.52M */
    { 21, _S(2), _IAM, 18, 2, 2, 1, 0, 0, _C(80), _R(300) }, /* 1.68M */
    { 20, _S(2), _IAM, 40, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 1.6M */
    { 36, _S(2), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 2.88M */
    { 0 }
}, adfs_type[] = {
    /* ADFS D/E: 5 * 1kB, 800k */
    {  5, _S(2), _IAM, 116, 1, 3, 0, 0, 1, _C(80), _R(300) },
    /* ADFS F: 10 * 1kB, 1600k */
    { 10, _S(2), _IAM, 116, 1, 3, 0, 0, 2, _C(80), _R(300) },
    /* ADFS L 640k */
    { 16, _S(2), _IAM,  57, 1, 1, 0, 0, 0, _C(80), _R(300) },
    /* ADFS M 320k */
    { 16, _S(1), _IAM,  57, 1, 1, 0, 0, 0, _C(80), _R(300) },
    /* ADFS S 160k */
    { 16, _S(1), _IAM,  57, 1, 1, 0, 0, 0, _C(40), _R(300) },
    { 0 }
}, akai_type[] = {
    /* Akai DD:  5*1kB sectors */
    {  5, _S(2), _IAM, 116, 1, 3, 1, 0, 0, _C(80), _R(300) },
    /* Akai HD: 10*1kB sectors */
    { 10, _S(2), _IAM, 116, 1, 3, 1, 0, 0, _C(80), _R(300) },
    { 0 }
}, casio_type[] = {
    { 8, _S(2), _IAM, 116, 3, 3, 1, 0, 0, _C(80), _R(360) }, /* 1280k */
    { 0 }
}, d81_type[] = {
    { 10, _S(2), _IAM, 30, 1, 2, 1, 0, 0, _C(80), _R(300) },
    { 0 }
}, dec_type[] = {
    { 10, _S(1), _IAM, 30, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* RX50 (400k) */
    { 0 } /* RX33 (1.2MB) from default list */
}, ensoniq_type[] = {
    { 10, _S(2), _IAM, 30, 1, 2, 0, 0, 0, _C(80), _R(300) }, /* 800kB */
    { 20, _S(2), _IAM, 40, 1, 2, 0, 0, 0, _C(80), _R(300) }, /* 1.6MB */
    { 0 }
}, fluke_type[] = {
    { 16, _S(2), _IAM, 57, 2, 1, 0, 0, 0, _C(80), _R(300) },
    { 0 }
}, kaypro_type[] = {
    { 10, _S(1), _IAM, 30, 3, 2, 0, _ITN, 0, _C(40), _R(300) }, /* 200k */
    { 10, _S(2), _IAM, 30, 3, 2, 0, _ITN, 0, _C(40), _R(300) }, /* 400k */
    { 10, _S(2), _IAM, 30, 3, 2, 0, _ITN, 0, _C(80), _R(300) }, /* 800k */
    { 0 }
}, mbd_type[] = {
    { 11, _S(2), _IAM,  30, 1, 3, 1, 0, 0, _C(80), _R(300) },
    {  5, _S(2), _IAM, 116, 3, 1, 1, 0, 0, _C(80), _R(300) },
    { 11, _S(2), _IAM,  30, 1, 3, 1, 0, 0, _C(40), _R(300) },
    {  5, _S(2), _IAM, 116, 3, 1, 1, 0, 0, _C(40), _R(300) },
    { 0 }
}, memotech_type[] = {
    { 16, _S(2), _IAM, 57, 3, 1, 1, 0, 0, _C(40), _R(300) }, /* Type 03 */
    { 16, _S(2), _IAM, 57, 3, 1, 1, 0, 0, _C(80), _R(300) }, /* Type 07 */
    { 0 }
}, msx_type[] = {
    { 8, _S(1), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 320k */
    { 9, _S(1), _IAM, 84, 1, 2, 1, 0, 0, _C(80), _R(300) }, /* 360k */
    { 0 } /* all other formats from default list */
}, nascom_type[] = {
    { 16, _S(1), _IAM, 57, 3, 1, 1, 0, 8, _C(80), _R(300) }, /* 320k */
    { 16, _S(2), _IAM, 57, 3, 1, 1, 0, 8, _C(80), _R(300) }, /* 360k */
    { 0 }
}, pc98_type[] = {
    { 8, _S(2), _IAM, 116, 1, 3, 1, 0, 0, _C(80), _R(360) }, /* 1232k */
    { 8, _S(2), _IAM, 116, 1, 2, 1, 0, 0, _C(80), _R(360) }, /* 640k */
    { 9, _S(2), _IAM, 116, 1, 2, 1, 0, 0, _C(80), _R(360) }, /* 720k */
    { 0 }
}, uknc_type[] = {
    { 10, _S(2), 0, 38, 1, 2, 1, 0, 0, _C(80), _R(300) },
    { 0 }
};

static FSIZE_t im_size(struct image *im)
{
    return (f_size(&im->fp) < im->img.base_off) ? 0
        : (f_size(&im->fp) - im->img.base_off);
}

static uint8_t sec_base(struct image *im)
{
    return im->img.sec_base[im->cur_track & (im->nr_sides - 1)];
}

static unsigned int enc_sec_sz(struct image *im)
{
    return im->img.idam_sz + im->img.dam_sz_pre
        + sec_sz(im) + im->img.dam_sz_post;
}

static bool_t _img_open(struct image *im, const struct img_type *type)
{
    unsigned int nr_cyls, cyl_sz, nr_sides;

    /* Walk the layout/type hints looking for a match on file size. */
    for (; type->nr_secs != 0; type++) {
        unsigned int min_cyls, max_cyls;
        switch (type->cyls) {
        case _C(40):
            min_cyls = 38;
            max_cyls = 42;
            break;
        case _C(80):
        default:
            min_cyls = 77;
            max_cyls = 85;
            break;
        }
        nr_sides = type->nr_sides + 1;
        cyl_sz = type->nr_secs * (128 << type->no) * nr_sides;
        for (nr_cyls = min_cyls; nr_cyls <= max_cyls; nr_cyls++)
            if ((nr_cyls * cyl_sz) == im_size(im))
                goto found;
    }

    return FALSE;

found:
    im->nr_cyls = nr_cyls;
    im->nr_sides = nr_sides;
    im->img.sec_no = type->no;
    im->img.interleave = type->interleave;
    im->img.skew = type->skew;
    im->img.nr_sectors = type->nr_secs;
    im->img.gap_3 = type->gap3;
    im->img.rpm = (type->rpm + 5) * 60;
    im->img.sec_base[0] = im->img.sec_base[1] = type->base;
    if (type->inter_track_numbering == _ITN)
        im->img.sec_base[1] += im->img.nr_sectors;
    im->img.has_iam = type->has_iam;

    return mfm_open(im);
}

static bool_t adfs_open(struct image *im)
{
    im->img.skew_cyls_only = TRUE;
    return _img_open(im, adfs_type);
}

static bool_t img_open(struct image *im)
{
    const struct img_type *type;

    switch (ff_cfg.host) {
    case HOST_akai:
    case HOST_gem:
        type = akai_type;
        break;
    case HOST_casio:
        type = casio_type;
        break;
    case HOST_dec:
        type = dec_type;
        break;
    case HOST_ensoniq:
        type = ensoniq_type;
        break;
    case HOST_fluke:
        type = fluke_type;
        break;
    case HOST_kaypro:
        type = kaypro_type;
        break;
    case HOST_memotech:
        type = memotech_type;
        break;
    case HOST_msx:
        if (msx_open(im))
            return TRUE;
        goto fallback;
    case HOST_nascom:
        type = nascom_type;
        im->img.skew_cyls_only = TRUE;
        break;
    case HOST_pc98:
        type = pc98_type;
        break;
    case HOST_pc_dos:
        if (pc_dos_open(im))
            return TRUE;
        goto fallback;
    case HOST_ti99:
        return ti99_open(im);
    case HOST_uknc:
        im->img.gap_2 = 24;
        im->img.gap_4a = 27;
        im->img.post_crc_syncs = 1;
        return _img_open(im, uknc_type);
    default:
        type = img_type;
        break;
    }

    /* Try specified host-specific geometries. */
    if (_img_open(im, type))
        return TRUE;

fallback:
    /* Fall back to default list. */
    memset(&im->img, 0, sizeof(im->img));
    return _img_open(im, img_type);
}

static bool_t d81_open(struct image *im)
{
    im->img.layout = LAYOUT_interleaved_swap_sides;
    return _img_open(im, d81_type);
}

static bool_t st_open(struct image *im)
{
    const struct img_type *in;
    struct img_type *out, *st_type;

    st_type = out = im->bufs.read_data.p;

    for (in = img_type; in->nr_secs != 0; in++) {
        if (in->cyls != _C(80))
            continue;
        memcpy(out, in, sizeof(*out));
        out->has_iam = FALSE;
        if (out->nr_secs == 9) {
            /* TOS formats 720kB disks with skew. */
            out->skew = 2;
        }
        out++;
    }

    memset(out, 0, sizeof(*out));

    return _img_open(im, st_type);
}

static bool_t mbd_open(struct image *im)
{
    return _img_open(im, mbd_type);
}

static bool_t mgt_open(struct image *im)
{
    return _img_open(im, img_type);
}

static bool_t pc98fdi_open(struct image *im)
{
    struct {
        uint32_t zero;
        uint32_t density;
        uint32_t header_size;
        uint32_t image_body_size;
        uint32_t sector_size_bytes;
        uint32_t nr_secs;
        uint32_t nr_sides;
        uint32_t cyls;
    } header;
    F_read(&im->fp, &header, sizeof(header), NULL);
    if (le32toh(header.density) == 0x30) {
        im->img.rpm = 300;
        im->img.gap_3 = 84;
    } else {
        im->img.rpm = 360;
        im->img.gap_3 = 116;
    }
    if (le32toh(header.sector_size_bytes) == 512) {
        im->img.sec_no = 2;
    } else {
        im->img.sec_no = 3;
    }
    im->nr_cyls = le32toh(header.cyls);
    im->nr_sides = le32toh(header.nr_sides);
    im->img.nr_sectors = le32toh(header.nr_secs);
    im->img.interleave = 1;
    im->img.sec_base[0] = im->img.sec_base[1] = 1;
    im->img.skew = 0;
    im->img.has_iam = TRUE;
    /* Skip 4096-byte header. */
    im->img.base_off = le32toh(header.header_size);
    return mfm_open(im);
}

static bool_t pc98hdm_open(struct image *im) {
    im->img.rpm = 360;
    im->img.gap_3 = 116;
    im->img.sec_no = 3;

    im->nr_cyls = 77;
    im->nr_sides = 2;
    im->img.nr_sectors = 8;
    im->img.interleave = 1;
    im->img.sec_base[0] = im->img.sec_base[1] = 1;
    im->img.skew = 0;
    im->img.has_iam = TRUE;
    im->img.base_off = 0;
    return mfm_open(im);
}

struct bpb {
    uint16_t sig;
    uint16_t bytes_per_sec;
    uint16_t sec_per_track;
    uint16_t num_heads;
    uint16_t tot_sec;
};

static void bpb_read(struct image *im, struct bpb *bpb)
{
    uint16_t x;

    F_lseek(&im->fp, 510); /* BS_55AA */
    F_read(&im->fp, &x, 2, NULL);
    bpb->sig = le16toh(x);

    F_lseek(&im->fp, 11); /* BPB_BytsPerSec */
    F_read(&im->fp, &x, 2, NULL);
    bpb->bytes_per_sec = le16toh(x);

    F_lseek(&im->fp, 24); /* BPB_SecPerTrk */
    F_read(&im->fp, &x, 2, NULL);
    bpb->sec_per_track = le16toh(x);

    F_lseek(&im->fp, 26); /* BPB_NumHeads */
    F_read(&im->fp, &x, 2, NULL);
    bpb->num_heads = le16toh(x);

    F_lseek(&im->fp, 19); /* BPB_TotSec */
    F_read(&im->fp, &x, 2, NULL);
    bpb->tot_sec = le16toh(x);
}

static bool_t msx_open(struct image *im)
{
    struct bpb bpb;

    /* Try to disambiguate overloaded image sizes via the boot sector. */
    switch (im_size(im)) {
    case 320*1024: /* 80/1/8 or 40/2/8? */
    case 360*1024: /* 80/1/9 or 40/2/9? */
        bpb_read(im, &bpb);
        /* BS_55AA (bpb.sig) is not valid in MSXDOS so don't check it. */
        if ((bpb.bytes_per_sec == 512)
            && ((bpb.num_heads == 1) || (bpb.num_heads == 2))
            && (bpb.tot_sec == (im_size(im) / bpb.bytes_per_sec))
            && ((bpb.sec_per_track == 8) || (bpb.sec_per_track == 9))) {
            im->img.sec_no = 2;
            im->img.nr_sectors = bpb.sec_per_track;
            im->nr_sides = bpb.num_heads;
            im->nr_cyls = (im->nr_sides == 1) ? 80 : 40;
            im->img.interleave = 1;
            im->img.sec_base[0] = im->img.sec_base[1] = 1;
            im->img.skew = 0;
            im->img.has_iam = TRUE;
            if (mfm_open(im))
                return TRUE;
        }
        break;
    }

    /* Use the MSX-specific list. */
    memset(&im->img, 0, sizeof(im->img));
    if (_img_open(im, msx_type))
        return TRUE;

    /* Caller falls back to the generic list. */
    return FALSE;
}

static bool_t pc_dos_open(struct image *im)
{
    struct bpb bpb;

    bpb_read(im, &bpb);

    if (bpb.sig != 0xaa55)
        goto fail;

    for (im->img.sec_no = 0; im->img.sec_no <= 6; im->img.sec_no++)
        if (sec_sz(im) == bpb.bytes_per_sec)
            break;
    if (im->img.sec_no > 6) /* >8kB? */
        goto fail;

    if ((bpb.sec_per_track == 0)
        || (bpb.sec_per_track > ARRAY_SIZE(im->img.sec_map)))
        goto fail;
    im->img.nr_sectors = bpb.sec_per_track;

    if ((bpb.num_heads != 1) && (bpb.num_heads != 2))
        goto fail;
    im->nr_sides = bpb.num_heads;

    im->nr_cyls = (bpb.tot_sec + im->img.nr_sectors*im->nr_sides - 1)
        / (im->img.nr_sectors * im->nr_sides);
    if (im->nr_cyls == 0)
        goto fail;

    im->img.interleave = 1;
    im->img.sec_base[0] = im->img.sec_base[1] = 1;
    im->img.skew = 0;
    im->img.has_iam = TRUE;
    return mfm_open(im);

fail:
    return FALSE;
}

static bool_t trd_open(struct image *im)
{
    uint8_t geometry;

    /* Interrogate TR-DOS geometry identifier. */
    F_lseek(&im->fp, 0x8e3);
    F_read(&im->fp, &geometry, 1, NULL);
    switch (geometry) {
    case 0x16:
        im->nr_cyls = 80;
        im->nr_sides = 2;
        break;
    case 0x17:
        im->nr_cyls = 40;
        im->nr_sides = 2;
        break;
    case 0x18:
        im->nr_cyls = 80;
        im->nr_sides = 1;
        break;
    case 0x19:
        im->nr_cyls = 40;
        im->nr_sides = 1;
        break;
    default:
        /* Guess geometry */
        if (im_size(im) <= 40*16*256) {
            im->nr_cyls = 40;
            im->nr_sides = 1;
        } else if (im_size(im) < 40*2*16*256) {
            im->nr_cyls = 40;
            im->nr_sides = 1;
        } else {
            im->nr_cyls = 80;
            im->nr_sides = 2;
        }
    }

    im->img.sec_no = 1; /* 256-byte */
    im->img.interleave = 1;
    im->img.skew = 0;
    im->img.sec_base[0] = im->img.sec_base[1] = 1;
    im->img.nr_sectors = 16;
    im->img.gap_3 = 57;
    im->img.has_iam = TRUE;

    return mfm_open(im);
}

static bool_t opd_open(struct image *im)
{
    switch (im_size(im)) {
    case 184320:
        im->nr_cyls = 40;
        im->nr_sides = 1;
        break;
    case 737280:
        im->nr_cyls = 80;
        im->nr_sides = 2;
        break;
    default:
        return FALSE;
    }

    im->img.sec_no = 1; /* 256-byte */
    im->img.interleave = 13;
    im->img.skew = 13;
    im->img.skew_cyls_only = TRUE;
    im->img.sec_base[0] = im->img.sec_base[1] = 0;
    im->img.nr_sectors = 18;
    im->img.gap_3 = 12;
    im->img.has_iam = TRUE;

    return mfm_open(im);
}

static bool_t dfs_open(struct image *im)
{
    im->nr_cyls = 80;
    im->img.interleave = 1;
    im->img.skew = 3;
    im->img.skew_cyls_only = TRUE;
    im->img.sec_no = 1; /* 256-byte */
    im->img.sec_base[0] = im->img.sec_base[1] = 0;
    im->img.nr_sectors = 10;
    im->img.gap_3 = 21;

    return fm_open(im);
}

static bool_t ssd_open(struct image *im)
{
    im->nr_sides = 1;
    return dfs_open(im);
}

static bool_t dsd_open(struct image *im)
{
    im->nr_sides = 2;
    return dfs_open(im);
}

static bool_t sdu_open(struct image *im)
{
    struct {
        uint8_t app[21], ver[5];
        uint16_t flags;
        uint16_t type;
        struct { uint16_t c, h, s; } max, used;
        uint16_t sec_size, trk_size;
    } header;

    /* Read basic (cyls, heads, spt) geometry from the image header. */
    F_read(&im->fp, &header, sizeof(header), NULL);
    im->nr_cyls = le16toh(header.max.c);
    im->nr_sides = le16toh(header.max.h);
    im->img.nr_sectors = le16toh(header.max.s);

    /* Check the geometry. Accept 180k/360k/720k/1.44M/2.88M PC sizes. */
    if (((im->nr_cyls != 40) && (im->nr_cyls != 80))
        || ((im->nr_sides != 1) && (im->nr_sides != 2))
        || ((im->img.nr_sectors != 9)
            && (im->img.nr_sectors != 18)
            && (im->img.nr_sectors != 36)))
        return FALSE;

    /* Fill in the rest of the geometry. */
    im->img.sec_no = 2; /* 512-byte sectors */
    im->img.interleave = 1; /* no interleave */
    im->img.sec_base[0] = im->img.sec_base[1] = 1; /* standard numbering */
    im->img.gap_3 = 84; /* standard gap3 */
    im->img.has_iam = TRUE;

    /* Skip 46-byte SABDU header. */
    im->img.base_off = 46;

    return mfm_open(im);
}

static bool_t ti99_open(struct image *im)
{
    struct vib {
        uint8_t name[10];
        uint16_t tot_secs;
        uint8_t secs_per_track;
        char id[3];
        uint8_t protection;
        uint8_t tracks_per_side;
        uint8_t sides;
        uint8_t density;
    } vib;
    bool_t have_vib;
    unsigned int fsize = im_size(im);

    /* Must be a multiple of 256 sectors. */
    if ((fsize % 256) != 0)
        return FALSE;
    fsize /= 256;

    /* Check for 3-sector footer containing a bad sector map. We ignore it. */
    if ((fsize % 10) == 3)
        fsize -= 3;

    /* Main image must be non-zero size. */
    if (fsize == 0)
        return FALSE;

    /* Check for Volume Information Block in sector 0. */
    F_read(&im->fp, &vib, sizeof(vib), NULL);
    have_vib = !strncmp(vib.id, "DSK", 3);

    im->img.has_iam = FALSE;
    im->img.interleave = 4;
    im->img.skew = 3;
    im->img.skew_cyls_only = TRUE;
    im->img.sec_no = 1;
    im->img.sec_base[0] = im->img.sec_base[1] = 0;
    im->img.layout = LAYOUT_sequential_reverse_side1;

    if ((fsize % (40*9)) == 0) {

        /* 9/18/36 sectors-per-track formats. */
        switch (fsize / (40*9)) {
        case 1: /* SSSD */
            im->nr_cyls = 40;
            im->nr_sides = 1;
            im->img.nr_sectors = 9;
            im->img.gap_3 = 44;
            return fm_open(im);
        case 2: /* DSSD (or SSDD) */
            if (have_vib && (vib.sides == 1)) {
                /* Disambiguated: This is SSDD. */
                im->nr_cyls = 40;
                im->nr_sides = 1;
                im->img.nr_sectors = 18;
                im->img.interleave = 5;
                im->img.gap_3 = 24;
                return mfm_open(im);
            }
            /* Assume DSSD. */
            im->nr_cyls = 40;
            im->nr_sides = 2;
            im->img.nr_sectors = 9;
            im->img.gap_3 = 44;
            return fm_open(im);
        case 4: /* DSDD (or DSSD80) */
            if (have_vib && (vib.tracks_per_side == 80)) {
                /* Disambiguated: This is DSSD80. */
                im->nr_cyls = 80;
                im->nr_sides = 2;
                im->img.nr_sectors = 9;
                im->img.gap_3 = 44;
                return fm_open(im);
            }
            /* Assume DSDD. */
            im->nr_cyls = 40;
            im->nr_sides = 2;
            im->img.nr_sectors = 18;
            im->img.interleave = 5;
            im->img.gap_3 = 24;
            return mfm_open(im);
        case 8: /* DSDD80 */
            im->nr_cyls = 80;
            im->nr_sides = 2;
            im->img.nr_sectors = 18;
            im->img.interleave = 5;
            im->img.gap_3 = 24;
            return mfm_open(im);
        case 16: /* DSHD80 */
            im->nr_cyls = 80;
            im->nr_sides = 2;
            im->img.nr_sectors = 36;
            im->img.interleave = 5;
            im->img.gap_3 = 24;
            return mfm_open(im);
        }

    } else if ((fsize % (40*16)) == 0) {

        /* SSDD/DSDD, 16 sectors */
        im->nr_sides = fsize / (40*16);
        if (im->nr_sides <= 2) {
            im->nr_cyls = 40;
            im->img.nr_sectors = 16;
            im->img.interleave = 5;
            im->img.gap_3 = 44;
            return mfm_open(im);
        }

    }

    return FALSE;
}

static bool_t jvc_open(struct image *im)
{
    struct jvc {
        uint8_t spt, sides, ssize_code, sec_id, attr;
    } jvc = {
        18, 1, 1, 1, 0
    };
    unsigned int bps, bpc;

    im->img.base_off = f_size(&im->fp) & 255;

    /* Check the image header. */
    F_read(&im->fp, &jvc,
           min_t(unsigned, im->img.base_off, sizeof(jvc)), NULL);
    if (jvc.attr || ((jvc.sides != 1) && (jvc.sides != 2)) || (jvc.spt == 0))
        return FALSE;

    im->nr_sides = jvc.sides;
    im->img.sec_no = jvc.ssize_code & 3;
    im->img.interleave = 3; /* RSDOS likes a 3:1 interleave (ref. xroar) */
    im->img.sec_base[0] = im->img.sec_base[1] = jvc.sec_id;
    im->img.nr_sectors = jvc.spt;

    /* Calculate number of cylinders. */
    bps = 128 << im->img.sec_no;
    bpc = bps * im->img.nr_sectors * im->nr_sides;
    im->nr_cyls = im_size(im) / bpc;
    if ((im->nr_cyls >= 88) && (im->nr_sides == 1)) {
        im->nr_sides++;
        im->nr_cyls /= 2;
        bpc *= 2;
    }
    if ((im_size(im) % bpc) >= bps)
        im->nr_cyls++;

    im->img.gap_3 = 20;
    im->img.gap_4a = 54;
    im->img.has_iam = TRUE;

    return mfm_open(im);
}

static bool_t vdk_open(struct image *im)
{
    struct vdk {
        char id[2];
        uint16_t hlen;
        uint8_t misc[4];
        uint8_t cyls, heads;
        uint8_t flags, compression;
    } vdk;

    /* Check the image header. */
    F_read(&im->fp, &vdk, sizeof(vdk), NULL);
    if (strncmp(vdk.id, "dk", 2) || le16toh(vdk.hlen < 12))
        return FALSE;

    /* Read (cyls, heads) geometry from the image header. */
    im->nr_cyls = vdk.cyls;
    im->nr_sides = vdk.heads;

    /* Check the geometry. */
    if ((im->nr_sides != 1) && (im->nr_sides != 2))
        return FALSE;

    /* Fill in the rest of the geometry. */
    im->img.sec_no = 1; /* 256-byte sectors */
    im->img.interleave = 2; /* DDOS likes a 2:1 interleave (ref. xroar) */
    im->img.sec_base[0] = im->img.sec_base[1] = 1;
    im->img.nr_sectors = 18;
    im->img.gap_3 = 20;
    im->img.gap_4a = 54;
    im->img.has_iam = TRUE;

    im->img.base_off = le16toh(vdk.hlen);

    return mfm_open(im);
}

const struct image_handler img_image_handler = {
    .open = img_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler d81_image_handler = {
    .open = d81_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler st_image_handler = {
    .open = st_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler adfs_image_handler = {
    .open = adfs_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler mbd_image_handler = {
    .open = mbd_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler mgt_image_handler = {
    .open = mgt_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler pc98fdi_image_handler = {
    .open = pc98fdi_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler pc98hdm_image_handler = {
    .open = pc98hdm_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler trd_image_handler = {
    .open = trd_open,
    .extend = img_extend,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler opd_image_handler = {
    .open = opd_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler ssd_image_handler = {
    .open = ssd_open,
    .extend = img_extend,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler dsd_image_handler = {
    .open = dsd_open,
    .extend = img_extend,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler sdu_image_handler = {
    .open = sdu_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler jvc_image_handler = {
    .open = jvc_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler vdk_image_handler = {
    .open = vdk_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};

const struct image_handler ti99_image_handler = {
    .open = ti99_open,
    .setup_track = img_setup_track,
    .read_track = img_read_track,
    .rdata_flux = bc_rdata_flux,
    .write_track = img_write_track,
};


/*
 * Generic Handlers
 */

static void img_extend(struct image *im)
{
    unsigned int sz = (im->img.nr_sectors * sec_sz(im)
                       * im->nr_sides * im->nr_cyls) + im->img.base_off;
    if (f_size(&im->fp) >= sz)
        return;
    F_lseek(&im->fp, sz);
    F_sync(&im->fp);
    if (f_tell(&im->fp) != sz)
        F_die(FR_DISK_FULL);
}

static void img_seek_track(
    struct image *im, uint16_t track, unsigned int cyl, unsigned int side)
{
    uint32_t trk_len;
    unsigned int base, i, pos, trk = cyl * im->nr_sides + side;

    im->cur_track = track;

    /* Create logical sector map in rotational order. */
    memset(im->img.sec_map, 0xff, im->img.nr_sectors);
    pos = ((im->img.skew_cyls_only ? cyl : trk) * im->img.skew)
        % im->img.nr_sectors;
    base = sec_base(im);
    for (i = 0; i < im->img.nr_sectors; i++) {
        while (im->img.sec_map[pos] != 0xff)
            pos = (pos + 1) % im->img.nr_sectors;
        im->img.sec_map[pos] = i + base;
        pos = (pos + im->img.interleave) % im->img.nr_sectors;
    }

    trk_len = im->img.nr_sectors * sec_sz(im);
    switch (im->img.layout) {
    case LAYOUT_sequential_reverse_side1:
        im->img.trk_off = (side ? 2*im->nr_cyls - cyl - 1 : cyl) * trk_len;
        break;
    case LAYOUT_interleaved_swap_sides:
        trk ^= im->nr_sides - 1;
        /* fall through */
    default:
        im->img.trk_off = trk * trk_len;
        break;
    }
    im->img.trk_off += im->img.base_off;
}

static uint32_t calc_start_pos(struct image *im)
{
    uint32_t decode_off;

    im->img.crc = 0xffff;
    im->img.trk_sec = im->img.rd_sec_pos = im->img.decode_data_pos = 0;
    decode_off = im->cur_bc / 16;
    if (decode_off < im->img.idx_sz) {
        /* Post-index track gap */
        im->img.decode_pos = 0;
    } else {
        unsigned int ess = enc_sec_sz(im);
        decode_off -= im->img.idx_sz;
        im->img.decode_pos = decode_off / ess;
        if (im->img.decode_pos < im->img.nr_sectors) {
            /* IDAM */
            im->img.trk_sec = im->img.decode_pos;
            im->img.decode_pos = im->img.decode_pos * 4 + 1;
            decode_off -= im->img.trk_sec * ess;
            if (decode_off >= im->img.idam_sz) {
                /* DAM */
                decode_off -= im->img.idam_sz;
                im->img.decode_pos++;
                if (decode_off >= im->img.dam_sz_pre) {
                    /* Data or Post Data */
                    decode_off -= im->img.dam_sz_pre;
                    im->img.decode_pos++;
                    if (decode_off < sec_sz(im)) {
                        /* Data */
                        im->img.rd_sec_pos = decode_off / 1024;
                        im->img.decode_data_pos = im->img.rd_sec_pos;
                        decode_off %= 1024;
                    } else {
                        /* Post Data */
                        decode_off -= sec_sz(im);
                        im->img.decode_pos++;
                        /* Start fetch at next sector. */
                        im->img.trk_sec = (im->img.trk_sec + 1)
                            % im->img.nr_sectors;
                    }
                }
            }
        } else {
            /* Pre-index track gap */
            decode_off -= im->img.nr_sectors * ess;
            im->img.decode_pos = im->img.nr_sectors * 4 + 1;
            im->img.decode_data_pos = decode_off / 1024;
            decode_off %= 1024;
        }
    }

    return decode_off;
}

static void img_setup_track(
    struct image *im, uint16_t track, uint32_t *start_pos)
{
    struct image_buf *rd = &im->bufs.read_data;
    struct image_buf *bc = &im->bufs.read_bc;
    uint32_t decode_off, sys_ticks = start_pos ? *start_pos : 0;
    uint8_t cyl = track/2, side = track&1;

    /* TODO: Fake out unformatted tracks. */
    cyl = min_t(uint8_t, cyl, im->nr_cyls-1);
    side = min_t(uint8_t, side, im->nr_sides-1);
    track = cyl*2 + side;

    if (track != im->cur_track)
        img_seek_track(im, track, cyl, side);

    im->img.write_sector = -1;

    im->cur_bc = (sys_ticks * 16) / im->ticks_per_cell;
    im->cur_bc &= ~15;
    if (im->cur_bc >= im->tracklen_bc)
        im->cur_bc = 0;
    im->cur_ticks = im->cur_bc * im->ticks_per_cell;
    im->ticks_since_flux = 0;

    decode_off = calc_start_pos(im);

    rd->prod = rd->cons = 0;
    bc->prod = bc->cons = 0;

    if (start_pos) {
        image_read_track(im);
        bc->cons = decode_off * 16;
        *start_pos = sys_ticks;
    }
}

static bool_t img_read_track(struct image *im)
{
    return (im->sync == SYNC_fm) ? fm_read_track(im) : mfm_read_track(im);
}

static bool_t img_write_track(struct image *im)
{
    return (im->sync == SYNC_fm) ? fm_write_track(im) : mfm_write_track(im);
}

static void img_dump_info(struct image *im)
{
    printk("%s %u-%u-%u:\n", (im->sync == SYNC_fm) ? "FM" : "MFM",
           im->nr_cyls, im->nr_sides, im->img.nr_sectors);
    printk(" rpm: %u, tracklen: %u, datarate: %u\n",
           im->img.rpm, im->tracklen_bc, im->img.data_rate);
    printk(" gap2: %u, gap3: %u, gap4a: %u, gap4: %u\n",
           im->img.gap_2, im->img.gap_3, im->img.gap_4a, im->img.gap_4);
    printk(" ticks_per_cell: %u, write_bc_ticks: %u has_iam: %u\n",
           im->ticks_per_cell, im->write_bc_ticks, im->img.has_iam);
}

static void img_fetch_data(struct image *im)
{
    struct image_buf *rd = &im->bufs.read_data;
    uint8_t sec, *buf = rd->p;
    uint16_t off, len;

    if (rd->prod != rd->cons)
        return;

    sec = im->img.sec_map[im->img.trk_sec] - sec_base(im);
    off = sec * sec_sz(im) + im->img.rd_sec_pos * 1024;
    len = sec_sz(im) - im->img.rd_sec_pos * 1024;

    if (len > 1024) {
        len = 1024;
        im->img.rd_sec_pos++;
    } else {
        im->img.rd_sec_pos = 0;
        if (++im->img.trk_sec >= im->img.nr_sectors)
            im->img.trk_sec = 0;
    }

    F_lseek(&im->fp, im->img.trk_off + off);
    F_read(&im->fp, buf, len, NULL);

    rd->prod++;
}


/*
 * MFM-Specific Handlers
 */

#define GAP_1    50 /* Post-IAM */
#define GAP_2    22 /* Post-IDAM */
#define GAP_4A   80 /* Post-Index */
#define GAP_SYNC 12

/* Shrink the IDAM pre-sync gap if sectors are close together. */
#define idam_gap_sync(im) min_t(uint8_t, (im)->img.gap_3, GAP_SYNC)

static bool_t mfm_open(struct image *im)
{
    const uint8_t GAP_3[] = { 32, 54, 84, 116, 255, 255, 255, 255 };
    uint32_t tracklen;
    unsigned int i;

    if ((im->nr_sides < 1) || (im->nr_sides > 2)
        || (im->nr_cyls < 1) || (im->nr_cyls > 254)
        || (im->img.nr_sectors < 1)
        || (im->img.nr_sectors > ARRAY_SIZE(im->img.sec_map)))
        return FALSE;

    im->img.rpm = im->img.rpm ?: 300;
    im->img.gap_2 = im->img.gap_2 ?: GAP_2;
    im->img.gap_3 = im->img.gap_3 ?: GAP_3[im->img.sec_no];
    im->img.gap_4a = im->img.gap_4a ?: GAP_4A;

    im->stk_per_rev = (stk_ms(200) * 300) / im->img.rpm;

    im->img.idx_sz = im->img.gap_4a;
    if (im->img.has_iam)
        im->img.idx_sz += GAP_SYNC + 4 + GAP_1;
    im->img.idam_sz = idam_gap_sync(im) + 8 + 2 + im->img.gap_2;
    im->img.dam_sz_pre = GAP_SYNC + 4;
    im->img.dam_sz_post = 2 + im->img.gap_3;

    im->img.idam_sz += im->img.post_crc_syncs;
    im->img.dam_sz_post += im->img.post_crc_syncs;

    /* Work out minimum track length (with no pre-index track gap). */
    tracklen = enc_sec_sz(im) * im->img.nr_sectors;
    tracklen += im->img.idx_sz;
    tracklen *= 16;

    /* Infer the data rate and hence the standard track length. */
    for (i = 0; i < 3; i++) { /* SD=0, DD=1, HD=2, ED=3 */
        uint32_t maxlen = (((50000u * 300) / im->img.rpm) << i) + 5000;
        if (tracklen < maxlen)
            break;
    }
    im->img.data_rate = 250u << i; /* SD=250, DD=500, HD=1000, ED=2000 */
    im->tracklen_bc = (im->img.data_rate * 200 * 300) / im->img.rpm;

    /* Does the track data fit within standard track length? */
    if (im->tracklen_bc < tracklen) {
        if ((tracklen - im->img.gap_4a*16) <= im->tracklen_bc) {
            /* Eliminate the post-index gap 4a if that suffices. */
            tracklen -= im->img.gap_4a*16;
            im->img.idx_sz -= im->img.gap_4a;
            im->img.gap_4a = 0;
        } else {
            /* Extend the track length ("long track"). */
            im->tracklen_bc = tracklen + 100;
        }
    }

    /* Round the track length up to a multiple of 32 bitcells. */
    im->tracklen_bc = (im->tracklen_bc + 31) & ~31;

    im->ticks_per_cell = ((sysclk_stk(im->stk_per_rev) * 16u)
                          / im->tracklen_bc);
    im->img.gap_4 = (im->tracklen_bc - tracklen) / 16;

    im->write_bc_ticks = sysclk_ms(1) / im->img.data_rate;

    im->sync = SYNC_mfm;

    img_dump_info(im);

    return TRUE;
}

static bool_t mfm_read_track(struct image *im)
{
    struct image_buf *rd = &im->bufs.read_data;
    struct image_buf *bc = &im->bufs.read_bc;
    uint8_t *buf = rd->p;
    uint16_t *bc_b = bc->p;
    uint32_t bc_len, bc_mask, bc_space, bc_p, bc_c;
    uint16_t pr = 0, crc;
    unsigned int i;

    img_fetch_data(im);

    /* Generate some MFM if there is space in the raw-bitcell ring buffer. */
    bc_p = bc->prod / 16; /* MFM words */
    bc_c = bc->cons / 16; /* MFM words */
    bc_len = bc->len / 2; /* MFM words */
    bc_mask = bc_len - 1;
    bc_space = bc_len - (uint16_t)(bc_p - bc_c);

#define emit_raw(r) ({                                   \
    uint16_t _r = (r);                                   \
    bc_b[bc_p++ & bc_mask] = htobe16(_r & ~(pr << 15));  \
    pr = _r; })
#define emit_byte(b) emit_raw(mfmtab[(uint8_t)(b)])

    if (im->img.decode_pos == 0) {
        /* Post-index track gap */
        if (bc_space < im->img.idx_sz)
            return FALSE;
        for (i = 0; i < im->img.gap_4a; i++)
            emit_byte(0x4e);
        if (im->img.has_iam) {
            /* IAM */
            for (i = 0; i < GAP_SYNC; i++)
                emit_byte(0x00);
            for (i = 0; i < 3; i++)
                emit_raw(0x5224);
            emit_byte(0xfc);
            for (i = 0; i < GAP_1; i++)
                emit_byte(0x4e);
        }
    } else if (im->img.decode_pos == (im->img.nr_sectors * 4 + 1)) {
        /* Pre-index track gap */
        uint16_t sz = im->img.gap_4 - im->img.decode_data_pos * 1024;
        if (bc_space < min_t(unsigned int, sz, 1024))
            return FALSE;
        if (sz > 1024) {
            sz = 1024;
            im->img.decode_data_pos++;
            im->img.decode_pos--;
        } else {
            im->img.decode_data_pos = 0;
            im->img.decode_pos = (im->img.idx_sz != 0) ? -1 : 0;
        }
        for (i = 0; i < sz; i++)
            emit_byte(0x4e);
    } else {
        switch ((im->img.decode_pos - 1) & 3) {
        case 0: /* IDAM */ {
            uint8_t cyl = im->cur_track/2, hd = im->cur_track&1;
            uint8_t sec = im->img.sec_map[(im->img.decode_pos-1) >> 2];
            uint8_t idam[8] = { 0xa1, 0xa1, 0xa1, 0xfe, cyl, hd, sec,
                                im->img.sec_no };
            if (bc_space < im->img.idam_sz)
                return FALSE;
            for (i = 0; i < idam_gap_sync(im); i++)
                emit_byte(0x00);
            for (i = 0; i < 3; i++)
                emit_raw(0x4489);
            for (; i < 8; i++)
                emit_byte(idam[i]);
            crc = crc16_ccitt(idam, sizeof(idam), 0xffff);
            emit_byte(crc >> 8);
            emit_byte(crc);
            for (i = 0; i < im->img.post_crc_syncs; i++)
                emit_raw(0x4489);
            for (i = 0; i < im->img.gap_2; i++)
                emit_byte(0x4e);
            break;
        }
        case 1: /* DAM */ {
            uint8_t dam[4] = { 0xa1, 0xa1, 0xa1, 0xfb };
            if (bc_space < im->img.dam_sz_pre)
                return FALSE;
            for (i = 0; i < GAP_SYNC; i++)
                emit_byte(0x00);
            for (i = 0; i < 3; i++)
                emit_raw(0x4489);
            emit_byte(dam[3]);
            im->img.crc = crc16_ccitt(dam, sizeof(dam), 0xffff);
            break;
        }
        case 2: /* Data */ {
            uint16_t sec_sz = sec_sz(im);
            sec_sz -= im->img.decode_data_pos * 1024;
            if (bc_space < min_t(unsigned int, sec_sz, 1024))
                return FALSE;
            if (sec_sz > 1024) {
                sec_sz = 1024;
                im->img.decode_data_pos++;
                im->img.decode_pos--;
            } else {
                im->img.decode_data_pos = 0;
            }
            for (i = 0; i < sec_sz; i++)
                emit_byte(buf[i]);
            im->img.crc = crc16_ccitt(buf, sec_sz, im->img.crc);
            rd->cons++;
            break;
        }
        case 3: /* Post Data */ {
            if (bc_space < im->img.dam_sz_post)
                return FALSE;
            crc = im->img.crc;
            emit_byte(crc >> 8);
            emit_byte(crc);
            for (i = 0; i < im->img.post_crc_syncs; i++)
                emit_raw(0x4489);
            for (i = 0; i < im->img.gap_3; i++)
                emit_byte(0x4e);
            break;
        }
        }
    }

#undef emit_raw
#undef emit_byte

    im->img.decode_pos++;
    bc->prod = bc_p * 16;

    return TRUE;
}

static bool_t mfm_write_track(struct image *im)
{
    const uint8_t header[] = { 0xa1, 0xa1, 0xa1, 0xfb };

    bool_t flush;
    struct write *write = get_write(im, im->wr_cons);
    struct image_buf *wr = &im->bufs.write_bc;
    uint16_t *buf = wr->p;
    unsigned int bufmask = (wr->len / 2) - 1;
    uint8_t *wrbuf = im->bufs.write_data.p;
    uint32_t c = wr->cons / 16, p = wr->prod / 16;
    uint32_t base = write->start / im->ticks_per_cell; /* in data bytes */
    unsigned int i;
    time_t t;
    uint16_t crc;
    uint8_t x;

    /* If we are processing final data then use the end index, rounded up. */
    barrier();
    flush = (im->wr_cons != im->wr_bc);
    if (flush)
        p = (write->bc_end + 15) / 16;

    if (im->img.write_sector == -1) {
        /* Convert write offset to sector number (in rotational order). */
        im->img.write_sector =
            (base - im->img.idx_sz - im->img.idam_sz + enc_sec_sz(im)/2)
            / enc_sec_sz(im);
        if (im->img.write_sector >= im->img.nr_sectors) {
            printk("IMG Bad Sector Offset: %u -> %u\n",
                   base, im->img.write_sector);
            im->img.write_sector = -2;
        } else {
            /* Convert rotational order to logical order. */
            im->img.write_sector = im->img.sec_map[im->img.write_sector];
            im->img.write_sector -= sec_base(im);
        }
    }

    while ((int16_t)(p - c) >= (3 + sec_sz(im) + 2)) {

        /* Scan for sync words and IDAM. Because of the way we sync we expect
         * to see only 2*4489 and thus consume only 3 words for the header. */
        if (be16toh(buf[c++ & bufmask]) != 0x4489)
            continue;
        for (i = 0; i < 2; i++)
            if ((x = mfmtobin(buf[c++ & bufmask])) != 0xa1)
                break;

        switch (x) {

        case 0xfe: /* IDAM */
            for (i = 0; i < 3; i++)
                wrbuf[i] = 0xa1;
            wrbuf[i++] = x;
            for (; i < 10; i++)
                wrbuf[i] = mfmtobin(buf[c++ & bufmask]);
            crc = crc16_ccitt(wrbuf, i, 0xffff);
            if (crc != 0) {
                printk("IMG IDAM Bad CRC %04x, sector %u\n", crc, wrbuf[6]);
                break;
            }
            im->img.write_sector = wrbuf[6] - sec_base(im);
            if ((uint8_t)im->img.write_sector >= im->img.nr_sectors) {
                printk("IMG IDAM Bad Sector: %u\n", wrbuf[6]);
                im->img.write_sector = -2;
            }
            break;

        case 0xfb: /* DAM */
            for (i = 0; i < (sec_sz(im) + 2); i++)
                wrbuf[i] = mfmtobin(buf[c++ & bufmask]);

            crc = crc16_ccitt(wrbuf, sec_sz(im) + 2,
                              crc16_ccitt(header, 4, 0xffff));
            if (crc != 0) {
                printk("IMG Bad CRC %04x, sector %u[%u]\n",
                       crc, im->img.write_sector,
                       im->img.write_sector + im->img.sec_base);
                break;
            }

            if (im->img.write_sector < 0) {
                printk("IMG DAM for unknown sector (%d)\n",
                       im->img.write_sector);
                break;
            }

            /* All good: write out to mass storage. */
            printk("Write %u[%u]/%u... ", im->img.write_sector,
                   im->img.write_sector + im->img.sec_base,
                   im->img.nr_sectors);
            t = time_now();
            F_lseek(&im->fp,
                    im->img.trk_off + im->img.write_sector*sec_sz(im));
            F_write(&im->fp, wrbuf, sec_sz(im), NULL);
            printk("%u us\n", time_diff(t, time_now()) / TIME_MHZ);
            break;
        }
    }

    wr->cons = c * 16;

    return flush;
}


/*
 * FM-Specific Handlers
 */

/* 8271 mini-diskette values */
#define FM_GAP_2 11
#define FM_GAP_4A 16
#define FM_GAP_SYNC 6

static bool_t fm_open(struct image *im)
{
    const uint8_t FM_GAP_3[] = { 27, 42, 58, 138, 255, 255, 255, 255 };
    uint32_t tracklen;

    if ((im->nr_sides < 1) || (im->nr_sides > 2)
        || (im->nr_cyls < 1) || (im->nr_cyls > 254)
        || (im->img.nr_sectors < 1)
        || (im->img.nr_sectors > ARRAY_SIZE(im->img.sec_map)))
        return FALSE;

    im->img.rpm = im->img.rpm ?: 300;
    im->img.gap_2 = im->img.gap_2 ?: FM_GAP_2;
    im->img.gap_3 = im->img.gap_3 ?: FM_GAP_3[im->img.sec_no];
    im->img.gap_4a = im->img.gap_4a ?: FM_GAP_4A;

    im->stk_per_rev = (stk_ms(200) * 300) / im->img.rpm;

    im->img.idx_sz = im->img.gap_4a;
    im->img.idam_sz = FM_GAP_SYNC + 5 + 2 + im->img.gap_2;
    im->img.dam_sz_pre = FM_GAP_SYNC + 1;
    im->img.dam_sz_post = 2 + im->img.gap_3;

    /* Work out minimum track length (with no pre-index track gap). */
    tracklen = enc_sec_sz(im) * im->img.nr_sectors;
    tracklen += im->img.idx_sz;
    tracklen *= 16;

    /* Data rate is always SD. */
    im->img.data_rate = 250; /* SD */
    im->tracklen_bc = (im->img.data_rate * 200 * 300) / im->img.rpm;

    ASSERT(im->tracklen_bc > tracklen);

    /* Round the track length up to a multiple of 32 bitcells. */
    im->tracklen_bc = (im->tracklen_bc + 31) & ~31;

    im->ticks_per_cell = ((sysclk_stk(im->stk_per_rev) * 16u)
                          / im->tracklen_bc);
    im->img.gap_4 = (im->tracklen_bc - tracklen) / 16;

    im->write_bc_ticks = sysclk_ms(1) / im->img.data_rate;

    im->sync = SYNC_fm;

    img_dump_info(im);

    return TRUE;
}

uint16_t fm_sync(uint8_t dat, uint8_t clk)
{
    uint16_t _dat = mfmtab[dat] & 0x5555;
    uint16_t _clk = (mfmtab[clk] & 0x5555) << 1;
    return _clk | _dat;
}

static bool_t fm_read_track(struct image *im)
{
    struct image_buf *rd = &im->bufs.read_data;
    struct image_buf *bc = &im->bufs.read_bc;
    uint8_t *buf = rd->p;
    uint16_t crc, *bc_b = bc->p;
    uint32_t bc_len, bc_mask, bc_space, bc_p, bc_c;
    unsigned int i;

    img_fetch_data(im);

    /* Generate some FM if there is space in the raw-bitcell ring buffer. */
    bc_p = bc->prod / 16; /* FM words */
    bc_c = bc->cons / 16; /* FM words */
    bc_len = bc->len / 2; /* FM words */
    bc_mask = bc_len - 1;
    bc_space = bc_len - (uint16_t)(bc_p - bc_c);

#define emit_raw(r) ({                          \
    uint16_t _r = (r);                          \
    bc_b[bc_p++ & bc_mask] = htobe16(_r); })
#define emit_byte(b) emit_raw(mfmtab[(uint8_t)(b)] | 0xaaaa)

    if (im->img.decode_pos == 0) {
        /* Post-index track gap */
        if (bc_space < im->img.idx_sz)
            return FALSE;
        for (i = 0; i < im->img.gap_4a; i++)
            emit_byte(0xff);
        ASSERT(!im->img.has_iam);
    } else if (im->img.decode_pos == (im->img.nr_sectors * 4 + 1)) {
        /* Pre-index track gap */
        uint16_t sz = im->img.gap_4 - im->img.decode_data_pos * 1024;
        if (bc_space < min_t(unsigned int, sz, 1024))
            return FALSE;
        if (sz > 1024) {
            sz = 1024;
            im->img.decode_data_pos++;
            im->img.decode_pos--;
        } else {
            im->img.decode_data_pos = 0;
            im->img.decode_pos = (im->img.idx_sz != 0) ? -1 : 0;
        }
        for (i = 0; i < sz; i++)
            emit_byte(0xff);
    } else {
        switch ((im->img.decode_pos - 1) & 3) {
        case 0: /* IDAM */ {
            uint8_t cyl = im->cur_track/2, hd = im->cur_track&1;
            uint8_t sec = im->img.sec_map[(im->img.decode_pos-1) >> 2];
            uint8_t idam[5] = { 0xfe, cyl, hd, sec, im->img.sec_no };
            if (bc_space < im->img.idam_sz)
                return FALSE;
            for (i = 0; i < FM_GAP_SYNC; i++)
                emit_byte(0x00);
            emit_raw(fm_sync(idam[0], FM_SYNC_CLK));
            for (i = 1; i < 5; i++)
                emit_byte(idam[i]);
            crc = crc16_ccitt(idam, sizeof(idam), 0xffff);
            emit_byte(crc >> 8);
            emit_byte(crc);
            for (i = 0; i < im->img.gap_2; i++)
                emit_byte(0xff);
            break;
        }
        case 1: /* DAM */ {
            uint8_t dam[1] = { 0xfb };
            if (bc_space < im->img.dam_sz_pre)
                return FALSE;
            for (i = 0; i < FM_GAP_SYNC; i++)
                emit_byte(0x00);
            emit_raw(fm_sync(dam[0], FM_SYNC_CLK));
            im->img.crc = crc16_ccitt(dam, sizeof(dam), 0xffff);
            break;
        }
        case 2: /* Data */ {
            uint16_t sec_sz = sec_sz(im);
            sec_sz -= im->img.decode_data_pos * 1024;
            if (bc_space < min_t(unsigned int, sec_sz, 1024))
                return FALSE;
            if (sec_sz > 1024) {
                sec_sz = 1024;
                im->img.decode_data_pos++;
                im->img.decode_pos--;
            } else {
                im->img.decode_data_pos = 0;
            }
            for (i = 0; i < sec_sz; i++)
                emit_byte(buf[i]);
            im->img.crc = crc16_ccitt(buf, sec_sz, im->img.crc);
            rd->cons++;
            break;
        }
        case 3: /* Post Data */ {
            if (bc_space < im->img.dam_sz_post)
                return FALSE;
            crc = im->img.crc;
            emit_byte(crc >> 8);
            emit_byte(crc);
            for (i = 0; i < im->img.gap_3; i++)
                emit_byte(0xff);
            break;
        }
        }
    }

#undef emit_raw
#undef emit_byte

    im->img.decode_pos++;
    bc->prod = bc_p * 16;

    return TRUE;
}

static bool_t fm_write_track(struct image *im)
{
    bool_t flush;
    struct write *write = get_write(im, im->wr_cons);
    struct image_buf *wr = &im->bufs.write_bc;
    uint16_t *buf = wr->p;
    unsigned int bufmask = (wr->len / 2) - 1;
    uint8_t *wrbuf = im->bufs.write_data.p;
    uint32_t c = wr->cons / 16, p = wr->prod / 16;
    uint32_t base = write->start / im->ticks_per_cell; /* in data bytes */
    unsigned int i;
    time_t t;
    uint16_t crc, sync;
    uint8_t x;

    /* If we are processing final data then use the end index, rounded up. */
    barrier();
    flush = (im->wr_cons != im->wr_bc);
    if (flush)
        p = (write->bc_end + 15) / 16;

    if (im->img.write_sector == -1) {
        /* Convert write offset to sector number (in rotational order). */
        im->img.write_sector =
            (base - im->img.idx_sz - im->img.idam_sz + enc_sec_sz(im)/2)
            / enc_sec_sz(im);
        if (im->img.write_sector >= im->img.nr_sectors) {
            printk("IMG Bad Sector Offset: %u -> %u\n",
                   base, im->img.write_sector);
            im->img.write_sector = -2;
        } else {
            /* Convert rotational order to logical order. */
            im->img.write_sector = im->img.sec_map[im->img.write_sector];
            im->img.write_sector -= sec_base(im);
        }
    }

    while ((int16_t)(p - c) >= (2 + sec_sz(im) + 2)) {

        if (buf[c++ & bufmask] != 0xaaaa)
            continue;
        sync = buf[c & bufmask];
        if (mfmtobin(sync >> 1) != FM_SYNC_CLK)
            continue;
        x = mfmtobin(sync);
        c++;

        switch (x) {

        case 0xfe: /* IDAM */
            wrbuf[0] = x;
            for (i = 1; i < 7; i++)
                wrbuf[i] = mfmtobin(buf[c++ & bufmask]);
            crc = crc16_ccitt(wrbuf, i, 0xffff);
            if (crc != 0) {
                printk("IMG IDAM Bad CRC %04x, sector %u\n", crc, wrbuf[3]);
                break;
            }
            im->img.write_sector = wrbuf[3] - sec_base(im);
            if ((uint8_t)im->img.write_sector >= im->img.nr_sectors) {
                printk("IMG IDAM Bad Sector: %u\n", wrbuf[3]);
                im->img.write_sector = -2;
            }
            break;

        case 0xfb: /* DAM */
            for (i = 0; i < (sec_sz(im) + 2); i++)
                wrbuf[i] = mfmtobin(buf[c++ & bufmask]);

            crc = crc16_ccitt(wrbuf, sec_sz(im) + 2,
                              crc16_ccitt(&x, 1, 0xffff));
            if (crc != 0) {
                printk("IMG Bad CRC %04x, sector %u[%u]\n",
                       crc, im->img.write_sector,
                       im->img.write_sector + im->img.sec_base);
                break;
            }

            if (im->img.write_sector < 0) {
                printk("IMG DAM for unknown sector (%d)\n",
                       im->img.write_sector);
                break;
            }

            /* All good: write out to mass storage. */
            printk("Write %u[%u]/%u... ", im->img.write_sector,
                   im->img.write_sector + im->img.sec_base,
                   im->img.nr_sectors);
            t = time_now();
            F_lseek(&im->fp,
                    im->img.trk_off + im->img.write_sector*sec_sz(im));
            F_write(&im->fp, wrbuf, sec_sz(im), NULL);
            printk("%u us\n", time_diff(t, time_now()) / TIME_MHZ);
            break;
        }
    }

    wr->cons = c * 16;

    return flush;
}

/*
 * Local variables:
 * mode: C
 * c-file-style: "Linux"
 * c-basic-offset: 4
 * tab-width: 4
 * indent-tabs-mode: nil
 * End:
 */
