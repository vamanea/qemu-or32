/*
 * OpenCores framebuffer device
 *
 * Copyright (c) 2014 Valentin Manea
 * Based on work by Sebastian Macke for jor1k http://s-macke.github.io/jor1k/
 * Based on Arm PrimeCell PL110 Color LCD Controller by Paul Brook
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/sysbus.h"
#include "hw/display/framebuffer.h"
#include "ui/console.h"

/* VGA defines */
#define VGA_CTRL       0x000
#define VGA_STAT       0x004
#define VGA_HTIM       0x008
#define VGA_VTIM       0x00c
#define VGA_HVLEN      0x010
#define VGA_VBARA      0x014
#define VGA_PALETTE    0x800

#define VGA_CTRL_VEN   0x00000001 /* Video Enable */
#define VGA_CTRL_HIE   0x00000002 /* HSync Interrupt Enable */
#define VGA_CTRL_PC    0x00000800 /* 8-bit Pseudo Color Enable*/
#define VGA_CTRL_CD8   0x00000000 /* Color Depth 8 */
#define VGA_CTRL_CD16  0x00000200 /* Color Depth 16 */
#define VGA_CTRL_CD24  0x00000400 /* Color Depth 24 */
#define VGA_CTRL_CD32  0x00000600 /* Color Depth 32 */
#define VGA_CTRL_CD    0x00000E00 /* Color Depth Mask */
#define VGA_CTRL_VBL1  0x00000000 /* Burst Length 1 */
#define VGA_CTRL_VBL2  0x00000080 /* Burst Length 2 */
#define VGA_CTRL_VBL4  0x00000100 /* Burst Length 4 */
#define VGA_CTRL_VBL8  0x00000180 /* Burst Length 8 */

#define PALETTE_SIZE   256

#define TYPE_OCFB "ocfb"
#define OCFB(obj) OBJECT_CHECK(OCFBState, (obj), TYPE_OCFB)

#ifdef DEBUG
#define DPRINTF(fmt, ...)                                \
    do { printf("ocfb: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

typedef struct OCFBState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    QemuConsole *con;
    /* RAM fragment containing framebuffer */
    MemoryRegionSection mem_section;
    uint32_t cr;
    uint8_t* fb;
    uint32_t fb_size;
    uint32_t fb_phys;
    uint32_t cols;
    uint32_t rows;
    uint32_t bpp;
    uint32_t invalidate;
    qemu_irq irq;
} OCFBState;

static int vmstate_ocfb_post_load(void *opaque, int version_id);

static const VMStateDescription vmstate_ocfb = {
    .name = "ocfb",
    .version_id = 2,
    .minimum_version_id = 1,
    .post_load = vmstate_ocfb_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cr, OCFBState),
        VMSTATE_UINT32(fb_size, OCFBState),
        VMSTATE_UINT32(fb_phys, OCFBState),
        VMSTATE_UINT32(cols, OCFBState),
        VMSTATE_UINT32(rows, OCFBState),
        VMSTATE_UINT32(bpp, OCFBState),
        VMSTATE_UINT32(invalidate, OCFBState),
        VMSTATE_END_OF_LIST()
    }
};

static int ocfb_enabled(OCFBState *s)
{
  return s->cr & VGA_CTRL_VEN;
}

static void ocfb_update_display(void *opaque)
{
    OCFBState *s = (OCFBState *)opaque;

    if (!ocfb_enabled(s)) {
        return;
    }
    
    memory_region_sync_dirty_bitmap(s->mem_section.mr);
    
    int dirty = memory_region_get_dirty(s->mem_section.mr,
          s->mem_section.offset_within_region, s->fb_size,
          DIRTY_MEMORY_VGA);

    if(dirty || s->invalidate)
        dpy_gfx_update(s->con, 0, 0, s->cols, s->rows);
    s->invalidate = 0;
    memory_region_reset_dirty(s->mem_section.mr,
          s->mem_section.offset_within_region, s->fb_size,
          DIRTY_MEMORY_VGA);
}

static void ocfb_invalidate_display(void * opaque)
{
    OCFBState *s = (OCFBState *)opaque;
    s->invalidate = 1;
}

static uint8_t* ocfb_map_fb(void *opaque, hwaddr base, hwaddr src_len)
{
    OCFBState *s = (OCFBState *)opaque;
    SysBusDevice *sbd = SYS_BUS_DEVICE(s);

    memory_region_unref(s->mem_section.mr);
    s->mem_section = memory_region_find(sysbus_address_space(sbd), base, src_len);
    assert(s->mem_section.mr);
    assert(s->mem_section.offset_within_address_space == base);

    if (int128_get64(s->mem_section.size) != src_len ||
            !memory_region_is_ram(s->mem_section.mr)) {
        return NULL;
    }

    return cpu_physical_memory_map(base, &src_len, 0);

}

static uint64_t ocfb_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    DPRINTF("read at 0x%08X\n", (unsigned int)offset);
    return 0;
}


static void ocfb_enable_console(OCFBState *s)
{
    DisplaySurface *surface;
    static const bool byteswap = true;
    if(!s->rows || !s->cols || !s->bpp)
        return;
    if(!s->fb_phys)
        return;
    int stride = (s->cols * s->bpp) / 8;
    s->fb_size = stride * s->rows;

    s->fb = ocfb_map_fb(s, s->fb_phys, s->fb_size);
    if(!s->fb) {
        hw_error("ocfb: bad framebuffer\n");
    }
    DPRINTF("fb pointer %p\n", s->fb);

    /* console.c supported depth -> buffer can be used directly */
    surface = qemu_create_displaysurface_from(s->cols, s->rows, s->bpp,
            stride, s->fb, byteswap);
    dpy_gfx_replace_surface(s->con, surface);
}

static inline void ocfb_read_htim(OCFBState *s, uint32_t val)
{
    /* uint32_t hsync_len = ((val >> 24) & 0xF) + 1;
    uint32_t right_margin = ((val >> 16) & 0xF) + 1;*/
    uint32_t xres = (val & 0xFFFF) + 1;
    s->cols = xres;
    DPRINTF("VGA_HTIM param, xres = %u!\n", s->cols);
}

static inline void ocfb_read_vtim(OCFBState *s, uint32_t val)
{
    /*uint32_t vsync_len = ((val >> 24) & 0xF) + 1;
    uint32_t lower_margin = ((val >> 16) & 0xF) + 1;*/
    uint32_t yres = (val & 0xFFFF) + 1;
    s->rows = yres;
    DPRINTF("VGA_VTIM param, yres = %u!\n", s->rows);
}

static void ocfb_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    OCFBState *s = (OCFBState *)opaque;

    DPRINTF("write at 0x%08X 0x%08X\n", 
             (unsigned int)offset, (unsigned int)val);

    switch (offset) {
    case VGA_CTRL:
        s->cr = val;

        if ((s->cr & VGA_CTRL_CD) == VGA_CTRL_CD32)
            s->bpp = 32;
        else if ((s->cr & VGA_CTRL_CD) == VGA_CTRL_CD24)
            s->bpp = 32;
        else if ((s->cr & VGA_CTRL_CD) == VGA_CTRL_CD16)
            s->bpp = 16;
        else
            hw_error("Unsupported framebuffer color mode!\n");

        ocfb_invalidate_display(s);
        if (ocfb_enabled(s)) {
            DPRINTF("Enable FB!\n");
            ocfb_enable_console(s);
        }

        break;
    case VGA_STAT:
        DPRINTF("VGA_STAT param!\n");
        break;
    case VGA_HTIM:
        ocfb_read_htim(s, val);
        break;
    case VGA_VTIM:
        ocfb_read_vtim(s, val);
        break;
    case VGA_HVLEN:
        DPRINTF("VGA_HVLEN param!\n");
        break;
    case VGA_VBARA:
        DPRINTF("framebuffer@0x%08X!\n", (unsigned int)val);
        s->fb_phys = val;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset %x\n", __func__, (int)offset);
    }
}

static const MemoryRegionOps ocfb_ops = {
    .read = ocfb_read,
    .write = ocfb_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static int vmstate_ocfb_post_load(void *opaque, int version_id)
{
    OCFBState *s = opaque;
    /* Make sure we redraw, and at the right size */
    ocfb_invalidate_display(s);
    return 0;
}

static const GraphicHwOps ocfb_gfx_ops = {
    .invalidate  = ocfb_invalidate_display,
    .gfx_update  = ocfb_update_display,
};

static int ocfb_initfn(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    OCFBState *s = OCFB(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ocfb_ops, s, "ocfb", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->con = graphic_console_init(dev, 0, &ocfb_gfx_ops, s);

    return 0;
}

static void ocfb_init(Object *obj)
{
    OCFBState *s = OCFB(obj);

    s->fb = NULL;
    s->fb_phys = 0;
    s->cols = 0;
    s->rows = 0;
    s->bpp = 0;
}

static void ocfb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ocfb_initfn;
    set_bit(DEVICE_CATEGORY_DISPLAY, dc->categories);
    dc->vmsd = &vmstate_ocfb;
}

static const TypeInfo ocfb_info = {
    .name          = TYPE_OCFB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OCFBState),
    .instance_init = ocfb_init,
    .class_init    = ocfb_class_init,
};

static void ocfb_register_types(void)
{
    type_register_static(&ocfb_info);
}

type_init(ocfb_register_types)
