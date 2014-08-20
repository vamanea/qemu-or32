/*
 * OpenCores framebuffer device
 *
 * Copyright (c) 2014 Valentin Manea
 * Based on work by Sebastian Macke for jor1k http://s-macke.github.io/jor1k/
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

#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "ui/input.h"
#include "qemu/timer.h"

/*
 * Touchscreen controller register offsets
 */
#define LPC32XX_TSC_STAT            0x00
#define LPC32XX_TSC_SEL             0x04
#define LPC32XX_TSC_CON             0x08
#define LPC32XX_TSC_FIFO            0x0C
#define LPC32XX_TSC_DTR             0x10
#define LPC32XX_TSC_RTR             0x14
#define LPC32XX_TSC_UTR             0x18
#define LPC32XX_TSC_TTR             0x1C
#define LPC32XX_TSC_DXP             0x20
#define LPC32XX_TSC_MIN_X           0x24
#define LPC32XX_TSC_MAX_X           0x28
#define LPC32XX_TSC_MIN_Y           0x2C
#define LPC32XX_TSC_MAX_Y           0x30
#define LPC32XX_TSC_AUX_UTR         0x34
#define LPC32XX_TSC_AUX_MIN         0x38
#define LPC32XX_TSC_AUX_MAX         0x3C

#define LPC32XX_TSC_STAT_FIFO_OVRRN     (1 << 8)
#define LPC32XX_TSC_STAT_FIFO_EMPTY     (1 << 7)
#define LPC32XX_TSC_FIFO_TS_P_LEVEL     (1 << 31)

#define LPC32XX_TSC_ADCCON_POWER_UP     (1 << 2)
#define LPC32XX_TSC_ADCCON_AUTO_EN      (1 << 0)

#define LPC32XX_TSC_FIFO_TS_P_LEVEL            (1 << 31)

#define LPC32XX_TSC_ADCDAT_VALUE_MASK          0x000003FF
#define LPC32XX_TSC_FIFO_X_VAL(x)    (((LPC32XX_TSC_ADCDAT_VALUE_MASK - x) & \
                                      LPC32XX_TSC_ADCDAT_VALUE_MASK) << 16)
#define LPC32XX_TSC_FIFO_Y_VAL(y)    ((LPC32XX_TSC_ADCDAT_VALUE_MASK - y) & \
                                      LPC32XX_TSC_ADCDAT_VALUE_MASK)


#define LPC32XX_TSC_MIN_XY_VAL      0x0
#define LPC32XX_TSC_MAX_XY_VAL      0x3FF


#define TYPE_LPC32XX "lpc32xx"
#define LPC32XX(obj) OBJECT_CHECK(LPC32XXState, (obj), TYPE_LPC32XX)


#ifdef DEBUG
#define DPRINTF(fmt, ...)                                \
    do { printf("lpc32xx: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif


typedef struct LPC32XXState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t control;
    uint32_t status;
    bool pressed;
    uint32_t move_count;
    uint32_t fifo;
    int32_t fifo_size;
} LPC32XXState;

static const VMStateDescription vmstate_lpc32xx = {
    .name = "lpc32xx",
    .version_id = 2,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(control, LPC32XXState),
        VMSTATE_UINT32(status, LPC32XXState),
        VMSTATE_UINT32(move_count, LPC32XXState),
        VMSTATE_UINT32(fifo, LPC32XXState),
        VMSTATE_INT32(fifo_size, LPC32XXState),
        VMSTATE_END_OF_LIST()
    }
};


static int lpc32xx_enabled(LPC32XXState *s)
{
  return s->control & LPC32XX_TSC_ADCCON_AUTO_EN;
}


static uint64_t lpc32xx_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    LPC32XXState *s = (LPC32XXState *)opaque;
    DPRINTF("read at 0x%08X\n", (unsigned int)offset);
    switch(offset)
    {
    case LPC32XX_TSC_CON:
        return s->control;
    case LPC32XX_TSC_STAT:
        qemu_irq_lower(s->irq);
        return s->status;
    case LPC32XX_TSC_FIFO:
        if (s->fifo_size <= 0) {
            s->status |= LPC32XX_TSC_STAT_FIFO_EMPTY;
        }
        else
            s->fifo_size--;
        return s->fifo;
    }
    return 0;
}

static void lpc32xx_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    LPC32XXState *s = (LPC32XXState *)opaque;

    DPRINTF("write at 0x%08X 0x%08X\n", (uint32_t)offset, (uint32_t)val);
    switch (offset) {
    case LPC32XX_TSC_CON:
        s->control = val;
        break;
    break;
    case LPC32XX_TSC_SEL:
    case LPC32XX_TSC_MIN_X:
    case LPC32XX_TSC_MAX_X:
    case LPC32XX_TSC_MIN_Y:
    case LPC32XX_TSC_MAX_Y:
    case LPC32XX_TSC_AUX_UTR:
    case LPC32XX_TSC_AUX_MIN:
    case LPC32XX_TSC_AUX_MAX:
    case LPC32XX_TSC_RTR:
    case LPC32XX_TSC_DTR:
    case LPC32XX_TSC_TTR:
    case LPC32XX_TSC_DXP:
    case LPC32XX_TSC_UTR:
         break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "lpc32xx: Bad offset %x\n", (int)offset);
    }
}

static void lpc32xx_touchscreen_event(void *opaque,
                int x, int y, int z, int buttons_state)
{
    LPC32XXState *s = (LPC32XXState *)opaque;
    /* Current driver has LPC32XX_TSC_MAX_XY_VAL hardcoded */
    x = qemu_input_scale_axis(x, INPUT_EVENT_ABS_SIZE, LPC32XX_TSC_MAX_XY_VAL);
    y = qemu_input_scale_axis(y, INPUT_EVENT_ABS_SIZE, LPC32XX_TSC_MAX_XY_VAL);

    printf("event %d %d %d\n", x, y, buttons_state);
    if(!lpc32xx_enabled(s))
        return;

    if(!buttons_state) {
        /* Finger up */
        if(s->pressed) {
            s->status &= ~LPC32XX_TSC_STAT_FIFO_EMPTY;
            s->fifo_size = 0; // just a button up event
            s->fifo = LPC32XX_TSC_FIFO_TS_P_LEVEL;
            s->pressed = false;
            qemu_irq_raise(s->irq);
            return;
        }
        /* Just mouse move */
        else
            return;
    }

    /* Move */
    if(buttons_state && s->pressed) {
        s->move_count++;
        /* handle mouse move only every fourth time */
        if (s->move_count & 3)
            return;
    }

    s->status &= ~LPC32XX_TSC_STAT_FIFO_EMPTY;
    s->fifo_size = 4;
    s->fifo = LPC32XX_TSC_FIFO_X_VAL(x);
    s->fifo |= LPC32XX_TSC_FIFO_Y_VAL(y);
    s->pressed = true;
    qemu_irq_raise(s->irq);
}

static const MemoryRegionOps lpc32xx_ops = {
    .read = lpc32xx_read,
    .write = lpc32xx_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int lpc32xx_initfn(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    LPC32XXState *s = LPC32XX(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &lpc32xx_ops, 
          s, "lpc32xx", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->irq);
    qemu_add_mouse_event_handler(lpc32xx_touchscreen_event, s, 1,
                "QEMU LPC32XX-driven Touchscreen");

    return 0;
}

static void lpc32xx_init(Object *obj)
{
    LPC32XXState *s = LPC32XX(obj);
	s->control = 0x0;
    s->status = LPC32XX_TSC_STAT_FIFO_EMPTY;
    s->pressed = false;
    s->move_count = 0;
    s->fifo = 0;
    s->fifo_size = 0;
}

static void lpc32xx_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = lpc32xx_initfn;
    dc->vmsd = &vmstate_lpc32xx;
}

static const TypeInfo lpc32xx_info = {
    .name          = TYPE_LPC32XX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LPC32XXState),
    .instance_init = lpc32xx_init,
    .class_init    = lpc32xx_class_init,
};

static void lpc32xx_register_types(void)
{
    type_register_static(&lpc32xx_info);
}

type_init(lpc32xx_register_types)
