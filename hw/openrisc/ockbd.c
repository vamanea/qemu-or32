/*
 * OpenCores Keyboard device
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
#include "ui/console.h"

#define TYPE_OCKB "ockb"
#define OCKB(obj) OBJECT_CHECK(OCKBState, (obj), TYPE_OCKB)

#ifdef DEBUG
#define DPRINTF(fmt, ...)                                \
    do { printf("ockb: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

typedef struct OCKBState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    uint8_t data[16];
    uint32_t rptr, wptr, count;
} OCKBState;


static void ockb_keycode(void *opaque, int keycode)
{
    OCKBState *s = (OCKBState *) opaque;
    /* The keycodes the driver expects are exactly the
       same as we receive them */
    if (s->count < sizeof(s->data)) {
        s->data[s->wptr] = keycode;
        if (++s->wptr == sizeof(s->data))
            s->wptr = 0;
        s->count++;
    }
    qemu_irq_raise(s->irq);
}

static uint64_t ockb_read(void *opaque, hwaddr offset,
                                 unsigned size)
{
    OCKBState *s = (OCKBState *) opaque;
    int keycode;


    if (offset >= 0x4)
        return 0;

    DPRINTF("read offset %u\n", (uint32_t)offset);
    if (s->count == 0) {
        qemu_irq_lower(s->irq);
        return 0;
    }

    keycode = s->data[s->rptr];
    if (++s->rptr == sizeof(s->data))
        s->rptr = 0;
    s->count--;

    return keycode;
}

static void ockb_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    /* Don't actually expect any write but don't fail */
    DPRINTF("read offset %u\n", (uint32_t)offset);
}

static const MemoryRegionOps ockb_ops = {
    .read = ockb_read,
    .write = ockb_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int ockb_initfn(SysBusDevice *sbd)
{
    DeviceState *dev = DEVICE(sbd);
    OCKBState *s = OCKB(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &ockb_ops, s, "ockb", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    qemu_add_kbd_event_handler(ockb_keycode, s);

    return 0;
}


static const VMStateDescription vmstate_ockb_regs = {
    .name = "ockb",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, OCKBState, 16),
        VMSTATE_UINT32(rptr, OCKBState),
        VMSTATE_UINT32(wptr, OCKBState),
        VMSTATE_UINT32(count, OCKBState),
        VMSTATE_END_OF_LIST(),
    },
};

static void ockb_init(Object *obj)
{
    OCKBState *s = OCKB(obj);

    memset(s->data, 0, sizeof(s->data));
    s->rptr = 0;
    s->wptr = 0;
    s->count = 0;
}

static void ockb_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ockb_initfn;
    dc->desc = "OpenCores Keyboard controller";
    dc->vmsd = &vmstate_ockb_regs;
}

static const TypeInfo ockb_info = {
    .name          = TYPE_OCKB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OCKBState),
    .instance_init = ockb_init,
    .class_init    = ockb_class_init,
};

static void ockb_register_types(void)
{
    type_register_static(&ockb_info);
}

type_init(ockb_register_types)
