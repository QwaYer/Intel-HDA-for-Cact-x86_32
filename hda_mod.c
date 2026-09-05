/*
 * Intel HD Audio (HDA) controller driver for Cact kmod.
 * Lives under source/Intel-HDA-for-Cact (outside kernel tree).
 * Build: make  (KERN_ROOT defaults to ../CactKernel-x86_32)
 *
 * Load (root): modload /lib/hda_audio.cctk
 *   Manifest binds PCI class 04:03 (HD Audio).
 *
 * I/O model: CORB/RIRB for codec command dispatch, BDL for PCM DMA.
 * Single PCM output stream with devfs interface /dev/hda_audio.
 */

#include <stddef.h>
#include <stdint.h>
#include "hda.h"
#include "pci_enum.h"
#include "devfs.h"
#include "sync.h"
#include "klib.h"

extern void     printk(const char* fmt, ...);
extern void     printk_hex(uint32_t v);

extern void*    kmalloc(uint32_t size);
extern void*    kmalloc_aligned(uint32_t size, uint32_t align);
extern void     kfree(void* p);
extern void     kfree(void* p);
extern void*    memset(void* s, int c, uint32_t n);
extern void*    memcpy(void* dst, const void* src, uint32_t n);


extern uint32_t pci_read_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg);
extern void     pci_write_config_dword(uint8_t bus, uint8_t dev, uint8_t fn, uint8_t reg,
                            uint32_t val);
extern uint8_t  inb (uint16_t port);
extern void     outb(uint16_t port, uint8_t val);
extern void     vmm_map(uint32_t* pd, uint32_t va, uint32_t pa, int flags);
extern void     irq_spinlock_init   (irq_spinlock_t* lock);
extern void     irq_spinlock_acquire(irq_spinlock_t* lock);
extern void     irq_spinlock_release(irq_spinlock_t* lock);

struct msix_table_entry {
    uint32_t msg_addr_lo;
    uint32_t msg_addr_hi;
    uint32_t msg_data;
    uint32_t vector_ctrl;
} __attribute__((packed));

extern int      msix_alloc_vector(void);
extern void     msix_free_vector(int vec);
extern int      msix_register_handler(int vec, void (*handler)(void));
extern void     msix_unregister_handler(int vec);
extern int      pci_msix_support(pci_device_t *dev);
extern int      pci_msix_table_map(pci_device_t *dev,
                                   volatile struct msix_table_entry **table_out,
                                   uint32_t *table_size_out);
extern int      pci_msix_enable(pci_device_t *dev, int vec,
                                volatile struct msix_table_entry *table,
                                unsigned int entry_idx);
extern void     sema_init   (semaphore_t* s, int val);
extern void     down   (semaphore_t* s);
extern void     up     (semaphore_t* s);
extern devfs_entry_t* register_chrdev  (const char* name, uint32_t flags,
                                       devfs_driver_t* drv, void* drv_priv);
extern int            unregister_chrdev(const char* name);

#define PAGE_PRESENT 0x1
#define PAGE_RW      0x2
#define PAGE_PWT     0x8
#define PAGE_PCD     0x10


static uint32_t          hda_mmio_base;
static uint32_t          hda_mmio_size;
static hda_codec_info_t  hda_codecs[HDA_MAX_CODECS];
static hda_widget_info_t hda_widgets[64];
static int               hda_num_codecs;
static int               hda_num_widgets;

static int               hda_attached;
static int               devfs_was_registered;
static int               hda_msix_vector;
static volatile struct msix_table_entry *hda_msix_table;
static uint32_t          saved_pci_cmd_dw;
static uint8_t           hda_bus, hda_dev, hda_fn;

static uint32_t         *corb_buf;
static uint32_t         *rirb_buf;
static uint32_t          corb_wp;
static uint32_t          rirb_rp;

static uint8_t          *pcm_buffer;
static hda_bdl_entry_t  *pcm_bdl;
static int               pcm_stream_tag;
static int               pcm_sd_idx;
static int               pcm_output_node;
static int               pcm_codec_addr;

static irq_spinlock_t    hda_lock;
static int               output_configured;

static inline uint32_t* get_current_pd(void) {
    uint32_t val;
    __asm__ __volatile__("mov %%cr3, %0" : "=r"(val));
    return (uint32_t*)val;
}

static inline uint32_t hda_reg_read(uint32_t reg) {
    return *(volatile uint32_t*)(uintptr_t)(hda_mmio_base + reg);
}

static inline void hda_reg_write(uint32_t reg, uint32_t val) {
    *(volatile uint32_t*)(uintptr_t)(hda_mmio_base + reg) = val;
}

static inline uint16_t hda_read_word(uint32_t reg) {
    return *(volatile uint16_t*)(uintptr_t)(hda_mmio_base + reg);
}

static inline void hda_write_word(uint32_t reg, uint16_t val) {
    *(volatile uint16_t*)(uintptr_t)(hda_mmio_base + reg) = val;
}

static inline uint8_t hda_read_byte(uint32_t reg) {
    return *(volatile uint8_t*)(uintptr_t)(hda_mmio_base + reg);
}

static inline void hda_write_byte(uint32_t reg, uint8_t val) {
    *(volatile uint8_t*)(uintptr_t)(hda_mmio_base + reg) = val;
}

const uint8_t cact_pci_class    = 0x04;
const uint8_t cact_pci_subclass = 0x03;

static void hda_udelay(uint32_t us) {
    for (volatile uint32_t i = 0; i < us * 10; i++)
        __asm__ __volatile__("pause");
}

static void hda_corb_rirb_init(void) {
    corb_buf = (uint32_t*)kmalloc_aligned(
                   HDA_CORB_SIZE * HDA_CORB_ENTRY_SIZE, 128);
    rirb_buf = (uint32_t*)kmalloc_aligned(
                   HDA_RIRB_SIZE * HDA_RIRB_ENTRY_SIZE, 128);
    if (!corb_buf || !rirb_buf) {
        printk("3" "HDA: CORB/RIRB buffer alloc failed");
        return;
    }
    memset(corb_buf, 0, HDA_CORB_SIZE * HDA_CORB_ENTRY_SIZE);
    memset(rirb_buf, 0, HDA_RIRB_SIZE * HDA_RIRB_ENTRY_SIZE);

    hda_write_byte(HDA_CORBCTL, 0);
    hda_write_byte(HDA_RIRBCTL, 0);

    hda_write_word(HDA_CORBRP, 0x8000);
    for (int i = 0; i < 500; i++) {
        if (hda_read_word(HDA_CORBRP) & 0x8000) break;
        hda_udelay(10);
    }
    hda_write_word(HDA_CORBRP, 0x0000);
    for (int i = 0; i < 500; i++) {
        if (!(hda_read_word(HDA_CORBRP) & 0x8000)) break;
        hda_udelay(10);
    }

    hda_reg_write(HDA_CORBBASE, (uint32_t)(uintptr_t)corb_buf);
    hda_reg_write(HDA_CORBUBASE, 0);
    hda_write_byte(HDA_CORBWP, 0);
    hda_write_byte(HDA_CORBSIZE, 0x02);
    hda_write_byte(HDA_CORBCTL, HDA_CORBCTL_RUN);

    hda_reg_write(HDA_RIRBBASE, (uint32_t)(uintptr_t)rirb_buf);
    hda_reg_write(HDA_RIRBUBASE, 0);
    hda_write_byte(HDA_RIRBSIZE, 0x02);
    hda_write_byte(HDA_RIRBCNT, 1);
    hda_write_byte(HDA_RIRBCTL, HDA_RIRBCTL_RUN);

    corb_wp = 0;
    rirb_rp = 0;
}

static uint32_t hda_send_verb(int codec_addr, int node_id,
                              uint32_t verb_id, uint32_t payload) {
    uint32_t verb = ((codec_addr & 0x0F) << 28) | ((node_id & 0xFF) << 20) |
                    ((verb_id & 0xFFF) << 8) | (payload & 0xFF);

    if (corb_buf && rirb_buf) {
        uint8_t wp = hda_read_byte(HDA_CORBWP);
        uint8_t rp = hda_read_word(HDA_CORBRP) & 0xFF;
        for (int i = 0; i < 1000; i++) {
            if ((uint8_t)(wp + 1 - rp) <= 256) break;
            __asm__ __volatile__("pause; pause; pause; pause");
            rp = hda_read_word(HDA_CORBRP) & 0xFF;
        }
        if ((uint8_t)(wp + 1 - rp) <= 256) {
            wp = (wp + 1) & 0xFF;
            corb_buf[wp] = verb;
            __asm__ __volatile__("" ::: "memory");
            hda_write_byte(HDA_CORBWP, wp);
            __asm__ __volatile__("" ::: "memory");
            for (int i = 0; i < 50000; i++) {
                uint8_t rirbwp = hda_read_byte(HDA_RIRBWP);
                uint8_t avail = (uint8_t)(rirbwp - rirb_rp);
                if (avail) {
                    for (uint8_t j = 0; j < avail; j++) {
                        uint8_t idx = (rirb_rp + 1 + j) & 0xFF;
                        uint32_t resp_lo = rirb_buf[idx * 2];
                        uint32_t resp_hi = rirb_buf[idx * 2 + 1];
                        if (!(resp_hi & 0x10)) {
                            rirb_rp = (rirb_rp + j + 1) & 0xFF;
                            return resp_lo;
                        }
                    }
                    rirb_rp = rirbwp;
                }
                __asm__ __volatile__("pause");
            }
        }
    }

    uint32_t ic = hda_reg_read(HDA_IC);
    if (ic & HDA_ICB) {
        for (int i = 0; i < 10000; i++) {
            __asm__ __volatile__("pause");
            ic = hda_reg_read(HDA_IC);
            if (!(ic & HDA_ICB)) break;
        }
        if (hda_reg_read(HDA_IC) & HDA_ICB) return 0xFFFFFFFF;
    }

    hda_reg_write(HDA_IC, 0);
    __asm__ __volatile__("" ::: "memory");
    hda_reg_write(HDA_ICOI, verb);
    __asm__ __volatile__("" ::: "memory");
    hda_reg_write(HDA_IC, HDA_ICB);
    __asm__ __volatile__("" ::: "memory");

    uint32_t start = hda_reg_read(HDA_WALCLK);
    if (start == 0) {
        printk("[HDA] WALCLK is ZERO, IC timeout disabled\n");
    }
    for (int i = 0; i < 500000; i++) {
        __asm__ __volatile__("pause");
        ic = hda_reg_read(HDA_IC);
        if (!(ic & HDA_ICB)) {
            uint32_t resp = hda_reg_read(HDA_IRII);
            return resp;
        }
        uint32_t now = hda_reg_read(HDA_WALCLK);
        if (now && (now - start) > 12000000) {
            printk("[HDA] IC WALCLK timeout after "); printk_hex(i); printk(" iters\n");
            break;
        }
    }

    printk("[HDA] IC timeout: IC="); printk_hex(hda_reg_read(HDA_IC));
    printk(" verb="); printk_hex(verb); printk("\n");
    return 0xFFFFFFFF;
}

static uint32_t hda_get_param(int codec_addr, int node_id, uint32_t param_id) {
    return hda_send_verb(codec_addr, node_id, 0xF00, param_id);
}

static int hda_verb_set(int codec_addr, int node_id,
                        uint32_t verb_id, uint32_t payload) {
    uint32_t r = hda_send_verb(codec_addr, node_id, verb_id, payload);
    return (r == 0xFFFFFFFF) ? -1 : 0;
}

static void hda_enumerate_codecs(void) {
    hda_num_codecs = 0;

    for (int i = 0; i < HDA_MAX_CODECS; i++) {
        uint32_t vendor = hda_get_param(i, 0, HDA_PARAM_VENDOR_ID);
        if (vendor == 0xFFFFFFFF || vendor == 0)
            continue;

        hda_codecs[hda_num_codecs].active = 1;
        hda_codecs[hda_num_codecs].addr   = i;
        hda_codecs[hda_num_codecs].vendor_id = vendor >> 16;
        hda_codecs[hda_num_codecs].device_id = vendor & 0xFFFF;

        uint32_t rev = hda_get_param(i, 0, HDA_PARAM_REVISION);
        hda_codecs[hda_num_codecs].revision = rev & 0xFF;

        uint32_t root_sub = hda_get_param(i, 0, HDA_PARAM_SUB_NODE_COUNT);
        uint8_t root_start = (root_sub >> 16) & 0xFF;
        uint8_t root_count = root_sub & 0xFF;

        uint8_t afg = 0;
        for (uint8_t n = 0; n < root_count; n++) {
            uint8_t nid = root_start + n;
            uint32_t ft = hda_get_param(i, nid, HDA_PARAM_FUNCTION_GROUP);
            if ((ft & 0xFF) == 0x01) {
                afg = nid;
                break;
            }
        }
        hda_codecs[hda_num_codecs].afg_node = afg;

        uint32_t sub = hda_get_param(i, afg, HDA_PARAM_SUB_NODE_COUNT);
        hda_codecs[hda_num_codecs].start_widget = (sub >> 16) & 0xFF;
        hda_codecs[hda_num_codecs].num_widgets  = sub & 0xFF;

        char hex[16];
        snprintf(hex, sizeof(hex), "%d", (int)(i));
        printk("[HDA] codec ");
        printk(hex);
        printk(": vendor=");
        snprintf(hex, sizeof(hex), "0x%x", (unsigned)(hda_codecs[hda_num_codecs].vendor_id));
        printk(hex);
        printk(" device=");
        snprintf(hex, sizeof(hex), "0x%x", (unsigned)(hda_codecs[hda_num_codecs].device_id));
        printk(hex);
        printk("\n");

        hda_num_codecs++;
    }
}

static void hda_enumerate_widgets(int codec_idx) {
    hda_codec_info_t *codec = &hda_codecs[codec_idx];
    if (!codec->active) return;

    int end = codec->start_widget + codec->num_widgets;
    for (int nid = codec->start_widget; nid < end && hda_num_widgets < 64; nid++) {
        uint32_t caps = hda_get_param(codec->addr, nid, HDA_PARAM_AUDIO_WIDGET);
        if (caps == 0xFFFFFFFF) continue;

        hda_widget_info_t *w = &hda_widgets[hda_num_widgets++];
        w->active   = 1;
        w->node_id  = nid;
        w->type     = (caps >> 20) & 0x0F;
        w->caps     = caps & 0xFFFF;

        if (w->type == HDA_WIDGET_TYPE_DAC || w->type == HDA_WIDGET_TYPE_ADC ||
            w->type == HDA_WIDGET_TYPE_PIN) {
            uint32_t pcm = hda_get_param(codec->addr, nid, HDA_PARAM_PCM);
            w->pcm_rates   = pcm & 0x7FFF;
            w->pcm_bits    = (pcm >> 16) & 0x7F;
            uint32_t fmt = hda_get_param(codec->addr, nid, HDA_PARAM_STREAM_FORMATS);
            w->pcm_formats = fmt & 0xFF;
        }
    }
}

static int hda_find_pcm_output(int codec_addr) {
    (void)codec_addr;
    for (int i = 0; i < hda_num_widgets; i++) {
        hda_widget_info_t *w = &hda_widgets[i];
        if (w->type == HDA_WIDGET_TYPE_DAC && w->active)
            return w->node_id;
    }
    return -1;
}

static int hda_setup_pcm_stream(int codec_addr, int converter_node) {
    if (output_configured) return 0;

    pcm_buffer = (uint8_t*)kmalloc_aligned(HDA_PCM_BUFFER_SIZE, 4096);
    pcm_bdl    = (hda_bdl_entry_t*)kmalloc_aligned(
                     sizeof(hda_bdl_entry_t) * HDA_BDL_ENTRIES, 4096);
    if (!pcm_buffer || !pcm_bdl) {
        printk("3" "HDA: PCM buffer/BDL alloc failed");
        return -1;
    }
    memset(pcm_buffer, 0, HDA_PCM_BUFFER_SIZE);
    memset(pcm_bdl, 0, sizeof(hda_bdl_entry_t) * HDA_BDL_ENTRIES);

    pcm_stream_tag = 1;
    uint32_t frag = HDA_PCM_BUFFER_SIZE / HDA_BDL_ENTRIES;
    for (int i = 0; i < HDA_BDL_ENTRIES; i++) {
        pcm_bdl[i].addr_low  = (uint32_t)(uintptr_t)(pcm_buffer + i * frag);
        pcm_bdl[i].addr_high = 0;
        pcm_bdl[i].length    = frag;
        pcm_bdl[i].ioc       = HDA_BDL_AIFLAG;
    }

    pcm_sd_idx = 0;
    uint32_t sd_base = HDA_SD_BASE_OUT + pcm_sd_idx * HDA_SD_STEP;
    uint32_t fmt = HDA_SDFMT_RATE_48KHZ | HDA_SDFMT_BITS_16;

    hda_reg_write(sd_base + 0x1C, fmt);
    hda_reg_write(sd_base + 0x08, HDA_PCM_BUFFER_SIZE);
    hda_reg_write(sd_base + 0x0C, HDA_BDL_ENTRIES - 1);
    hda_reg_write(sd_base + 0x28, (uint32_t)(uintptr_t)pcm_bdl);
    hda_reg_write(sd_base + 0x00, HDA_SDCTL_DEIE | HDA_SDCTL_FEIE |
                  (pcm_stream_tag << HDA_SDCTL_STRM_TAG_SHIFT));

    uint32_t sdctl = hda_reg_read(sd_base + 0x00);
    sdctl |= HDA_SDCTL_RUN;
    hda_reg_write(sd_base + 0x00, sdctl);

    hda_verb_set(codec_addr, converter_node, 0x706, pcm_stream_tag << 4);
    hda_verb_set(codec_addr, converter_node, 0x701, fmt);

    output_configured = 1;
    pcm_codec_addr = codec_addr;
    pcm_output_node = converter_node;
    printk("6" "HDA: PCM output stream configured (48kHz/16bit)");
    return 0;
}

static int hda_reset_controller(void) {
    uint32_t gctl = hda_reg_read(HDA_GCTL);
    gctl &= ~HDA_GCTL_CRST;
    hda_reg_write(HDA_GCTL, gctl);

    for (int i = 0; i < 1000; i++) {
        if (!(hda_reg_read(HDA_GCTL) & HDA_GCTL_CRST)) break;
        hda_udelay(1000);
    }

    gctl = hda_reg_read(HDA_GCTL);
    gctl |= HDA_GCTL_CRST;
    hda_reg_write(HDA_GCTL, gctl);

    for (int i = 0; i < 1000; i++) {
        if (hda_reg_read(HDA_GCTL) & HDA_GCTL_CRST) break;
        hda_udelay(1000);
    }

    if (!(hda_reg_read(HDA_GCTL) & HDA_GCTL_CRST)) {
        printk("3" "HDA: controller reset timeout");
        return -1;
    }

    hda_udelay(100000);
    printk("6" "HDA: controller reset OK");
    return 0;
}

static int hda_init_controller(uint32_t mmio, uint32_t mmio_size) {
    for (uint32_t off = 0; off < mmio_size; off += 0x1000)
        vmm_map(get_current_pd(), mmio + off, mmio + off,
                PAGE_PRESENT | PAGE_RW | PAGE_PCD | PAGE_PWT);

    hda_mmio_base = mmio;
    hda_mmio_size = mmio_size;

    if (hda_reset_controller() < 0) return -1;

    {
        uint32_t gcap = hda_reg_read(HDA_GCAP);
        uint32_t dword0c = hda_reg_read(HDA_WAKEEN);
        char hex[16];
        printk("[HDA] gcap="); snprintf(hex, sizeof(hex), "0x%x", (unsigned)(gcap)); printk(hex);
        printk(" dword0c="); snprintf(hex, sizeof(hex), "0x%x", (unsigned)(dword0c)); printk(hex);
        printk("\n");
    }

    corb_buf = 0;
    rirb_buf = 0;
    corb_wp = 0;
    rirb_rp = 0;

    hda_corb_rirb_init();

    printk("6" "HDA: CORB/RIRB initialised");

    {
        uint32_t wake = hda_reg_read(HDA_WAKEEN);
        wake |= HDA_WAKEEN_SDI;
        hda_reg_write(HDA_WAKEEN, wake);

        uint16_t states = 0;
        for (int i = 0; i < 500; i++) {
            states = (uint16_t)(hda_reg_read(HDA_WAKEEN) >> 16);
            if (states) break;
            hda_udelay(1000);
        }
        if (states) {
            hda_reg_write(HDA_WAKEEN, (wake & 0x0000FFFF) | ((uint32_t)states << 16));
        }
        if (states == 0) {
            printk("6" "HDA: STATESTS=0, codecs may be unresponsive");
        } else {
            printk("[HDA] STATESTS="); printk_hex(states); printk("\n");
        }
    }

    hda_enumerate_codecs();
    if (hda_num_codecs == 0) {
        printk("4" "HDA: no codecs detected");
        return 0;
    }

    hda_enumerate_widgets(0);

    int output_node = hda_find_pcm_output(hda_codecs[0].addr);
    if (output_node < 0) {
        printk("4" "HDA: no PCM output widget found");
        return 0;
    }

    if (hda_setup_pcm_stream(hda_codecs[0].addr, output_node) < 0) {
        printk("4" "HDA: PCM stream setup failed");
        return 0;
    }

    hda_reg_write(HDA_INTCTL, HDA_INTS_GLB | 0x01);
    return 0;
}

static int _hda_devfs_read(void *p, uint32_t off, uint32_t size, char *buf) {
    (void)p;
    if (!output_configured || !pcm_buffer) return -1;
    uint32_t read_size = size;
    if (off + read_size > HDA_PCM_BUFFER_SIZE)
        read_size = HDA_PCM_BUFFER_SIZE - off;
    memcpy(buf, pcm_buffer + off, read_size);
    return (int)read_size;
}

static int _hda_devfs_write(void *p, uint32_t off, uint32_t size, char *buf) {
    (void)p;
    if (!output_configured || !pcm_buffer) return -1;
    uint32_t write_size = size;
    if (off + write_size > HDA_PCM_BUFFER_SIZE)
        write_size = HDA_PCM_BUFFER_SIZE - off;
    memcpy(pcm_buffer + off, buf, write_size);
    return (int)write_size;
}

static int _hda_devfs_status(void *p, char *buf, uint32_t size) {
    (void)p;
    const char *h1 = "device: hda_audio\ntype: Intel HD Audio\n";
    int pos = 0;
    while (h1[pos] && (uint32_t)pos < size - 1) { buf[pos] = h1[pos]; pos++; }

    if (hda_num_codecs > 0 && hda_codecs[0].active) {
        char hex[16];
        const char *s1 = "codec_vendor: ";
        int i = 0;
        while (s1[i] && (uint32_t)pos < size - 1) { buf[pos++] = s1[i++]; }
        snprintf(hex, sizeof(hex), "0x%x", (unsigned)(hda_codecs[0].vendor_id));
        for (i = 0; hex[i] && (uint32_t)pos < size - 1; i++)
            buf[pos++] = hex[i];
        buf[pos++] = '\n';

        i = 0;
        const char *s2 = "codec_device: ";
        while (s2[i] && (uint32_t)pos < size - 1) { buf[pos++] = s2[i++]; }
        snprintf(hex, sizeof(hex), "0x%x", (unsigned)(hda_codecs[0].device_id));
        for (i = 0; hex[i] && (uint32_t)pos < size - 1; i++)
            buf[pos++] = hex[i];
        buf[pos++] = '\n';
    }

    if (output_configured) {
        const char *s3 = "state: playback active (48kHz/16bit)\n";
        int i = 0;
        while (s3[i] && (uint32_t)pos < size - 1) { buf[pos++] = s3[i++]; }
    } else {
        const char *s4 = "state: no PCM output\n";
        int i = 0;
        while (s4[i] && (uint32_t)pos < size - 1) { buf[pos++] = s4[i++]; }
    }

    buf[pos] = '\0';
    return pos;
}

static int _hda_devfs_ioctl(void *p, uint32_t cmd, void *arg) {
    (void)p;
    switch (cmd) {
        case 0x01: { uint32_t *rate = (uint32_t*)arg; if (!rate) return -1; *rate = 48000; return 0; }
        case 0x02: { uint32_t *freq = (uint32_t*)arg; if (!freq) return -1; *freq = 48000; return 0; }
        default: return -1;
    }
}

static devfs_driver_t drv_hda = {
    .read   = _hda_devfs_read,
    .write  = _hda_devfs_write,
    .status = _hda_devfs_status,
    .ioctl  = _hda_devfs_ioctl,
};

static void hda_detach(void) {
    if (output_configured && hda_mmio_base) {
        uint32_t sd_base = HDA_SD_BASE_OUT + pcm_sd_idx * HDA_SD_STEP;
        uint32_t sdctl = hda_reg_read(sd_base + 0x00);
        sdctl &= ~HDA_SDCTL_RUN;
        hda_reg_write(sd_base + 0x00, sdctl);
        hda_reg_write(sd_base + 0x04, 0);
        output_configured = 0;
    }

    if (hda_mmio_base) {
        hda_reg_write(HDA_INTCTL, 0);
        uint32_t gctl = hda_reg_read(HDA_GCTL);
        gctl &= ~HDA_GCTL_CRST;
        hda_reg_write(HDA_GCTL, gctl);
    }

    if (devfs_was_registered) {
        unregister_chrdev("hda_audio");
        devfs_was_registered = 0;
    }

    if (hda_msix_vector >= 0) {
        msix_unregister_handler(hda_msix_vector);
        msix_free_vector(hda_msix_vector);
        hda_msix_vector = -1;
        hda_msix_table  = NULL;
    }

    if (hda_attached)
        pci_write_config_dword(hda_bus, hda_dev, hda_fn, 0x04, saved_pci_cmd_dw);

    if (pcm_buffer) { kfree(pcm_buffer); pcm_buffer = 0; }
    if (pcm_bdl)    { kfree(pcm_bdl);    pcm_bdl    = 0; }
    if (corb_buf)   { kfree(corb_buf);   corb_buf   = 0; }
    if (rirb_buf)   { kfree(rirb_buf);   rirb_buf   = 0; }

    hda_mmio_base   = 0;
    hda_attached    = 0;
    hda_num_codecs  = 0;
    hda_num_widgets = 0;
    saved_pci_cmd_dw = 0;
    hda_bus = hda_dev = hda_fn = 0;
    memset(hda_codecs, 0, sizeof(hda_codecs));
    memset(hda_widgets, 0, sizeof(hda_widgets));
}

static void hda_isr(void) {
    uint32_t intsts = hda_reg_read(HDA_INTSTS);
    (void)intsts;
}

int pci_driver_probe(pci_device_t *pdev) {
    if (!pdev) return -1;
    if (hda_attached) {
        printk("[HDA] already attached, skipping\n");
        return -1;
    }

    irq_spinlock_init(&hda_lock);

    printk("[HDA] probe bus=");
    printk_hex(pdev->bus); printk(" dev="); printk_hex(pdev->dev);
    printk(" fn="); printk_hex(pdev->fn);
    printk(" irq="); printk_hex(pdev->irq_line); printk("\n");

    hda_bus = pdev->bus;
    hda_dev = pdev->dev;
    hda_fn  = pdev->fn;

    uint32_t mmio = 0;
    uint32_t mmio_size = 0;
    for (int i = 0; i < 6; i++) {
        if (!pdev->bars[i].is_io && pdev->bars[i].base) {
            mmio = pdev->bars[i].base;
            mmio_size = pdev->bars[i].size;
            break;
        }
    }
    if (!mmio) {
        mmio = pci_read_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x10) & ~0xFu;
        mmio_size = 0x4000;
    }
    if (!mmio) { printk("3" "HDA: no MMIO BAR"); return -1; }
    if (mmio_size < 0x4000) mmio_size = 0x4000;

    saved_pci_cmd_dw = pci_read_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x04);
    uint32_t cmd = saved_pci_cmd_dw;
    cmd |= 0x06u;
    cmd &= ~(1u << 10);
    pci_write_config_dword(pdev->bus, pdev->dev, pdev->fn, 0x04, cmd);

    {
        volatile struct msix_table_entry *table = NULL;
        uint32_t table_size = 0;
        hda_msix_vector = -1;
        hda_msix_table  = NULL;
        int cap = pci_msix_support(pdev);
        if (cap && pci_msix_table_map(pdev, &table, &table_size) == 0 && table_size > 0) {
            int vec = msix_alloc_vector();
            if (vec > 0) {
                msix_register_handler(vec, hda_isr);
                pci_msix_enable(pdev, vec, table, 0);
                hda_msix_vector = vec;
                hda_msix_table  = table;
                printk("6" "HDA: MSI-X enabled");
            }
        }
        if (hda_msix_vector < 0) printk("4" "HDA: MSI-X unavailable");
    }

    if (hda_init_controller(mmio, mmio_size) < 0) {
        printk("3" "HDA: controller init failed");
        hda_detach();
        return -1;
    }

    hda_attached = 1;

    if (register_chrdev("hda_audio", DEVFS_F_CHAR, &drv_hda, 0)) {
        devfs_was_registered = 1;
    } else {
        printk("4" "HDA: register_chrdev('hda_audio') failed");
    }

    if (output_configured) {
        printk("6" "HDA: audio driver attached — /dev/hda_audio (PCM playback)");
    } else {
        printk("4" "HDA: attached but no PCM output — /dev/hda_audio (control only)");
    }
    return 0;
}

void pci_driver_remove(pci_device_t *dev) {
    (void)dev;
    int had_any = hda_attached || devfs_was_registered || hda_mmio_base;
    hda_detach();
    if (had_any) printk("6" "HDA: unloaded");
}
