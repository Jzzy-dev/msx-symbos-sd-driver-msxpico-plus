// Test harness: runs the SymbOS SDXC driver on a Z80 core against a model of
// the MSX Pico+ "Nextor SD" cartridge (FBLabs SDXC SPI window) and an SD card
// speaking SPI-mode protocol.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "z80.h"

// ---------------------------------------------------------------- SD model
enum { CARD_SDHC, CARD_SDSC_V2, CARD_SDV1, CARD_MMC };
typedef struct {
    int type;
    uint8_t *img; uint32_t sectors;
    int cs;               // 1 = selected
    int idle, app, inits;
    uint8_t cmd[6]; int ncmd;
    uint8_t out[1100]; int outn, outp;
    int wstate;           // 0 none, 1 wait token, 2 receiving
    uint8_t wbuf[514]; int wn; uint32_t wlba;
    int busy;             // busy bytes after a write
    long bytes, cmds, crc_errors, reads, writes;
} sdcard;

static void q(sdcard *c, uint8_t b) { c->out[c->outn++] = b; }

static void sd_command(sdcard *c) {
    uint8_t cmd = c->cmd[0] & 0x3f;
    uint32_t arg = (c->cmd[1] << 24) | (c->cmd[2] << 16) | (c->cmd[3] << 8) | c->cmd[4];
    uint8_t crc = c->cmd[5];
    int app = c->app; c->app = 0;
    c->outn = c->outp = 0;
    c->cmds++;
    if (getenv("TRACE")) fprintf(stderr, "  [sd] CMD%d arg=%08X idle=%d\n", cmd, arg, c->idle);
    q(c, 0xff);                                   // Ncr = 1
    if (cmd == 0) {
        if (crc != 0x95) { c->crc_errors++; q(c, 0x09); return; }
        c->idle = 1; c->inits = 0; q(c, 0x01); return;
    }
    if (cmd == 8) {
        if (c->type == CARD_SDV1 || c->type == CARD_MMC) { q(c, 0x05); return; }
        if (crc != 0x87) { c->crc_errors++; q(c, 0x09); return; }
        q(c, c->idle); q(c, 0); q(c, 0); q(c, (arg >> 8) & 0x0f); q(c, arg & 0xff); return;
    }
    if (cmd == 55) {
        if (c->type == CARD_MMC) { q(c, 0x04 | c->idle); return; }
        c->app = 1; q(c, c->idle); return;
    }
    if (cmd == 41 && app) {
        if (c->type == CARD_SDHC && !(arg & 0x40000000)) { q(c, 0x01); return; } // SDHC never ready w/o HCS
        if (++c->inits > 3) c->idle = 0;
        q(c, c->idle); return;
    }
    if (cmd == 1) {
        if (c->type != CARD_MMC) { q(c, 0x04 | c->idle); return; }
        if (++c->inits > 3) c->idle = 0;
        q(c, c->idle); return;
    }
    if (c->idle) { q(c, 0x05); return; }          // everything else illegal while idle
    if (cmd == 58) {
        uint32_t ocr = 0x80FF8000u | (c->type == CARD_SDHC ? 0x40000000u : 0);
        q(c, 0); q(c, ocr >> 24); q(c, ocr >> 16); q(c, ocr >> 8); q(c, ocr); return;
    }
    if (cmd == 16) { q(c, arg == 512 ? 0 : 0x40); return; }
    uint32_t lba;
    if (cmd == 17 || cmd == 24) {
        if (c->type == CARD_SDHC) lba = arg;
        else { if (arg & 511) { q(c, 0x20); return; } lba = arg >> 9; }
        if (lba >= c->sectors) { q(c, 0x40); return; }
        if (cmd == 17) {
            c->reads++;
            q(c, 0); q(c, 0xff); q(c, 0xff); q(c, 0xff); q(c, 0xfe);   // R1, Nac, token
            memcpy(&c->out[c->outn], c->img + (size_t)lba * 512, 512); c->outn += 512;
            q(c, 0x12); q(c, 0x34);
        } else {
            q(c, 0); c->wstate = 1; c->wlba = lba; c->wn = 0;
        }
        return;
    }
    q(c, 0x04);                                   // illegal command
}

static uint8_t sd_xfer(sdcard *c, uint8_t mosi) {
    if (!c->cs) return 0xff;
    c->bytes++;
    uint8_t miso = 0xff;
    if (c->outp < c->outn) miso = c->out[c->outp++];
    else if (c->busy > 0) { c->busy--; miso = 0x00; }
    if (c->wstate == 1) { if (mosi == 0xfe) { c->wstate = 2; c->wn = 0; } return miso; }
    if (c->wstate == 2) {
        c->wbuf[c->wn++] = mosi;
        if (c->wn == 514) {
            memcpy(c->img + (size_t)c->wlba * 512, c->wbuf, 512);
            c->writes++;
            c->wstate = 0; c->outn = c->outp = 0;
            q(c, 0xe5); c->busy = 40;
        }
        return miso;
    }
    if (c->ncmd == 0) { if ((mosi & 0xc0) == 0x40) c->cmd[c->ncmd++] = mosi; }
    else { c->cmd[c->ncmd++] = mosi; if (c->ncmd == 6) { c->ncmd = 0; sd_command(c); } }
    return miso;
}

static void sd_set_cs(sdcard *c, int sel) {
    if (!sel && c->cs) { c->ncmd = 0; c->outn = c->outp = 0; c->wstate = 0; }
    c->cs = sel;
}

// ---------------------------------------------------------------- machine
static uint8_t page0seg[8][16384];
static uint8_t ram1[16384];
static uint8_t ram23[32768];
static uint8_t portA8 = 0, portFC = 0;
static sdcard card;
static int irq_enabled_by_bnkdofx = 0;
static long cart_reads, cart_writes, ctrl_writes;
static int gate_bank = 0;          // 1 = firmware v2.12 behaviour: SPI regs only when ROM bank 7 is selected
static uint8_t rom_bank = 0;       // 6000h register (Nextor leaves another bank selected)
static unsigned long *cycp;        // CPU T-state counter for the 7FF1 timer
static unsigned long timer_deadline;
static int spi_on(void) { return !gate_bank || rom_bank == 7; }

static int page1_is_cart(void) { return ((portA8 >> 2) & 3) == 1; }

static uint8_t rd(void *u, uint16_t a) {
    (void)u;
    if (a < 0x4000) return page0seg[portFC & 7][a];
    if (a < 0x8000) {
        if (!page1_is_cart()) return ram1[a - 0x4000];
        if (spi_on() && a >= 0x7b00 && a < 0x7f00) { cart_reads++; return sd_xfer(&card, 0xff); }
        if (spi_on() && a == 0x7ff0) return 0x80;
        if (spi_on() && a == 0x7ff1) { long r = (long)(timer_deadline - *cycp); return r <= 0 ? 0 : (r / 39 > 255 ? 255 : r / 39); }
        return 0xff;                              // ROM content irrelevant here
    }
    return ram23[a - 0x8000];
}
static void wr(void *u, uint16_t a, uint8_t v) {
    (void)u;
    if (a < 0x4000) { page0seg[portFC & 7][a] = v; return; }
    if (a < 0x8000) {
        if (!page1_is_cart()) { ram1[a - 0x4000] = v; return; }
        if (a == 0x6000) { rom_bank = v & 7; return; }
        if (!spi_on()) return;
        if (a >= 0x7b00 && a < 0x7f00) { cart_writes++; sd_xfer(&card, v); return; }
        if (a == 0x7ff0) { ctrl_writes++; sd_set_cs(&card, v & 1); return; }   // MSX Pico+: GPIO13 = !bit0
        if (a == 0x7ff1) { timer_deadline = *cycp + (unsigned long)v * 39; return; } // ~11 us per tick
        return;
    }
    ram23[a - 0x8000] = v;
}
static uint8_t pin(z80 *z, uint8_t p) { (void)z; if (p == 0xa8) return portA8; return 0xff; }
static void pout(z80 *z, uint8_t p, uint8_t v) { (void)z; if (p == 0xa8) portA8 = v; if (p == 0xfc) portFC = v; }

// ABI addresses
#define BNKMONX 0x8112
#define BNKMOFX 0x8115
#define BNKDOFX 0x8136
#define STOADRX 0x8157
#define STOBNKX 0x815A
#define STOBUFX 0x815B
#define RECBASE 0xA000
#define SYSBUF  0xB000
#define RETTRAP 0xFFF0
#define LOADHDR 0x8FE0   // header goes here -> code at 0x9000

static uint16_t w16(uint16_t a) { return rd(0, a) | (rd(0, a + 1) << 8); }
static void sw16(uint16_t a, uint16_t v) { wr(0, a, v & 0xff); wr(0, a + 1, v >> 8); }

static z80 cpu;
static void push(uint16_t v) { cpu.sp -= 2; sw16(cpu.sp, v); }
static uint16_t pop(void) { uint16_t v = w16(cpu.sp); cpu.sp += 2; return v; }

static uint16_t drv_entry[4];
static uint16_t drvslt_addr;

static int call(uint16_t entry, unsigned long *cycles) {
    cpu.sp = 0xF000; push(RETTRAP); cpu.pc = entry; cpu.halted = 0;
    unsigned long c0 = cpu.cyc; long steps = 0;
    while (cpu.pc != RETTRAP) {
        switch (cpu.pc) {
        case BNKMONX: { // map block of HL into page 0, HL -> 0000-3FFF (pages 2/3 stay as they are)
            if (cpu.h >= 0x80) { cpu.a = portFC; cpu.pc = pop(); continue; }
            uint8_t seg = 1 + ((cpu.h << 8 | cpu.l) >> 14);
            portFC = seg; cpu.a = seg; cpu.h &= 0x3f; cpu.pc = pop(); continue; }
        case BNKMOFX: portFC = 0; cpu.pc = pop(); continue;
        case BNKDOFX: portA8 &= ~0x0c; cpu.iff1 = cpu.iff2 = 1; irq_enabled_by_bnkdofx++; cpu.pc = pop(); continue;
        case STOADRX: { uint16_t r = RECBASE + cpu.a * 32; cpu.h = r >> 8; cpu.l = r & 0xff; cpu.pc = pop(); continue; }
        }
        z80_step(&cpu);
        if (++steps > 20000000L) { printf("  !! runaway at PC=%04X\n", cpu.pc); return -1; }
    }
    if (cycles) *cycles = cpu.cyc - c0;
    return 0;
}

static int load_driver(const char *fn) {
    FILE *f = fopen(fn, "rb"); if (!f) { perror(fn); return -1; }
    static uint8_t buf[8192]; size_t n = fread(buf, 1, sizeof buf, f); fclose(f);
    if (memcmp(buf, "SMD3", 4)) { printf("bad id\n"); return -1; }
    uint16_t codelen = buf[4] | buf[5] << 8, nrel = buf[6] | buf[7] << 8;
    if (32 + codelen + 2 * nrel != n) { printf("size mismatch %zu vs %u\n", n, 32 + codelen + 2 * nrel); return -1; }
    for (int i = 0; i < 32 + codelen; i++) wr(0, LOADHDR + i, buf[i]);
    uint16_t delta = LOADHDR - 0x0FE0;
    for (int i = 0; i < nrel; i++) {
        uint16_t at = buf[32 + codelen + 2 * i] | buf[33 + codelen + 2 * i] << 8;
        uint16_t p = at + delta; sw16(p, w16(p) + delta);
    }
    for (int i = 0; i < 4; i++) drv_entry[i] = w16(0x9000 + 2 * i);
    drvslt_addr = 0x9000 + 8 + 3 + 1 + 4;   // drvjmp(8) + RET,DW(3) + DB(1) + DS4
    printf("driver loaded: code %u bytes, %u relocations, entries %04X %04X %04X\n",
           codelen, nrel, drv_entry[0], drv_entry[1], drv_entry[2]);
    return 0;
}

static int fails = 0;
#define CHECK(cond, ...) do { if (cond) printf("  ok   " __VA_ARGS__); else { printf("  FAIL " __VA_ARGS__); fails++; } printf("\n"); } while (0)

static void make_image(int type) {
    memset(&card, 0, sizeof card);
    card.type = type; card.sectors = 65536 * 2;   // 64 MB
    card.img = malloc((size_t)card.sectors * 512);
    for (uint32_t s = 0; s < card.sectors; s++)
        for (int i = 0; i < 512; i++) card.img[(size_t)s * 512 + i] = (uint8_t)(s * 7 + i * 13 + (s >> 8));
    uint8_t *mbr = card.img;
    memset(mbr + 0x1be, 0, 64);
    // partition 1: FAT32 LBA at 2048, partition 3: type 0x83 (unsupported)
    mbr[0x1be + 4] = 0x0c; mbr[0x1be + 8] = 0x00; mbr[0x1be + 9] = 0x08; mbr[0x1be + 12] = 0x00; mbr[0x1be + 13] = 0xf8;
    mbr[0x1ce + 4] = 0x0e; uint32_t p2 = 0x12345 & 0xffff; mbr[0x1ce + 8] = p2 & 0xff; mbr[0x1ce + 9] = (p2 >> 8) & 0xff; mbr[0x1ce + 10] = 1; // start 0x0001xxxx
    mbr[0x1de + 4] = 0x83; mbr[0x1de + 8] = 1;
    mbr[510] = 0x55; mbr[511] = 0xaa;
}

static int act(int dev, uint8_t sub) {
    uint16_t rec = RECBASE + dev * 32;
    for (int i = 0; i < 32; i++) wr(0, rec + i, 0);
    wr(0, rec + 26, sub);
    cpu.a = dev; call(drv_entry[2], NULL);
    return cpu.cf ? cpu.a : -0x100;
}
static uint32_t rec_start(int dev) { uint16_t r = RECBASE + dev * 32 + 12; return w16(r) | (uint32_t)w16(r + 2) << 16; }

static int xfer(int out, int dev, uint32_t lsec, int count, uint16_t addr, unsigned long *cyc) {
    cpu.a = dev; cpu.ix = lsec & 0xffff; cpu.iy = lsec >> 16; cpu.b = count; cpu.d = addr >> 8; cpu.e = addr & 0xff;
    call(drv_entry[out ? 1 : 0], cyc);
    return cpu.cf ? cpu.a : -0x100;
}

static const char *tname[] = { "SDHC/SDXC (block addr)", "SDSC v2 (byte addr)", "SD v1 (byte addr)", "MMC (CMD1, byte addr)" };

static const char *drvfile;
static int load_driver(const char *fn);
static void run_card(int type) {
    load_driver(drvfile);   /* fresh driver state per card = fresh boot */
    printf("\n=== card type: %s ===\n", tname[type]);
    make_image(type);
    portA8 = 0; portFC = 0; rom_bank = 0;
    // slot config: page1 -> primary slot 1, not expanded (SymbOS fills this)
    wr(0, drvslt_addr + 0, 0x04); wr(0, drvslt_addr + 1, 0x00); wr(0, drvslt_addr + 2, 0x04);
    wr(0, STOBNKX, 0); sw16(STOBUFX, SYSBUF);

    int r = act(0, 0x01);
    CHECK(r == -0x100, "drvact channel0/partition1 -> %s (A=%d)", r == -0x100 ? "ok" : "error", r & 0xff);
    CHECK(rd(0, RECBASE + 0) == 1 && rd(0, RECBASE + 1) == 17, "device record status=%d media=%d", rd(0, RECBASE), rd(0, RECBASE + 1));
    CHECK(rec_start(0) == 2048, "partition start = %u (expect 2048)", rec_start(0));
    CHECK(card.crc_errors == 0, "no CRC errors on CMD0/CMD8 (%ld)", card.crc_errors);
    CHECK(memcmp(&ram23[SYSBUF - 0x8000], card.img, 512) == 0, "MBR copied to stobufx");
    CHECK((portA8 & 0x0c) == 0, "page 1 restored after drvact");

    // read 3 sectors at logical 5 into 0x2200 (page 0 -> segment 1)
    unsigned long cyc = 0;
    memset(page0seg[1], 0, sizeof page0seg[1]);
    r = xfer(0, 0, 5, 3, 0x2200, &cyc);
    CHECK(r == -0x100, "drvinp 3 sectors -> %s", r == -0x100 ? "ok" : "error");
    CHECK(memcmp(&page0seg[1][0x2200], card.img + (size_t)(2048 + 5) * 512, 1536) == 0, "read data matches LBA 2053-2055");
    printf("       %lu T-states for 1536 bytes -> %.1f KB/s at 3.58 MHz\n", cyc, 1536.0 / (cyc / 3579545.0) / 1024);

    // read into a buffer in page 2 (stays visible) and one at 0x4000 (mapped to page 0, seg 2)
    r = xfer(0, 0, 0, 1, 0x8400, NULL);
    CHECK(r == -0x100 && memcmp(&ram23[0x0400], card.img + (size_t)2048 * 512, 512) == 0, "read into page-2 buffer");

    // write 2 sectors from 0x1000 (segment 1) to logical 100
    for (int i = 0; i < 1024; i++) page0seg[1][0x1000 + i] = (uint8_t)(0xA5 ^ i ^ (i >> 3));
    r = xfer(1, 0, 100, 2, 0x1000, &cyc);
    CHECK(r == -0x100, "drvout 2 sectors -> %s", r == -0x100 ? "ok" : "error");
    CHECK(memcmp(card.img + (size_t)(2048 + 100) * 512, &page0seg[1][0x1000], 1024) == 0, "written data landed at LBA 2148-2149");
    CHECK(card.img[(size_t)(2048 + 102) * 512] == (uint8_t)((2048 + 102) * 7 + ((2048 + 102) >> 8)), "neighbour sector untouched");
    printf("       %lu T-states for 1024 bytes -> %.1f KB/s at 3.58 MHz\n", cyc, 1024.0 / (cyc / 3579545.0) / 1024);

    // read back
    memset(page0seg[2], 0, 2048);
    r = xfer(0, 0, 100, 2, 0x4000, NULL);   // 0x4000 -> seg 2 offset 0
    CHECK(r == -0x100 && memcmp(page0seg[2], &page0seg[1][0x1000], 1024) == 0, "read-back of written sectors");

    // 32-bit LBA via partition 2 (start 0x00012345) + logical 0x10000 -> beyond 64MB: expect error
    r = act(1, 0x02);
    CHECK(r == -0x100 && rec_start(1) == 0x00012345, "partition 2 start = 0x%08X", rec_start(1));
    r = xfer(0, 1, 0x10000, 1, 0x2000, NULL);
    CHECK(r == 6, "out-of-range sector -> error %d (expect 6)", r & 0xff);
    r = xfer(0, 0, 7, 1, 0x2000, NULL);     // driver re-inits after error
    CHECK(r == -0x100 && memcmp(&page0seg[1][0x2000], card.img + (size_t)(2048 + 7) * 512, 512) == 0, "recovers (re-init) after error");

    r = act(2, 0x00);
    CHECK(r == -0x100 && rec_start(2) == 0, "partition 0 = whole card (superfloppy)");
    r = act(3, 0x04);
    CHECK(r == 4, "empty partition 4 -> error %d (expect 4)", r & 0xff);
    r = act(3, 0x03);
    CHECK(r == 5, "partition type 0x83 -> error %d (expect 5)", r & 0xff);
    r = act(4, 0x11);
    CHECK(r == 26, "channel 1 (no 2nd card on MSX Pico+) -> error %d (expect 26)", r & 0xff);
    r = act(5, 0x21);
    CHECK(r == 32, "channel 2 -> error %d (expect 32)", r & 0xff);
    CHECK(!card.cs, "card deselected at the end");
    printf("       SD commands %ld, sector reads %ld, writes %ld, bnkdofx calls %d\n", card.cmds, card.reads, card.writes, irq_enabled_by_bnkdofx);
    free(card.img);
}

static void swap_test(void) {
    printf("\n=== card swap without reboot: SDHC -> SDSC, then re-activate ===\n");
    load_driver(drvfile);
    make_image(CARD_SDHC);
    portA8 = 0; portFC = 0; rom_bank = 0;
    wr(0, drvslt_addr + 0, 0x04); wr(0, drvslt_addr + 1, 0x00); wr(0, drvslt_addr + 2, 0x04);
    wr(0, STOBNKX, 0); sw16(STOBUFX, SYSBUF);
    int r = act(0, 0x01); CHECK(r == -0x100, "activate on SDHC card");
    free(card.img); make_image(CARD_SDSC_V2);          /* user swaps the card */
    r = act(0, 0x01); CHECK(r == -0x100, "re-activate after swap -> %d", r & 0xff);
    memset(page0seg[1], 0, 4096);
    r = xfer(0, 0, 5, 1, 0x2200, NULL);
    CHECK(r == -0x100 && memcmp(&page0seg[1][0x2200], card.img + (size_t)(2048 + 5) * 512, 512) == 0, "read correct data from the new card");
}

static void put_entry(uint8_t *sec, int n, uint8_t type, uint32_t start, uint32_t len) {
    uint8_t *e = sec + 0x1be + 16 * n;
    memset(e, 0, 16); e[4] = type;
    for (int i = 0; i < 4; i++) { e[8 + i] = start >> (8 * i); e[12 + i] = len >> (8 * i); }
}
static void sign(uint8_t *sec) { sec[510] = 0x55; sec[511] = 0xaa; }
static uint8_t *S(uint32_t lba) { return card.img + (size_t)lba * 512; }

/* Nextor 3 layout: primary FAT16 + extended (20000..119999) holding 3 logical volumes */
static void nextor_image(int type) {
    make_image(type);
    memset(S(0) + 0x1be, 0, 66);
    put_entry(S(0), 0, 0x06, 64, 16000);
    put_entry(S(0), 1, 0x05, 20000, 100000);
    sign(S(0));
    memset(S(20000) + 0x1be, 0, 66);                   /* EBR1 */
    put_entry(S(20000), 0, 0x06, 63, 9000);            /* -> 20063 */
    put_entry(S(20000), 1, 0x05, 10000, 20000);        /* next EBR at 20000+10000 */
    sign(S(20000));
    memset(S(30000) + 0x1be, 0, 66);                   /* EBR2 */
    put_entry(S(30000), 0, 0x0e, 1, 20000);            /* -> 30001 */
    put_entry(S(30000), 1, 0x05, 40000, 60000);        /* next EBR at 60000 */
    sign(S(30000));
    memset(S(60000) + 0x1be, 0, 66);                   /* EBR3 */
    put_entry(S(60000), 0, 0x0c, 32, 50000);           /* -> 60032 */
    sign(S(60000));
}
static void mp_setup(int type) {
    load_driver(drvfile);
    nextor_image(type);
    portA8 = 0; portFC = 0; rom_bank = 0;
    wr(0, drvslt_addr + 0, 0x04); wr(0, drvslt_addr + 1, 0x00); wr(0, drvslt_addr + 2, 0x04);
    wr(0, STOBNKX, 0); sw16(STOBUFX, SYSBUF);
}
static void mp_test(int type) {
    int r;
    printf("\n=== multi-partition, %s ===\n", tname[type]);
    mp_setup(type);
    r = act(0, 0x00); CHECK(r == -0x100 && rec_start(0) == 0, "setting 0 -> whole card (start %u)", rec_start(0));
    r = act(1, 0x01); CHECK(r == -0x100 && rec_start(1) == 64, "setting 1 -> primary at %u (expect 64)", rec_start(1));
    r = act(2, 0x02); CHECK(r == -0x100 && rec_start(2) == 20063, "setting 2 -> logical 1 at %u (expect 20063)", rec_start(2));
    r = act(3, 0x03); CHECK(r == -0x100 && rec_start(3) == 30001, "setting 3 -> logical 2 at %u (expect 30001)", rec_start(3));
    r = act(4, 0x04); CHECK(r == -0x100 && rec_start(4) == 60032, "setting 4 -> logical 3 at %u (expect 60032)", rec_start(4));
    CHECK(rd(0, RECBASE + 4 * 32 + 1) == 17, "media type 17 (SD) on logical volume");
    r = act(5, 0x05); CHECK(r == 4, "setting 5 -> error %d (expect 4)", r & 0xff);
    memset(page0seg[1], 0, 4096);
    r = xfer(0, 2, 5, 2, 0x2200, NULL);
    CHECK(r == -0x100 && memcmp(&page0seg[1][0x2200], S(20068), 1024) == 0, "read logical 1, sectors 5-6 = LBA 20068-20069");
    for (int i = 0; i < 512; i++) page0seg[1][0x1000 + i] = (uint8_t)(i * 3 + 1);
    r = xfer(1, 4, 7, 1, 0x1000, NULL);
    CHECK(r == -0x100 && memcmp(S(60039), &page0seg[1][0x1000], 512) == 0, "write logical 3, sector 7 -> LBA 60039");
    r = xfer(0, 3, 0, 1, 0x2000, NULL);
    CHECK(r == -0x100 && memcmp(&page0seg[1][0x2000], S(30001), 512) == 0, "read logical 2, sector 0 = LBA 30001");
    free(card.img);

    printf("  -- broken / other layouts --\n");
    mp_setup(type);                                   /* chain loops back to EBR1 */
    put_entry(S(30000), 1, 0x05, 0, 60000);
    r = act(0, 0x03); CHECK(r == -0x100 && rec_start(0) == 30001, "looping chain: logical 2 still found");
    r = act(0, 0x04); CHECK(r == 4, "looping chain: logical 3 -> error %d (expect 4, no hang)", r & 0xff);
    free(card.img);
    mp_setup(type);                                   /* volume runs past extended end */
    put_entry(S(60000), 0, 0x0c, 32, 70000);
    r = act(0, 0x04); CHECK(r == 4, "volume past extended end -> error %d (expect 4)", r & 0xff);
    free(card.img);
    mp_setup(type);                                   /* unsupported logical type */
    put_entry(S(30000), 0, 0x83, 1, 20000);
    r = act(0, 0x03); CHECK(r == 5, "logical type 0x83 -> error %d (expect 5)", r & 0xff);
    free(card.img);
    mp_setup(type);                                   /* EBR without signature */
    S(30000)[510] = 0;
    r = act(0, 0x03); CHECK(r == 4, "EBR without 55AA -> error %d (expect 4)", r & 0xff);
    free(card.img);
    mp_setup(type);                                   /* not a Nextor layout: entry 3 used */
    put_entry(S(0), 2, 0x0b, 120000, 1000);
    r = act(0, 0x03); CHECK(r == -0x100 && rec_start(0) == 120000, "non-Nextor MBR: setting 3 = MBR entry 3 (%u)", rec_start(0));
    r = act(0, 0x02); CHECK(r == 5, "non-Nextor MBR: setting 2 = extended entry -> error %d (expect 5)", r & 0xff);
    free(card.img);
    mp_setup(type);                                   /* Nextor layout, unsigned MBR */
    S(0)[511] = 0;
    r = act(0, 0x02); CHECK(r == 4, "Nextor layout, MBR without 55AA -> error %d (expect 4)", r & 0xff);
    r = act(0, 0x01); CHECK(r == -0x100 && rec_start(0) == 64, "...setting 1 still works");
    free(card.img);
}
int main(int argc, char **argv) {
    z80_init(&cpu); cycp = &cpu.cyc;
    if (argc > 2) gate_bank = atoi(argv[2]);
    printf("SPI window gated on ROM bank 7: %s\n", gate_bank ? "yes (firmware v2.12)" : "no (old model)");
    cpu.read_byte = rd; cpu.write_byte = wr; cpu.port_in = pin; cpu.port_out = pout;
    drvfile = argc > 1 ? argv[1] : "SDXC.smd";
    if (load_driver(drvfile)) return 1;
    if (argc > 3 && !strcmp(argv[3], "mp")) { mp_test(CARD_SDHC); mp_test(CARD_SDSC_V2); }
    else if (argc > 3) swap_test(); else for (int t = 0; t < 4; t++) run_card(t);
    printf("\n%s (%d failures)\n", fails ? "FAILED" : "ALL TESTS PASSED", fails);
    return fails != 0;
}
