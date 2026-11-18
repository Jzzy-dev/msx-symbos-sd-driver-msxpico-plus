# msx-symbos-sd-driver-msxpico-plus

A SymbOS 4 mass-storage driver for the SD card of the **MSX Pico+** cartridge, plus what we learned getting there.

The driver talks to the SD card through the FBLabs SDXC-style SPI port that the MSX Pico+ already exposes in its "Nextor SD" modes. No firmware change and no Sunrise IDE emulation is needed.

<img width="2000" height="1500" alt="picoplusmultipartition" src="https://github.com/user-attachments/assets/8d310a6b-0789-45f4-a4ae-dc9be07fef36" />

## Drivers

| File | Source | Partitions | Status |
|------|--------|------------|--------|
| `MSXPICMP.DRV` | `MSXPICO_SD_SYMBOS_MP.asm` | Multiple, including Nextor 3 logical volumes | v0.9, works on real hardware |
| `MSXPICO.DRV` | `MSXPICO_SD_SYMBOS.asm` | Primary partitions only | JeroenT's v0.8, works on real hardware |
| none (reference only) | `research/Drv-SDXC.asm` | Primary partitions only | JZZY's v1.1, tested in the emulator harness only |

Build with [sjasmplus](https://github.com/z00m128/sjasmplus):

```
sjasmplus --dirbol --raw=MSXPICMP.DRV MSXPICO_SD_SYMBOS_MP.asm
sjasmplus --dirbol --raw=MSXPICO.DRV MSXPICO_SD_SYMBOS.asm
```

Both sources assemble byte-for-byte into the `.DRV` files in this repo. Channel 0 is the SD slot.

## MSXPICMP.DRV: multiple partitions (v0.9)

`MSXPICMP.DRV` (SymbOS name "MSXPico SD MP") is JeroenT's v0.8 driver with three additions:

- **Nextor 3 logical volumes:** SeD_NcL's EBR (extended boot record) walker, ported from [Jzzy-dev/msx-symbos-sunrise-ide-driver-picoverse](https://github.com/Jzzy-dev/msx-symbos-sunrise-ide-driver-picoverse). It checks every EBR and volume address against the extended partition bounds and the 28-bit LBA limit, and stops if the chain doesn't move forward.
- **MMC cards:** if a card rejects CMD55/ACMD41 as illegal, the driver initialises it with CMD1 instead (byte addressing, CMD16 512). Taken from `research/Drv-SDXC.asm`.
- **Card swaps:** the card is re-initialised every time SymbOS activates the drive. Firmware v2.12 never sets the `#7FF0` change bit, so without this a card swapped without a reboot kept the old card's addressing mode. Taken from `research/Drv-SDXC.asm`.

The SD access code and the write handshake are JeroenT's, unchanged.

Partition mapping (Control Panel / Mass Storage, "Primary partition" setting):

| Setting | Nextor 3 layout* | Any other layout |
|---------|------------------|------------------|
| 0 | whole card | whole card |
| 1 | primary partition (first volume) | MBR entry 1 |
| 2 | logical volume 1 | MBR entry 2 |
| 3 | logical volume 2 | MBR entry 3 |
| 4 | logical volume 3 | MBR entry 4 |

\* MBR entry 2 is an extended partition (`#05`/`#0F`), entries 3 and 4 are empty, and the MBR has a valid `#55AA` signature.

Supported volume types are FAT12, FAT16 and FAT32. As with the PicoVerse multi-partition driver, SeD_NcL's patched `SYM.COM` and `SYMBOS.BIN` ("Sector Fix v7") fix a SymbOS 4.0 kernel bug that reports disk error 032 on some volumes. They're available from the msx.org SymbOS thread.

`MSXPICO.DRV` (JeroenT's v0.8) uses settings `1`-`4` as MBR primary partitions only.

## How the MSX Pico+ exposes the SD card

Reverse engineered from firmware v2.12 (`msxpico_noroms_v212.uf2`, RP2350 on a Pimoroni Pico Plus 2). In "Nextor SD" mode the RP2350 does not emulate a disk. It passes SPI bytes straight between the Z80 and the card, and the Z80 driver speaks the SD protocol itself (CMD0, CMD8, ACMD41, CMD58, CMD17, CMD24...).

| MSX address | Read | Write |
|-------------|------|-------|
| `#4000-#7AFF` | Nextor ROM, current 16 KB bank | ignored |
| `#6000` | ROM | ROM bank select (value AND 7), works in every bank |
| `#7B00-#7EFF` **bank 7 only** | clocks out `#FF`, returns the byte received from the card | clocks out the byte |
| `#7FF0` **bank 7 only** | status, always `#80` on v2.12 | card select: bit 0 = 1 pulls SD /CS low |
| `#7FF1` **bank 7 only** | countdown timer, about 11 µs per tick | starts the countdown |

- **The SPI registers only respond while ROM bank 7 is selected** (`#6000` = 7). In any other bank, reads return ROM bytes and writes are ignored. Bank 7 is where the Nextor driver lives, which is why Nextor itself always works. A SymbOS driver must write 7 to `#6000` every time it maps the cartridge into page 1.
- The whole `#7B00-#7EFF` window aliases the same data port, so `LDI`/`LDIR` can stream a 512-byte sector.
- On v2.12, `#7FF0` always reads `#80`: one slot, card present, not write-protected, no card change. A driver can't detect a card swap from it.
- The card sits on RP2350 SPI1: MISO GPIO12, /CS GPIO13, SCK GPIO14, MOSI GPIO15.
- The UF2 holds a USB mass-storage bootloader, two app builds (`msxpico_scc` and `msxpico_opll_scc`) and a Nextor 2.1.4 ROM with the FBLabs SDXC driver at flash `0x064000`. The firmware has no IDE/ATA code.

## Why our first driver failed

Our first driver (`research/Drv-SDXC.asm` v1.0) never selected ROM bank 7. Every SPI read came back as ROM data, CMD0 was never answered, and SymbOS reported error 26 (device not ready).

The cause was a misreading during reverse engineering. In the firmware's bus handler, `cmp r3, #7` compares the **current ROM bank**, and we took it for a mapper-type check. Our emulator harness modelled the hardware the same wrong way, so all its tests passed. Once the harness gates the SPI window on bank 7, v1.0 fails every test, and v1.1 (one extra write to `#6000`) passes them all.

Lesson: a test harness built from the same reverse-engineering notes as the driver can't catch a misreading of those notes. Real hardware, or a second independent reading of the firmware, can.

## JeroenT's driver compared with ours

Only the bank-7 write is a fatal difference. Results in the updated harness (bank-7 gate and 7FF1 timer modelled, simulated cards):

| Harness run | Ours v1.0 | Ours v1.1 | JeroenT v0.8 |
|-------------|-----------|-----------|--------------|
| SDHC/SDXC, SDSC v2, SD v1 | fail (error 26) | pass | pass |
| MMC (CMD1) | fail | pass | fail (no CMD1 fallback) |
| Card swapped, re-activated without reboot | fail | pass | fail (keeps old card's addressing) |
| Read / write, SDHC at 3.58 MHz | n/a | 181 / 151 KB/s | 172 / 142 KB/s |

Other differences, none fatal:

| Area | JeroenT v0.8 | Ours v1.1 |
|------|--------------|-----------|
| Timeouts | `#7FF1` hardware timer, independent of CPU speed | Poll counts, shorter on R800 / turbo |
| `#7FF0` status | Uses model, absent, write-protect and change bits (constant `#80` on v2.12) | Not used |
| Second channel | Error 32 when the model bit says one slot | Init attempt, then error 26 |
| Re-init | On change bit, after an I/O error, or first use | On every activation and after an I/O error |
| Buffer mapping | Caller's buffer mapped only during the 512-byte copy | `bnkmonx` mapping kept for the whole transfer (Sunrise style) |

The speeds ignore MSX wait states and any delay the Pico+ adds.

## Comparison with PicoVerse 2350

[PicoVerse 2350](https://github.com/cristianoag/msx-picoverse-public) takes the other route. Its RP2350 firmware emulates a Sunrise IDE register file and serves sectors from the SD card on its second core, so SymbOS uses a Sunrise IDE driver ([Jzzy-dev/msx-symbos-sunrise-ide-driver-picoverse](https://github.com/Jzzy-dev/msx-symbos-sunrise-ide-driver-picoverse)).

| | MSX Pico+ v2.12 | PicoVerse 2350 |
|-|-----------------|----------------|
| What the MSX sees | FBLabs SDXC SPI port, bank 7 | Sunrise IDE: `#4104` control, `#7C00` data, `#7E00-#7E0F` ATA registers |
| Who speaks the SD protocol | Z80 driver | RP2350 core 1 |
| SymbOS driver | this repo | Sunrise IDE driver without the `#F8` probe |
| Firmware licence | closed, activation | CC BY-NC-SA 4.0 |

The stock SymbOS Sunrise driver sends ATA `#F8` (READ NATIVE MAX ADDRESS) on activation, and PicoVerse doesn't implement it. Sunrise emulation is possible on MSX Pico+ hardware, but only in its closed firmware, so that is up to its author. The native SPI driver makes it unnecessary for SymbOS.

## Known limitations

- `MSXPICMP.DRV`: only the first three logical volumes can be reached (settings 2-4).
- `MSXPICO.DRV`: MMC cards don't initialise, and on v2.12 a swapped card isn't noticed until an I/O error forces re-initialisation. Both are fixed in `MSXPICMP.DRV`.
- Hot-swapping a card while a volume on it is in use can corrupt cached data. Deactivate the drive before removing the card.
- Keep the Pico+ "USB SD-card access" setting off while SymbOS runs, so the firmware and SymbOS don't both use the card.

## Test harness

`test/harness.c` runs a driver binary on a Z80 core against a model of the MSX Pico+ register map (bank-7 gate, `#7FF1` timer) and an SPI-mode SD card (SDHC, SDSC v2, SD v1, MMC). It relocates the driver to `#9000` to exercise the relocation table, and stubs the SymbOS banking calls.

```
git clone --depth 1 https://github.com/superzazu/z80 z80core
gcc -O2 -I z80core -o harness test/harness.c z80core/z80.c
./harness MSXPICMP.DRV 1          # standard suite, bank-7 gate on
./harness MSXPICMP.DRV 1 swap     # card-swap scenario
./harness MSXPICMP.DRV 1 mp       # Nextor 3 logical-volume layouts
```

The `mp` suite checks settings 0-5 on a Nextor 3 layout, reads and writes inside logical volumes, and broken layouts: a looping EBR chain, a volume past the extended partition's end, an unsupported type, missing signatures and a non-Nextor MBR.

| Suite | `MSXPICMP.DRV` v0.9 | `MSXPICO.DRV` v0.8 |
|-------|---------------------|--------------------|
| standard | 4 failures (channel-1 checks: error 32 by design) | 18 failures (channel-1 + MMC) |
| swap | pass | fail |
| mp (SDHC and SDSC) | pass, 38 checks | fails settings 2-4 (no logical volumes) |

The card model was written from the SD specification, not checked against real cards.

## Credits

- SymbOS driver: JeroenT (MSX Pico+)
- Nextor 3 logical partition support (EBR walker): [SeD_NcL](https://www.msx.org/users/sed-ncl)
- SymbOS and its mass-storage driver API: Prodatron / SymbiosiS (Jörn Mika), [symdrv-msx-massstorage](https://github.com/Prodatron/symdrv-msx-massstorage)
- FBLabs SD Mapper / SDXC and its Nextor driver: Fabio Belavenuto, Fabio R. Schmidlin
- Reverse engineering, comparison, harness: JZZY

Licence: GPL-3.0, see `LICENSE`.
