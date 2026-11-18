; SPDX-License-Identifier: GPL-3.0-only
;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;@                                                                            @
;@            S y m b O S   -   M S X   D e v i c e   D r i v e r             @
;@                 FBLABS SDXC SPI INTERFACE  (MSX Pico+ Nextor SD)           @
;@                                                                            @
;@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
;
; Draft driver. SymbOS ABI/structure follows Drv-IDESunrise.asm and
; Drv-WonderTANG.asm from Prodatron/symdrv-msx-massstorage (GPL-3.0).
;
; Hardware interface (identical on FBLabs SDXC and on the MSX Pico+ when it
; boots in one of its "Nextor SD" modes; reverse engineered from the MSX Pico+
; v2.12 firmware and its built-in Nextor "FBLabs SDXC" driver):
;
;   #7B00-#7EFF  SPI data window. A READ clocks out #FF and returns the byte
;                received from the card (MISO). A WRITE clocks out the byte.
;                The whole 1 KB window aliases the same port, so LDI/LDIR can
;                stream a 512-byte sector in or out.
;   #7FF0        write: card select. bit0=1 -> card 1 selected (/CS low),
;                bit1=1 -> card 2 (FBLabs dual-slot only). 0 -> deselected.
;                read : status. On MSX Pico+ always #80 (card present,
;                no disk-change, single slot). Not used by this driver.
;   #7FF1        countdown timer (not used; loops are counted instead).
;   #6000        ROM bank register of the Nextor ROM. On MSX Pico+ v2.12 the
;                registers above are ONLY live while bank 7 is selected, so
;                drvshw writes 7 here every time the cartridge is shown.
;
; The card speaks plain SD SPI-mode protocol; everything (CMD0/8/41/58,
; CMD17/CMD24, tokens, busy) is done here on the Z80 side.
;
; Assemble: sjasmplus --dirbol --raw=SDXC.smd Drv-SDXC.asm

; -----------------------------------------------------------------------------
; SymbOS 4 MSX mass-storage ABI (same values as Drv-IDESunrise.asm)

stobnkx        EQU 0815Ah      ; caller buffer banking configuration
bnkmonx        EQU 08112h      ; map caller buffer (A=stobnkx, HL=address)
bnkmofx        EQU 08115h      ; restore normal mapping
bnkdofx        EQU 08136h      ; hide device ROM in page 1, re-enable irqs
stoadrx        EQU 08157h      ; HL = data record of device A
stobufx        EQU 0815Bh      ; address of the shared 512-byte buffer

; storage error codes (SymbOS-File.txt)
stoerrpno      EQU 4           ; partition does not exist
stoerrptp      EQU 5           ; unsupported media or partition
stoerrsec      EQU 6           ; error while sector read/write
stoerrrdy      EQU 26          ; device is not ready
stoerrxch      EQU 32          ; device channel not available

; device data record (values as used by Drv-WonderTANG.asm; please verify
; against SymbOS-File-Const.asm before release)
stotypoky      EQU 1
stomedsdc      EQU 17          ; media type "SD card"
stodatsta      EQU 0
stodattyp      EQU 1
stodatbeg      EQU 12
stodatsub      EQU 26

ideparadr      EQU 01BEh       ; partition table in MBR
idepartyp      EQU 4
ideparbeg      EQU 8

; hardware
SPI_DATA       EQU 07B00h
SPI_CTRL       EQU 07FF0h
ROM_BANK       EQU 06000h      ; ASCII16-style bank register of the Nextor ROM
SDXC_BANK      EQU 7           ; bank in which 7B00h-7FF1h are live

; SD commands (with start/transmission bits)
CMD0           EQU 040h+0
CMD8           EQU 040h+8
CMD16          EQU 040h+16
CMD17          EQU 040h+17
CMD24          EQU 040h+24
CMD55          EQU 040h+55
CMD58          EQU 040h+58
ACMD41         EQU 040h+41
CMD1           EQU 040h+1

DRV_ID         EQU 13          ; placeholder - ask Prodatron for an official ID

; card flag bits (per channel)
CF_INIT        EQU 0           ; card initialised
CF_BYTE        EQU 1           ; byte addressing (SDSC/MMC), else block (SDHC/XC)

; -----------------------------------------------------------------------------
; Driver header

ORG 01000h-32
RELOCATE_START

DB "SMD3"                      ; ID
DW drvend-drvjmp               ; code length
DW relocate_count              ; number of relocation entries
DS 8                           ; reserved
DB 1,1,2                       ; version 1.1, type 2 = SD
DB "SDXC FBLabs  "             ; comment (13 chars)

drvjmp:
        DW drvinp,drvout,drvact,drvmof
drvmof:
        RET
        DW 0
        DB 32*2+DRV_ID         ; bit0-4 driver ID, bit5-7 storage type (2=SD)
        DS 4
drvslt:
        DS 3                   ; filled by SymbOS: slot switching for page 1

idepartok:
        DB 01h,11h,04h,06h,0Eh,14h,16h,1Eh,0Bh,0Ch,1Bh,1Ch
idepartan      EQU 12

sec_cnt:    DB 0               ; sectors left in current transfer
buf_seg:    DB 0               ; page-0 mapping returned by bnkmonx
card_mask:  DB 1               ; value for SPI_CTRL (1=card1, 2=card2)
chan_flag:  DW card_flags      ; -> flags byte of current channel
card_flags: DB 0,0             ; per channel: CF_INIT / CF_BYTE
cur_lba:    DS 4               ; little endian
sd_v2:      DB 0               ; scratch during init

; =============================================================================
; PUBLIC ENTRY POINTS
; =============================================================================

;### DRVINP -> read sectors
;### In  A=device, IY:IX=logical sector, B=count, DE=destination, (stobnkx)
;### Out CF=0 ok / CF=1 A=error
drvinp:
        PUSH AF
        LD A,B
        LD (sec_cnt),A
        POP AF
        PUSH DE
        CALL drvsec             ; BC:DE = physical LBA, channel selected
        CALL store_lba
        POP HL
        LD A,0
        JR drvio

;### DRVOUT -> write sectors (same parameters, DE=source)
drvout:
        PUSH AF
        LD A,B
        LD (sec_cnt),A
        POP AF
        PUSH DE
        CALL drvsec
        CALL store_lba
        POP HL
        LD A,1

; common transfer loop. A=0 read, A=1 write, HL=caller address
drvio:
        LD (drvio_dir+1),A
        CALL drvshw             ; DI, cartridge in page 1
        CALL ensure_init
        JR C,drvio_noinit
        LD A,(stobnkx)
        CALL bnkmonx            ; maps caller block, HL corrected
        LD (buf_seg),A
drvio_loop:
drvio_dir:
        LD A,0                  ; patched: 0=read 1=write
        OR A
        JR NZ,drvio_w
        CALL sd_read_sector
        JR drvio_chk
drvio_w:
        CALL sd_write_sector
drvio_chk:
        JR C,drvio_exit
        LD A,(sec_cnt)
        DEC A
        LD (sec_cnt),A
        JR Z,drvio_exit         ; A=0, CF=0
        PUSH HL
        CALL bnkmofx            ; let interrupts in between sectors
        CALL bnkdofx
        POP HL
        CALL drvshw
        LD A,(buf_seg)
        OUT (0FCh),A
        JR drvio_loop
drvio_exit:
        PUSH AF
        CALL bnkmofx
        CALL bnkdofx
        POP AF
        RET
drvio_noinit:                   ; caller block was not mapped yet
        PUSH AF
        CALL bnkdofx
        POP AF
        RET

;### DRVACT -> init card, read MBR, select partition
;### In  A=device. stodatsub: high nibble = channel (0/1), low nibble =
;###     partition (0 = whole card / superfloppy, 1-4 = primary partition)
drvact:
        PUSH AF
        CALL stoadrx
        POP BC                  ; B = device
        PUSH HL                 ; device record
        LD DE,stodatsub
        ADD HL,DE
        LD A,(HL)
        RRCA
        RRCA
        RRCA
        RRCA
        AND 0Fh
        CP 2
        JR C,drvact_ch
        POP HL
        LD A,stoerrxch
        SCF
        RET
drvact_ch:
        CALL set_channel
        LD HL,(chan_flag)
        LD (HL),0               ; force a fresh card init
        CALL drvshw
        CALL sd_init
        PUSH AF
        CALL bnkdofx
        POP AF
        JR NC,drvact_mbr
        POP HL
        RET
drvact_mbr:
        LD A,1                  ; read physical sector 0 into stobufx
        LD (sec_cnt),A
        LD BC,0
        LD DE,0
        CALL store_lba
        LD HL,(stobufx)
        XOR A
        CALL drvio
        POP DE                  ; DE = device record
        RET C

        LD HL,stodatsub
        ADD HL,DE
        LD A,(HL)
        AND 0Fh
        LD BC,0                 ; partition start = 0 (whole card)
        LD IY,0
        JR Z,drvact_ready
        CP 5
        JR NC,drvact_nopart

        LD IX,(stobufx)
        LD BC,ideparadr-16
        ADD IX,BC
        LD BC,16
drvact_pl:
        ADD IX,BC
        DEC A
        JR NZ,drvact_pl

        LD A,(IX+idepartyp)
        OR A
        JR Z,drvact_nopart
        LD HL,idepartok
        LD B,idepartan
drvact_tl:
        CP (HL)
        JR Z,drvact_tok
        INC HL
        DJNZ drvact_tl
        LD A,stoerrptp
        SCF
        RET
drvact_nopart:
        LD A,stoerrpno
        SCF
        RET
drvact_tok:
        LD C,(IX+ideparbeg+2)
        LD B,(IX+ideparbeg+3)
        PUSH BC
        POP IY
        LD C,(IX+ideparbeg+0)
        LD B,(IX+ideparbeg+1)
drvact_ready:                   ; BC = start low word, IY = start high word
        EX DE,HL                ; HL = device record
        LD (HL),stotypoky
        INC HL
        LD (HL),stomedsdc
        LD DE,stodatbeg-stodattyp
        ADD HL,DE
        LD (HL),C
        INC HL
        LD (HL),B
        INC HL
        PUSH IY
        POP BC
        LD (HL),C
        INC HL
        LD (HL),B
        XOR A
        RET

; =============================================================================
; HELPERS
; =============================================================================

;### DRVSEC -> IY:IX + partition start -> BC:DE, select channel of device A
drvsec:
        CALL stoadrx            ; HL = record
        PUSH HL
        LD BC,stodatsub
        ADD HL,BC
        LD A,(HL)
        RRCA
        RRCA
        RRCA
        RRCA
        AND 01h
        CALL set_channel
        POP HL
        LD BC,stodatbeg
        ADD HL,BC
        LD C,(HL)
        INC HL
        LD B,(HL)
        INC HL
        ADD IX,BC
        LD C,(HL)
        INC HL
        LD B,(HL)
        JR NC,drvsec1
        INC BC
drvsec1:
        ADD IY,BC
        PUSH IX
        POP DE
        PUSH IY
        POP BC
        RET

;### SET_CHANNEL -> A=0/1
set_channel:
        LD HL,card_flags
        LD E,A
        LD D,0
        ADD HL,DE
        LD (chan_flag),HL
        INC A
        LD (card_mask),A
        RET

;### STORE_LBA -> BC:DE -> cur_lba
store_lba:
        LD (cur_lba+0),DE
        LD (cur_lba+2),BC
        RET

;### INC_LBA
inc_lba:
        PUSH HL
        LD HL,cur_lba
        INC (HL)
        JR NZ,inc_lba1
        INC HL
        INC (HL)
        JR NZ,inc_lba1
        INC HL
        INC (HL)
        JR NZ,inc_lba1
        INC HL
        INC (HL)
inc_lba1:
        POP HL
        RET

;### LBA_ARG -> BC:DE = command argument for cur_lba (block or byte address)
lba_arg:
        PUSH HL
        LD DE,(cur_lba+0)
        LD BC,(cur_lba+2)
        LD HL,(chan_flag)
        BIT CF_BYTE,(HL)
        POP HL
        RET Z
        LD B,C                  ; byte address = LBA * 512
        LD C,D
        LD D,E
        LD E,0
        SLA D
        RL C
        RL B
        RET

;### ENSURE_INIT -> (cartridge visible) init card of current channel if needed
;### Out CF=0 ok / CF=1 A=error. Preserves HL.
ensure_init:
        PUSH HL
        LD HL,(chan_flag)
        LD A,(HL)
        AND 1 << CF_INIT        ; AND also clears CF
        JR NZ,ensure_init1
        CALL sd_init
ensure_init1:
        POP HL
        RET

;### DRVSHW -> show cartridge in page 1 (DI)
drvshw:
        LD A,(drvslt+0)
        DI
        OUT (0A8h),A
        LD A,(drvslt+1)
        LD (0FFFFh),A
        LD A,(drvslt+2)
        OUT (0A8h),A
        LD A,SDXC_BANK          ; MSX Pico+ v2.12: SPI registers only respond
        LD (ROM_BANK),A         ; while ROM bank 7 (the driver bank) is selected
        RET

; =============================================================================
; SD CARD (SPI MODE)
; =============================================================================

;### SD_DESEL -> /CS high plus 8 clocks
sd_desel:
        XOR A
        LD (SPI_CTRL),A
        LD A,(SPI_DATA)
        RET

;### SD_CMD -> send command
;### In  A=command byte (#40+n), B,C,D,E = argument (MSB first)
;### Out A=R1, CF=1 on response timeout. Card stays selected.
;### Destroys F,B
sd_cmd:
        PUSH AF
        XOR A                   ; deselect, 8 clocks, select (like Nextor)
        LD (SPI_CTRL),A
        LD A,(SPI_DATA)
        LD A,(card_mask)
        LD (SPI_CTRL),A
        LD A,(SPI_DATA)         ; one more byte with /CS low
        POP AF
        LD (SPI_DATA),A
        PUSH AF
        LD A,B
        LD (SPI_DATA),A
        LD A,C
        LD (SPI_DATA),A
        LD A,D
        LD (SPI_DATA),A
        LD A,E
        LD (SPI_DATA),A
        POP AF
        LD B,095h               ; valid CRC for CMD0
        CP CMD0
        JR Z,sd_cmd_crc
        LD B,087h               ; valid CRC for CMD8 (arg #1AA)
        CP CMD8
        JR Z,sd_cmd_crc
        LD B,0FFh
sd_cmd_crc:
        LD A,B
        LD (SPI_DATA),A
        LD B,16                 ; Ncr: up to 8 bytes, be generous
sd_cmd_r:
        LD A,(SPI_DATA)
        OR A
        RET P                   ; bit7=0 -> R1 received, CF=0 (OR clears it)
        DJNZ sd_cmd_r
        SCF
        RET

;### SD_INIT -> initialise card of current channel (cartridge visible)
;### Out CF=0 ok / CF=1 A=error
sd_init:
        CALL sd_desel
        LD B,10                 ; >=74 clocks with /CS high
        LD A,0FFh
sd_init_clk:
        LD (SPI_DATA),A
        DJNZ sd_init_clk

        LD H,32                 ; CMD0 -> idle
sd_init_c0:
        LD A,CMD0
        LD BC,0
        LD DE,0
        CALL sd_cmd
        JR C,sd_init_c0n
        CP 01h
        JR Z,sd_init_c8
sd_init_c0n:
        DEC H
        JR NZ,sd_init_c0
        JP sd_init_fail

sd_init_c8:
        XOR A
        LD (sd_v2),A
        LD A,CMD8               ; SEND_IF_COND 2.7-3.6V, pattern #AA
        LD BC,0
        LD DE,01AAh
        CALL sd_cmd
        JP C,sd_init_fail
        BIT 2,A                 ; illegal command -> SD v1 or MMC
        JR NZ,sd_init_a41
        LD A,(SPI_DATA)
        LD A,(SPI_DATA)
        LD A,(SPI_DATA)
        AND 0Fh
        CP 01h                  ; voltage accepted
        JP NZ,sd_init_fail
        LD A,(SPI_DATA)
        CP 0AAh                 ; echo pattern
        JP NZ,sd_init_fail
        LD A,1
        LD (sd_v2),A

sd_init_a41:
        LD HL,8000              ; ACMD41 retries (~1s+ at 3.58MHz)
sd_init_a41l:
        LD A,CMD55
        LD BC,0
        LD DE,0
        CALL sd_cmd
        JP C,sd_init_fail
        CP 2                    ; 0 or 1 = ok, illegal -> MMC
        JR NC,sd_init_mmc
        LD A,(sd_v2)
        OR A
        LD B,0
        JR Z,sd_init_hcs
        LD B,040h               ; HCS: we support SDHC/SDXC
sd_init_hcs:
        LD A,ACMD41
        LD C,0
        LD DE,0
        CALL sd_cmd
        JP C,sd_init_fail
        OR A
        JR Z,sd_init_rdy
        CP 01h
        JR NZ,sd_init_mmc       ; ACMD41 rejected -> try MMC CMD1
        DEC HL
        LD A,H
        OR L
        JR NZ,sd_init_a41l
        JP sd_init_fail

sd_init_mmc:                    ; MMC: CMD1 until ready, byte addressing
        XOR A
        LD (sd_v2),A
        LD HL,8000
sd_init_mmcl:
        LD A,CMD1
        LD BC,0
        LD DE,0
        CALL sd_cmd
        JP C,sd_init_fail
        OR A
        JR Z,sd_init_rdy
        DEC HL
        LD A,H
        OR L
        JR NZ,sd_init_mmcl
        JP sd_init_fail

sd_init_rdy:
        LD C,1 << CF_BYTE       ; default: byte addressing
        LD A,(sd_v2)
        OR A
        JR Z,sd_init_blk
        PUSH BC
        LD A,CMD58              ; READ_OCR -> CCS bit
        LD BC,0
        LD DE,0
        CALL sd_cmd
        POP BC
        JR C,sd_init_fail
        OR A
        JR NZ,sd_init_fail
        LD A,(SPI_DATA)         ; OCR[31:24]
        LD B,A
        LD A,(SPI_DATA)
        LD A,(SPI_DATA)
        LD A,(SPI_DATA)
        BIT 6,B                 ; CCS=1 -> block addressing
        JR Z,sd_init_blk
        LD C,0
sd_init_blk:
        BIT CF_BYTE,C
        JR Z,sd_init_ok
        PUSH BC
        LD A,CMD16              ; SET_BLOCKLEN 512 for byte-addressed cards
        LD BC,0
        LD DE,0200h
        CALL sd_cmd
        POP BC
        JR C,sd_init_fail
        OR A
        JR NZ,sd_init_fail
sd_init_ok:
        SET CF_INIT,C
        LD HL,(chan_flag)
        LD (HL),C
        CALL sd_desel
        XOR A
        RET
sd_init_fail:
        CALL sd_desel
        LD HL,(chan_flag)
        LD (HL),0
        LD A,stoerrrdy
        SCF
        RET

;### SD_READ_SECTOR -> CMD17, HL=destination (page-mapped), cur_lba
;### Out HL+=512, CF=1 A=error
sd_read_sector:
        CALL lba_arg
        LD A,CMD17
        CALL sd_cmd
        JP C,sd_rw_err
        OR A
        JP NZ,sd_rw_err
        LD DE,0                 ; wait for data token (65536 polls max)
sd_rd_tok:
        LD A,(SPI_DATA)
        CP 0FEh
        JR Z,sd_rd_data
        INC A                   ; #FF -> still waiting
        JP NZ,sd_rw_err         ; anything else = error token
        DEC DE
        LD A,D
        OR E
        JR NZ,sd_rd_tok
        JP sd_rw_err
sd_rd_data:
        EX DE,HL                ; DE = destination
        LD HL,SPI_DATA          ; source walks #7B00-#7CFF (all aliases)
        LD A,512/32
sd_rd_blk:
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        DEC A
        JP NZ,sd_rd_blk
        EX DE,HL                ; HL = destination+512
        LD A,(SPI_DATA)         ; CRC16 (ignored)
        LD A,(SPI_DATA)
        CALL sd_desel
        CALL inc_lba
        XOR A
        RET

sd_rw_err:
        CALL sd_desel
        LD HL,(chan_flag)       ; force re-init on next access
        RES CF_INIT,(HL)
        LD A,stoerrsec
        SCF
        RET

;### SD_WRITE_SECTOR -> CMD24, HL=source (page-mapped), cur_lba
;### Out HL+=512, CF=1 A=error
sd_write_sector:
        CALL lba_arg
        LD A,CMD24
        CALL sd_cmd
        JP C,sd_rw_err
        OR A
        JP NZ,sd_rw_err
        LD A,0FFh               ; Nwr gap
        LD (SPI_DATA),A
        LD A,0FEh               ; start block token
        LD (SPI_DATA),A
        LD DE,SPI_DATA          ; destination walks #7B00-#7CFF
        LD A,512/32
sd_wr_blk:
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        LDI
        DEC A
        JP NZ,sd_wr_blk
        LD A,0FFh               ; dummy CRC
        LD (SPI_DATA),A
        LD (SPI_DATA),A
        LD B,16                 ; data response xxx0sss1
sd_wr_resp:
        LD A,(SPI_DATA)
        CP 0FFh
        JR NZ,sd_wr_got
        DJNZ sd_wr_resp
        JP sd_rw_err
sd_wr_got:
        AND 1Fh
        CP 05h                  ; data accepted
        JP NZ,sd_rw_err
        LD C,16                 ; busy wait: up to 16*65536 polls
        LD DE,0
sd_wr_busy:
        LD A,(SPI_DATA)
        INC A
        JR Z,sd_wr_done
        DEC DE
        LD A,D
        OR E
        JR NZ,sd_wr_busy
        DEC C
        JR NZ,sd_wr_busy
        JP sd_rw_err
sd_wr_done:
        CALL sd_desel
        CALL inc_lba
        XOR A
        RET

drvend:

RELOCATE_TABLE
RELOCATE_END
