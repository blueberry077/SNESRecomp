#ifndef SNES__H
#define SNES__H

typedef struct IO {
  uint8_t NMITIMEN;
  uint8_t WRMPYA;
  uint8_t WRMPYB;
  uint8_t WRDIVL;
  uint8_t WRDIVH;
  uint8_t WRDIVB;
  uint8_t RDDIVL;
  uint8_t RDDIVH;
  uint8_t RDMPYL;
  uint8_t RDMPYH;
  uint8_t RDNMI;
  uint8_t HVBJOY;
  uint8_t JOY1L;
  uint8_t JOY1H;
  // for cycle-accurate recomp
  uint8_t RDDIVL_old;
  uint8_t RDDIVH_old;
  uint8_t RDMPYL_old;
  uint8_t RDMPYH_old;
  
  int div_done_cycle;
  int mul_done_cycle;
} IO;

void multiply_5A22(IO *io)
{
  uint16_t res = io->WRMPYA * io->WRMPYB;
  io->RDMPYL_old = io->RDMPYL;
  io->RDMPYH_old = io->RDMPYH;
  
  io->RDMPYL = res & 0xFF;
  io->RDMPYH = (res >> 8) & 0xFF;
  io->mul_done_cycle = CycleCount + 8;
  // printf("multiply_5A22(%02X, %02X)=%04X\n", io->WRMPYA, io->WRMPYB, res);
}

void divide_5A22(IO *io)
{
  uint16_t A = (io->WRDIVH << 8) | io->WRDIVL;
  uint16_t B = io->WRDIVB;
  uint16_t res = (B == 0) ? 0xFFFF : A / B;
  uint16_t rem = (B == 0) ? A : A % B;
  
  io->RDDIVL_old = io->RDDIVL;
  io->RDDIVH_old = io->RDDIVH;
  io->RDDIVL_old = io->RDMPYL;
  io->RDMPYH_old = io->RDMPYH;
  
  io->RDDIVL = res & 0xFF;
  io->RDDIVH = (res >> 8) & 0xFF;
  io->RDMPYL = rem & 0xFF;
  io->RDMPYH = (rem >> 8) & 0xFF;
  io->div_done_cycle = CycleCount + 16;
}

typedef struct PPU {
  // Registers
  uint8_t INIDISP;
  uint8_t OBJSEL;
  uint8_t OAMADDL;
  uint8_t OAMADDH;
  uint8_t BGMODE;
  uint8_t MOSAIC;
  uint8_t BG1SC;
  uint8_t BG2SC;
  uint8_t BG3SC;
  uint8_t BG4SC;
  uint8_t BG12NBA;
  uint8_t BG34NBA;
  uint16_t M7HOFS;
  uint16_t M7VOFS;
  uint16_t BG1HOFS;
  uint16_t BG1VOFS;
  uint16_t BG2HOFS;
  uint16_t BG2VOFS;
  uint16_t BG3HOFS;
  uint16_t BG3VOFS;
  uint16_t BG4HOFS;
  uint16_t BG4VOFS;
  uint8_t BG2OFS[2];
  uint8_t BG3OFS[2];
  uint8_t BG4OFS[2];
  uint8_t VMAIN;
  uint8_t VMADDL;
  uint8_t VMADDH;
  uint8_t VMDATAL;
  uint8_t VMDATAH;
  uint16_t CGADD;
  uint8_t CGDATA;
  uint8_t TM;
  uint8_t STAT77;
  uint8_t STAT78;
  // Values
  uint16_t vram_addr;
  int scroll_value;
  // Buffers
  uint8_t vram[64 * 1024];
  uint8_t cgram[512];
  uint8_t oam[544];
  uint8_t oam2[544]; // Sprite Evaluation
} PPU;

#define WRAM_SIZE (128 * 1024)
uint8_t wram[WRAM_SIZE];

#define MAX_SRAM_SIZE (8 * 1024)
uint8_t sram[MAX_SRAM_SIZE];

void write_cgram(PPU *ppu, uint8_t data)
{
  // printf("write_cgram(data=0x%02X) at %04X\n", data, ppu->CGADD);
  ppu->cgram[ppu->CGADD] = data;
  ppu->CGADD++;
}

void write_oam(PPU *ppu, uint8_t data)
{
  uint16_t addr = (ppu->OAMADDH << 8) | ppu->OAMADDL;
  ppu->oam[addr] = data;
  addr = (addr + 1) % 544;
  ppu->OAMADDH = (addr >> 8) & 0xFF;
  ppu->OAMADDL = addr & 0xFF;
}

int vram_inc[4] = { 1, 32, 128, 128 };
void write_vram(PPU *ppu, uint8_t data, int high, int inc)
{
  ppu->vram[ppu->vram_addr + high] = data;
  if (inc) {
    uint16_t addr = (ppu->VMADDH << 8) | ppu->VMADDL;
    addr += vram_inc[ppu->VMAIN & 3];
    ppu->VMADDL = addr & 0xFF;
    ppu->VMADDH = (addr >> 8) & 0xFF;
    ppu->vram_addr = addr * 2;
  }
}

typedef struct DMAChannel {
  int DMAP;
  int BBAD;
  int A1TL;
  int A1TH;
  int A1B;
  int DASL;
  int DASH;
  int DASB;
  int A2AL;
  int A2AH;
  int NLTR;
} DMAChannel;

typedef struct DMA {
  int MDMAEN;
  int HDMAEN;
  DMAChannel ch[8];
} DMA;

void dma_trasfer(DMA *dma, int channel);

void write_dma(DMA *dma, uint64_t addr, uint8_t v)
{
  for (int i = 0; i < 8; ++i) {
    int i_off = i << 4;
    if (addr == 0x4300+i_off /* DMAP0 */) {
      dma->ch[i].DMAP = v;
    } else
    if (addr == 0x4301+i_off /* BBAD0 */) {
      dma->ch[i].BBAD = v;
      // printf("BBAD%i=%02X\n", i, v);
    } else
    if (addr == 0x4302+i_off /* A1T0L */) {
      dma->ch[i].A1TL = v;
      // printf("dma->ch[i].A1TL=%02X\n", dma->ch[i].A1TL);
    } else
    if (addr == 0x4303+i_off /* A1T0H */) {
      dma->ch[i].A1TH = v;
      // printf("dma->ch[i].A1TH=%02X\n", dma->ch[i].A1TH);
    } else
    if (addr == 0x4304+i_off /* A1B0 */) {
      dma->ch[i].A1B = v;
      // printf("dma->ch[i].A1B=%02X\n", dma->ch[i].A1B);
    } else
    if (addr == 0x4305+i_off /* DAS0L */) {
      dma->ch[i].DASL = v;
      // printf("dma->ch[i].DASL=%02X\n", dma->ch[i].DASL);
    } else
    if (addr == 0x4306+i_off /* DAS0H */) {
      dma->ch[i].DASH = v;
      // printf("dma->ch[i].DASH=%02X\n", dma->ch[i].DASH);
    } else
    if (addr == 0x4307+i_off /* DASB0 */) {
      dma->ch[i].DASB = v;
      // printf("dma->ch[i].DASB=%02X\n", dma->ch[i].DASB);
    } else
    if (addr == 0x4308+i_off /* A2A0L */) {
      dma->ch[i].A2AL = v;
      // printf("dma->ch[i].A2AL=%02X\n", dma->ch[i].A2AL);
    } else
    if (addr == 0x4309+i_off /* A2A0H */) {
      dma->ch[i].A2AH = v;
      // printf("dma->ch[i].A2AH=%02X\n", dma->ch[i].A2AH);
    } else
    if (addr == 0x430A+i_off /* NLTR0 */) {
      dma->ch[i].NLTR = v;
      // printf("dma->ch[i].NLTR=%02X\n", dma->ch[i].NLTR);
    }
  }
  if (addr == 0x420B /* MDMAEN */) {
    dma->MDMAEN = v;
    int mask = dma->MDMAEN;
    for (int i = 0; i < 8; ++i) {
      if (mask & 1) {
        dma_trasfer(dma, i);
      }
      mask >>= 1;
    }
  }
}

void dma_trasfer(DMA *dma, int channel)
{
  DMAChannel *ch = &dma->ch[channel];
  
  uint16_t len = ch->DASH<<8 | ch->DASL;
  uint32_t A_addr = (ch->A1B << 16) | (ch->A1TH << 8) | ch->A1TL;
  uint16_t B_addr = 0x2100 | ch->BBAD;
  
  int pattern = ch->DMAP & 3;
  int dir = (ch->DMAP >> 3) & 3;
  switch (dir) {
    case 0: dir = 1;  break;
    case 2: dir = -1; break;
    default:
      dir = 0;
  }
  int directionAB = (ch->DMAP & 0x80) == 0;

  int B_save = B_addr;
  for (uint16_t i = 0; i < len; ++i) {
    if (directionAB) {
      if (pattern == 1) {
        B_addr = B_save + i % 2;
      }
      uint8_t v = __READ8(A_addr);
      __WRITE8(B_addr, v);
    } else {
      uint8_t v = __READ8(B_addr);
      __WRITE8(A_addr, v);
    }
    A_addr += dir;
  }
  
  // Update DMA Channel
  ch->A1TL = A_addr & 0xFF;
  ch->A1TH = (A_addr >> 8) & 0xFF;
  ch->A1B  = (A_addr >> 16) & 0xFF;
  ch->DASL = 0;
  ch->DASH = 0;
}

#endif // SNES__H