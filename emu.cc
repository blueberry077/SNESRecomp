/*

TITLE SNES Recomp Emulation Layer
AUTHOR Marc-Daniel DALEBA
FILE emu.cc
DATE 2025-11-14
DESCRIPTION
  This is the emulation layer for the recompiler. The SNES Architecture
  is so far from PC that unfortunately this file is necessary to make
  input handling, rendering and CPU synchronization possible.

*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <windows.h>

#include <iostream>

#define SNES_WIDTH  (256)
#define SNES_HEIGHT (224)
#define NMI_CYCLES  (100000) // Should probably be 89342 (NTSC)

int inNMI = 0;
uint8_t regDBR=0;
uint16_t regA=0, regX=0, regY=0, regS=0, regDP=0;
uint8_t N_Flag=0, V_Flag=0, M_Flag=0, X_Flag=0, D_Flag=0;
uint8_t I_Flag=0, Z_Flag=0, C_Flag=0, B_Flag=0, E_Flag=0;
uint8_t MapMode = 0; // 0=LoROM, 1=HiROM, 5=ExHiROM
uint64_t CycleCount = 0;
uint64_t CycleCountNextNMI = NMI_CYCLES;
HANDLE hCPUReady = CreateEvent(NULL, FALSE, FALSE, NULL);
HANDLE hMainReady = CreateEvent(NULL, FALSE, FALSE, NULL);

extern "C" void __CPUSync(void);

extern "C" void __PHP(void);
extern "C" void __PLP(void);
extern "C" void __MVN(uint64_t src, uint64_t dst);
extern "C" void __MVP(uint64_t src, uint64_t dst);

extern "C" void __UpdateNZ_A8(void);
extern "C" void __UpdateNZ_X8(void);
extern "C" void __UpdateNZ_Y8(void);
extern "C" void __UpdateNZ_A16(void);
extern "C" void __UpdateNZ_X16(void);
extern "C" void __UpdateNZ_Y16(void);

extern "C" void __BIT8(uint64_t value);
extern "C" void __BIT16(uint64_t value);
extern "C" uint64_t __TSB(uint64_t value);
extern "C" uint64_t __TRB(uint64_t value);

extern "C" void __TESTNZ8(uint64_t value);
extern "C" void __TESTNZ16(uint64_t value);

extern "C" uint8_t __INC8(uint64_t value);
extern "C" uint16_t __INC16(uint64_t value);
extern "C" uint8_t __ADC8(uint64_t value1, uint64_t value2);
extern "C" uint16_t __ADC16(uint64_t value1, uint64_t value2);
extern "C" uint8_t __SBC8(uint64_t value1, uint64_t value2);
extern "C" uint16_t __SBC16(uint64_t value1, uint64_t value2);

extern "C" uint8_t __ASL8(uint64_t value);
extern "C" uint16_t __ASL16(uint64_t value);
extern "C" uint8_t __LSR8(uint64_t value);
extern "C" uint16_t __LSR16(uint64_t value);
extern "C" uint8_t __ROL8(uint64_t value);
extern "C" uint16_t __ROL16(uint64_t value);
extern "C" uint8_t __ROR8(uint64_t value);
extern "C" uint16_t __ROR16(uint64_t value);

extern "C" void __COMPARE8(uint64_t value1, uint64_t value2);
extern "C" void __COMPARE16(uint64_t value1, uint64_t value2);
extern "C" void __XCE(void);
extern "C" void __WDM(uint8_t v);
extern "C" void __WAI(void);
extern "C" void __WRITE8(uint64_t addr, uint64_t value);
extern "C" void __WRITE16(uint64_t addr, uint64_t value);
extern "C" void __REP(uint64_t mask);
extern "C" void __SEP(uint64_t mask);
extern "C" void __CALL_SHOW(uint64_t addr, uint64_t pc);
extern "C" uint8_t __READ8(uint64_t addr);
extern "C" uint16_t __READ16(uint64_t addr);
extern "C" uint32_t __READ24(uint64_t addr);
extern "C" uint32_t __READ_INS(uint64_t addr);
extern "C" void __PUSH8(uint64_t value);
extern "C" void __PUSH16(uint64_t value);
extern "C" uint8_t __PULL8(void);
extern "C" uint16_t __PULL16(void);
extern "C" void __PRINT_INS(uint64_t pc, uint64_t ins);
extern "C" void __JUMP_FAILED(uint64_t target, uint64_t pc);
extern "C" void __REG_DUMP(void);

extern "C" uint32_t pc_map(uint32_t pc);

#include <snes.h>

extern "C" void Start(void);
extern "C" void Label_Reset(void);
extern "C" void Label_NMI(void);

uint8_t io_RDNMI = 0;
uint8_t io_NMITIMEN = 0;
uint8_t *rom = NULL;
int RecompBreak = 0;
int ROM_OFFSET = 0; // 0 or 512, nothing else
IO io;
PPU ppu;
DMA dma;
int APUIO0 = 0xAA; // For some SMW tests
int APUIO1 = 0xBB; // 
uint32_t framebuf[SNES_WIDTH * SNES_HEIGHT];
uint16_t input1 = 0;

HWND ghwnd;
BITMAPINFO bmi;

uint32_t snes_color_to_rgb(uint16_t c)
{
  uint8_t r = (c & 0x1F) << 3;
  uint8_t g = ((c >> 5) & 0x1F) << 3;
  uint8_t b = ((c >> 10) & 0x1F) << 3;
  return (r << 16) | (g << 8) | b;
}

void draw_palette(void);
void draw_sprite_8x8_8bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip);
void draw_sprite_8x8_4bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip);
void draw_sprite_8x8_2bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip);
void draw_vram_2bpp(int offset);
void draw_vram_4bpp(int offset);
void draw_oam(void);

void draw_screen(void);
void draw_mode0(void);
void draw_mode1(void);
void draw_mode3(void);

bool Running = true;
LRESULT WindowProc(HWND h, UINT m, WPARAM w, LPARAM l);
int InitApplication(void)
{
  WNDCLASS wc;
  HINSTANCE hInstance = GetModuleHandle(NULL);
  
  memset(&wc, '\0', sizeof(wc));
  wc.lpfnWndProc = WindowProc;
  wc.lpszClassName = "SNES RECOMP EMU";
  wc.hInstance = hInstance;
  RegisterClass(&wc);
  
  RECT rect;
  rect.left = 0;
  rect.top = 0;
  rect.right = SNES_WIDTH * 3;
  rect.bottom = SNES_HEIGHT * 3;
  AdjustWindowRect(&rect, WS_OVERLAPPEDWINDOW, FALSE);
  
  ghwnd = CreateWindowEx(
    0,
    "SNES RECOMP EMU",
    "SNES Recomp Emulation Layer",
    WS_OVERLAPPEDWINDOW,
    CW_USEDEFAULT,
    CW_USEDEFAULT,
    rect.right - rect.left,
    rect.bottom - rect.top,
    NULL,
    NULL,
    hInstance,
    NULL
  );
  if (!ghwnd) {
    return -1;
  }
  ShowWindow(ghwnd, SW_NORMAL);
  return 0;
}

int main(int argc, char **argv)
{
  FILE *f = fopen(argv[1], "rb");
  // FILE *f = fopen("roms\\eb_noaud.sfc", "rb");
  // FILE *f = fopen("roms\\cputest-basic.sfc", "rb");
  if (!f) {
    printf("error: couldn't open %s\n", argv[1]);
    return -1;
  }
  fseek(f, 0, SEEK_END);
  size_t len = ftell(f);
  fseek(f, 0, SEEK_SET);
  ROM_OFFSET = len % 1024;
  printf("len=%d\n", len);
  printf("ROM_OFFSET=%d\n", ROM_OFFSET);
  if ((ROM_OFFSET != 0) && (ROM_OFFSET != 512)) {
    printf("error: ROM size might be wrong. Check ROM.\n");
    fclose(f);
    return -1;
  }
  
  rom = (uint8_t*)malloc(len);
  fread(rom, 1, len, f);
  fclose(f);

  if (InitApplication() < 0) {
    printf("error: InitApplication() failed.\n");
    return -1;
  }
  
  timeBeginPeriod(1);
  
  // Init
  memset(&bmi, '\0', sizeof(bmi));
  bmi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
  bmi.bmiHeader.biWidth = SNES_WIDTH;
  bmi.bmiHeader.biHeight = -SNES_HEIGHT;
  bmi.bmiHeader.biPlanes = 1;
  bmi.bmiHeader.biBitCount = 32;
  bmi.bmiHeader.biCompression = BI_RGB;
  
  // Reset Flags
  X_Flag = 1;
  M_Flag = 1;
  I_Flag = 1;
  E_Flag = 1;
  io_NMITIMEN = 0;
  ppu.BGMODE = 0;
  ppu.vram_addr = 0;
  ppu.STAT77 = 0x00;
  ppu.STAT78 = 0x81;
  ppu.scroll_value = 0;
  
  // Main loop
  Start();
  Label_Reset();
  return 0;
}

void Cleanup(void)
{
  timeEndPeriod(1);
  
  FILE *fwram = fopen("wram_dump.x", "wb");
  if (fwram) {
    fwrite(wram, 1, WRAM_SIZE, fwram);
    fclose(fwram);
    printf("DUMPED WRAM in wram_dump.x\n");
  }
  
  FILE *fsram = fopen("sram_dump.x", "wb");
  if (fsram) {
    fwrite(sram, 1, MAX_SRAM_SIZE, fsram);
    fclose(fsram);
    printf("DUMPED SRAM in sram_dump.x\n");
  }
  
  FILE *fvram = fopen("vram_dump.x", "wb");
  if (fvram) {
    fwrite(ppu.vram, 1, 64 * 1024, fvram);
    fclose(fvram);
    printf("DUMPED VRAM in vram_dump.x\n");
  }
  
  FILE *foam = fopen("oam_dump.x", "wb");
  if (foam) {
    fwrite(ppu.oam, 1, 544, foam);
    fclose(foam);
    printf("DUMPED OAM in oam_dump.x\n");
  }
  
  free(rom);
  DestroyWindow(ghwnd);
}

void DoMessages(void)
{
  MSG msg;
  memset(&msg, '\0', sizeof(msg));
  while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
    if (msg.message == WM_QUIT) {
      Cleanup();
      ExitProcess(0);
    }
    TranslateMessage(&msg); 
    DispatchMessage(&msg);
  }
}

int NMI = 0;
extern "C" void Render(void)
{
  if (NMI)
    return;
  
  if ((CycleCount >= NMI_CYCLES) || RecompBreak) {
    io.JOY1H = (input1 >> 8) & 0xFF;
    io.JOY1L = input1 & 0xFF;
    input1 = 0;
    
    draw_screen();
    // draw_vram_4bpp(0xC000);
    // draw_palette();
    
    InvalidateRect(ghwnd, NULL, TRUE);
    UpdateWindow(ghwnd);
    DoMessages();
    
    CycleCount = 0;
    NMI = 1;
  }
}

LRESULT WindowProc(HWND h, UINT m, WPARAM w, LPARAM l)
{
  switch (m) {
    case WM_CLOSE:
      PostQuitMessage(0);
      break;
    
    case WM_PAINT:
      PAINTSTRUCT ps;
      BeginPaint(ghwnd, &ps);
      
      StretchDIBits(
        ps.hdc,
        0,
        0,
        ps.rcPaint.right - ps.rcPaint.left,
        ps.rcPaint.bottom - ps.rcPaint.top,
        0,
        0,
        SNES_WIDTH,
        SNES_HEIGHT,
        (VOID*)framebuf,
        &bmi,
        DIB_RGB_COLORS,
        SRCCOPY
      );
      
      EndPaint(ghwnd, &ps);
      break;
    
    case WM_KEYDOWN:
      if (w == 'A')       input1 |= 0x0080;
      if (w == VK_RETURN) input1 |= 0x1000;
      if (w == VK_UP)     input1 |= 0x0800;
      if (w == VK_DOWN)   input1 |= 0x0400;
      if (w == VK_LEFT)   input1 |= 0x0200;
      if (w == VK_RIGHT)  input1 |= 0x0100;
      break;
    
    default:
      return DefWindowProc(h, m, w, l);
  }
  return 0;
}

void __UpdateNZ_A8(void) { Z_Flag = ((regA & 0xFF) == 0); N_Flag = (regA & 0x80) != 0; }
void __UpdateNZ_X8(void) { Z_Flag = ((regX & 0xFF) == 0); N_Flag = (regX & 0x80) != 0; }
void __UpdateNZ_Y8(void) { Z_Flag = ((regY & 0xFF) == 0); N_Flag = (regY & 0x80) != 0; }

void __UpdateNZ_A16(void) { Z_Flag = (regA == 0); N_Flag = (regA & 0x8000) != 0; }
void __UpdateNZ_X16(void) { Z_Flag = (regX == 0); N_Flag = (regX & 0x8000) != 0; }
void __UpdateNZ_Y16(void) { Z_Flag = (regY == 0); N_Flag = (regY & 0x8000) != 0; }

extern "C" uint8_t __INC8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  v++;
  Z_Flag = ((v & 0xFF) == 0);
  N_Flag = (v & 0x80) != 0;
  return v;
}
extern "C" uint16_t __INC16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  v++;
  Z_Flag = (v == 0);
  N_Flag = (v & 0x8000) != 0;
  return v;
}

extern "C" void __BIT8(uint64_t value)
{
  uint8_t A = regA & 0xFF;
  uint8_t v = value & 0xFF;
  
  // printf("__BIT8(value=%02X)\n", value);
  Z_Flag = (A & v) == 0;
  N_Flag = (v & 0x80) != 0;
  V_Flag = (v & 0x40) != 0;
}
extern "C" void __BIT16(uint64_t value)
{
  uint16_t A = regA & 0xFFFF;
  uint16_t v = value & 0xFFFF;
  
  Z_Flag = (A & v) == 0;
  N_Flag = (v & 0x8000) != 0;
  V_Flag = (v & 0x4000) != 0;
}

extern "C" uint64_t __TSB(uint64_t value)
{
  if (M_Flag) {
    uint8_t A = regA & 0xFF;
    uint8_t v = value & 0xFF;
    Z_Flag = (v & A) == 0;
    v |= A;
    return v;
  } else {
    uint16_t A = regA;
    uint16_t v = value & 0xFFFF;
    Z_Flag = (v & A) == 0;
    v |= A;
    return v;
  }
}
extern "C" uint64_t __TRB(uint64_t value)
{
  if (M_Flag) {
    uint8_t A = regA & 0xFF;
    uint8_t v = value & 0xFF;
    Z_Flag = (v & A) == 0;
    v &= ~A;
    return v;
  } else {
    uint16_t A = regA;
    uint16_t v = value & 0xFFFF;
    Z_Flag = (v & A) == 0;
    v &= ~A;
    return v;
  }
}

extern "C" void __TESTNZ8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  Z_Flag = (v == 0);
  N_Flag = (v & 0x80) != 0;
}

extern "C" void __TESTNZ16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  Z_Flag = (v == 0);
  N_Flag = (v & 0x8000) != 0;
}

extern "C" void __PHP(void)
{
  uint8_t P = 0;
  P |= N_Flag << 7;
  P |= V_Flag << 6;
  P |= M_Flag << 5;
  P |= X_Flag << 4;
  P |= D_Flag << 3;
  P |= I_Flag << 2;
  P |= Z_Flag << 1;
  P |= C_Flag << 0;
  __PUSH8(P);
}

extern "C" void __PLP(void)
{
  uint8_t P = __PULL8();
  
  N_Flag = (P >> 7) & 1;
  V_Flag = (P >> 6) & 1;
  M_Flag = (P >> 5) & 1;
  X_Flag = (P >> 4) & 1;
  D_Flag = (P >> 3) & 1;
  I_Flag = (P >> 2) & 1;
  Z_Flag = (P >> 1) & 1;
  C_Flag = (P >> 0) & 1;
}

extern "C" void __MVN(uint64_t bankS, uint64_t bankD)
{
  bankS &= 0xFF;
  bankD &= 0xFF;
  regDBR = bankD;
  do {
    CycleCount += 7;
    uint8_t b = __READ8((bankS << 16) | regX);
    __WRITE8((bankD << 16) | regY, b);
    
    regX++;
    regY++;
    if (X_Flag) {
      regX &= 0xFF;
      regY &= 0xFF;
    }
    
    regA--;
  } while (regA != 0xFFFF);
}
extern "C" void __MVP(uint64_t bankS, uint64_t bankD)
{
  bankS &= 0xFF;
  bankD &= 0xFF;
  regDBR = bankD;
  do {
    CycleCount += 7;
    uint8_t b = __READ8((bankS << 16) | regX);
    __WRITE8((bankD << 16) | regY, b);
    
    regX--;
    regY--;
    if (X_Flag) {
      regX &= 0xFF;
      regY &= 0xFF;
    }
    
    regA--;
  } while (regA != 0xFFFF);
}

extern "C" uint8_t __ADC8(uint64_t value1, uint64_t value2)
{
  uint8_t v1 = value1 & 0xFF;
  uint8_t v2 = value2 & 0xFF;

  uint8_t carry = C_Flag ? 1 : 0;
  uint16_t result = 0;
  if (D_Flag) {
    uint16_t res = 0;
    res += (v1 & 0xF) + (v2 & 0xF) + carry;
    if (res >= 0xA) res += 0x6;
    res += (v1 & 0xF0) + (v2 & 0xF0);
    if ((res & 0xF0) >= 0xA0) res += 0x60;
    result = res;
  } else {
    result = (uint16_t)(v1 + v2 + carry);
  }
  
  C_Flag = result > 0xFF;
  Z_Flag = (uint8_t)result == 0;
  N_Flag = (result & 0x80) != 0;
  V_Flag = !((v1 ^ v2) & 0x80) && ((v1 ^ result) & 0x80); // Overflow check
  return (uint8_t)result;
}

extern "C" uint16_t __ADC16(uint64_t value1, uint64_t value2)
{
  uint16_t v1 = value1 & 0xFFFF;
  uint16_t v2 = value2 & 0xFFFF;
  
  uint16_t carry = C_Flag ? 1 : 0;
  uint32_t result = 0;
  if (D_Flag) {
    uint32_t res = 0;
    res += (v1 & 0xF) + (v2 & 0xF) + carry;
    if (res >= 0xA) res += 0x6;
    res += (v1 & 0xF0) + (v2 & 0xF0);
    if ((res & 0xF0) >= 0xA0) res += 0x60;
    res += (v1 & 0xF00) + (v2 & 0xF00);
    if ((res & 0xF00) >= 0xA00) res += 0x600;
    res += (v1 & 0xF000) + (v2 & 0xF000);
    if ((res & 0xF000) >= 0xA000) res += 0x6000;
    result = res;
  } else {
    result = (uint32_t)(v1 + v2 + carry);
  }
  
  C_Flag = result > 0xFFFF;
  Z_Flag = (uint16_t)result == 0;
  N_Flag = (result & 0x8000) != 0;
  V_Flag = !((v1 ^ v2) & 0x8000) && ((v1 ^ result) & 0x8000); // Overflow check
  return (uint16_t)result;
}

extern "C" uint8_t __SBC8(uint64_t value1, uint64_t value2)
{
  uint8_t v1 = value1 & 0xFF;
  uint8_t v2 = (value2 & 0xFF) ^ 0xFF;
  
  uint8_t carry = C_Flag ? 1 : 0;
  int result = 0;
  if (D_Flag) {
    // code from LakeSnes
    result = (v1 & 0xf) + (v2 & 0xf) + carry;
    if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
    result = (v1 & 0xf0) + (v2 & 0xf0) + result;
  } else {
    result = v1 + v2 + carry;
  }
  
  V_Flag = (v1 & 0x80) == (v2 & 0x80) && (v2 & 0x80) != (result & 0x80);
  if (D_Flag && result < 0x100) result -= 0x60;
  C_Flag = result >= 0x100;
  Z_Flag = (uint8_t)result == 0;
  N_Flag = (result & 0x80) != 0;
  return (uint8_t)result;
}

extern "C" uint16_t __SBC16(uint64_t value1, uint64_t value2)
{
  uint16_t v1 = value1 & 0xFFFF;
  uint16_t v2 = (value2 & 0xFFFF) ^ 0xFFFF;
  
  uint8_t carry = C_Flag ? 1 : 0;
  int result = 0;
  if (D_Flag) {
    // code from LakeSnes
    result = (v1 & 0xf) + (v2 & 0xf) + carry;
    if(result < 0x10) result = (result - 0x6) & ((result - 0x6 < 0) ? 0xf : 0x1f);
    result = (v1 & 0xf0) + (v2 & 0xf0) + result;
    if(result < 0x100) result = (result - 0x60) & ((result - 0x60 < 0) ? 0xff : 0x1ff);
    result = (v1 & 0xf00) + (v2 & 0xf00) + result;
    if(result < 0x1000) result = (result - 0x600) & ((result - 0x600 < 0) ? 0xfff : 0x1fff);
    result = (v1 & 0xf000) + (v2 & 0xf000) + result;
  } else {
    result = v1 + v2 + carry;
  }
  
  V_Flag = (v1 & 0x8000) == (v2 & 0x8000) && (v2 & 0x8000) != (result & 0x8000);
  if (D_Flag && (result < 0x10000)) result -= 0x6000;
  C_Flag = result >= 0x10000;
  Z_Flag = (uint16_t)result == 0;
  N_Flag = (result & 0x8000) != 0;
  return (uint16_t)result;
}

extern "C" uint8_t __ASL8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  uint8_t result = v << 1;
  
  C_Flag = (v & 0x80) != 0;
  Z_Flag = (result == 0);
  N_Flag = (result & 0x80) != 0;
  return result;
}

extern "C" uint16_t __ASL16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  uint16_t result = v << 1;
  
  C_Flag = (v & 0x8000) != 0;
  Z_Flag = (result == 0);
  N_Flag = (result & 0x8000) != 0;
  return result;
}

extern "C" uint8_t __LSR8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  uint8_t result = v >> 1;
  
  C_Flag = v & 1;
  Z_Flag = (result == 0);
  N_Flag = (result & 0x80) != 0;
  return result;
}

extern "C" uint16_t __LSR16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  uint16_t result = v >> 1;
  
  C_Flag = v & 1;
  Z_Flag = (result == 0);
  N_Flag = (result & 0x8000) != 0;
  return result;
}

extern "C" uint8_t __ROL8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  int c = (v & 0x80) != 0;
  uint16_t result = v << 1;
  
  if (C_Flag) { result |= 1; }
  C_Flag = c;
  Z_Flag = (uint8_t)result == 0;
  N_Flag = (result & 0x80) != 0;
  return (uint8_t)result;
}
extern "C" uint16_t __ROL16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  int c = (v & 0x8000) != 0;
  uint32_t result = v << 1;
  
  if (C_Flag) { result |= 1; }
  C_Flag = c;
  Z_Flag = (uint16_t)result == 0;
  N_Flag = (result & 0x8000) != 0;
  return (uint16_t)result;
}

extern "C" uint8_t __ROR8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  int c = value & 1;
  uint16_t result = v >> 1;
  
  if (C_Flag) { result |= 0x80; }
  C_Flag = c;
  Z_Flag = (uint8_t)result == 0;
  N_Flag = (result & 0x80) != 0;
  return (uint8_t)result;
}
extern "C" uint16_t __ROR16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  int c = v & 1;
  uint32_t result = v >> 1;
  
  if (C_Flag) { result |= 0x8000; }
  C_Flag = c;
  Z_Flag = (uint16_t)result == 0;
  N_Flag = (result & 0x8000) != 0;
  return (uint16_t)result;
}

uint8_t read8_wram_LoROM(uint64_t addr, int *read);
uint8_t read8_wram_HiROM(uint64_t addr, int *read);

uint8_t read8_wram(uint64_t addr, int *read)
{
  *read = 0;
  switch (MapMode) {
    case 0: return read8_wram_LoROM(addr, read);
    case 1: return read8_wram_HiROM(addr, read);
    default:
      *read = 0;
  }
  return 0;
}

uint8_t read8_wram_LoROM(uint64_t addr, int *read)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  uint32_t address = addr & 0xFFFFFF;
  
  if (((bank >= 0x00 && bank < 0x40) ||
      (bank >= 0x80 && bank <= 0xC0)) &&
      offset < 0x2000) {
    // Lo RAM
    *read = 1;
    return wram[offset];
  } else
  if ((bank == 0x7E) || (bank == 0x7F)) {
    // WRAM
    *read = 1;
    return wram[((bank - 0x7E) << 16) | offset];
  } else
  if ((bank >= 0x70) && (bank < 0x7E) && (offset < 0x8000)) {
    // SRAM
    *read = 1;
    return sram[offset];
  }
  return 0;
}

uint8_t read8_wram_HiROM(uint64_t addr, int *read)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  uint32_t address = (addr & 0xFFFFFF);
  
  *read = 0;
  if (((bank < 0x40) || (bank >= 0x80 && bank <= 0xC0))
      && offset < 0x2000) { // Lo RAM
    // printf("__READ8(addr=%04X)\n", address);
    *read = 1;
    return wram[offset];
  } else
  if ((bank == 0x7E) || (bank == 0x7F)) {
    // WRAM
    *read = 1;
    return wram[((bank - 0x7E) << 16) | offset];
  } else
  if ((bank >= 0x30) && (bank < 0x40) && (offset >= 0x6000) && (offset < 0x8000)) {
    // SRAM
    *read = 1;
    return sram[offset - 0x6000];
  }
  return 0;
}

uint8_t read8_rom_LoROM(uint64_t addr, int* read);
uint8_t read8_rom_HiROM(uint64_t addr, int* read);

static uint8_t read8_rom(uint64_t addr, int* read)
{
  switch (MapMode) {
    case 0: return read8_rom_LoROM(addr, read);
    case 1: return read8_rom_HiROM(addr, read);
    default:
      *read = 0;
  }
  return 0;
}

uint8_t read8_rom_LoROM(uint64_t addr, int* read)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  uint32_t address = (addr & 0xFFFFFF);
  
  if (((bank >= 0x00 && bank < 0x7E) || (bank >= 0x80 && bank <= 0xFF)) &
      (address & 0xFFFF) >= 0x8000) {
    // ROM
    *read = 1;
    uint8_t ret = rom[(address - 0x8000) + ROM_OFFSET];
    return ret;
  }
  return 0;
}

uint8_t read8_rom_HiROM(uint64_t addr, int* read)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  uint32_t rom_offset = 0;
  
  *read = 0;
  if (bank < 0x40) {
    // ROM
    if (offset >= 0x8000) {
      *read = 1;
      rom_offset = (bank * 0x10000) + offset;
      return rom[rom_offset % 0x400000];
    }
    return 0;
  } else
  if (bank >= 0x40 && bank < 0x7E) {
    // ROM
    *read = 1;
    rom_offset = (bank * 0x10000) + offset;
    return rom[rom_offset % 0x400000];
  } else
  if (bank >= 0x80 && bank < 0xC0) {
    // ROM
    if (offset >= 0x8000) {
      *read = 1;
      rom_offset = ((bank - 0x80) * 0x10000) + offset;
      return rom[rom_offset % 0x400000];
    }
    return 0;
  } else
  if (bank >= 0xC0 && bank <= 0xFF) {
    // ROM
    *read = 1;
    rom_offset = ((bank - 0xC0) * 0x10000) + offset;
    return rom[rom_offset % 0x400000];
  }
  return 0;
}

extern "C" uint8_t __READ8(uint64_t addr)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  uint32_t address = addr & 0xFFFFFF;
  
  // printf("__READ8(addr=%04X)\n", addr);
  if (addr == 0x213E /* STAT77 */) {
    return ppu.STAT77;
  } else
  if (addr == 0x213F /* STAT78 */) {
    return ppu.STAT78;
  } else
  if (addr == 0x2140 /* APUIO0 */) {
    return APUIO0;
  } else
  if (addr == 0x2141 /* APUIO1 */) {
    return APUIO1;
  } else
  if (addr == 0x4214 /* RDDIVL */) {
    return io.RDDIVL;
  } else
  if (addr == 0x4215 /* RDDIVH */) {
    return io.RDDIVH;
  } else
  if (addr == 0x4216 /* RDMPYL */) {
    return io.RDMPYL;
  } else
  if (addr == 0x4217 /* RDMPYH */) {
    return io.RDMPYH;
  } else
  if (addr == 0x4210 /* RDNMI */) {
    uint8_t rdnmi = io_RDNMI;
    io_RDNMI &= 0x7F;
    return rdnmi;
  } else
  if (addr == 0x4212 /* HVBJOY */) {
    return io.HVBJOY;
  } else
  if (addr == 0x4219 /* JOY1H */) {
    return io.JOY1H;
  } else
  if (addr == 0x4218 /* JOY1L */) {
    return io.JOY1L;
  }
  
  int read = 0;
  uint8_t value = read8_rom(addr, &read);
  if (read) {
    return value;
  }
  
  value = read8_wram(addr, &read);
  if (read) {
    return value;
  }
  return 0;
}

extern "C" uint16_t __READ16(uint64_t addr)
{
  uint32_t address = addr & 0xFFFFFF;
  uint16_t ret = 0;
  
  ret |= (__READ8(addr+1) & 0xFF) << 8;
  ret |= __READ8(addr) & 0xFF;
  return ret;
}

extern "C" uint32_t __READ24(uint64_t addr)
{
  uint32_t address = addr & 0xFFFFFF;
  uint32_t ret = 0;
  
  ret |= (__READ8(addr+2) & 0xFF) << 16;
  ret |= (__READ8(addr+1) & 0xFF) << 8;
  ret |= __READ8(addr) & 0xFF;
  // printf("__READ24(addr=%06X)=%06X\n", addr, ret);
  return ret;
}

extern "C" uint32_t __READ_INS(uint64_t addr)
{
  uint32_t address = addr & 0xFFFFFF;
  uint32_t ret = 0;
  
  ret |= (__READ8(addr) & 0xFF) << 24;
  ret |= (__READ8(addr+1) & 0xFF) << 16;
  ret |= (__READ8(addr+2) & 0xFF) << 8;
  ret |= __READ8(addr+3) & 0xFF;
  // printf("__READ_INS(%06X)=%08X\n", addr, ret);
  return ret;
}

int write8_sram_HiROM(uint64_t addr, uint8_t value)
{
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;

  if (offset >= 0x6000 && offset < 0x8000) {
    if (bank >= 0x20 && bank < 0x40) {
      sram[offset - 0x6000] = value;
      return 1;
    } else
    if (bank >= 0xA0 && bank < 0xC0) {
      sram[offset - 0x6000] = value;
      return 1;
    }
    return 0;
  }
  return 0;
}

extern "C" void __WRITE8(uint64_t addr, uint64_t value)
{
  uint8_t v = value & 0xFF;
  uint8_t bank = (addr & 0xFF0000) >> 16;
  uint16_t offset = addr & 0xFFFF;
  
  // if (offset == 0x210D)
    // printf("__WRITE8(addr=0x%06X, value=0x%02X)\n", addr, v);
  
  if (write8_sram_HiROM(addr, v)) {
    return;
  } else
  if (((bank < 0x40) || (bank >= 0x80 && bank < 0xC0)) && offset < 0x2000) {
    // Low RAM
    wram[offset] = v;
  } else
  if ((bank == 0x7E) || (bank == 0x7F)) {
    // WRAM
    wram[((bank - 0x7E) << 16) | offset] = v;
    return;
  }
  // ------------ MMIO Registers -----------
  if (addr == 0x4200 /* NMITIMEN */) {
    io_NMITIMEN = v;
    // printf("WROTE %02X TO NMITIMEN\n", v);
  } else
  if (addr == 0x4202 /* WRMPYA */) {
    io.WRMPYA = v;
  } else
  if (addr == 0x4203 /* WRMPYB */) {
    io.WRMPYB = v;
    multiply_5A22(&io);
  } else
  if (addr == 0x4204 /* WRDIVL */) {
    io.WRDIVL = v;
  } else
  if (addr == 0x4205 /* WRDIVH */) {
    io.WRDIVH = v;
  } else
  if (addr == 0x4206 /* WRDIVB */) {
    io.WRDIVB = v;
    divide_5A22(&io);
  } else // ------------ PPU Registers ------------
  if (addr == 0x2100 /* INIDISP */) {
    ppu.INIDISP = v;
  } else
  if (addr == 0x2121 /* CGADD */) {
    // printf("WRITE8(%02X) to CGADD\n", v);
    ppu.CGADD = v * 2;
  } else
  if (addr == 0x2122 /* CGDATA */) {
    write_cgram(&ppu, value);
    // printf("WRITE8(%02X) to CGDATA\n", value & 0xFF);
  } else
  if (addr == 0x2105 /* BGMODE */) {
    ppu.BGMODE = v;
  } else
  if (addr == 0x2107 /* BG1SC */) {
    ppu.BG1SC = v;
  } else
  if (addr == 0x2108 /* BG2SC */) {
    ppu.BG2SC = v;
  } else
  if (addr == 0x2109 /* BG3SC */) {
    ppu.BG3SC = v;
  } else
  if (addr == 0x210B /* BG12NBA */) {
    ppu.BG12NBA = v;
  } else
  if (addr == 0x210C /* BG34NBA */) {
    ppu.BG34NBA = v;
  } else
  if (addr == 0x210D /* BG1HOFS */) {
    if (ppu.scroll_value) {
      ppu.BG1HOFS = ((v & 3) << 8) | (ppu.BG1HOFS & 0xFF);
    } else {
      ppu.BG1HOFS = (ppu.BG1HOFS & 0xFF00) | v;
    }
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x210E /* BG1VOFS */) {
    if (ppu.scroll_value) {
      ppu.BG1VOFS = ((v & 3) << 8) | (ppu.BG1VOFS & 0xFF);
    } else {
      ppu.BG1VOFS = (ppu.BG1VOFS & 0xFF00) | v;
    }
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x210F /* BG2HOFS */) {
    if (ppu.scroll_value) {
      ppu.BG2HOFS = ((v & 3) << 8) | (ppu.BG2HOFS & 0xFF);
    } else {
      ppu.BG2HOFS = (ppu.BG2HOFS & 0xFF00) | v;
    }
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2110 /* BG2VOFS */) {
    if (ppu.scroll_value) {
      ppu.BG2VOFS = ((v & 3) << 8) | (ppu.BG2VOFS & 0xFF);
    } else {
      ppu.BG2VOFS = (ppu.BG2VOFS & 0xFF00) | v;
    }
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2111 /* BG3HOFS */) {
    ppu.BG3HOFS = v;
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2112 /* BG3VOFS */) {
    ppu.BG3VOFS = v;
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2113 /* BG4HOFS */) {
    ppu.BG4HOFS = v;
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2114 /* BG4VOFS */) {
    ppu.BG4VOFS = v;
    ppu.scroll_value ^= 1;
  } else
  if (addr == 0x2101 /* OBJSEL */) {
    ppu.OBJSEL = v;
    // printf("ppu.OBJSEL=%02X\n", v);
  } else
  if (addr == 0x2102 /* OAMADDL */) {
    ppu.OAMADDL = v;
  } else
  if (addr == 0x2103 /* OAMADDH */) {
    ppu.OAMADDH = v;
  } else
  if (addr == 0x2104 /* OAMDATA */) {
    write_oam(&ppu, v);
  } else
  if (addr == 0x2115 /* VMAIN */) {
    ppu.VMAIN = v;
  } else
  if (addr == 0x2116 /* VMADDL */) {
    printf("WRITE8(%02X) to VMADDL\n", v);
    ppu.VMADDL = v;
    ppu.vram_addr = (ppu.VMADDH << 8) | ppu.VMADDL;
    ppu.vram_addr &= 0x7FFF;
    ppu.vram_addr *= 2;
    // printf("ppu.vram_addr=%04X\n", ppu.vram_addr);
  } else
  if (addr == 0x2117 /* VMADDH */) {
    printf("WRITE8(%02X) to VMADDH\n", v);
    ppu.VMADDH = v;
    ppu.vram_addr = (ppu.VMADDH << 8) | ppu.VMADDL;
    ppu.vram_addr &= 0x7FFF;
    ppu.vram_addr *= 2;
    // printf("ppu.vram_addr=%04X\n", ppu.vram_addr);
  } else
  if (addr == 0x212C /* TM */) {
    ppu.TM = v;
  } else
  if (addr == 0x2118 /* VMDATAL */) {
    // printf("WRITE8(%02X) to VMDATAL\n", v);
    write_vram(&ppu, v, 0, (ppu.VMAIN & 0x80) == 0);
  } else
  if (addr == 0x2119 /* VMDATAH */) {
    // printf("WRITE8(%02X) to VMDATAH\n", v);
    write_vram(&ppu, v, 1, (ppu.VMAIN & 0x80) != 0);
  }
  write_dma(&dma, addr, v);
}

extern "C" void __WRITE16(uint64_t addr, uint64_t value)
{
  // printf("__WRITE16(addr=0x%04X, value=0x%04X)\n", addr, value);
  __WRITE8(addr, value & 0xFF);
  __WRITE8(addr + 1, (value >> 8) & 0xFF);
}

extern "C" void __REP(uint64_t mask)
{
  if (mask & 0x01) C_Flag = 0;
  if (mask & 0x02) Z_Flag = 0;
  if (mask & 0x04) I_Flag = 0;
  if (mask & 0x08) D_Flag = 0;
  if (!E_Flag) {
    if (mask & 0x10) X_Flag = 0;
    if (mask & 0x20) M_Flag = 0;
  }
  if (mask & 0x40) V_Flag = 0;
  if (mask & 0x80) N_Flag = 0;
}

extern "C" void __SEP(uint64_t mask)
{
  // printf("__SEP(mask=%02X)\n", mask);
  if (mask & 0x01) C_Flag = 1;
  if (mask & 0x02) Z_Flag = 1;
  if (mask & 0x04) I_Flag = 1;
  if (mask & 0x08) D_Flag = 1;
  if (!E_Flag) {
    if (mask & 0x10) X_Flag = 1;
    if (mask & 0x20) M_Flag = 1;
  }
  if (mask & 0x40) V_Flag = 1;
  if (mask & 0x80) N_Flag = 1;
  if (X_Flag) {
    regY &= 0xFF;
    regX &= 0xFF;
  }
}

extern "C" void __REG_DUMP(void)
{
  uint8_t P = 0;
  P |= N_Flag << 7;
  P |= V_Flag << 6;
  P |= M_Flag << 5;
  P |= X_Flag << 4;
  P |= D_Flag << 3;
  P |= I_Flag << 2;
  P |= Z_Flag << 1;
  P |= C_Flag << 0;
  
  printf("              __REG_DUMP\n");
  printf("regA  =%04X regX=%04X regY=%04X\n", regA, regX, regY);
  printf("regS  =%04X\n", regS);
  printf("regDP =%04X\n", regDP);
  printf("regDBR=%02X\n", regDBR);
  printf("P     =%02X\n", P);
  printf("CycleCount=%d\n", CycleCount);
  printf("\n             STACK DUMP\n");
  for (int i = 0; i < 48; ++i) {
    int c = wram[regS + i + 1];
    printf("%02X ", c);
    if ((i + 1) % 16 == 0) {
      printf("\n");
    }
  }
  printf("\n");
  // RecompBreak = 1;
  // for (;;)
    // ;
}

extern "C" void __PUSH8(uint64_t value)
{
  uint8_t v = value & 0xFF;
  __WRITE8(regS--, v);
}

extern "C" void __PUSH16(uint64_t value)
{
  uint16_t v = value & 0xFFFF;
  __PUSH8(v >> 8);
  __PUSH8(v);
}

extern "C" uint8_t __PULL8(void)
{
  regS++;
  uint8_t ret = __READ8(regS);
  // printf("__PULL8()=%02X\n", ret);
  
  // N_Flag = (ret & 0x80) != 0;
  // Z_Flag = (ret == 0);
  return ret;
}

extern "C" uint16_t __PULL16(void)
{
  uint16_t ret = 0;
  ret |= __READ8(++regS);
  ret |= __READ8(++regS) << 8;
  // printf("__PULL16()=%04X\n", ret);
  
  // N_Flag = (ret & 0x8000) != 0;
  // Z_Flag = (ret == 0);
  return ret;
}

extern "C" void __PRINT_INS(uint64_t pc, uint64_t ins)
{
  printf("\nUNKNOWN INSTRUCTION %08X AT %06X\n\n", ins, pc);
  __REG_DUMP();
}

extern "C" void __JUMP_FAILED(uint64_t target, uint64_t pc)
{
  printf("\nFAILED TO JUMP TO ADDRESS %06X AT %06X\n\n", target, pc);
  __REG_DUMP();
  Cleanup();
  ExitProcess(0);
}

extern "C" void __CALL_SHOW(uint64_t addr, uint64_t pc)
{
  // printf("__CALL_ADDRESS(addr=%06X) FROM %06X\n", addr, pc);
}

extern "C" void __WAI(void)
{
  // stub
}

extern "C" void __COMPARE8(uint64_t value1, uint64_t value2)
{
  uint8_t v1 = value1 & 0xFF;
  uint8_t v2 = value2 & 0xFF;
  uint8_t result = v1 - v2;
  // printf("__COMPARE8(value1=%04X, value2=%04X)\n", value1, value2);
  
  Z_Flag = (result == 0);
  C_Flag = (v1 >= v2);
  N_Flag = (result & 0x80) != 0;
}

extern "C" void __COMPARE16(uint64_t value1, uint64_t value2)
{
  uint16_t v1 = value1 & 0xFFFF;
  uint16_t v2 = value2 & 0xFFFF;
  uint16_t result = v1 - v2;
  
  // printf("__COMPARE16(value1=%04X, value2=%04X)\n", value1, value2);
  
  Z_Flag = (result == 0);
  C_Flag = (v1 >= v2);
  N_Flag = (result & 0x8000) != 0;
}

extern "C" void __XCE(void)
{
  uint8_t tmp_Flag = C_Flag;
  C_Flag = E_Flag;
  E_Flag = tmp_Flag;
  if (E_Flag) {
    M_Flag = 1;
    X_Flag = 1;
    regX &= 0xFF;
    regY &= 0xFF;
    regS = 0x100 | (regS & 0xFF);
  }
}

extern "C" void __WDM(uint8_t v)
{
  // printf("__WDM()\n");
}

void draw_palette(void)
{
  for (int i = 0; i < 256; ++i) {
    int idx = i * 2;
    uint32_t color = snes_color_to_rgb(ppu.cgram[idx]+ppu.cgram[idx+1]*256);
    framebuf[(i % 16) + SNES_WIDTH * (i / 16)] = color;
  }
}

void draw_sprite_8x8_8bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip)
{
  int tile = tile_index;
  
  palindex *= 256;
  for (int y = 0; y < 8; ++y) {
    uint8_t p0 = ppu.vram[tile + y*2 + 0];
    uint8_t p1 = ppu.vram[tile + y*2 + 1];
    uint8_t p2 = ppu.vram[tile + 16 + y*2 + 0];
    uint8_t p3 = ppu.vram[tile + 16 + y*2 + 1];
    uint8_t p4 = ppu.vram[tile + 32 + y*2 + 0];
    uint8_t p5 = ppu.vram[tile + 32 + y*2 + 1];
    uint8_t p6 = ppu.vram[tile + 48 + y*2 + 0];
    uint8_t p7 = ppu.vram[tile + 48 + y*2 + 1];
    int dy = py + y;
    if (Vflip) {
      dy = py + (7 - y);
    }
    
    if ((dy < 0) || dy >= SNES_HEIGHT) continue;
    for (int x = 0; x < 8; x++) {
      int dx = px + x;
      if ((dx < 0) || dx >= SNES_WIDTH) continue;
      
      int bit = Hflip ? x : (7 - x);
      uint8_t pindex =
        ((p0 >> bit) & 1) |
        (((p1 >> bit) & 1) << 1) |
        (((p2 >> bit) & 1) << 2) |
        (((p3 >> bit) & 1) << 3) |
        (((p4 >> bit) & 1) << 4) |
        (((p5 >> bit) & 1) << 5) |
        (((p6 >> bit) & 1) << 6) |
        (((p7 >> bit) & 1) << 7);
      
      uint16_t color16_0 = ppu.cgram[palindex + 2 * pindex];
      uint16_t color16_1 = ppu.cgram[palindex + 2 * pindex + 1];
      uint32_t color32 = snes_color_to_rgb(color16_0+color16_1*256);
      if (pindex != 0) {
        framebuf[dx + SNES_WIDTH * dy] = color32;
      }
    }
  }
}

uint8_t dump_palette[] = {
  // Some funky SNES Palette
  0x00, 0x00, 
  0x08, 0x08, 
  0x10, 0x10, 
  0x18, 0x18, 
  0x20, 0x20, 
  0x28, 0x28, 
  0x30, 0x30, 
  0x38, 0x38, 
  0x3F, 0x3F, 
  0x47, 0x47, 
  0x4F, 0x4F, 
  0x57, 0x57,
  0x5F, 0x5F,
  0x67, 0x67,
  0x6F, 0x6F,
  0x7F, 0x7F
};

void draw_sprite_8x8_4bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip)
{
  int tile = tile_index;
  
  palindex *= 0x20;
  for (int y = 0; y < 8; ++y) {
    uint8_t p0 = ppu.vram[tile + y * 2 + 0];      // bitplane 0
    uint8_t p1 = ppu.vram[tile + y * 2 + 1];      // bitplane 1
    uint8_t p2 = ppu.vram[tile + 16 + y * 2 + 0]; // bitplane 2
    uint8_t p3 = ppu.vram[tile + 16 + y * 2 + 1]; // bitplane 3
    int dy = py + y;
    if (Vflip) {
      dy = py + (7 - y);
    }
    
    if ((dy < 0) || dy >= SNES_HEIGHT) continue;
    for (int x = 0; x < 8; x++) {
      int dx = px + x;
      if ((dx < 0) || dx >= SNES_WIDTH) continue;
      
      int bit = Hflip ? x : (7 - x);
      uint8_t pindex =
        ((p0 >> bit) & 1) |
        (((p1 >> bit) & 1) << 1) |
        (((p2 >> bit) & 1) << 2) |
        (((p3 >> bit) & 1) << 3);
      
      uint16_t color16_0;
      uint16_t color16_1;
      uint32_t color32;
      if (palindex < 0) {
        color16_0 = dump_palette[pindex];
        color16_1 = dump_palette[pindex + 1];
        color32 = snes_color_to_rgb(color16_0+color16_1*256);
      } else {
        color16_0 = ppu.cgram[palindex + 2 * pindex];
        color16_1 = ppu.cgram[palindex + 2 * pindex + 1];
        color32 = snes_color_to_rgb(color16_0+color16_1*256);
      }
      if (pindex != 0) {
        framebuf[dx + SNES_WIDTH * dy] = color32;
      }
    }
  }
}

void draw_sprite_8x8_2bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip)
{
  int tile = tile_index;
  
  palindex *= 8;
  for (int y = 0; y < 8; ++y) {
    int p = tile + 2 * y;
    if (p > 0xFFEF) { p = 0; }
    uint8_t p0 = ppu.vram[p];
    uint8_t p1 = ppu.vram[p + 1];
    int dy = py + y;
    if (Vflip) {
      dy = py + (7 - y);
    }
    
    if ((dy < 0) || dy >= SNES_HEIGHT) continue;
    for (int x = 0; x < 8; x++) {
      int dx = px + x;
      if ((dx < 0) || dx >= SNES_WIDTH) continue;
      
      int bit = Hflip ? x : (7 - x);
      uint8_t pindex =
        ((p0 >> bit) & 1) |
        (((p1 >> bit) & 1) << 1);
      
      uint16_t color16_0 = ppu.cgram[palindex + 2 * pindex];
      uint16_t color16_1 = ppu.cgram[palindex + 2 * pindex + 1];
      uint32_t color32 = snes_color_to_rgb(color16_0|(color16_1<<8));
      int pos = dx + SNES_WIDTH * dy;
      if (pindex != 0) {
        framebuf[pos] = color32;
      }
    }
  }
}

void draw_vram_2bpp(int offset)
{
  for (int i = 0; i < 1024; ++i) {
    int idx = offset + i*16;
    draw_sprite_8x8_2bpp((i%16)*8, (i/16)*8, idx, 0, 0, 0);
  }
}

void draw_vram_4bpp(int offset)
{
  for (int i = 0; i < 1024; ++i) {
    int idx = offset + i * 32;
    draw_sprite_8x8_4bpp((i%16)*8, (i/16)*8, idx, -1, 0, 0);
  }
}

void draw_screen(void)
{
  // clear framebuf
  int len = SNES_WIDTH * SNES_HEIGHT;
  for (int i = 0; i < len; ++i) {
    framebuf[i] = snes_color_to_rgb(ppu.cgram[0]|(ppu.cgram[1]<<8));
  }
  
  // printf("ppu.BGMODE=%d\n", ppu.BGMODE);
  switch (ppu.BGMODE & 7) {
    case 0: draw_mode0(); break;
    case 1: draw_mode1(); break;
    case 3: draw_mode3(); break;
    default:
      break;
  }
  if (ppu.TM & 0x10) draw_oam();
  
  // apply brightness
  for (int i = 0; i < len; ++i) {
    float brightness = (float)(ppu.INIDISP & 0xF) / 15.0;
    uint8_t r = ((framebuf[i] >> 24)  & 0xFF) * brightness;
    uint8_t g = ((framebuf[i] >> 16) & 0xFF) * brightness;
    uint8_t b = ((framebuf[i] >> 8)  & 0xFF) * brightness;
    uint8_t a = ((framebuf[i] >> 0)  & 0xFF) * brightness;
    framebuf[i] = (r << 24) | (g << 16) | (b << 8) | a;
  }
}

void draw_sprite_16x16_4bpp(int px, int py, int tile_index, int palindex, int Hflip, int Vflip)
{
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      int _x = (Hflip) ? (1 - x) : x;
      int _y = (Vflip) ? (1 - y) : y;
      int dx = px + _x * 8;
      int dy = py + _y * 8;
      int tile = tile_index + (x + y * 16) * 0x20;
      draw_sprite_8x8_4bpp(dx, dy, tile, palindex, Hflip, Vflip);
    }
  }
}

void draw_oam(void)
{
  int objsize = (ppu.OBJSEL >> 5) & 7;
  int nb_sel = (ppu.OBJSEL >> 3) & 3;
  int nb_addr = (ppu.OBJSEL & 7) << 14;
  
  for (int i = 0; i < 128; ++i) {
    int base = i * 4;
    int x = ppu.oam[base + 0];
    int y = ppu.oam[base + 1] + 1;
    int attr = ppu.oam[base + 3];
    int palette = (attr >> 1) & 7;
    int hflip = (attr >> 6) & 1;
    int vflip = (attr >> 7) & 1;
    int name_table = attr & 1;
    int x_hi = (ppu.oam[512 + i / 4] >> (i % 4) * 2) & 1;
    int s = (ppu.oam[512 + i / 4] >> ((i % 4) * 2 + 1)) & 1;
    x = (x_hi << 8) | x;
    int tile = ppu.oam[base + 2] + (name_table * 0x100);
    if (objsize == 0) {
      if (s == 0) {
        draw_sprite_8x8_4bpp(x, y, nb_addr + (tile * 0x20), 8 + palette, hflip, vflip);
      } else {
        draw_sprite_8x8_4bpp(x,     y,     nb_addr + (tile        * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x + 8, y,     nb_addr + ((tile + 1)  * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x,     y + 8, nb_addr + ((tile + 16) * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x + 8, y + 8, nb_addr + ((tile + 17) * 0x20), 8 + palette, hflip, vflip);
      }
    } else
    if (objsize == 3) {
      if (s == 0) {
        draw_sprite_16x16_4bpp(x, y, nb_addr + (tile * 0x20), 8 + palette, hflip, vflip);
      } else {
        draw_sprite_8x8_4bpp(x,     y,     nb_addr + (tile        * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x + 8, y,     nb_addr + ((tile + 1)  * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x,     y + 8, nb_addr + ((tile + 16) * 0x20), 8 + palette, hflip, vflip);
        draw_sprite_8x8_4bpp(x + 8, y + 8, nb_addr + ((tile + 17) * 0x20), 8 + palette, hflip, vflip);
      }
    }
  }
}

void draw_mode0(void)
{
  // Draw BG1
  int bg1_base = ((((ppu.BG1SC >> 2) & 0x3F) << 10) * 2) & 0xFFFF;
  int bg1_chrbase = ((ppu.BG12NBA & 0xF) << 12) * 2;
  
  for (int y = 0; y < 32; ++y) {
    for (int x = 0; x < 32; ++x) {
      int p = bg1_base+(y*32+x)*2;
      p %= 0xFFFE;
      uint16_t tiledata = *(uint16_t*)&ppu.vram[p];
      uint16_t tile = (tiledata & 0x3FF);
      uint8_t palette = (tiledata >> 10) & 3;
      uint8_t Hflip = (tiledata >> 14) & 1;
      
      draw_sprite_8x8_2bpp(
        x * 8,
        y * 8,
        bg1_chrbase + (tile * 16),
        0,
        Hflip,
        0
      );
    }
  }
}

void draw_bg1(void)
{
  int width  = ((ppu.BG1SC & 1) ? 64 : 32);
  int height = ((ppu.BG1SC & 2) ? 64 : 32);
  int bg1_base = (((ppu.BG1SC >> 2) & 0x3F) << 10) * 2;
  int bg1_chrbase = ((ppu.BG12NBA & 0xF) << 12) * 2;
  int scrollx = ppu.BG1HOFS;
  int scrolly = ppu.BG1VOFS;
  
  for (int y = 0; y < 28; ++y) {
    for (int x = 0; x < 32; ++x) {
      int _x = abs((x + (scrollx >> 3)) % width);
      int _y = abs((y + (scrolly >> 3)) % height);
      int offset = (_y * width + _x) * 2;
      
      uint16_t tiledata = *(uint16_t*)&ppu.vram[bg1_base+offset];
      uint16_t tile = tiledata & 0x3FF;
      uint8_t palette = (tiledata >> 10) & 3;
      uint8_t Hflip = (tiledata >> 14) & 1;
      uint8_t Vflip = (tiledata >> 15) & 1;
      
      int tx = x * 8;
      int ty = y * 8;
      
      draw_sprite_8x8_4bpp(tx, ty, bg1_chrbase + (tile * 32), palette, Hflip, Vflip);
    }
  }
}

void draw_bg2(void)
{
  int width  = ((ppu.BG2SC & 1) ? 64 : 32);
  int height = ((ppu.BG2SC & 2) ? 64 : 32);
  int bg_base = (((ppu.BG2SC >> 2) & 0x3F) << 11);
  int bg_chrbase = (((ppu.BG12NBA >> 4) & 0xF) << 12) * 2;
  int scrollx = ppu.BG2HOFS;
  int scrolly = ppu.BG2VOFS;
  
  for (int y = 0; y < 28; ++y) {
    for (int x = 0; x < 32; ++x) {
      int _x = abs((x + (scrollx >> 3)) % width);
      int _y = abs((y + (scrolly >> 3)) % height);
      int offset = (_y * width + _x) * 2;
      
      uint16_t tiledata = *(uint16_t*)&ppu.vram[bg_base+offset];
      uint16_t tile = tiledata & 0x3FF;
      uint8_t palette = (tiledata >> 10) & 3;
      uint8_t Hflip = (tiledata >> 14) & 1;
      uint8_t Vflip = (tiledata >> 15) & 1;
      
      int tx = x * 8;
      int ty = y * 8;
      
      draw_sprite_8x8_4bpp(tx, ty, bg_chrbase + (tile * 32), palette, Hflip, Vflip);
    }
  }
}

void draw_bg3(void)
{
  int width = (ppu.BG3SC & 1) + 1; width *= 32;
  int height = ((ppu.BG3SC >> 1) & 1) + 1; height *= 32;
  int bg_base = (((ppu.BG3SC >> 2) & 0x3F) << 11);
  int bg_chrbase = ((ppu.BG34NBA & 0xF) << 12) * 2;
  
  for (int y = 0; y < 28; ++y) {
    for (int x = 0; x < 32; ++x) {
      uint16_t tiledata = *(uint16_t*)&ppu.vram[bg_base+(y*width+x)*2];
      uint16_t tile = tiledata & 0x3FF;
      uint8_t palette = (tiledata >> 10) & 3;
      uint8_t Hflip = (tiledata >> 14) & 1;
      uint8_t Vflip = (tiledata >> 15) & 1;
      
      int tx = x * 8;
      int ty = y * 8;
      
      draw_sprite_8x8_2bpp(tx, ty, (tile * 16) + bg_chrbase, palette, Hflip, Vflip);
    }
  }
}

void draw_mode1(void)
{
  if (ppu.TM & 0x01) draw_bg1();
  if (ppu.TM & 0x02) draw_bg2();
  if (ppu.TM & 0x04) draw_bg3();
}

void draw_mode3(void)
{
  if (ppu.TM & 0x01) {
    int width = (ppu.BG1SC & 1) + 1; width *= 32;
    int height = ((ppu.BG1SC >> 1) & 1) + 1; height *= 32;
    int bg1_base = (((ppu.BG1SC >> 2) & 0x3F) << 11);
    int bg1_chrbase = ((ppu.BG12NBA & 0xF) << 12) * 2;
    
    for (int y = 0; y < 28; ++y) {
      for (int x = 0; x < 32; ++x) {
        uint16_t tiledata = *(uint16_t*)&ppu.vram[bg1_base+(y*width+x)*2];
        uint16_t tile = tiledata & 0x3FF;
        uint8_t palette = (tiledata >> 10) & 3;
        uint8_t Hflip = (tiledata >> 14) & 1;
        uint8_t Vflip = (tiledata >> 15) & 1;
        
        int tx = x * 8;
        int ty = y * 8;
        
        draw_sprite_8x8_8bpp(tx, ty, bg1_chrbase + (tile * 64), 0, Hflip, Vflip);
      }
    }
  }
  if (ppu.TM & 0x02) draw_bg2();
}

uint32_t pc_to_HiRom(uint32_t pc)
{
  // map addresses to $C00000-$FFFFFF
  uint32_t k = (pc >> 16) & 0xFF;
  uint32_t b = pc & 0xFFFF;
  if (k < 0x40) {
    return ((0xC0 + k) << 16) | b;
  }
  if (k >= 0x40 && k < 0x7E) {
    return ((0xC0 + (k - 0x40)) << 16) | b;
  }
  if ((k >= 0x80) && (k < 0xC0)) {
    return ((0xC0 + k - 0x80) << 16) | b;
  }
  return pc;
}

uint32_t pc_to_LoRom(uint32_t pc)
{
  // map addresses to $808000-$FFFFFF
  uint32_t k = (pc >> 16) & 0xFF;
  uint32_t b = pc & 0xFFFF;
  if (k < 0x7E) {
    return ((k + 0x80) << 16) | b;
  }
  return pc;
}

uint32_t pc_to_dummy(uint32_t pc)
{
  return pc;
}

extern "C" uint32_t pc_map(uint32_t pc)
{
  switch (MapMode) {
    case 0: return pc_to_LoRom(pc);
    case 1: return pc_to_HiRom(pc);
    default:
      return pc_to_dummy(pc);
  }
}