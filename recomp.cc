/*

TITLE SNES Recompiler
AUTHOR Marc-Daniel DALEBA
FILE recomp.cc
DATE 2025-11-13
DESCRIPTION
  This is the heart of the project. This program turns the CPU trace
  into x86_64 assembly. The reason it doesn't turn the code into C is
  because it is necessary as SNES games are originally written in assembly
  and turning the code into C would be to complex.

*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <set>

typedef struct CodeAddr {
  int M, X, K;
  int type;
  uint32_t pc;
  uint32_t ins;
  
  bool operator<(const CodeAddr& other) const {
    if (pc != other.pc) { return pc < other.pc; }
    if (M != other.M) { return M < other.M; }
    return X != other.X;
  }
} CodeAddr;
std::multiset<CodeAddr> addr;
std::vector<uint32_t> routines;
uint32_t ResetVector = 0;
uint32_t NMIVector = 0;
uint32_t BRKVector = 0;
uint32_t COPVector = 0;
int MapMode = 0;
int FoundNMI = 0;
int FoundBRK = 0;
int FoundCOP = 0;

// INSTRUCTION : OP + A0 + A1 + A2
#define INS_OP(ins) ((ins >> 24) & 0xFF)
#define INS_GETA0(ins) ((ins >> 16) & 0xFF)                                             // A0
#define INS_GETA1(ins) ((ins >> 8) & 0xFF)                                              // A1
#define INS_GETA10(ins) ((ins & 0xFF00) | ((ins >> 16) & 0xFF))                         // A1+A0
#define INS_GETA210(ins) (((ins & 0xFF) << 16) | (ins & 0xFF00) | ((ins >> 16) & 0xFF)) // A2+A1+A0

#define CALL_FUNCTION_STK(FNAME) \
  printf("  sub rsp, 32\n"); \
  printf("  call %s\n", FNAME); \
  printf("  add rsp, 32\n");

void decode_65C816(CodeAddr ca);
int is_routines(uint32_t addr);

uint32_t pc_to_LoRom(uint32_t pc);
uint32_t pc_to_HiRom(uint32_t pc);
uint32_t pc_to_dummy(uint32_t pc);
static int in_wram(uint32_t pc);

uint32_t pc_map(uint32_t pc)
{
  switch (MapMode) {
    case 0: return pc_to_LoRom(pc);
    case 1: return pc_to_HiRom(pc);
    default:
      return pc_to_dummy(pc);
  }
}

int main(int argc, char **argv)
{
  FILE *f = fopen(argv[1], "r");
  if (!f) {
    printf("failed to open %s\n", argv[1]);
    return -1;
  }
  
  uint32_t headerloc;
  fscanf(f, "%X", &headerloc);
  fscanf(f, "%d", &MapMode);
  
  CodeAddr ca0;
  while((fscanf(f, "%d %d %d %X %X %X", &ca0.M, &ca0.X, &ca0.type, &ca0.K, &ca0.pc, &ca0.ins)) == 6) {
    addr.insert(ca0);
  }
  fclose(f);
  
  // Get Interrupts
  FILE *from = fopen(argv[2], "rb");
  
  fseek(from, headerloc + 0x3C, SEEK_SET); // Reset Vector
  fread(&ResetVector, 1, 2, from);
  if (MapMode == 0) {
    ResetVector += 0x800000;
  } else
  if (MapMode == 1) {
    ResetVector += 0xC00000;
  }
  routines.push_back(ResetVector);
  
  fseek(from, headerloc + 0x2A, SEEK_SET); // NMI Vector
  fread(&NMIVector, 1, 2, from);
  if (NMIVector) {
    if (MapMode == 0 && NMIVector >= 0x8000) {
      NMIVector += 0x800000;
    } else
    if (MapMode == 1) {
      NMIVector += 0xC00000;
    }
    routines.push_back(NMIVector);
  }
  
  fseek(from, headerloc + 0x26, SEEK_SET); // BRK Vector
  fread(&BRKVector, 1, 2, from);
  if (BRKVector) {
    if (MapMode == 0 && BRKVector >= 0x8000) {
      // We dont remap RAM
      BRKVector += 0x800000;
    } else
    if (MapMode == 1) {
      BRKVector += 0xC00000;
    }
    routines.push_back(BRKVector);
  }
  
  fseek(from, headerloc + 0x24, SEEK_SET); // COP Vector
  fread(&COPVector, 1, 2, from);
  if (COPVector) {
    if (MapMode == 0 && COPVector >= 0x8000) {
      // We dont remap RAM
      COPVector += 0x800000;
    } else
    if (MapMode == 1) {
      COPVector += 0xC00000;
    }
    routines.push_back(COPVector);
  }
  
  fclose(from);
  
  for (auto ca : addr) {
    if (ca.type == 1) { // ROUTINE
      routines.push_back(ca.pc);
    }
  }
  
  printf("extern MapMode, CycleCount\n");
  printf("extern regA, regX, regY, regS, regDP, regDBR\n");
  printf("extern B_Flag, C_Flag, D_Flag, E_Flag, I_Flag\n");
  printf("extern M_Flag, N_Flag, V_Flag, X_Flag, Z_Flag\n");
  printf("extern inNMI, NMI, io_RDNMI, io_NMITIMEN, DoMessages, Render\n");
  printf("extern pc_map, __JUMP_FAILED, __CALL_SHOW, __REG_DUMP, __READ_INS\n");
  printf("extern __UpdateNZ_A8, __UpdateNZ_X8, __UpdateNZ_Y8\n");
  printf("extern __UpdateNZ_A16, __UpdateNZ_X16, __UpdateNZ_Y16\n");
  printf("extern __MVN, __MVP, __REP, __SEP, __XCE, __WDM, __WAI, __PRINT_INS\n");
  printf("extern __TESTNZ8, __TESTNZ16, __COMPARE8, __COMPARE16\n");
  printf("extern __ASL8, __ASL16, __LSR8, __LSR16, __ROR8, __ROR16, __ROL8, __ROL16\n");
  printf("extern __TSB, __TRB, __BIT8, __BIT16, __INC8, __INC16, __ADC8, __ADC16, __SBC8, __SBC16\n");
  printf("extern __PHP, __PLP, __PUSH8, __PUSH16, __PULL8, __PULL16\n");
  printf("extern __WRITE8, __WRITE16, __READ8, __READ16, __READ24\n");
  printf("\n");
  
  printf("  section .text\n");
  printf("__CALL_ADDRESS:\n");
  printf("  mov r12, rcx\n");
  printf("  sub rsp, 32\n");
  printf("  call __CALL_SHOW\n");
  printf("  add rsp, 32\n");
  printf("  mov rcx, r12\n");
  printf("  and rcx, 0xFFFFFF\n");
  int i = 0;
  for (auto r : routines) {
    printf("  cmp rcx, 0x%06X\n", r);
    printf("  jne .next_%d\n", i);
    printf("  jmp Label_%06X\n", r);
    printf(".next_%d:\n", i);
    i++;
  }
  printf("  mov rdx, rbx\n");
  printf("  call __JUMP_FAILED\n\n");
  
  printf("__CPUSync:\n");
  printf("  sub rsp, 32\n");
  printf("  call Render\n");
  printf("  add rsp, 32\n");
  printf("  \n");
  // printf("  mov al, byte [rel inNMI]\n");
  // printf("  cmp al, 0\n");
  // printf("  jne .return\n");
  // printf("  \n");
  printf("  mov rax, [rel NMI]\n");
  printf("  cmp al, 0\n");
  printf("  je .return\n");
  printf("  \n");
  printf("  mov al, [rel io_RDNMI]\n");
  printf("  or al, 0x80\n");
  printf("  mov [rel io_RDNMI], al\n");
  printf("  \n");
  printf("  mov byte [rel NMI], 0\n");
  printf("  mov al, [rel io_NMITIMEN]\n");
  printf("  and al, 0x80\n");
  printf("  cmp al, 0\n");
  printf("  je .return\n");
  printf("  \n");
  printf("  add rsp, 32\n");
  printf("  pop rax ; pop return address\n");
  printf("  \n");
  printf("  mov byte [rel inNMI], 1\n");
  printf("  mov rax, r12\n");
  printf("  shr rax, 16\n");
  printf("  mov cl, al\n");
  CALL_FUNCTION_STK("__PUSH8");
  printf("  mov rax, r12\n");
  printf("  mov cl, ah\n");
  CALL_FUNCTION_STK("__PUSH8");
  printf("  mov rax, r12\n");
  printf("  mov cl, al\n");
  CALL_FUNCTION_STK("__PUSH8");
  CALL_FUNCTION_STK("__PHP");
  // MOV_REG8_IMM("I_Flag", 1); // ?
  // MOV_REG8_IMM("D_Flag", 0);
  printf("  jmp Label_NMI\n");
  printf("  \n");
  printf(".return\n");
  printf("  ret\n\n");
  
  printf("\n");
  printf("  global Start\n");
  printf("Start:\n");
  printf("  mov byte [rel MapMode], %d\n", MapMode);
  printf("  ret\n");
  
  std::set<uint32_t> vram_addrs;
  int skips = 0;
  for (auto ca : addr) {
    if (is_routines(ca.pc)) {
      printf("\n");
      if (ca.pc == ResetVector) {
        printf("  global Label_Reset\n");
        printf("Label_Reset:\n");
      } else
      if (ca.pc == NMIVector) {
        printf("  global Label_NMI\n");
        printf("Label_NMI:\n");
        FoundNMI = 1;
      } else
      if (ca.pc == BRKVector) {
        printf("  global Label_BRK\n");
        printf("Label_BRK:\n");
        FoundBRK = 1;
      }
      if (ca.pc == COPVector) {
        printf("  global Label_COP\n");
        printf("Label_COP:\n");
        FoundCOP = 1;
      }
      if (in_wram(ca.pc)) {
        if (std::find(vram_addrs.begin(), vram_addrs.end(), ca.pc) == vram_addrs.end()) {
          printf("  global Label_%06X\n", ca.pc);
          printf("Label_%06X:\n", ca.pc);
          vram_addrs.insert(ca.pc);
        }
      } else {
        printf("  global Label_%06X\n", ca.pc);
        printf("Label_%06X:\n", ca.pc);
      }
    }
    printf("  ; -- %06X --\n", ca.pc);
    if (in_wram(ca.pc)) {
      printf("  mov rcx, 0x%06X\n", ca.pc);
      printf("  sub rsp, 32\n");
      printf("  call __READ_INS\n"); // a little risky :(
      printf("  add rsp, 32\n");
      printf("  cmp eax, 0x%08X\n", ca.ins);
      printf("  jne Label_%06X_skip%d\n", ca.pc, skips);
    }
    printf("  mov r12, 0x%06X\n", ca.pc);
    printf("  sub rsp, 32\n");
    printf("  call __CPUSync\n");
    printf("  add rsp, 32\n");
    decode_65C816(ca);
    if (in_wram(ca.pc)) {
      printf("Label_%06X_skip%d:\n", ca.pc, skips);
      skips++;
    }
    if ((ca.pc & 0xFFFF) == 0xFFFF) {
      // pc wraps
      printf("  jmp Label_%02X0000\n", ca.pc >> 16);
    }
  }
  
  if (!NMIVector || !FoundNMI) {
    printf("  global Label_NMI\n");
    printf("Label_NMI:\n");
    printf("  mov byte [rel inNMI], 0\n");
    CALL_FUNCTION_STK("__PLP");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  and rbx, 0xFFFF\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  movzx rcx, al\n");
    printf("  shl rcx, 16\n");
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0xFFFFF0\n");
    printf("  jmp __CALL_ADDRESS\n\n");
  }
  if (!BRKVector || !FoundBRK) {
    printf("  global Label_BRK\n");
    printf("Label_BRK:\n");
    printf("  mov byte [rel inNMI], 0\n");
    CALL_FUNCTION_STK("__PLP");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  and rbx, 0xFFFF\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  movzx rcx, al\n");
    printf("  shl rcx, 16\n");
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0xFFFFF1\n");
    printf("  jmp __CALL_ADDRESS\n\n");
  }
  if (!COPVector || !FoundCOP) {
    printf("  global Label_COP\n");
    printf("Label_COP:\n");
    printf("  mov byte [rel inNMI], 0\n");
    CALL_FUNCTION_STK("__PLP");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  and rbx, 0xFFFF\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  movzx rcx, al\n");
    printf("  shl rcx, 16\n");
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0xFFFFF2\n");
    printf("  jmp __CALL_ADDRESS\n");
  }
  return 0;
}

int is_routines(uint32_t addr)
{
  return std::find(routines.begin(), routines.end(), addr) != routines.end();
}

#define ADD_CYCLES(cycles) printf("  add qword [rel CycleCount], %d\n", cycles)
#define UPDATE_NZ_A(M) (M) ? (printf("  call __UpdateNZ_A8\n")) : (printf("  call __UpdateNZ_A16\n"))
#define UPDATE_NZ_X(X) (X) ? (printf("  call __UpdateNZ_X8\n")) : (printf("  call __UpdateNZ_X16\n"))
#define UPDATE_NZ_Y(X) (X) ? (printf("  call __UpdateNZ_Y8\n")) : (printf("  call __UpdateNZ_Y16\n"))
#define CALL_FUNCTION(FNAME)     printf("  call %s\n", FNAME);
#define MOV_REG8_IMM(REG, IMM) printf("  mov byte [rel %s], 0x%02X\n", REG, IMM)
#define MOV_REG16_IMM(REG, IMM) printf("  mov word [rel %s], 0x%04X\n", REG, IMM)
#define LOAD_IMM(REG, IMM) printf("  mov %s, 0x%X\n", REG, IMM)
#define LOAD_DBR_ADDR(REG, ADDR) \
  printf("  movzx %s, byte [rel regDBR]\n", REG); \
  printf("  shl %s, 16\n", REG); \
  printf("  add %s, 0x%04X\n", REG, ADDR);
#define LOAD_DP(REG, OFFSET) \
  printf("  movzx %s, word [rel regDP]\n", REG); \
  printf("  add %s, 0x%02X\n", REG, OFFSET); \
  printf("  and %s, 0xFFFF\n", REG);
#define LOAD_SR(REG, OFFSET) \
  printf("  movzx %s, word [rel regS]\n", REG); \
  printf("  add %s, 0x%02X\n", REG, OFFSET);
#define APPLY_IDX_OFFSET(REGDST, REGSRC, IDX) \
  printf("  movzx %s, word [rel %s]\n", REGSRC, IDX); \
  if (ca.X) { printf("  and %s, 0xFF\n", REGSRC); } \
  printf("  add %s, %s\n", REGDST, REGSRC);

void decode_65C816(CodeAddr ca)
{
  int op = INS_OP(ca.ins);
  if (op == 0x00 || op == 0x02) { // brk #imm / cop #imm
    uint16_t p = (ca.pc & 0xFFFF) + 2;
    uint32_t retaddr = (ca.pc & 0xFF0000) | p;
    printf("  mov rcx, 0x%02X\n", (retaddr >> 16) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", (retaddr >> 8) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", retaddr & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    CALL_FUNCTION_STK("__PHP");
    MOV_REG8_IMM("I_Flag", 1);
    MOV_REG8_IMM("D_Flag", 0);
    if (op == 0x00) printf("  jmp Label_BRK\n");
    if (op == 0x02) printf("  jmp Label_COP\n");
    ADD_CYCLES(7);
  } else
  if (op == 0xEA) { // nop
    printf("  nop\n");
    ADD_CYCLES(2);
  } else
  if (op == 0x58) { // cli
    MOV_REG8_IMM("I_Flag", 0);
    ADD_CYCLES(2);
  } else
  if (op == 0x78) { // sei
    MOV_REG8_IMM("I_Flag", 1);
    ADD_CYCLES(2);
  } else
  if (op == 0x18) { // clc
    MOV_REG8_IMM("C_Flag", 0);
    ADD_CYCLES(2);
  } else
  if (op == 0x38) { // sec
    MOV_REG8_IMM("C_Flag", 1);
    ADD_CYCLES(2);
  } else
  if (op == 0xB8) { // clv
    MOV_REG8_IMM("V_Flag", 0);
    ADD_CYCLES(2);
  } else
  if (op == 0xD8) { // cld
    MOV_REG8_IMM("D_Flag", 0);
    ADD_CYCLES(2);
  } else
  if (op == 0xF8) { // sed
    MOV_REG8_IMM("D_Flag", 1);
    ADD_CYCLES(2);
  } else
  if (op == 0x5B) { // tcd
    printf("  mov ax, word [rel regA]\n");
    printf("  mov word [rel regDP], ax\n");
    printf("  mov cx, ax\n");
    CALL_FUNCTION_STK("__TESTNZ16");
    ADD_CYCLES(2);
  } else
  if (op == 0x7B) { // tdc
    printf("  mov ax, word [rel regDP]\n");
    printf("  mov word [rel regA], ax\n");
    UPDATE_NZ_A(0);
    ADD_CYCLES(2);
  } else
  if (op == 0x08) { // php
    CALL_FUNCTION_STK("__PHP");
    ADD_CYCLES(3);
  } else
  if (op == 0x28) { // plp
    CALL_FUNCTION_STK("__PLP");
    ADD_CYCLES(4);
  } else
  if (op == 0x14) { // trb dp
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TRB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TRB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x1C) { // trb addr
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TRB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TRB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x04) { // tsb dp
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TSB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TSB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x0C) { // tsb addr
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TSB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__TSB");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x0B) { // phd
    printf("  movzx rcx, word [rel regDP]\n");
    CALL_FUNCTION_STK("__PUSH16");
    ADD_CYCLES(3);
  } else
  if (op == 0x2B) { // pld
    CALL_FUNCTION_STK("__PULL16");
    printf("  mov word [rel regDP], ax\n");
    printf("  mov rcx, rax\n");
    CALL_FUNCTION_STK("__TESTNZ16");
    ADD_CYCLES(3);
  } else
  if (op == 0xD4) { // pei dp
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  mov rcx, rax\n");
    CALL_FUNCTION_STK("__PUSH16");
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF4) { // pea addr
    uint16_t address = INS_GETA10(ca.ins);
    printf("  mov rcx, 0x%04X\n", address);
    CALL_FUNCTION_STK("__PUSH16");
    ADD_CYCLES(5);
  } else
  if (op == 0x62) { // per rel
    int16_t rel = INS_GETA10(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 3;
    printf("  mov rcx, 0x%04X\n", ca.pc | p);
    CALL_FUNCTION_STK("__PUSH16");
    ADD_CYCLES(5);
  } else
  if (op == 0x48) { // pha
    int cycles = 3;
    printf("  movzx rcx, word [rel regA]\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__PUSH8");
    } else {
      CALL_FUNCTION_STK("__PUSH16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xDA) { // phx
    int cycles = 3;
    printf("  movzx rcx, word [rel regX]\n");
    if (ca.X) {
      CALL_FUNCTION_STK("__PUSH8");
    } else {
      CALL_FUNCTION_STK("__PUSH16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x5A) { // phy
    int cycles = 3;
    printf("  movzx rcx, word [rel regY]\n");
    if (ca.X) {
      CALL_FUNCTION_STK("__PUSH8");
    } else {
      CALL_FUNCTION_STK("__PUSH16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x68) { // pla
    int cycles = 4;
    if (ca.M) {
      CALL_FUNCTION_STK("__PULL8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__PULL16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xFA) { // plx
    int cycles = 4;
    if (ca.X) {
      CALL_FUNCTION_STK("__PULL8");
      printf("  mov byte [rel regX], al\n");
    } else {
      CALL_FUNCTION_STK("__PULL16");
      printf("  mov word [rel regX], ax\n");
      cycles++;
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x7A) { // ply
    int cycles = 4;
    if (ca.X) {
      CALL_FUNCTION_STK("__PULL8");
      printf("  mov byte [rel regY], al\n");
    } else {
      CALL_FUNCTION_STK("__PULL16");
      printf("  mov word [rel regY], ax\n");
      cycles++;
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x4B) { // phk
    printf("  mov eax, 0x%02X\n", ca.K);
    printf("  movzx rcx, ax\n");
    CALL_FUNCTION_STK("__PUSH8");
    ADD_CYCLES(3);
  } else
  if (op == 0x8B) { // phb
    printf("  movzx rcx, byte [rel regDBR]\n");
    CALL_FUNCTION_STK("__PUSH8");
    ADD_CYCLES(3);
  } else
  if (op == 0xAB) { // plb
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov byte [rel regDBR], al\n");
    printf("  mov rcx, rax\n");
    CALL_FUNCTION_STK("__TESTNZ8");
    ADD_CYCLES(4);
  } else
  if (op == 0xCB) { // wai
    CALL_FUNCTION("__WAI");
  } else
  if (op == 0x9A) { // txs
    printf("  mov ax, word [rel regX]\n");
    printf("  mov word [rel regS], ax\n");
    ADD_CYCLES(2);
  } else
  if (op == 0xBA) { // tsx
    printf("  mov ax, word [rel regS]\n");
    if (ca.X) { printf("  and ax, 0xFF\n"); }
    printf("  mov word [rel regX], ax\n");
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0x3B) { // tsc
    printf("  mov ax, word [rel regS]\n");
    printf("  mov word [rel regA], ax\n");
    UPDATE_NZ_A(0);
    ADD_CYCLES(2);
  } else
  if (op == 0x1B) { // tcs
    printf("  mov ax, word [rel regA]\n");
    printf("  mov word [rel regS], ax\n");
    ADD_CYCLES(2);
  } else
  if (op == 0xAA) { // tax
    if (ca.X) {
      printf("  mov al, byte [rel regA]\n");
      printf("  mov byte [rel regX], al\n");
    } else {
      printf("  mov ax, word [rel regA]\n");
      printf("  mov word [rel regX], ax\n");
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0x8A) { // txa
    if (ca.M) {
      printf("  mov al, byte [rel regX]\n");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  mov ax, word [rel regX]\n");
      printf("  mov word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(2);
  } else
  if (op == 0xA8) { // tay
    if (ca.X) {
      printf("  mov al, byte [rel regA]\n");
      printf("  mov byte [rel regY], al\n");
    } else {
      printf("  mov ax, word [rel regA]\n");
      printf("  mov word [rel regY], ax\n");
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0x98) { // tya
    if (ca.M) {
      printf("  mov al, byte [rel regY]\n");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  mov ax, word [rel regY]\n");
      printf("  mov word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(2);
  } else
  if (op == 0x9B) { // txy
    if (ca.X) {
      printf("  mov al, byte [rel regX]\n");
      printf("  mov byte [rel regY], al\n");
    } else {
      printf("  mov ax, word [rel regX]\n");
      printf("  mov word [rel regY], ax\n");
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0xBB) { // tyx
    if (ca.X) {
      printf("  mov al, byte [rel regY]\n");
      printf("  mov byte [rel regX], al\n");
    } else {
      printf("  mov ax, word [rel regY]\n");
      printf("  mov word [rel regX], ax\n");
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0x40) { // rti
    printf("  mov byte [rel inNMI], 0\n");
    ADD_CYCLES(6);
    CALL_FUNCTION_STK("__PLP");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  and rbx, 0xFFFF\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  movzx rcx, al\n");
    printf("  shl rcx, 16\n");
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x54) { // mvn src, dest
    uint8_t dest = INS_GETA0(ca.ins);
    uint8_t src = INS_GETA1(ca.ins);
    LOAD_IMM("rcx", src);
    LOAD_IMM("rdx", dest);
    CALL_FUNCTION_STK("__MVN");
  } else
  if (op == 0x44) { // mvp src, dest
    uint8_t dest = INS_GETA0(ca.ins);
    uint8_t src = INS_GETA1(ca.ins);
    LOAD_IMM("rcx", src);
    LOAD_IMM("rdx", dest);
    CALL_FUNCTION_STK("__MVP");
  } else
  if (op == 0x49) { // eor #imm
    int cycles = 2;
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  mov al, byte [rel regA]\n");
      printf("  xor al, 0x%02X\n", v);
      printf("  mov byte [rel regA], al\n");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      printf("  mov ax, word [rel regA]\n");
      printf("  xor ax, 0x%04X\n", v);
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x4D) { // eor addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x5D) { // eor addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x59) { // eor addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x4F) { // eor long
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x5F) { // eor long, x
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x45) { // eor dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x55) { // eor dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x52) { // eor (dp)
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x51) { // eor (dp), y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x41) { // eor (dp, x)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  xor rcx, rax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x47) { // eor [dp]
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x57) { // eor [dp], y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x43) { // eor sr, s
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x53) { // eor (sr, s), y
    int cycles = 7;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov bl, byte [rel regA]\n");
      printf("  xor bl, al\n");
      printf("  mov byte [rel regA], bl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov bx, word [rel regA]\n");
      printf("  xor bx, ax\n");
      printf("  mov word [rel regA], bx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x4A) { // lsr a
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__LSR8");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__LSR16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0x4E) { // lsr addr
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(5);
  } else
  if (op == 0x5E) { // lsr addr, x
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(6);
  } else
  if (op == 0x46) { // lsr dp
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x56) { // lsr dp, x
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__LSR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(5);
  } else
  if (op == 0x0A) { // asl a
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ASL8");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ASL16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0x0E) { // asl addr
    int cycles = 6;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles += 2;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x1E) { // asl addr, X
    int cycles = 7;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles += 2;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x06) { // asl dp
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles += 2;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x16) { // asl dp, X
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ASL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles += 2;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x2A) { // rol a
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ROL8");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ROL16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0x2E) { // rol addr
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x3E) { // rol addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x26) { // rol dp
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x36) { // rol dp, x
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    // printf("  and rcx, 0xFFFF\n");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROL16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x6A) { // ror a
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ROR8");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ROR16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0x6E) { // ror addr
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x7E) { // ror addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x66) { // ror dp
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x76) { // ror dp, x
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    // printf("  and rcx, 0xFFFF\n");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__ROR16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x89) { // bit #imm
    int cycles = 2;
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, 0x%02X\n", v);
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, 0x%04X\n", v);
      cycles++;
    }
    printf("  and rcx, rdx\n");
    printf("  cmp rcx, 0\n");
    printf("  sete byte [rel Z_Flag]\n");
    ADD_CYCLES(cycles);
  } else
  if (op == 0x2C) { // bit addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      cycles++;
    }
    printf("  movzx rcx, ax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__BIT8");
    } else {
      CALL_FUNCTION_STK("__BIT16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x3C) { // bit addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__BIT8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__BIT16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x24) { // bit dp
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, ax\n");
      CALL_FUNCTION_STK("__BIT8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, ax\n");
      CALL_FUNCTION_STK("__BIT16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x34) { // bit dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, ax\n");
      CALL_FUNCTION_STK("__BIT8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, ax\n");
      CALL_FUNCTION_STK("__BIT16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x69) { // adc #imm
    int cycles = 2;
    printf("  movzx rcx, word [rel regA]\n");
    if (ca.M) {      
      uint8_t v = INS_GETA0(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x6D) { // adc addr
    int cycles = 3;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x79) { // adc addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x7D) { // adc addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x65) { // adc dp
    int cycles = 3;
    uint8_t offset = INS_GETA10(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x75) { // adc dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA10(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    // printf("  and rcx, 0xFFFF\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x61) { // adc (dp, x)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x72) { // adc (dp)
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x71) { // adc (dp), y
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    printf("  movzx rax, word [rel regY]\n");
    if (ca.X) {
      printf("  and rax, 0xFF\n");
      cycles++;
    }
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x67) { // adc [dp]
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x77) { // adc [dp], y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    printf("  movzx rax, word [rel regY]\n");
    if (ca.X) {
      printf("  and rax, 0xFF\n");
    }
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x6F) { // adc long
    int cycles = 4;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x7F) { // adc long, x
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x63) { // adc sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x73) { // adc (sr, s), y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    printf("  movzx rax, word [rel regY]\n");
    if (ca.X) {
      printf("  and rax, 0xFF\n");
      cycles++;
    }
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__ADC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf(" mov rcx, rax\n");
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__ADC16");
      printf("  mov word [rel regA], ax\n");
    }
  } else
  if (op == 0xE9) { // sbc #imm
    int cycles = 2;
    printf("  movzx rcx, word [rel regA]\n");
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  mov rdx, 0x%02X\n", v);
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      printf("  mov rdx, 0x%04X\n", v);
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE5) { // sbc dp
    int cycles = 5;
    uint8_t offset = INS_GETA10(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF5) { // sbc dp, x
    int cycles = 5;
    uint8_t offset = INS_GETA10(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF2) { // sbc (dp)
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF1) { // sbc (dp), y
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE1) { // sbc (dp, x)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xED) { // sbc addr
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xFD) { // sbc addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF9) { // sbc addr, y
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xEF) { // sbc long
    int cycles = 5;
    uint32_t longaddr = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", longaddr);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xFF) { // sbc long, x
    int cycles = 5;
    uint32_t longaddr = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", longaddr);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE7) { // sbc [dp]
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF7) { // sbc [dp], y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE3) { // sbc sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF3) { // sbc (sr, s), y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__SBC16");
      printf("  mov word [rel regA], ax\n");
    }
  } else
  if (op == 0x29) { // and #imm
    int cycles = 2;
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  mov al, byte [rel regA]\n");
      printf("  and al, 0x%02X\n", v);
      printf("  mov byte [rel regA], al\n");
    } else {
      cycles++;
      uint16_t v = INS_GETA10(ca.ins);
      printf("  mov ax, word [rel regA]\n");
      printf("  and ax, 0x%04X\n", v);
      printf("  mov word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x2D) { // and addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x3D) { // and addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x39) { // and addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x2F) { // and long
    int cycles = 5;
    uint32_t longaddr = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", longaddr);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x3F) { // and long, x
    int cycles = 5;
    uint32_t longaddr = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", longaddr);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x25) { // and dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x35) { // and dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  movzx rax, word [rel regX]\n");
    if (ca.X) {
      printf("  and rax, 0xFF\n");
    }
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x32) { // and (dp)
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x31) { // and (dp), y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x21) { // and (dp, x)
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    printf("  movzx rax, word [rel regX]\n");
    if (ca.X) {
      printf("  and rax, 0xFF\n");
    }
    printf("  add rcx, rax\n");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x27) { // and [dp] (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x37) { // and [dp], y (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x23) { // and sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x33) { // and (sr, s), y
    int cycles = 7;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regY]\n");
    if (ca.X) {
      printf("  and rcx, 0xFF\n");
    }
    printf("  add rcx, rax\n");
    printf("  movzx rax, byte [rel regDBR]\n");
    printf("  shl rax, 16\n");
    printf("  or rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  and byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  and word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x09) { // ora #imm
    int cycles = 2;
    printf("  mov ax, word [rel regA]\n");
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  or al, 0x%02X\n", v);
      printf("  mov byte [rel regA], al\n");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      printf("  or ax, 0x%04X\n", v);
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x0D) { // ora addr
    int cycles = 3;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x1D) { // ora addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x19) { // ora addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x0F) { // ora long
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x1F) { // ora long, x
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x05) { // ora dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x15) { // ora dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x12) { // ora (dp)
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x11) { // ora (dp), y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x01) { // ora (dp, x)
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x07) { // ora [dp] (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x17) { // ora [dp], y (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x03) { // ora sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x13) { // ora (sr, s), y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  or cl, al\n");
      printf("  mov byte [rel regA], cl\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  or cx, ax\n");
      printf("  mov word [rel regA], cx\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x1A) { // inc
    if (ca.M) {
      printf("  mov al, byte [rel regA]\n");
      printf("  inc al\n");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  mov ax, word [rel regA]\n");
      printf("  inc ax\n");
      printf("  mov word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(2);
  } else
  if (op == 0xEE) { // inc addr
    int cycles = 6;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xFE) { // inc addr, x
    int cycles = 7;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE6) { // inc dp
    int cycles = 2;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC8");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC16");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xF6) { // INC DirectPage, X
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    if (ca.X) {
      printf("  movzx rax, byte [rel regX]\n");
    } else {
      printf("  movzx rax, word [rel regX]\n");
    }
    printf("  add rcx, rax\n");
    printf("  mov r12, rcx\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rcx, rax\n");
      CALL_FUNCTION_STK("__INC16");
      cycles++;
    }
    printf("  mov rcx, r12\n");
    printf("  mov rdx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x3A) { // dec
    if (ca.M) {
      printf("  mov al, byte [rel regA]\n");
      printf("  dec al\n");
      printf("  mov byte [rel regA], al\n");
    } else {
      printf("  mov ax, word [rel regA]\n");
      printf("  dec ax\n");
      printf("  mov word [rel regA], ax\n");
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(2);
  } else
  if (op == 0xCE) { // dec addr
    int cycles = 6; // NA
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  dec al\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  dec ax\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xDE) { // dec addr, x
    int cycles = 6; // NA
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  dec al\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  dec ax\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC6) { // dec dp
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  dec al\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  dec ax\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD6) { // dec dp, x
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ8");
      printf("  dec al\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE8");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ8");
    } else {
      printf("  mov r12, rcx\n");
      CALL_FUNCTION_STK("__READ16");
      printf("  dec ax\n");
      printf("  mov rcx, r12\n");
      printf("  mov rdx, rax\n");
      printf("  mov rbx, rax\n");
      CALL_FUNCTION_STK("__WRITE16");
      printf("  mov rcx, rbx\n");
      CALL_FUNCTION_STK("__TESTNZ16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE8) { // inx
    if (ca.X) {
      printf("  movzx rcx, byte [rel regX]\n");
      CALL_FUNCTION_STK("__INC8");
      printf("  mov byte [rel regX], al\n");
    } else {
      printf("  movzx rcx, word [rel regX]\n");
      CALL_FUNCTION_STK("__INC16");
      printf("  mov word [rel regX], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0xCA) { // DEX
    printf("  mov ax, word [rel regX]\n");
    printf("  dec ax\n");
    printf("  mov word [rel regX], ax\n");
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0xC8) { // INY
    if (ca.X) {
      printf("  movzx rcx, byte [rel regY]\n");
      CALL_FUNCTION_STK("__INC8");
      printf("  mov byte [rel regY], al\n");
    } else {
      printf("  movzx rcx, word [rel regY]\n");
      CALL_FUNCTION_STK("__INC16");
      printf("  mov word [rel regY], ax\n");
    }
    ADD_CYCLES(2);
  } else
  if (op == 0x88) { // dey
    printf("  mov ax, word [rel regY]\n");
    printf("  dec ax\n");
    printf("  mov word [rel regY], ax\n");
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0x60) { // rts
    ADD_CYCLES(6);
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  inc bx\n");
    printf("  and rbx, 0xFFFF\n");
    printf("  mov rcx, 0x%06X\n", ca.pc & 0xFF0000);
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x6B) { // rtl
    ADD_CYCLES(6);
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bl, al\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  mov bh, al\n");
    printf("  inc bx\n");
    printf("  and rbx, 0xFFFF\n");
    CALL_FUNCTION_STK("__PULL8");
    printf("  movzx rcx, al\n");
    printf("  shl rcx, 16\n");
    printf("  or rcx, rbx\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0xEB) { // xba
    printf("  mov ax, word [rel regA]\n");
    printf("  mov bl, al\n");
    printf("  mov al, ah\n");
    printf("  mov ah, bl\n");
    printf("  mov word [rel regA], ax\n");
    UPDATE_NZ_A(1);
    ADD_CYCLES(3);
  } else
  if (op == 0xFB) { // xce
    CALL_FUNCTION_STK("__XCE");
    ADD_CYCLES(2);
  } else
  if (op == 0x42) { // wdm
    uint8_t v = INS_GETA0(ca.ins);
    CALL_FUNCTION("__WDM");
  } else
  if (op == 0xC2) { // rep
    uint8_t mask = INS_GETA0(ca.ins);
    LOAD_IMM("rcx", mask);
    CALL_FUNCTION_STK("__REP");
    ADD_CYCLES(3);
  } else
  if (op == 0xE2) { // SEP
    uint8_t mask = INS_GETA0(ca.ins);
    LOAD_IMM("rcx", mask);
    CALL_FUNCTION_STK("__SEP");
    ADD_CYCLES(3);
  } else
  if (op == 0xC9) { // cmp #imm
    int cycles = 2;
    printf("  movzx rcx, word [rel regA]\n");
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xCD) { // cmp addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xDD) { // cmp addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD9) { // cmp addr, y
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov rdx, rax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xDD) { // cmp addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      cycles++;
    }
    printf("  mov rdx, rax\n");
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xCF) { // cmp long
    int cycles = 6;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      cycles++;
    }
    printf("  mov rdx, rax\n");
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xDF) { // cmp long, x
    int cycles = 4;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      cycles++;
    }
    printf("  mov rdx, rax\n");
    if (ca.M) {
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC5) { // cmp dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD5) { // cmp dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n");
      printf("  movzx rcx, word [rel regA]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD2) { // cmp (dp)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD1) { // cmp (dp), y
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC1) { // cmp (dp, x)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC7) { // cmp [dp] (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD7) { // cmp [dp], Y (Long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    printf("  movzx rcx, word [rel regDP]\n");
    printf("  add rcx, 0x%02X\n", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  movzx ebx, word [rel regY]\n");
    printf("  add eax, ebx\n");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC3) { // cmp sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
     if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xD3) { // cmp (sr, s), y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, word [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
     if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rcx, byte [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rcx, word [rel regA]\n");
      printf("  mov rdx, rax\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE0) { // cpx #imm
    int cycles = 2;
    printf("  mov ax, word [rel regX]\n", 0);
    printf("  movzx rcx, ax\n");
    if (ca.X) {
      uint8_t v = INS_GETA0(ca.ins);
      printf("  mov rdx, 0x%02X\n", v);
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      cycles++;
      uint16_t v = INS_GETA10(ca.ins);
      printf("  mov rdx, 0x%04X\n", v);
      CALL_FUNCTION_STK("__COMPARE16");
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xEC) { // cpx addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regX]\n", 0);
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n", 0);
      printf("  movzx rcx, word [rel regX]\n", 0);
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xE4) { // cpx dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regX]\n", 0);
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n", 0);
      printf("  movzx rcx, word [rel regX]\n", 0);
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC0) { // cpy #imm
    int cycles = 2;
    printf("  movzx rcx, word [rel regY]\n");
    if (ca.X) {
      uint8_t v = INS_GETA0(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      LOAD_IMM("rdx", v);
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xCC) { // cpy addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regY]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n");
      printf("  movzx rcx, word [rel regY]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0xC4) { // cpy dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  movzx rdx, al\n");
      printf("  movzx rcx, byte [rel regY]\n");
      CALL_FUNCTION_STK("__COMPARE8");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  movzx rdx, ax\n");
      printf("  movzx rcx, word [rel regY]\n");
      CALL_FUNCTION_STK("__COMPARE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x80) { // bra rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(3);
    if (is_routines(absolute)) {
      printf("  jmp Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x82) { // brl rel (long)
    int16_t rel = INS_GETA10(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 3;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(4);
    if (is_routines(absolute)) {
      printf("  jmp Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0xD0) { // BNE Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel Z_Flag], 0\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0xF0) { // BEQ Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel Z_Flag], 1\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x90) { // BCC Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel C_Flag], 0\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0xB0) { // BCS Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel C_Flag], 1\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x30) { // BMI Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel N_Flag], 1\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x10) { // BPL Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel N_Flag], 0\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x50) { // BVC Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel V_Flag], 0\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0x70) { // BVS Rel
    int8_t rel = INS_GETA0(ca.ins);
    uint16_t p = ca.pc & 0xFFFF;
    p += rel + 2;
    uint32_t absolute = ca.pc & 0xFF0000 | p;
    ADD_CYCLES(2);
    if (is_routines(absolute)) {
      printf("  cmp byte [rel V_Flag], 1\n");
      printf("  je Label_%06X\n", absolute);
    } else {
      printf("  ; Never branches to %06X\n", absolute);
    }
  } else
  if (op == 0xA9) { // lda #imm
    int cycles = 2;
    if (ca.M) {
      uint8_t v = INS_GETA0(ca.ins);
      MOV_REG8_IMM("regA", v);
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      MOV_REG16_IMM("regA", v);
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xAD) { // lda addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB9) { // lda addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xAF) { // lda long
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA5) { // lda dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB5) { // lda dp, X
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8\n");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16\n");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB2) { // lda (dp)
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB1) { // lda (dp), y
    int cycles = 5;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  movzx rax, byte [rel regDBR]\n");
    printf("  shl rax, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA1) { // lda (dp, x)
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA7) { // lda [dp] (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB7) { // lda [dp], y (long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xBD) { // lda addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB9) { // lda addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xBF) { // lda long, X
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA3) { // lda sr, s
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB3) { // lda (sr, s), y
    int cycles = 7;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  movzx rax, byte [rel regDBR]\n");
    printf("  shl rax, 16\n");
    printf("  add rcx, rax\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regA], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regA], ax\n");
      cycles++;
    }
    UPDATE_NZ_A(ca.M);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA2) { // ldx #imm
    int cycles = 2;
    if (ca.X) {
      uint8_t v = INS_GETA0(ca.ins);
      MOV_REG8_IMM("regX", v);
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      MOV_REG16_IMM("regX", v);
      cycles++;
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xAE) { // ldx addr
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regX], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regX], ax\n");
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(4);
  } else
  if (op == 0xBE) { // ldx addr, y
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regX], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regX], ax\n");
      cycles++;
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA6) { // ldx dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regX], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regX], ax\n");
      cycles++;
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xB6) { // ldx dp, y
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  and rcx, 0xFFFF\n");
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regX], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regX], ax\n");
      cycles++;
    }
    UPDATE_NZ_X(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA0) { // ldy #imm
    if (ca.X) {
      uint8_t v = INS_GETA0(ca.ins);
      MOV_REG8_IMM("regY", v);
    } else {
      uint16_t v = INS_GETA10(ca.ins);
      MOV_REG16_IMM("regY", v);
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(2);
  } else
  if (op == 0xAC) { // ldy addr
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regY], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regY], ax\n");
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(4);
  } else
  if (op == 0xBC) { // ldy addr, x
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regY], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regY], ax\n");
      cycles++;
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0xA4) { // ldy dp
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regY], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regY], ax\n");
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(3);
  } else
  if (op == 0xB4) { // ldy dp, x
    int cycles = 4;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    if (ca.X) {
      CALL_FUNCTION_STK("__READ8");
      printf("  mov byte [rel regY], al\n");
    } else {
      CALL_FUNCTION_STK("__READ16");
      printf("  mov word [rel regY], ax\n");
      cycles++;
    }
    UPDATE_NZ_Y(ca.X);
    ADD_CYCLES(cycles);
  } else
  if (op == 0x85) { // sta dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x95) { // sta dp, X
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(4);
  } else
  if (op == 0x92) { // sta (dp)
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x91) { // sta (dp), y
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x81) { // sta (dp, x)
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x87) { // sta [dp] (Long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    if (ca.M) {
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x8D) { // sta addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x9D) { // sta addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    if (ca.M) {
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x99) { // sta addr, y
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x97) { // sta [dp], y (Long)
    int cycles = 6;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    if (ca.M) {
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x83) { // sta sr, s
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x93) { // sta (sr, s), y
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_SR("rcx", offset);
    CALL_FUNCTION_STK("__READ16");
    printf("  movzx rcx, byte [rel regDBR]\n");
    printf("  shl rcx, 16\n");
    printf("  add rcx, rax\n");
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x8E) { // stx addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  movzx rdx, word [rel regX]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x86) { // stx dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  movzx rdx, word [rel regX]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x96) { // stx dp, y
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regY");
    printf("  and rcx, 0xFFFF\n");
    printf("  movzx rdx, word [rel regX]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x8C) { // sty addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    printf("  movzx rdx, word [rel regY]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x84) { // sty dp
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    printf("  movzx rdx, word [rel regY]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x94) { // sty dp, x
    int cycles = 3;
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    printf("  movzx rdx, word [rel regY]\n");
    if (ca.X) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x64) { // stz dp
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    LOAD_IMM("rdx", 0);
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(3);
  } else
  if (op == 0x74) { // stz dp, x
    uint8_t offset = INS_GETA0(ca.ins);
    LOAD_DP("rcx", offset);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    LOAD_IMM("rdx", 0);
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
    }
    ADD_CYCLES(3);
  } else
  if (op == 0x9C) { // stz addr
    int cycles = 4;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    LOAD_IMM("rdx", 0);
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x9E) { // stz addr, x
    int cycles = 5;
    uint16_t address = INS_GETA10(ca.ins);
    LOAD_DBR_ADDR("rcx", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    LOAD_IMM("rdx", 0);
    if (ca.M) {
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x8F) { // sta long
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    if (ca.M) {
      printf("  movzx rdx, byte [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      printf("  movzx rdx, word [rel regA]\n");
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x9F) { // sta long, x
    int cycles = 5;
    uint32_t address = INS_GETA210(ca.ins);
    printf("  mov rcx, 0x%06X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  movzx rdx, word [rel regA]\n");
    if (ca.M) {
      printf("  and rdx, 0xFF\n");
      CALL_FUNCTION_STK("__WRITE8");
    } else {
      CALL_FUNCTION_STK("__WRITE16");
      cycles++;
    }
    ADD_CYCLES(cycles);
  } else
  if (op == 0x4C) { // jmp addr
    uint32_t address = INS_GETA10(ca.ins);
    address = (ca.pc & 0xFF0000) | address;
    ADD_CYCLES(3);
    printf("  mov rcx, 0x%06X\n", address);
    CALL_FUNCTION_STK("pc_map");
    printf("  mov rcx, rax\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x6C) { // jmp (addr)
    uint32_t address = INS_GETA10(ca.ins);
    ADD_CYCLES(5);
    printf("  mov rcx, 0x%04X\n", address);
    CALL_FUNCTION_STK("__READ16");
    printf("  or rax, 0x%06X\n", ca.pc & 0xFF0000);
    printf("  mov rcx, rax\n");
    CALL_FUNCTION_STK("pc_map");
    printf("  mov rcx, rax\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x7C) { // jmp (addr, x)
    uint32_t address = INS_GETA10(ca.ins);
    ADD_CYCLES(6);
    printf("  mov rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  mov rcx, rax\n");
    printf("  or rcx, 0x%06X\n", ca.pc & 0xFF0000);
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x22) { // jsl long
    uint32_t address = pc_map(INS_GETA210(ca.ins));
    ADD_CYCLES(8);
    printf("  mov rcx, 0x%02X\n", ((ca.pc + 3) >> 16) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", ((ca.pc + 3) >> 8) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", (ca.pc + 3) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%06X\n", address);
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x20) { // jsr addr
    uint32_t address = INS_GETA10(ca.ins);
    address = pc_map((ca.pc & 0xFF0000) | address);
    ADD_CYCLES(6);
    printf("  mov rcx, 0x%02X\n", ((ca.pc + 2) >> 8) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", (ca.pc + 2) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%06X\n", address);
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0xFC) { // jsr (addr, x)
    uint32_t address = INS_GETA10(ca.ins);
    ADD_CYCLES(3);
    printf("  mov rcx, 0x%04X\n", address);
    APPLY_IDX_OFFSET("rcx", "rax", "regX");
    printf("  and rcx, 0xFFFF\n");
    CALL_FUNCTION_STK("__READ16");
    printf("  mov rcx, 0x%06X\n", ca.pc & 0xFF0000);
    printf("  or rcx, rax\n");
    printf("  mov r12, rcx\n");
    printf("  mov rcx, 0x%02X\n", ((ca.pc + 2) >> 8) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, 0x%02X\n", (ca.pc + 2) & 0xFF);
    CALL_FUNCTION_STK("__PUSH8");
    printf("  mov rcx, r12\n");
    CALL_FUNCTION_STK("pc_map");
    printf("  mov rcx, rax\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0x5C) { // jml long
    uint32_t address = INS_GETA210(ca.ins);
    ADD_CYCLES(4);
    LOAD_IMM("rcx", pc_map(address));
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else
  if (op == 0xDC) { // jml [addr]
    int cycles = 6;
    uint16_t address = INS_GETA10(ca.ins);
    ADD_CYCLES(cycles);
    printf("  mov rcx, 0x%04X\n", address);
    CALL_FUNCTION_STK("__READ24");
    printf("  mov rcx, rax\n");
    CALL_FUNCTION_STK("pc_map");
    printf("  mov rcx, rax\n");
    printf("  mov rbx, 0x%06X\n", ca.pc);
    printf("  jmp __CALL_ADDRESS\n");
  } else {
    printf("  ; UNKNOWN %08X\n", ca.ins);
    printf("  mov rcx, 0x%08X\n", ca.pc);
    printf("  mov rdx, 0x%08X\n", ca.ins);
    printf("  call __PRINT_INS\n");
  }
}

static int in_wram(uint32_t pc)
{
  uint8_t k = (pc >> 16) & 0xFF;
  uint16_t p = pc & 0xFFFF;
  if (k == 0x7E || k == 0x7F) {
    return 1;
  }
  if (k < 0x40 || (k >= 80 && k < 0xC0)) {
    if (p < 0x2000) {
      return 1;
    }
  }
  return 0;
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
    if (b >= 0x8000) {
      return ((k + 0x80) << 16) | b;
    }
  }
  return pc;
}

uint32_t pc_to_dummy(uint32_t pc)
{
  return pc;
}