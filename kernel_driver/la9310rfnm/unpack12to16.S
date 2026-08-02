/* Supplementary macro for setting function attributes */
.macro asm_function fname
#ifdef __APPLE__
    .private_extern _\fname
    .globl _\fname
_\fname:
#else
    .global \fname
#ifdef __ELF__
    .hidden \fname
    .type \fname, %function
#endif
\fname:
#endif
.endm



asm_function rfnm_unpack12to16_aarch64

dest .req x0
src .req x1
count .req x2



.text
.align 5
.global rfnm_unpack12to16_aarch64


//PRFM PLDL1STRM,   [src]
//PRFM PLDL1STRM,   [src, #48]
//PRFM PLDL1STRM,   [src, #96]
//PRFM PLDL1STRM,   [src, #144]

.align 5

loop:

ld3     {v20.16b, v21.16b, v22.16b}, [src], #48
ld3     {v23.16b, v24.16b, v25.16b}, [src], #48
ld3     {v26.16b, v27.16b, v28.16b}, [src], #48
ld3     {v29.16b, v30.16b, v31.16b}, [src], #48

cmp     count, #256
beq     sky_prefetch

PRFM PLDL1STRM,   [src]
PRFM PLDL1STRM,   [src, #48]
PRFM PLDL1STRM,   [src, #96]
PRFM PLDL1STRM,   [src, #144]

sky_prefetch:

// I/Q emit order swapped (conjugate fix): X -> v2/v3, Y -> v0/v1
SHL    v2.16b, v20.16b, #4
USHR   v3.16b, v20.16b, #4
SHL    v1.16b, v21.16b, #4
ORR    v3.16b, v3.16b, v1.16b

USHR   v0.16b, v21.16b, #4
SHL    v0.16b, v0.16b, #4
MOV    v1.16b, v22.16b
 

// I/Q emit order swapped (conjugate fix): X -> v6/v7, Y -> v4/v5
SHL    v6.16b, v23.16b, #4
USHR   v7.16b, v23.16b, #4
SHL    v5.16b, v24.16b, #4
ORR    v7.16b, v7.16b, v5.16b

USHR   v4.16b, v24.16b, #4
SHL    v4.16b, v4.16b, #4
MOV    v5.16b, v25.16b
 

// I/Q emit order swapped (conjugate fix): X -> v10/v11, Y -> v8/v9
SHL    v10.16b, v26.16b, #4
USHR   v11.16b, v26.16b, #4
SHL    v9.16b, v27.16b, #4
ORR    v11.16b, v11.16b, v9.16b

USHR   v8.16b, v27.16b, #4
SHL    v8.16b, v8.16b, #4
MOV    v9.16b, v28.16b
 

// I/Q emit order swapped (conjugate fix): X -> v14/v15, Y -> v12/v13
SHL    v14.16b, v29.16b, #4
USHR   v15.16b, v29.16b, #4
SHL    v13.16b, v30.16b, #4
ORR    v15.16b, v15.16b, v13.16b

USHR   v12.16b, v30.16b, #4
SHL    v12.16b, v12.16b, #4
MOV    v13.16b, v31.16b



// store

st4     {v0.16b, v1.16b, v2.16b, v3.16b}, [dest], #64
st4     {v4.16b, v5.16b, v6.16b, v7.16b}, [dest], #64
st4     {v8.16b, v9.16b, v10.16b, v11.16b}, [dest], #64
st4     {v12.16b, v13.16b, v14.16b, v15.16b}, [dest], #64

subs    count, count, #256

bne     loop




ret

.unreq dest
.unreq src
.unreq count



/*
 
 <?php

 for($i = 0; $i < 4; $i++) {
     
     $sreg = 20 + (3 * $i);
     
     $reg[0] = $sreg + 0;
     $reg[1] = $sreg + 1;
     $reg[2] = $sreg + 2;
     
     $v[0] = ($i*4) + 0;
     $v[1] = ($i*4) + 1;
     $v[2] = ($i*4) + 2;
     $v[3] = ($i*4) + 3;
     
     echo <<<EOF
    
    
    SHL    v{$v[0]}.16b, v{$reg[0]}.16b, #4
    USHR   v{$v[1]}.16b, v{$reg[0]}.16b, #4
    SHL    v{$v[2]}.16b, v{$reg[1]}.16b, #4
    ORR    v{$v[1]}.16b, v{$v[1]}.16b, v{$v[2]}.16b
    
    USHR   v{$v[2]}.16b, v{$reg[1]}.16b, #4
    SHL    v{$v[2]}.16b, v{$v[2]}.16b, #4
    MOV    v{$v[3]}.16b, v{$reg[2]}.16b
 
EOF;
 }
  

 
 */
