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



asm_function rfnm_remap_packed_vspa_aarch64
        add     x4, x0, 1728
        add     x5, x1, 2304
.L2:
        // load all of line 0
        ld4     {v0.16b - v3.16b}, [x1], 64
        ld4     {v4.16b - v7.16b}, [x1], 64
        // move the first extra bits into our longer-term storage
        mov     v16.16b, v0.16b
        mov     v17.16b, v4.16b
        // and store the second set of extra bits alongside
        sli     v16.16b, v2.16b, 4
        sli     v17.16b, v6.16b, 4

        sri     v2.16b, v1.16b, 4
        sri     v6.16b, v5.16b, 4
        shl     v1.16b, v1.16b, 4
        shl     v5.16b, v5.16b, 4
        sri     v1.16b, v0.16b, 4
        sri     v5.16b, v4.16b, 4
        st3     {v1.16b - v3.16b}, [x0], 48
        st3     {v5.16b - v7.16b}, [x0], 48
        // done with normal portions of line 0

        // load all of line 1
        ld4     {v0.16b - v3.16b}, [x1], 64
        ld4     {v4.16b - v7.16b}, [x1], 64
        // move the first extra bits into our longer-term storage
        mov     v18.16b, v0.16b
        mov     v19.16b, v4.16b
        // and store the second set of extra bits alongside
        sli     v18.16b, v2.16b, 4
        sli     v19.16b, v6.16b, 4

        sri     v2.16b, v1.16b, 4
        sri     v6.16b, v5.16b, 4
        shl     v1.16b, v1.16b, 4
        shl     v5.16b, v5.16b, 4
        sri     v1.16b, v0.16b, 4
        sri     v5.16b, v4.16b, 4
        st3     {v1.16b - v3.16b}, [x0], 48
        st3     {v5.16b - v7.16b}, [x0], 48
        // done with normal portions of line 1

        // load all of line 2
        ld4     {v0.16b - v3.16b}, [x1], 64
        ld4     {v4.16b - v7.16b}, [x1], 64
        // move the first extra bits into our longer-term storage
        mov     v20.16b, v0.16b
        mov     v21.16b, v4.16b
        // and store the second set of extra bits alongside
        sli     v20.16b, v2.16b, 4
        sli     v21.16b, v6.16b, 4

        sri     v2.16b, v1.16b, 4
        sri     v6.16b, v5.16b, 4
        shl     v1.16b, v1.16b, 4
        shl     v5.16b, v5.16b, 4
        sri     v1.16b, v0.16b, 4
        sri     v5.16b, v4.16b, 4
        st3     {v1.16b - v3.16b}, [x0], 48
        st3     {v5.16b - v7.16b}, [x0], 48
        // done with normal portions of line 2

        movi    v1.16b, 0xf
        mov     v0.16b, v16.16b
        bsl     v1.16b, v20.16b, v16.16b
        mov     v2.16b, v20.16b
        sli     v0.16b, v18.16b, 4
        sri     v2.16b, v18.16b, 4

        movi    v4.16b, 0xf
        mov     v3.16b, v17.16b
        bsl     v4.16b, v21.16b, v17.16b
        mov     v5.16b, v21.16b
        sli     v3.16b, v19.16b, 4
        sri     v5.16b, v19.16b, 4

        st3     {v0.16b - v2.16b}, [x4], 48
        st3     {v3.16b - v5.16b}, [x4], 48

        cmp     x5, x1
        bne     .L2
        ret