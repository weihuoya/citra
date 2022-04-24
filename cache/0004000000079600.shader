// shader: 8B31, F2622744950F3FD8

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};

layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

// accurate_mul: fast
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_191();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_190();
bool sub_148_160();
bool sub_160_185();
bool sub_161_179();
bool sub_179_184();
bool sub_191_208();
bool sub_208_284();
bool sub_247_267();
bool sub_248_253();
bool sub_253_266();
bool sub_257_260();
bool sub_260_265();
bool sub_267_280();
bool sub_268_273();
bool sub_273_279();
bool sub_284_299();
bool sub_290_295();
bool sub_292_293();
bool sub_297_298();
bool sub_460_4096();

bool exec_shader() {
    sub_460_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 8: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 13: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 17: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 18: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 19: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 22: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 26: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 27: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 28: dp3
    reg_tmp5.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 29: dp3
    reg_tmp5.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 30: dp3
    reg_tmp5.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 31: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_191() {
    // 34: mul
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_190();
    }
    // 190: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 42: mov
    reg_tmp7 = uniforms.f[90].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[90].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[90].xxxx;
    // 45: mul
    reg_tmp2 = mul_s(uniforms.f[90].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 72: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 73: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 74: call
    {
        sub_191_208();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 98: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 99: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 100: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 101: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 102: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 103: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 107: call
    {
        sub_208_284();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_190() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 142: mova
    address_registers.x = (ivec2(uniforms.f[90].ww)).x;
    // 143: dp4
    reg_tmp10.x = dot_s(uniforms.f[31], reg_tmp15);
    // 144: dp4
    reg_tmp10.y = dot_s(uniforms.f[32], reg_tmp15);
    // 145: dp4
    reg_tmp10.z = dot_s(uniforms.f[33], reg_tmp15);
    // 146: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 147: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_148_160();
    } else {
        sub_160_185();
    }
    // 185: mov
    vs_out_attr2 = -reg_tmp15;
    // 186: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 187: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 188: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 189: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_148_160() {
    // 148: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 149: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 150: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 151: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 152: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 153: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 154: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 155: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 156: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 157: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 158: call
    {
        sub_191_208();
    }
    // 159: nop
    return false;
}
bool sub_160_185() {
    // 160: ifc
    if (all(conditional_code)) {
        sub_161_179();
    } else {
        sub_179_184();
    }
    // 184: nop
    return false;
}
bool sub_161_179() {
    // 161: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 162: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 163: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 164: dp3
    reg_tmp11.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 165: dp3
    reg_tmp11.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 166: dp3
    reg_tmp11.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 167: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 168: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 169: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 170: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 172: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 173: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 174: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 175: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 176: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 177: call
    {
        sub_208_284();
    }
    // 178: nop
    return false;
}
bool sub_179_184() {
    // 179: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 180: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 181: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 182: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 183: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_191_208() {
    uint jmp_to = 191u;
    while (true) {
        switch (jmp_to) {
        case 191u: {
            // 191: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 192: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 193: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 194: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 195: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 196: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 197: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 198: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 207u; break; }
            }
            // 199: add
            reg_tmp4 = uniforms.f[90].yyyy + reg_tmp14.zzzz;
            // 200: mul
            reg_tmp4 = mul_s(uniforms.f[91].zzzz, reg_tmp4);
            // 201: cmp
            conditional_code = greaterThanEqual(uniforms.f[90].xx, reg_tmp4.xx);
            // 202: rsq
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            // 203: mul
            reg_tmp5 = mul_s(uniforms.f[91].zzzz, reg_tmp14);
            // 204: jmpc
            if (conditional_code.x) {
                { jmp_to = 207u; break; }
            }
            // 205: rcp
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            // 206: mul
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 207u: {
            // 207: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_208_284() {
    uint jmp_to = 208u;
    while (true) {
        switch (jmp_to) {
        case 208u: {
            // 208: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 209: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 210: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 211: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 212: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 213: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 214: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 215: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 216: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 217: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 283u; break; }
            }
            // 218: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 219: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 220: mul
            reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 221: mad
            reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 222: dp3
            reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
            // 223: rsq
            reg_tmp5.w = rsq_s(reg_tmp5.w);
            // 224: mul
            reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
            // 225: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 226: mul
            reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 227: mad
            reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 228: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 229: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 230: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 231: add
            reg_tmp6.w = (uniforms.f[90].yyyy + reg_tmp6).w;
            // 232: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 233: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 234: cmp
            conditional_code = lessThan(uniforms.f[91].yy, reg_tmp6.ww);
            // 235: mov
            reg_tmp6.x = (uniforms.f[90].yyyy).x;
            // 236: mov
            reg_tmp6.y = (-uniforms.f[90].yyyy).y;
            // 237: jmpc
            if (!conditional_code.x) {
                { jmp_to = 245u; break; }
            }
            // 238: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 239: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 240: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 241: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            // 242: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 243: mul
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            // 244: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 283u; break; }
            }
        }
        case 245u: {
            // 245: cmp
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            // 246: ifc
            if (conditional_code.x) {
                sub_247_267();
            } else {
                sub_267_280();
            }
            // 280: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            // 281: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 282: mul
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 283u: {
            // 283: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_247_267() {
    // 247: ifc
    if (conditional_code.y) {
        sub_248_253();
    } else {
        sub_253_266();
    }
    // 266: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_248_253() {
    // 248: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 249: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 250: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 251: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 252: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_253_266() {
    // 253: cmp
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    // 254: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 255: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 256: ifc
    if (conditional_code.x) {
        sub_257_260();
    } else {
        sub_260_265();
    }
    // 265: nop
    return false;
}
bool sub_257_260() {
    // 257: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 258: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 259: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_260_265() {
    // 260: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 261: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 262: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 263: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 264: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_267_280() {
    // 267: ifc
    if (conditional_code.y) {
        sub_268_273();
    } else {
        sub_273_279();
    }
    // 279: nop
    return false;
}
bool sub_268_273() {
    // 268: mul
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 269: add
    reg_tmp8.y = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).y;
    // 270: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 271: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 272: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_273_279() {
    // 273: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 274: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 275: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 276: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 277: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 278: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_284_299() {
    // 284: mov
    reg_tmp8.xy = (uniforms.f[90].xxxx).xy;
    // 285: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 286: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.xy);
    // 287: mov
    reg_tmp9.xyz = (uniforms.f[90].xxxx).xyz;
    // 288: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 289: ifc
    if (conditional_code.y) {
        sub_290_295();
    }
    // 295: cmp
    conditional_code = equal(uniforms.f[90].xx, reg_tmp8.xy);
    // 296: ifc
    if (all(conditional_code)) {
        sub_297_298();
    }
    // 298: max
    vs_out_attr3 = max(uniforms.f[90].xxxx, reg_tmp9);
    return false;
}
bool sub_290_295() {
    // 290: mul
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    // 291: ifu
    if (uniforms.b[7]) {
        sub_292_293();
    }
    // 293: mul
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 294: mov
    reg_tmp8.x = (uniforms.f[90].yyyy).x;
    return false;
}
bool sub_292_293() {
    // 292: mul
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_297_298() {
    // 297: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_460_4096() {
    // 460: call
    {
        sub_34_191();
    }
    // 461: call
    {
        sub_284_299();
    }
    // 462: end
    return true;
}
// reference: 94A1126E0A0C2E29, F2622744950F3FD8
// shader: 8DD9, 4BD70AD09292A3DA

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];

struct Vertex {
    vec4 attributes[4];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(0.0, 0.0);
    texcoord1 = vec2(0.0, 0.0);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);

    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[4](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0]);
    prim_buffer[1].attributes = vec4[4](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1]);
    prim_buffer[2].attributes = vec4[4](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 3901EC4BEC56958E, 4BD70AD09292A3DA
// shader: 8B30, AEDA558FB63A096E
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BF90F3870E2BDA72, AEDA558FB63A096E
// program: F2622744950F3FD8, 4BD70AD09292A3DA, AEDA558FB63A096E
// reference: A1F58204104EABF1, AEDA558FB63A096E
// reference: 1F41C5C1D1B58F6E, F2622744950F3FD8
// shader: 8B31, 485AB8989E17D97B

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};

layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;
 out vec4 vs_out_attr5;
 out vec4 vs_out_attr6;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr5 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr6 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

// accurate_mul: fast
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

bool sub_0_4096();
bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_191();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_190();
bool sub_148_160();
bool sub_160_185();
bool sub_161_179();
bool sub_179_184();
bool sub_191_208();
bool sub_208_284();
bool sub_247_267();
bool sub_248_253();
bool sub_253_266();
bool sub_257_260();
bool sub_260_265();
bool sub_267_280();
bool sub_268_273();
bool sub_273_279();
bool sub_284_299();
bool sub_290_295();
bool sub_292_293();
bool sub_297_298();
bool sub_299_326();
bool sub_301_305();
bool sub_305_324();
bool sub_308_314();
bool sub_314_323();
bool sub_315_319();
bool sub_319_322();
bool sub_326_339();
bool sub_327_328();
bool sub_328_335();
bool sub_331_333();
bool sub_333_334();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_358_392();
bool sub_392_420();
bool sub_394_400();
bool sub_400_419();
bool sub_401_417();
bool sub_405_412();
bool sub_412_415();
bool sub_417_418();
bool sub_420_425();
bool sub_425_444();
bool sub_427_433();
bool sub_433_443();
bool sub_434_441();
bool sub_441_442();
bool sub_444_449();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: call
    {
        sub_34_191();
    }
    // 1: call
    {
        sub_284_299();
    }
    // 2: call
    {
        sub_299_326();
    }
    // 3: call
    {
        sub_392_420();
    }
    // 4: call
    {
        sub_425_444();
    }
    // 5: end
    return true;
}
bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 8: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 13: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 17: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 18: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 19: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 22: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 26: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 27: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 28: dp3
    reg_tmp5.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 29: dp3
    reg_tmp5.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 30: dp3
    reg_tmp5.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 31: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_191() {
    // 34: mul
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_190();
    }
    // 190: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 42: mov
    reg_tmp7 = uniforms.f[90].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[90].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[90].xxxx;
    // 45: mul
    reg_tmp2 = mul_s(uniforms.f[90].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 72: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 73: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 74: call
    {
        sub_191_208();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 98: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 99: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 100: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 101: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 102: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 103: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 107: call
    {
        sub_208_284();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_190() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 142: mova
    address_registers.x = (ivec2(uniforms.f[90].ww)).x;
    // 143: dp4
    reg_tmp10.x = dot_s(uniforms.f[31], reg_tmp15);
    // 144: dp4
    reg_tmp10.y = dot_s(uniforms.f[32], reg_tmp15);
    // 145: dp4
    reg_tmp10.z = dot_s(uniforms.f[33], reg_tmp15);
    // 146: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 147: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_148_160();
    } else {
        sub_160_185();
    }
    // 185: mov
    vs_out_attr2 = -reg_tmp15;
    // 186: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 187: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 188: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 189: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_148_160() {
    // 148: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 149: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 150: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 151: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 152: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 153: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 154: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 155: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 156: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 157: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 158: call
    {
        sub_191_208();
    }
    // 159: nop
    return false;
}
bool sub_160_185() {
    // 160: ifc
    if (all(conditional_code)) {
        sub_161_179();
    } else {
        sub_179_184();
    }
    // 184: nop
    return false;
}
bool sub_161_179() {
    // 161: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 162: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 163: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 164: dp3
    reg_tmp11.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 165: dp3
    reg_tmp11.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 166: dp3
    reg_tmp11.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 167: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 168: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 169: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 170: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 172: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 173: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 174: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 175: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 176: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 177: call
    {
        sub_208_284();
    }
    // 178: nop
    return false;
}
bool sub_179_184() {
    // 179: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 180: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 181: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 182: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 183: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_191_208() {
    uint jmp_to = 191u;
    while (true) {
        switch (jmp_to) {
        case 191u: {
            // 191: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 192: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 193: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 194: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 195: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 196: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 197: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 198: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 207u; break; }
            }
            // 199: add
            reg_tmp4 = uniforms.f[90].yyyy + reg_tmp14.zzzz;
            // 200: mul
            reg_tmp4 = mul_s(uniforms.f[91].zzzz, reg_tmp4);
            // 201: cmp
            conditional_code = greaterThanEqual(uniforms.f[90].xx, reg_tmp4.xx);
            // 202: rsq
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            // 203: mul
            reg_tmp5 = mul_s(uniforms.f[91].zzzz, reg_tmp14);
            // 204: jmpc
            if (conditional_code.x) {
                { jmp_to = 207u; break; }
            }
            // 205: rcp
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            // 206: mul
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 207u: {
            // 207: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_208_284() {
    uint jmp_to = 208u;
    while (true) {
        switch (jmp_to) {
        case 208u: {
            // 208: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 209: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 210: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 211: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 212: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 213: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 214: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 215: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 216: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 217: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 283u; break; }
            }
            // 218: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 219: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 220: mul
            reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 221: mad
            reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 222: dp3
            reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
            // 223: rsq
            reg_tmp5.w = rsq_s(reg_tmp5.w);
            // 224: mul
            reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
            // 225: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 226: mul
            reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 227: mad
            reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 228: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 229: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 230: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 231: add
            reg_tmp6.w = (uniforms.f[90].yyyy + reg_tmp6).w;
            // 232: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 233: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 234: cmp
            conditional_code = lessThan(uniforms.f[91].yy, reg_tmp6.ww);
            // 235: mov
            reg_tmp6.x = (uniforms.f[90].yyyy).x;
            // 236: mov
            reg_tmp6.y = (-uniforms.f[90].yyyy).y;
            // 237: jmpc
            if (!conditional_code.x) {
                { jmp_to = 245u; break; }
            }
            // 238: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 239: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 240: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 241: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            // 242: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 243: mul
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            // 244: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 283u; break; }
            }
        }
        case 245u: {
            // 245: cmp
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            // 246: ifc
            if (conditional_code.x) {
                sub_247_267();
            } else {
                sub_267_280();
            }
            // 280: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            // 281: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 282: mul
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 283u: {
            // 283: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_247_267() {
    // 247: ifc
    if (conditional_code.y) {
        sub_248_253();
    } else {
        sub_253_266();
    }
    // 266: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_248_253() {
    // 248: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 249: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 250: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 251: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 252: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_253_266() {
    // 253: cmp
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    // 254: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 255: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 256: ifc
    if (conditional_code.x) {
        sub_257_260();
    } else {
        sub_260_265();
    }
    // 265: nop
    return false;
}
bool sub_257_260() {
    // 257: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 258: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 259: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_260_265() {
    // 260: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 261: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 262: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 263: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 264: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_267_280() {
    // 267: ifc
    if (conditional_code.y) {
        sub_268_273();
    } else {
        sub_273_279();
    }
    // 279: nop
    return false;
}
bool sub_268_273() {
    // 268: mul
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 269: add
    reg_tmp8.y = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).y;
    // 270: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 271: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 272: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_273_279() {
    // 273: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 274: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 275: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 276: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 277: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 278: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_284_299() {
    // 284: mov
    reg_tmp8.xy = (uniforms.f[90].xxxx).xy;
    // 285: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 286: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.xy);
    // 287: mov
    reg_tmp9.xyz = (uniforms.f[90].xxxx).xyz;
    // 288: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 289: ifc
    if (conditional_code.y) {
        sub_290_295();
    }
    // 295: cmp
    conditional_code = equal(uniforms.f[90].xx, reg_tmp8.xy);
    // 296: ifc
    if (all(conditional_code)) {
        sub_297_298();
    }
    // 298: max
    vs_out_attr3 = max(uniforms.f[90].xxxx, reg_tmp9);
    return false;
}
bool sub_290_295() {
    // 290: mul
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    // 291: ifu
    if (uniforms.b[7]) {
        sub_292_293();
    }
    // 293: mul
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 294: mov
    reg_tmp8.x = (uniforms.f[90].yyyy).x;
    return false;
}
bool sub_292_293() {
    // 292: mul
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_297_298() {
    // 297: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_299_326() {
    // 299: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 300: ifu
    if (uniforms.b[9]) {
        sub_301_305();
    } else {
        sub_305_324();
    }
    // 324: callu
    if (uniforms.b[2]) {
        sub_326_339();
    }
    // 325: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_301_305() {
    // 301: call
    {
        sub_339_347();
    }
    // 302: dp4
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    // 303: dp4
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    // 304: mov
    reg_tmp3.zw = (uniforms.f[90].xxxx).zw;
    return false;
}
bool sub_305_324() {
    // 305: cmp
    conditional_code = equal(uniforms.f[92].xy, reg_tmp0.xy);
    // 306: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 307: ifc
    if (all(not(conditional_code))) {
        sub_308_314();
    } else {
        sub_314_323();
    }
    // 323: nop
    return false;
}
bool sub_308_314() {
    // 308: mov
    reg_tmp6 = reg_tmp10;
    // 309: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 310: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    // 311: dp4
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    // 312: mul
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 313: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_314_323() {
    // 314: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_315_319();
    } else {
        sub_319_322();
    }
    // 322: nop
    return false;
}
bool sub_315_319() {
    // 315: call
    {
        sub_347_354();
    }
    // 316: dp3
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    // 317: dp3
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    // 318: dp3
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_319_322() {
    // 319: call
    {
        sub_354_358();
    }
    // 320: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 321: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_326_339() {
    // 326: ifu
    if (uniforms.b[9]) {
        sub_327_328();
    } else {
        sub_328_335();
    }
    // 335: mov
    reg_tmp0 = reg_tmp3;
    // 336: mova
    address_registers.x = (ivec2(uniforms.f[90].xx)).x;
    // 337: call
    {
        sub_358_392();
    }
    // 338: mov
    reg_tmp3 = reg_tmp0;
    return false;
}
bool sub_327_328() {
    // 327: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_328_335() {
    // 328: mov
    reg_tmp7 = uniforms.f[11];
    // 329: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp7.zz);
    // 330: ifc
    if (all(conditional_code)) {
        sub_331_333();
    } else {
        sub_333_334();
    }
    // 334: nop
    return false;
}
bool sub_331_333() {
    // 331: mov
    reg_tmp7.xy = (reg_tmp10.xzzz).xy;
    // 332: add
    reg_tmp7.y = (reg_tmp7.yyyy + reg_tmp10.yyyy).y;
    return false;
}
bool sub_333_334() {
    // 333: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(uniforms.f[90].yz, reg_tmp0.xy);
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    // 349: rsq
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    // 350: mul
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[91].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[90].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_358_392() {
    // 358: mov
    reg_tmp6.zw = (uniforms.f[22].xxxx).zw;
    // 359: mul
    reg_tmp6.z = (mul_s(uniforms.f[24 + address_registers.x].zzzz, reg_tmp6.zzzz)).z;
    // 360: mul
    reg_tmp6.w = (mul_s(uniforms.f[24 + address_registers.x].wwww, reg_tmp6.wwww)).w;
    // 361: add
    reg_tmp6.xy = (-uniforms.f[91].zzzz + reg_tmp7.yxxx).xy;
    // 362: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].yzzz, reg_tmp7.xyyy)).xy;
    // 363: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].xxxx, reg_tmp6.zwww)).xy;
    // 364: add
    reg_tmp6.xy = (uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 365: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].wwww, reg_tmp6.xyyy)).xy;
    // 366: flr
    reg_tmp7.xy = (floor(reg_tmp6.xyyy)).xy;
    // 367: add
    reg_tmp7.xy = (reg_tmp6.xyyy + -reg_tmp7).xy;
    // 368: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].xxxx, reg_tmp7.xyyy)).xy;
    // 369: add
    reg_tmp6.xy = (-uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 370: mov
    reg_tmp8 = uniforms.f[95];
    // 371: mov
    reg_tmp9 = uniforms.f[94];
    // 372: mul
    reg_tmp7.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
    // 373: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 374: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 375: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 376: mov
    reg_tmp8 = uniforms.f[93];
    // 377: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 378: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 379: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.xxxx)).y;
    // 380: mad
    reg_tmp0.x = (fma_s(reg_tmp7.xxxx, uniforms.f[24 + address_registers.x].xxxx, reg_tmp0.xxxx)).x;
    // 381: mov
    reg_tmp8 = uniforms.f[95];
    // 382: mov
    reg_tmp9 = uniforms.f[94];
    // 383: mul
    reg_tmp7.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
    // 384: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 385: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 386: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 387: mov
    reg_tmp8 = uniforms.f[93];
    // 388: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 389: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 390: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.yyyy)).y;
    // 391: mad
    reg_tmp0.y = (fma_s(reg_tmp7.yyyy, uniforms.f[24 + address_registers.x].yyyy, reg_tmp0.yyyy)).y;
    return false;
}
bool sub_392_420() {
    // 392: mov
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    // 393: ifu
    if (uniforms.b[10]) {
        sub_394_400();
    } else {
        sub_400_419();
    }
    // 419: nop
    return false;
}
bool sub_394_400() {
    // 394: call
    {
        sub_339_347();
    }
    // 395: dp4
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    // 396: dp4
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    // 397: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 398: callu
    if (uniforms.b[3]) {
        sub_420_425();
    }
    // 399: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_400_419() {
    // 400: ifu
    if (uniforms.b[13]) {
        sub_401_417();
    } else {
        sub_417_418();
    }
    // 418: nop
    return false;
}
bool sub_401_417() {
    // 401: cmp
    conditional_code = equal(uniforms.f[92].xy, reg_tmp0.xy);
    // 402: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 403: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 404: ifc
    if (all(not(conditional_code))) {
        sub_405_412();
    } else {
        sub_412_415();
    }
    // 415: callu
    if (uniforms.b[3]) {
        sub_420_425();
    }
    // 416: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_405_412() {
    // 405: mov
    reg_tmp6 = reg_tmp10;
    // 406: dp4
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    // 407: dp4
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    // 408: dp4
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    // 409: rcp
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    // 410: mul
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    // 411: add
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_412_415() {
    // 412: call
    {
        sub_354_358();
    }
    // 413: dp4
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    // 414: dp4
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_417_418() {
    // 417: mov
    vs_out_attr5 = uniforms.f[90].xxxx;
    return false;
}
bool sub_420_425() {
    // 420: mov
    reg_tmp7.xy = (reg_tmp4.xyyy).xy;
    // 421: mov
    reg_tmp0 = reg_tmp4;
    // 422: mova
    address_registers.x = (ivec2(uniforms.f[90].zz)).x;
    // 423: call
    {
        sub_358_392();
    }
    // 424: mov
    reg_tmp4 = reg_tmp0;
    return false;
}
bool sub_425_444() {
    // 425: mov
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    // 426: ifu
    if (uniforms.b[11]) {
        sub_427_433();
    } else {
        sub_433_443();
    }
    // 443: nop
    return false;
}
bool sub_427_433() {
    // 427: call
    {
        sub_339_347();
    }
    // 428: dp4
    reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
    // 429: dp4
    reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
    // 430: mov
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    // 431: callu
    if (uniforms.b[4]) {
        sub_444_449();
    }
    // 432: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_433_443() {
    // 433: ifu
    if (uniforms.b[14]) {
        sub_434_441();
    } else {
        sub_441_442();
    }
    // 442: nop
    return false;
}
bool sub_434_441() {
    // 434: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 435: mov
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    // 436: call
    {
        sub_354_358();
    }
    // 437: dp4
    reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
    // 438: dp4
    reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
    // 439: callu
    if (uniforms.b[4]) {
        sub_444_449();
    }
    // 440: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_441_442() {
    // 441: mov
    vs_out_attr6 = uniforms.f[90].xxxx;
    return false;
}
bool sub_444_449() {
    // 444: mov
    reg_tmp7.xy = (reg_tmp5.xyyy).xy;
    // 445: mov
    reg_tmp0 = reg_tmp5;
    // 446: mova
    address_registers.x = (ivec2(uniforms.f[92].yy)).x;
    // 447: call
    {
        sub_358_392();
    }
    // 448: mov
    reg_tmp5 = reg_tmp0;
    return false;
}
// reference: 287550ED69E1F1DC, 485AB8989E17D97B
// shader: 8DD9, AD7922A63ED54CA7

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];
 in vec4 vs_out_attr5[];
 in vec4 vs_out_attr6[];

struct Vertex {
    vec4 attributes[7];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = vtx.attributes[4].z;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);

    texcoord2 = vec2(vtx.attributes[6].x, vtx.attributes[6].y);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[7](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0], vs_out_attr6[0]);
    prim_buffer[1].attributes = vec4[7](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1], vs_out_attr6[1]);
    prim_buffer[2].attributes = vec4[7](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2], vs_out_attr6[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: FC74FA4ACA1C8C74, AD7922A63ED54CA7
// shader: 8B30, BCEB383EA713B553
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((texcolor2.rgb) + (const_color[3].aaa), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F82A463247AFDDA8, BCEB383EA713B553
// program: 485AB8989E17D97B, AD7922A63ED54CA7, BCEB383EA713B553
// shader: 8B31, 4F9DAEA1DCDB2FC6

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};

layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

// accurate_mul: fast
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_191();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_190();
bool sub_148_160();
bool sub_160_185();
bool sub_161_179();
bool sub_179_184();
bool sub_191_208();
bool sub_208_284();
bool sub_247_267();
bool sub_248_253();
bool sub_253_266();
bool sub_257_260();
bool sub_260_265();
bool sub_267_280();
bool sub_268_273();
bool sub_273_279();
bool sub_284_299();
bool sub_290_295();
bool sub_292_293();
bool sub_297_298();
bool sub_299_326();
bool sub_301_305();
bool sub_305_324();
bool sub_308_314();
bool sub_314_323();
bool sub_315_319();
bool sub_319_322();
bool sub_326_339();
bool sub_327_328();
bool sub_328_335();
bool sub_331_333();
bool sub_333_334();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_358_392();
bool sub_455_4096();

bool exec_shader() {
    sub_455_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 8: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 13: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 17: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 18: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 19: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 22: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 26: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 27: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 28: dp3
    reg_tmp5.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 29: dp3
    reg_tmp5.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 30: dp3
    reg_tmp5.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 31: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_191() {
    // 34: mul
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_190();
    }
    // 190: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 42: mov
    reg_tmp7 = uniforms.f[90].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[90].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[90].xxxx;
    // 45: mul
    reg_tmp2 = mul_s(uniforms.f[90].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 72: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 73: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 74: call
    {
        sub_191_208();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 98: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 99: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 100: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 101: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 102: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 103: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 107: call
    {
        sub_208_284();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_190() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 142: mova
    address_registers.x = (ivec2(uniforms.f[90].ww)).x;
    // 143: dp4
    reg_tmp10.x = dot_s(uniforms.f[31], reg_tmp15);
    // 144: dp4
    reg_tmp10.y = dot_s(uniforms.f[32], reg_tmp15);
    // 145: dp4
    reg_tmp10.z = dot_s(uniforms.f[33], reg_tmp15);
    // 146: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 147: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_148_160();
    } else {
        sub_160_185();
    }
    // 185: mov
    vs_out_attr2 = -reg_tmp15;
    // 186: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 187: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 188: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 189: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_148_160() {
    // 148: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 149: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 150: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 151: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 152: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 153: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 154: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 155: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 156: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 157: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 158: call
    {
        sub_191_208();
    }
    // 159: nop
    return false;
}
bool sub_160_185() {
    // 160: ifc
    if (all(conditional_code)) {
        sub_161_179();
    } else {
        sub_179_184();
    }
    // 184: nop
    return false;
}
bool sub_161_179() {
    // 161: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 162: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 163: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 164: dp3
    reg_tmp11.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 165: dp3
    reg_tmp11.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 166: dp3
    reg_tmp11.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 167: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 168: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 169: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 170: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 172: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 173: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 174: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 175: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 176: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 177: call
    {
        sub_208_284();
    }
    // 178: nop
    return false;
}
bool sub_179_184() {
    // 179: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 180: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 181: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 182: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 183: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_191_208() {
    uint jmp_to = 191u;
    while (true) {
        switch (jmp_to) {
        case 191u: {
            // 191: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 192: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 193: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 194: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 195: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 196: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 197: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 198: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 207u; break; }
            }
            // 199: add
            reg_tmp4 = uniforms.f[90].yyyy + reg_tmp14.zzzz;
            // 200: mul
            reg_tmp4 = mul_s(uniforms.f[91].zzzz, reg_tmp4);
            // 201: cmp
            conditional_code = greaterThanEqual(uniforms.f[90].xx, reg_tmp4.xx);
            // 202: rsq
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            // 203: mul
            reg_tmp5 = mul_s(uniforms.f[91].zzzz, reg_tmp14);
            // 204: jmpc
            if (conditional_code.x) {
                { jmp_to = 207u; break; }
            }
            // 205: rcp
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            // 206: mul
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 207u: {
            // 207: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_208_284() {
    uint jmp_to = 208u;
    while (true) {
        switch (jmp_to) {
        case 208u: {
            // 208: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 209: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 210: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 211: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 212: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 213: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 214: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 215: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 216: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 217: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 283u; break; }
            }
            // 218: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 219: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 220: mul
            reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 221: mad
            reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 222: dp3
            reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
            // 223: rsq
            reg_tmp5.w = rsq_s(reg_tmp5.w);
            // 224: mul
            reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
            // 225: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 226: mul
            reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 227: mad
            reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 228: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 229: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 230: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 231: add
            reg_tmp6.w = (uniforms.f[90].yyyy + reg_tmp6).w;
            // 232: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 233: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 234: cmp
            conditional_code = lessThan(uniforms.f[91].yy, reg_tmp6.ww);
            // 235: mov
            reg_tmp6.x = (uniforms.f[90].yyyy).x;
            // 236: mov
            reg_tmp6.y = (-uniforms.f[90].yyyy).y;
            // 237: jmpc
            if (!conditional_code.x) {
                { jmp_to = 245u; break; }
            }
            // 238: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 239: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 240: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 241: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            // 242: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 243: mul
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            // 244: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 283u; break; }
            }
        }
        case 245u: {
            // 245: cmp
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            // 246: ifc
            if (conditional_code.x) {
                sub_247_267();
            } else {
                sub_267_280();
            }
            // 280: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            // 281: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 282: mul
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 283u: {
            // 283: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_247_267() {
    // 247: ifc
    if (conditional_code.y) {
        sub_248_253();
    } else {
        sub_253_266();
    }
    // 266: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_248_253() {
    // 248: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 249: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 250: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 251: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 252: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_253_266() {
    // 253: cmp
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    // 254: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 255: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 256: ifc
    if (conditional_code.x) {
        sub_257_260();
    } else {
        sub_260_265();
    }
    // 265: nop
    return false;
}
bool sub_257_260() {
    // 257: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 258: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 259: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_260_265() {
    // 260: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 261: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 262: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 263: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 264: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_267_280() {
    // 267: ifc
    if (conditional_code.y) {
        sub_268_273();
    } else {
        sub_273_279();
    }
    // 279: nop
    return false;
}
bool sub_268_273() {
    // 268: mul
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 269: add
    reg_tmp8.y = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).y;
    // 270: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 271: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 272: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_273_279() {
    // 273: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 274: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 275: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 276: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 277: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 278: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_284_299() {
    // 284: mov
    reg_tmp8.xy = (uniforms.f[90].xxxx).xy;
    // 285: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 286: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.xy);
    // 287: mov
    reg_tmp9.xyz = (uniforms.f[90].xxxx).xyz;
    // 288: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 289: ifc
    if (conditional_code.y) {
        sub_290_295();
    }
    // 295: cmp
    conditional_code = equal(uniforms.f[90].xx, reg_tmp8.xy);
    // 296: ifc
    if (all(conditional_code)) {
        sub_297_298();
    }
    // 298: max
    vs_out_attr3 = max(uniforms.f[90].xxxx, reg_tmp9);
    return false;
}
bool sub_290_295() {
    // 290: mul
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    // 291: ifu
    if (uniforms.b[7]) {
        sub_292_293();
    }
    // 293: mul
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 294: mov
    reg_tmp8.x = (uniforms.f[90].yyyy).x;
    return false;
}
bool sub_292_293() {
    // 292: mul
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_297_298() {
    // 297: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_299_326() {
    // 299: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 300: ifu
    if (uniforms.b[9]) {
        sub_301_305();
    } else {
        sub_305_324();
    }
    // 324: callu
    if (uniforms.b[2]) {
        sub_326_339();
    }
    // 325: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_301_305() {
    // 301: call
    {
        sub_339_347();
    }
    // 302: dp4
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    // 303: dp4
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    // 304: mov
    reg_tmp3.zw = (uniforms.f[90].xxxx).zw;
    return false;
}
bool sub_305_324() {
    // 305: cmp
    conditional_code = equal(uniforms.f[92].xy, reg_tmp0.xy);
    // 306: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 307: ifc
    if (all(not(conditional_code))) {
        sub_308_314();
    } else {
        sub_314_323();
    }
    // 323: nop
    return false;
}
bool sub_308_314() {
    // 308: mov
    reg_tmp6 = reg_tmp10;
    // 309: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 310: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    // 311: dp4
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    // 312: mul
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 313: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_314_323() {
    // 314: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_315_319();
    } else {
        sub_319_322();
    }
    // 322: nop
    return false;
}
bool sub_315_319() {
    // 315: call
    {
        sub_347_354();
    }
    // 316: dp3
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    // 317: dp3
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    // 318: dp3
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_319_322() {
    // 319: call
    {
        sub_354_358();
    }
    // 320: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 321: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_326_339() {
    // 326: ifu
    if (uniforms.b[9]) {
        sub_327_328();
    } else {
        sub_328_335();
    }
    // 335: mov
    reg_tmp0 = reg_tmp3;
    // 336: mova
    address_registers.x = (ivec2(uniforms.f[90].xx)).x;
    // 337: call
    {
        sub_358_392();
    }
    // 338: mov
    reg_tmp3 = reg_tmp0;
    return false;
}
bool sub_327_328() {
    // 327: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_328_335() {
    // 328: mov
    reg_tmp7 = uniforms.f[11];
    // 329: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp7.zz);
    // 330: ifc
    if (all(conditional_code)) {
        sub_331_333();
    } else {
        sub_333_334();
    }
    // 334: nop
    return false;
}
bool sub_331_333() {
    // 331: mov
    reg_tmp7.xy = (reg_tmp10.xzzz).xy;
    // 332: add
    reg_tmp7.y = (reg_tmp7.yyyy + reg_tmp10.yyyy).y;
    return false;
}
bool sub_333_334() {
    // 333: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(uniforms.f[90].yz, reg_tmp0.xy);
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    // 349: rsq
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    // 350: mul
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[91].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[90].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_358_392() {
    // 358: mov
    reg_tmp6.zw = (uniforms.f[22].xxxx).zw;
    // 359: mul
    reg_tmp6.z = (mul_s(uniforms.f[24 + address_registers.x].zzzz, reg_tmp6.zzzz)).z;
    // 360: mul
    reg_tmp6.w = (mul_s(uniforms.f[24 + address_registers.x].wwww, reg_tmp6.wwww)).w;
    // 361: add
    reg_tmp6.xy = (-uniforms.f[91].zzzz + reg_tmp7.yxxx).xy;
    // 362: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].yzzz, reg_tmp7.xyyy)).xy;
    // 363: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].xxxx, reg_tmp6.zwww)).xy;
    // 364: add
    reg_tmp6.xy = (uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 365: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].wwww, reg_tmp6.xyyy)).xy;
    // 366: flr
    reg_tmp7.xy = (floor(reg_tmp6.xyyy)).xy;
    // 367: add
    reg_tmp7.xy = (reg_tmp6.xyyy + -reg_tmp7).xy;
    // 368: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].xxxx, reg_tmp7.xyyy)).xy;
    // 369: add
    reg_tmp6.xy = (-uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 370: mov
    reg_tmp8 = uniforms.f[95];
    // 371: mov
    reg_tmp9 = uniforms.f[94];
    // 372: mul
    reg_tmp7.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
    // 373: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 374: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 375: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 376: mov
    reg_tmp8 = uniforms.f[93];
    // 377: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 378: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 379: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.xxxx)).y;
    // 380: mad
    reg_tmp0.x = (fma_s(reg_tmp7.xxxx, uniforms.f[24 + address_registers.x].xxxx, reg_tmp0.xxxx)).x;
    // 381: mov
    reg_tmp8 = uniforms.f[95];
    // 382: mov
    reg_tmp9 = uniforms.f[94];
    // 383: mul
    reg_tmp7.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
    // 384: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 385: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 386: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 387: mov
    reg_tmp8 = uniforms.f[93];
    // 388: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 389: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 390: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.yyyy)).y;
    // 391: mad
    reg_tmp0.y = (fma_s(reg_tmp7.yyyy, uniforms.f[24 + address_registers.x].yyyy, reg_tmp0.yyyy)).y;
    return false;
}
bool sub_455_4096() {
    // 455: call
    {
        sub_34_191();
    }
    // 456: call
    {
        sub_284_299();
    }
    // 457: call
    {
        sub_299_326();
    }
    // 458: end
    return true;
}
// reference: 266DA58D5AC31F5E, 4F9DAEA1DCDB2FC6
// shader: 8DD9, 6CF3F3B70E23AA85

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];

struct Vertex {
    vec4 attributes[5];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(0.0, 0.0);

    texcoord0_w = vtx.attributes[4].z;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);

    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[5](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0]);
    prim_buffer[1].attributes = vec4[5](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1]);
    prim_buffer[2].attributes = vec4[5](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: CD8210802464D9AE, 6CF3F3B70E23AA85
// shader: 8B30, F5F586805D7D88AD
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D9A939CBF5CBABCB, F5F586805D7D88AD
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, F5F586805D7D88AD
// reference: AD8D7222817ABE19, 4F9DAEA1DCDB2FC6
// shader: 8B30, C1FEDE1C4F0B4CB9
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D9A939CBA6CE08A2, C1FEDE1C4F0B4CB9
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, C1FEDE1C4F0B4CB9
// shader: 8B30, F4F521EBD49743B3
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CE312E68BF687954, F4F521EBD49743B3
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, F4F521EBD49743B3
// shader: 8B30, AFD6A906439B3437
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.b) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1B73E29D56F3C0B7, AFD6A906439B3437
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, AFD6A906439B3437
// shader: 8B31, 1A2F11F6C34A4059

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout (std140) uniform vs_config {
    pica_uniforms uniforms;
};

layout(location = 0) in vec4 vs_in_reg0;
layout(location = 1) in vec4 vs_in_reg1;
layout(location = 2) in vec4 vs_in_reg2;
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;
layout(location = 8) in vec4 vs_in_reg8;

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;
 out vec4 vs_out_attr3;
 out vec4 vs_out_attr4;
 out vec4 vs_out_attr5;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr3 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr4 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr5 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

// accurate_mul: fast
bvec2 conditional_code = bvec2(false);
ivec3 address_registers = ivec3(0);
vec4 reg_tmp0 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp1 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp2 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp3 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp4 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp5 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp6 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp7 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp8 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp9 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp10 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp11 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp12 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp13 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp14 = vec4(0.0, 0.0, 0.0, 1.0);
vec4 reg_tmp15 = vec4(0.0, 0.0, 0.0, 1.0);

bool sub_7_12();
bool sub_12_21();
bool sub_21_34();
bool sub_34_191();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_190();
bool sub_148_160();
bool sub_160_185();
bool sub_161_179();
bool sub_179_184();
bool sub_191_208();
bool sub_208_284();
bool sub_247_267();
bool sub_248_253();
bool sub_253_266();
bool sub_257_260();
bool sub_260_265();
bool sub_267_280();
bool sub_268_273();
bool sub_273_279();
bool sub_284_299();
bool sub_290_295();
bool sub_292_293();
bool sub_297_298();
bool sub_299_326();
bool sub_301_305();
bool sub_305_324();
bool sub_308_314();
bool sub_314_323();
bool sub_315_319();
bool sub_319_322();
bool sub_326_339();
bool sub_327_328();
bool sub_328_335();
bool sub_331_333();
bool sub_333_334();
bool sub_339_347();
bool sub_341_342();
bool sub_342_346();
bool sub_343_344();
bool sub_344_345();
bool sub_347_354();
bool sub_354_358();
bool sub_358_392();
bool sub_392_420();
bool sub_394_400();
bool sub_400_419();
bool sub_401_417();
bool sub_405_412();
bool sub_412_415();
bool sub_417_418();
bool sub_420_425();
bool sub_449_4096();

bool exec_shader() {
    sub_449_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 8: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 13: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 17: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 18: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 19: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xx)).x;
    // 22: dp4
    reg_tmp3.x = dot_s(uniforms.f[31 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot_s(uniforms.f[32 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot_s(uniforms.f[33 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 26: dp3
    reg_tmp4.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 27: dp3
    reg_tmp4.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 28: dp3
    reg_tmp5.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 29: dp3
    reg_tmp5.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 30: dp3
    reg_tmp5.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 31: mad
    reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_191() {
    // 34: mul
    reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_190();
    }
    // 190: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 42: mov
    reg_tmp7 = uniforms.f[90].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[90].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[90].xxxx;
    // 45: mul
    reg_tmp2 = mul_s(uniforms.f[90].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 72: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 73: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 74: call
    {
        sub_191_208();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 60: callc
    if (conditional_code.y) {
        sub_12_21();
    }
    // 61: nop
    return false;
}
bool sub_76_135() {
    // 76: ifc
    if (all(conditional_code)) {
        sub_77_109();
    } else {
        sub_109_134();
    }
    // 134: nop
    return false;
}
bool sub_77_109() {
    // 77: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 98: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 99: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 100: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 101: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 102: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 103: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 107: call
    {
        sub_208_284();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(uniforms.f[90].xx, vs_in_reg8.zw);
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[90].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_190() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.yz);
    // 142: mova
    address_registers.x = (ivec2(uniforms.f[90].ww)).x;
    // 143: dp4
    reg_tmp10.x = dot_s(uniforms.f[31], reg_tmp15);
    // 144: dp4
    reg_tmp10.y = dot_s(uniforms.f[32], reg_tmp15);
    // 145: dp4
    reg_tmp10.z = dot_s(uniforms.f[33], reg_tmp15);
    // 146: mov
    reg_tmp10.w = (uniforms.f[90].yyyy).w;
    // 147: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_148_160();
    } else {
        sub_160_185();
    }
    // 185: mov
    vs_out_attr2 = -reg_tmp15;
    // 186: dp4
    vs_out_attr0.x = dot_s(uniforms.f[82], reg_tmp15);
    // 187: dp4
    vs_out_attr0.y = dot_s(uniforms.f[83], reg_tmp15);
    // 188: dp4
    vs_out_attr0.z = dot_s(uniforms.f[84], reg_tmp15);
    // 189: dp4
    vs_out_attr0.w = dot_s(uniforms.f[85], reg_tmp15);
    return false;
}
bool sub_148_160() {
    // 148: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 149: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 150: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 151: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 152: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 153: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 154: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 155: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 156: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 157: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 158: call
    {
        sub_191_208();
    }
    // 159: nop
    return false;
}
bool sub_160_185() {
    // 160: ifc
    if (all(conditional_code)) {
        sub_161_179();
    } else {
        sub_179_184();
    }
    // 184: nop
    return false;
}
bool sub_161_179() {
    // 161: dp3
    reg_tmp12.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp14.xyz);
    // 162: dp3
    reg_tmp12.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp14.xyz);
    // 163: dp3
    reg_tmp12.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp14.xyz);
    // 164: dp3
    reg_tmp11.x = dot_3(uniforms.f[31 + address_registers.x].xyz, reg_tmp13.xyz);
    // 165: dp3
    reg_tmp11.y = dot_3(uniforms.f[32 + address_registers.x].xyz, reg_tmp13.xyz);
    // 166: dp3
    reg_tmp11.z = dot_3(uniforms.f[33 + address_registers.x].xyz, reg_tmp13.xyz);
    // 167: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 168: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 169: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 170: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 171: dp3
    reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
    // 172: dp3
    reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
    // 173: dp3
    reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
    // 174: dp3
    reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
    // 175: dp3
    reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
    // 176: dp3
    reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
    // 177: call
    {
        sub_208_284();
    }
    // 178: nop
    return false;
}
bool sub_179_184() {
    // 179: dp4
    reg_tmp15.x = dot_s(uniforms.f[86], reg_tmp10);
    // 180: dp4
    reg_tmp15.y = dot_s(uniforms.f[87], reg_tmp10);
    // 181: dp4
    reg_tmp15.z = dot_s(uniforms.f[88], reg_tmp10);
    // 182: mov
    reg_tmp15.w = (uniforms.f[90].yyyy).w;
    // 183: mov
    vs_out_attr1 = uniforms.f[90].xxxx;
    return false;
}
bool sub_191_208() {
    uint jmp_to = 191u;
    while (true) {
        switch (jmp_to) {
        case 191u: {
            // 191: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 192: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 193: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 194: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 195: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 196: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 197: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 198: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 207u; break; }
            }
            // 199: add
            reg_tmp4 = uniforms.f[90].yyyy + reg_tmp14.zzzz;
            // 200: mul
            reg_tmp4 = mul_s(uniforms.f[91].zzzz, reg_tmp4);
            // 201: cmp
            conditional_code = greaterThanEqual(uniforms.f[90].xx, reg_tmp4.xx);
            // 202: rsq
            reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
            // 203: mul
            reg_tmp5 = mul_s(uniforms.f[91].zzzz, reg_tmp14);
            // 204: jmpc
            if (conditional_code.x) {
                { jmp_to = 207u; break; }
            }
            // 205: rcp
            reg_tmp0.z = rcp_s(reg_tmp4.x);
            // 206: mul
            reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
        }
        case 207u: {
            // 207: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_208_284() {
    uint jmp_to = 208u;
    while (true) {
        switch (jmp_to) {
        case 208u: {
            // 208: dp3
            reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
            // 209: dp3
            reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
            // 210: rsq
            reg_tmp6.x = rsq_s(reg_tmp6.x);
            // 211: rsq
            reg_tmp7.x = rsq_s(reg_tmp7.x);
            // 212: mul
            reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 213: mul
            reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 214: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 215: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 216: mov
            reg_tmp0 = uniforms.f[90].yxxx;
            // 217: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 283u; break; }
            }
            // 218: mul
            reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 219: mul
            reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 220: mul
            reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 221: mad
            reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 222: dp3
            reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
            // 223: rsq
            reg_tmp5.w = rsq_s(reg_tmp5.w);
            // 224: mul
            reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
            // 225: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 226: mul
            reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 227: mad
            reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 228: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 229: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 230: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 231: add
            reg_tmp6.w = (uniforms.f[90].yyyy + reg_tmp6).w;
            // 232: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 233: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 234: cmp
            conditional_code = lessThan(uniforms.f[91].yy, reg_tmp6.ww);
            // 235: mov
            reg_tmp6.x = (uniforms.f[90].yyyy).x;
            // 236: mov
            reg_tmp6.y = (-uniforms.f[90].yyyy).y;
            // 237: jmpc
            if (!conditional_code.x) {
                { jmp_to = 245u; break; }
            }
            // 238: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 239: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 240: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 241: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
            // 242: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 243: mul
            reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
            // 244: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 283u; break; }
            }
        }
        case 245u: {
            // 245: cmp
            conditional_code = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
            // 246: ifc
            if (conditional_code.x) {
                sub_247_267();
            } else {
                sub_267_280();
            }
            // 280: dp4
            reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
            // 281: rsq
            reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
            // 282: mul
            reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
        }
        case 283u: {
            // 283: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_247_267() {
    // 247: ifc
    if (conditional_code.y) {
        sub_248_253();
    } else {
        sub_253_266();
    }
    // 266: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_248_253() {
    // 248: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 249: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 250: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 251: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 252: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_253_266() {
    // 253: cmp
    conditional_code = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
    // 254: mul
    reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 255: add
    reg_tmp8.x = (uniforms.f[90].yyyy + -reg_tmp5.yyyy).x;
    // 256: ifc
    if (conditional_code.x) {
        sub_257_260();
    } else {
        sub_260_265();
    }
    // 265: nop
    return false;
}
bool sub_257_260() {
    // 257: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 258: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 259: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_260_265() {
    // 260: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 261: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 262: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 263: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 264: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_267_280() {
    // 267: ifc
    if (conditional_code.y) {
        sub_268_273();
    } else {
        sub_273_279();
    }
    // 279: nop
    return false;
}
bool sub_268_273() {
    // 268: mul
    reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 269: add
    reg_tmp8.y = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).y;
    // 270: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 271: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 272: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_273_279() {
    // 273: mul
    reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 274: add
    reg_tmp8.z = (uniforms.f[90].yyyy + -reg_tmp5.zzzz).z;
    // 275: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 276: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 277: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 278: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_284_299() {
    // 284: mov
    reg_tmp8.xy = (uniforms.f[90].xxxx).xy;
    // 285: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 286: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp0.xy);
    // 287: mov
    reg_tmp9.xyz = (uniforms.f[90].xxxx).xyz;
    // 288: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 289: ifc
    if (conditional_code.y) {
        sub_290_295();
    }
    // 295: cmp
    conditional_code = equal(uniforms.f[90].xx, reg_tmp8.xy);
    // 296: ifc
    if (all(conditional_code)) {
        sub_297_298();
    }
    // 298: max
    vs_out_attr3 = max(uniforms.f[90].xxxx, reg_tmp9);
    return false;
}
bool sub_290_295() {
    // 290: mul
    reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
    // 291: ifu
    if (uniforms.b[7]) {
        sub_292_293();
    }
    // 293: mul
    reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 294: mov
    reg_tmp8.x = (uniforms.f[90].yyyy).x;
    return false;
}
bool sub_292_293() {
    // 292: mul
    reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_297_298() {
    // 297: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_299_326() {
    // 299: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 300: ifu
    if (uniforms.b[9]) {
        sub_301_305();
    } else {
        sub_305_324();
    }
    // 324: callu
    if (uniforms.b[2]) {
        sub_326_339();
    }
    // 325: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_301_305() {
    // 301: call
    {
        sub_339_347();
    }
    // 302: dp4
    reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
    // 303: dp4
    reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
    // 304: mov
    reg_tmp3.zw = (uniforms.f[90].xxxx).zw;
    return false;
}
bool sub_305_324() {
    // 305: cmp
    conditional_code = equal(uniforms.f[92].xy, reg_tmp0.xy);
    // 306: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 307: ifc
    if (all(not(conditional_code))) {
        sub_308_314();
    } else {
        sub_314_323();
    }
    // 323: nop
    return false;
}
bool sub_308_314() {
    // 308: mov
    reg_tmp6 = reg_tmp10;
    // 309: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 310: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    // 311: dp4
    reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
    // 312: mul
    reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 313: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_314_323() {
    // 314: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_315_319();
    } else {
        sub_319_322();
    }
    // 322: nop
    return false;
}
bool sub_315_319() {
    // 315: call
    {
        sub_347_354();
    }
    // 316: dp3
    reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
    // 317: dp3
    reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
    // 318: dp3
    reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
    return false;
}
bool sub_319_322() {
    // 319: call
    {
        sub_354_358();
    }
    // 320: dp4
    reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
    // 321: dp4
    reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_326_339() {
    // 326: ifu
    if (uniforms.b[9]) {
        sub_327_328();
    } else {
        sub_328_335();
    }
    // 335: mov
    reg_tmp0 = reg_tmp3;
    // 336: mova
    address_registers.x = (ivec2(uniforms.f[90].xx)).x;
    // 337: call
    {
        sub_358_392();
    }
    // 338: mov
    reg_tmp3 = reg_tmp0;
    return false;
}
bool sub_327_328() {
    // 327: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_328_335() {
    // 328: mov
    reg_tmp7 = uniforms.f[11];
    // 329: cmp
    conditional_code = notEqual(uniforms.f[90].xx, reg_tmp7.zz);
    // 330: ifc
    if (all(conditional_code)) {
        sub_331_333();
    } else {
        sub_333_334();
    }
    // 334: nop
    return false;
}
bool sub_331_333() {
    // 331: mov
    reg_tmp7.xy = (reg_tmp10.xzzz).xy;
    // 332: add
    reg_tmp7.y = (reg_tmp7.yyyy + reg_tmp10.yyyy).y;
    return false;
}
bool sub_333_334() {
    // 333: mov
    reg_tmp7.xy = (reg_tmp3.xyyy).xy;
    return false;
}
bool sub_339_347() {
    // 339: cmp
    conditional_code = equal(uniforms.f[90].yz, reg_tmp0.xy);
    // 340: ifc
    if (all(not(conditional_code))) {
        sub_341_342();
    } else {
        sub_342_346();
    }
    // 346: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_341_342() {
    // 341: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_342_346() {
    // 342: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_343_344();
    } else {
        sub_344_345();
    }
    // 345: nop
    return false;
}
bool sub_343_344() {
    // 343: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_344_345() {
    // 344: mul
    reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_347_354() {
    // 347: mov
    reg_tmp2 = -reg_tmp15;
    // 348: dp3
    reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
    // 349: rsq
    reg_tmp2.w = rsq_s(reg_tmp2.w);
    // 350: mul
    reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
    // 351: dp3
    reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
    // 352: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 353: mad
    reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_354_358() {
    // 354: mov
    reg_tmp1.xy = (uniforms.f[91].zzzz).xy;
    // 355: mov
    reg_tmp1.zw = (uniforms.f[90].xxxx).zw;
    // 356: mad
    reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
    // 357: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    return false;
}
bool sub_358_392() {
    // 358: mov
    reg_tmp6.zw = (uniforms.f[22].xxxx).zw;
    // 359: mul
    reg_tmp6.z = (mul_s(uniforms.f[24 + address_registers.x].zzzz, reg_tmp6.zzzz)).z;
    // 360: mul
    reg_tmp6.w = (mul_s(uniforms.f[24 + address_registers.x].wwww, reg_tmp6.wwww)).w;
    // 361: add
    reg_tmp6.xy = (-uniforms.f[91].zzzz + reg_tmp7.yxxx).xy;
    // 362: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].yzzz, reg_tmp7.xyyy)).xy;
    // 363: mad
    reg_tmp6.xy = (fma_s(reg_tmp6.xyyy, uniforms.f[23 + address_registers.x].xxxx, reg_tmp6.zwww)).xy;
    // 364: add
    reg_tmp6.xy = (uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 365: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].wwww, reg_tmp6.xyyy)).xy;
    // 366: flr
    reg_tmp7.xy = (floor(reg_tmp6.xyyy)).xy;
    // 367: add
    reg_tmp7.xy = (reg_tmp6.xyyy + -reg_tmp7).xy;
    // 368: mul
    reg_tmp6.xy = (mul_s(uniforms.f[91].xxxx, reg_tmp7.xyyy)).xy;
    // 369: add
    reg_tmp6.xy = (-uniforms.f[92].zzzz + reg_tmp6.xyyy).xy;
    // 370: mov
    reg_tmp8 = uniforms.f[95];
    // 371: mov
    reg_tmp9 = uniforms.f[94];
    // 372: mul
    reg_tmp7.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.xxxx)).z;
    // 373: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 374: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 375: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 376: mov
    reg_tmp8 = uniforms.f[93];
    // 377: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 378: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 379: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.xxxx)).y;
    // 380: mad
    reg_tmp0.x = (fma_s(reg_tmp7.xxxx, uniforms.f[24 + address_registers.x].xxxx, reg_tmp0.xxxx)).x;
    // 381: mov
    reg_tmp8 = uniforms.f[95];
    // 382: mov
    reg_tmp9 = uniforms.f[94];
    // 383: mul
    reg_tmp7.z = (mul_s(reg_tmp6.yyyy, reg_tmp6.yyyy)).z;
    // 384: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp8.xyyy, reg_tmp8.zwww)).xy;
    // 385: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.xyyy)).xy;
    // 386: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp9.zwww)).xy;
    // 387: mov
    reg_tmp8 = uniforms.f[93];
    // 388: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.xyyy)).xy;
    // 389: mad
    reg_tmp7.xy = (fma_s(reg_tmp7.zzzz, reg_tmp7.xyyy, reg_tmp8.zwww)).xy;
    // 390: mul
    reg_tmp7.y = (mul_s(reg_tmp7.yyyy, reg_tmp6.yyyy)).y;
    // 391: mad
    reg_tmp0.y = (fma_s(reg_tmp7.yyyy, uniforms.f[24 + address_registers.x].yyyy, reg_tmp0.yyyy)).y;
    return false;
}
bool sub_392_420() {
    // 392: mov
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    // 393: ifu
    if (uniforms.b[10]) {
        sub_394_400();
    } else {
        sub_400_419();
    }
    // 419: nop
    return false;
}
bool sub_394_400() {
    // 394: call
    {
        sub_339_347();
    }
    // 395: dp4
    reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
    // 396: dp4
    reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
    // 397: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 398: callu
    if (uniforms.b[3]) {
        sub_420_425();
    }
    // 399: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_400_419() {
    // 400: ifu
    if (uniforms.b[13]) {
        sub_401_417();
    } else {
        sub_417_418();
    }
    // 418: nop
    return false;
}
bool sub_401_417() {
    // 401: cmp
    conditional_code = equal(uniforms.f[92].xy, reg_tmp0.xy);
    // 402: mov
    reg_tmp6.zw = (uniforms.f[90].xxyy).zw;
    // 403: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 404: ifc
    if (all(not(conditional_code))) {
        sub_405_412();
    } else {
        sub_412_415();
    }
    // 415: callu
    if (uniforms.b[3]) {
        sub_420_425();
    }
    // 416: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_405_412() {
    // 405: mov
    reg_tmp6 = reg_tmp10;
    // 406: dp4
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    // 407: dp4
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    // 408: dp4
    reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
    // 409: rcp
    reg_tmp6.w = rcp_s(reg_tmp4.z);
    // 410: mul
    reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    // 411: add
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_412_415() {
    // 412: call
    {
        sub_354_358();
    }
    // 413: dp4
    reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
    // 414: dp4
    reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_417_418() {
    // 417: mov
    vs_out_attr5 = uniforms.f[90].xxxx;
    return false;
}
bool sub_420_425() {
    // 420: mov
    reg_tmp7.xy = (reg_tmp4.xyyy).xy;
    // 421: mov
    reg_tmp0 = reg_tmp4;
    // 422: mova
    address_registers.x = (ivec2(uniforms.f[90].zz)).x;
    // 423: call
    {
        sub_358_392();
    }
    // 424: mov
    reg_tmp4 = reg_tmp0;
    return false;
}
bool sub_449_4096() {
    // 449: call
    {
        sub_34_191();
    }
    // 450: call
    {
        sub_284_299();
    }
    // 451: call
    {
        sub_299_326();
    }
    // 452: call
    {
        sub_392_420();
    }
    // 453: end
    return true;
}
// reference: 8731C2AA434B5B4C, 1A2F11F6C34A4059
// shader: 8DD9, 3B3AE026C742C7D5

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

out vec4 primary_color;
out vec2 texcoord0;
out vec2 texcoord1;
out vec2 texcoord2;
out float texcoord0_w;
out vec4 normquat;
out vec3 view;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

 in vec4 vs_out_attr0[];
 in vec4 vs_out_attr1[];
 in vec4 vs_out_attr2[];
 in vec4 vs_out_attr3[];
 in vec4 vs_out_attr4[];
 in vec4 vs_out_attr5[];

struct Vertex {
    vec4 attributes[6];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
}

void EmitVtx(Vertex vtx, bool quats_opposite) {
    vec4 vtx_pos = vec4(vtx.attributes[0].x, vtx.attributes[0].y, vtx.attributes[0].z, vtx.attributes[0].w);
    gl_Position = vtx_pos;
#if !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)
    gl_ClipDistance[0] = -vtx_pos.z;
    gl_ClipDistance[1] = dot(clip_coef, vtx_pos);
#endif // !defined(CITRA_GLES) || defined(GL_EXT_clip_cull_distance)

    vec4 vtx_quat = GetVertexQuaternion(vtx);
    normquat = mix(vtx_quat, -vtx_quat, bvec4(quats_opposite));

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);
    texcoord1 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

    texcoord0_w = vtx.attributes[4].z;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);

    texcoord2 = vec2(0.0, 0.0);

    EmitVertex();
}

bool AreQuaternionsOpposite(vec4 qa, vec4 qb) {
    return (dot(qa, qb) < 0.0);
}

void EmitPrim(Vertex vtx0, Vertex vtx1, Vertex vtx2) {
    EmitVtx(vtx0, false);
    EmitVtx(vtx1, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx1)));
    EmitVtx(vtx2, AreQuaternionsOpposite(GetVertexQuaternion(vtx0), GetVertexQuaternion(vtx2)));
    EndPrimitive();
}

void main() {
    Vertex prim_buffer[3];
    prim_buffer[0].attributes = vec4[6](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0], vs_out_attr3[0], vs_out_attr4[0], vs_out_attr5[0]);
    prim_buffer[1].attributes = vec4[6](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1], vs_out_attr3[1], vs_out_attr4[1], vs_out_attr5[1]);
    prim_buffer[2].attributes = vec4[6](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2], vs_out_attr3[2], vs_out_attr4[2], vs_out_attr5[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 46A0C2E6B155D5CD, 3B3AE026C742C7D5
// shader: 8B30, A9EFEF8EE7A8DFB4
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.ggg) * (const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.g) * (const_color[1].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 936E957946058781, A9EFEF8EE7A8DFB4
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, A9EFEF8EE7A8DFB4
// shader: 8B30, CF3C268A4A063B13
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C3D7DDB55D2E791B, CF3C268A4A063B13
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, CF3C268A4A063B13
// shader: 8B30, 32003A0E5C4D29D5
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor2.rrr) * (texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.g) * (texcolor1.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 1.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((textureProj(tex0, vec3(texcoord0, texcoord0_w)).rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (rounded_primary_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 68CDFF32E29DF4AE, 32003A0E5C4D29D5
// program: 485AB8989E17D97B, AD7922A63ED54CA7, 32003A0E5C4D29D5
// shader: 8B30, 9FBEC10EBFC6E455
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (rounded_primary_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AD2267F9905192CD, 9FBEC10EBFC6E455
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 9FBEC10EBFC6E455
// shader: 8B30, 639D23622716DEE5
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.ggg) * (rounded_primary_color.rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FB671A7534AAEFE4, 639D23622716DEE5
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 639D23622716DEE5
// shader: 8B30, AFD6A906DF8DF6FB
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7C16B93C56F3C0B7, AFD6A906DF8DF6FB
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, AFD6A906DF8DF6FB
// shader: 8B30, DE1D12BE71648A2B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((1.0 - texcolor0.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 68A600435D2E791B, DE1D12BE71648A2B
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, DE1D12BE71648A2B
// shader: 8B30, B3802344941D3961
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7823778960F6489, B3802344941D3961
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, B3802344941D3961
// shader: 8B30, FAB46CC7AEF078C6
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 66685D422B9CC220, FAB46CC7AEF078C6
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, FAB46CC7AEF078C6
// shader: 8B30, A1084323DC64A9F7
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((rounded_primary_color.rgb) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0C2F332BF6992A15, A1084323DC64A9F7
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, A1084323DC64A9F7
// shader: 8B30, CD13112AA4A6172C
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp(min((rounded_primary_color.rgb) + (const_color[1].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0C2F332BB84F314A, CD13112AA4A6172C
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, CD13112AA4A6172C
// shader: 8B30, EDE7A4593D67EF4A
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor0.bbb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((primary_fragment_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 28C7EFDE4E161610, EDE7A4593D67EF4A
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, EDE7A4593D67EF4A
// shader: 8B30, B0346E41DBE263FF
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rrr) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp(min((rounded_primary_color.rgb) + (const_color[2].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D06477874CCFB997, B0346E41DBE263FF
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, B0346E41DBE263FF
// shader: 8B30, E494C412FBFAA372
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BF90F387122AE2F6, E494C412FBFAA372
// program: F2622744950F3FD8, 4BD70AD09292A3DA, E494C412FBFAA372
// reference: D9A939CBE81813FD, C1FEDE1C4F0B4CB9
// reference: 68CDFF32AC4BEFF1, 32003A0E5C4D29D5
// reference: AD2267F9DE878992, 9FBEC10EBFC6E455
// shader: 8B30, 1470DE93E936B856
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A6DB2FBA60C3418, 1470DE93E936B856
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 1470DE93E936B856
// shader: 8B30, CFD785A8A3C7359D
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((vec3(1.0) - texcolor1.rgb) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (last_tex_env_out.rgb) + (const_color[1].rgb) * (vec3(1.0) - (last_tex_env_out.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a) + (texcolor2.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (const_color[2].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6330AF5D2B65CB92, CFD785A8A3C7359D
// program: 485AB8989E17D97B, AD7922A63ED54CA7, CFD785A8A3C7359D
// reference: AD8D72225AC31F5E, 4F9DAEA1DCDB2FC6
// reference: 287550EDB258509B, 485AB8989E17D97B
// shader: 8B30, B728EF44248FB46B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3C2A5066307F7788, B728EF44248FB46B
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, B728EF44248FB46B
// shader: 8B30, F1DAE34E95760D05
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A6DB2FBF5099771, F1DAE34E95760D05
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, F1DAE34E95760D05
// reference: D9A939CBBB1DB094, F5F586805D7D88AD
// reference: 1B73E29D1825DBE8, AFD6A906439B3437
// shader: 8B30, 5D03B1DD7300A899
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4E1F0C3F52E89DB1, 5D03B1DD7300A899
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 5D03B1DD7300A899
// shader: 8B30, 843DA6896E752887
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FCBEFA074491BEC0, 843DA6896E752887
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 843DA6896E752887
// reference: F146E8D85CFCF9A9, E494C412FBFAA372
// reference: 977F2294A6CE08A2, C1FEDE1C4F0B4CB9
// reference: 261BE46DE29DF4AE, 32003A0E5C4D29D5
// shader: 8B30, 8D6B66D0C7E94583
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rgb) + (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E5398F4F17BF85D4, 8D6B66D0C7E94583
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 8D6B66D0C7E94583
// shader: 8B30, 0D24D4CC4BC1E16B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (const_color[2].rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A4B19CDF1978BA6C, 0D24D4CC4BC1E16B
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 0D24D4CC4BC1E16B
// reference: BF90F3870E0D7AE3, E494C412FBFAA372
// shader: 8B30, 471B469FCE1DE21B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 66685D42A73A2881, 471B469FCE1DE21B
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 471B469FCE1DE21B
// reference: D9A939CBF43F8BE8, C1FEDE1C4F0B4CB9
// reference: 68CDFF32B06C77E4, 32003A0E5C4D29D5
// shader: 8B30, 28DA555DE88FD115
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.aaa) + (const_color[2].rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A4B19CDF77232453, 28DA555DE88FD115
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 28DA555DE88FD115
// reference: 66685D42654AD97F, FAB46CC7AEF078C6
// shader: 8B30, FAB46CC7CEA6D794
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 071863D4654AD97F, FAB46CC7CEA6D794
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, FAB46CC7CEA6D794
// shader: 8B30, 730F2D240EDC7094
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C3D7DDB53DCF57F3, 730F2D240EDC7094
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 730F2D240EDC7094
// shader: 8B30, 6C0723FDFB41C4AD
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor2.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4679C0B8DF66DB5, 6C0723FDFB41C4AD
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 6C0723FDFB41C4AD
// shader: 8B30, 64B76F90472D3C6F
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 25BB072F498E0497, 64B76F90472D3C6F
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 64B76F90472D3C6F
// shader: 8B30, 1CB28E8C73CF0E85
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A6DB2FB53E8B89D, 1CB28E8C73CF0E85
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 1CB28E8C73CF0E85
// shader: 8B30, D04823D9D20A9288
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 613BF225151C45D1, D04823D9D20A9288
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, D04823D9D20A9288
// shader: 8B30, 861C8D757CCBE4EE
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2FEDE97AD1073910, 861C8D757CCBE4EE
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 861C8D757CCBE4EE
// reference: F146E8D8122AE2F6, E494C412FBFAA372
// shader: 8B30, CEFE1B057178439E
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb) + (texcolor0.ggg), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5CB876F38F206CCB, CEFE1B057178439E
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, CEFE1B057178439E
// shader: 8B30, C9699A407F6104BB
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 49750C6413F86244, C9699A407F6104BB
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, C9699A407F6104BB
// reference: 977F2294E81813FD, C1FEDE1C4F0B4CB9
// reference: 261BE46DAC4BEFF1, 32003A0E5C4D29D5
// shader: 8B30, 4DE070D6D1CE357A
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 13F8D220F5C8FF6A, 4DE070D6D1CE357A
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 4DE070D6D1CE357A
// shader: 8B30, DA3B241BF1886166
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C36CA95AEDC1576D, DA3B241BF1886166
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, DA3B241BF1886166
// shader: 8B30, AFD6A906987D56FC
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.ggg) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 66E98FD71825DBE8, AFD6A906987D56FC
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, AFD6A906987D56FC
// reference: 8731C2AA98F2FA0B, 1A2F11F6C34A4059
// shader: 8B30, 9DF04F70C65E748B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.b), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EFF3610FA635F81E, 9DF04F70C65E748B
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 9DF04F70C65E748B
// shader: 8B30, AAED980EF749D7C9
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((texcolor0.rgb) + (const_color[0].rgb), vec3(1.0)) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2 * 2.0, alpha_output_2 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A614556129DE5621, AAED980EF749D7C9
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, AAED980EF749D7C9
// shader: 8B30, 32463E4038AA304C
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 57C223680BD817DE, 32463E4038AA304C
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 32463E4038AA304C
// shader: 8B30, 3DDBB1E076F190FF
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
specular_sum.a = (lut_scale_fr * LookupLightingLUTSigned(3, dot(normal, normalize(half_vector))));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor1.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2FEDE97A369D1B7E, 3DDBB1E076F190FF
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 3DDBB1E076F190FF
// shader: 8B30, EC622732927D46B3
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (vec3(1.0) - secondary_fragment_color.aaa) + (const_color[0].rgb) * (vec3(1.0) - (vec3(1.0) - secondary_fragment_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) + (secondary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE68A58FE516C0D9, EC622732927D46B3
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, EC622732927D46B3
// reference: 977F2294198258F7, C1FEDE1C4F0B4CB9
// reference: 261BE46D5DD1A4FB, 32003A0E5C4D29D5
// reference: F146E8D8E3B0A9FC, E494C412FBFAA372
// shader: 8B30, AFD6A906CA010A49
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8DBAB205E9BF90E2, AFD6A906CA010A49
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, AFD6A906CA010A49
// shader: 8B30, 6BADA84C8A1979D2
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 53B09CF5874FBFA5, 6BADA84C8A1979D2
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 6BADA84C8A1979D2
// reference: F146E8D8B1678A27, AEDA558FB63A096E
// shader: 8B30, E54CAF8828CB019B
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D38DFA722A4C545E, E54CAF8828CB019B
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, E54CAF8828CB019B
// shader: 8B30, 51EAC3B629C0BB29
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AD3A8D8D881E671C, 51EAC3B629C0BB29
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 51EAC3B629C0BB29
// reference: A7D40B2C4A87FB9E, F5F586805D7D88AD
// shader: 8B30, 377F147C2AA5CC71
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0.0, 0.0, 1.0);
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1A5FF62DE5AF7F46, 377F147C2AA5CC71
// program: F2622744950F3FD8, 4BD70AD09292A3DA, 377F147C2AA5CC71
// reference: BF90F387E3B0A9FC, E494C412FBFAA372
// reference: D9A939CB198258F7, C1FEDE1C4F0B4CB9
// reference: 68CDFF325DD1A4FB, 32003A0E5C4D29D5
// shader: 8B30, 7D91BAFA102EE3FA
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp(min((rounded_primary_color.rgb) + (const_color[2].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E91436650EBA61E9, 7D91BAFA102EE3FA
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, 7D91BAFA102EE3FA
// shader: 8B30, AFD00F38938C6208
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.b), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp(min((rounded_primary_color.rgb) + (const_color[2].rgb), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) < alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 83285049DE416DB2, AFD00F38938C6208
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, AFD00F38938C6208
// reference: BF90F38767B53C2C, E494C412FBFAA372
// reference: D9A939CB9D87CD27, C1FEDE1C4F0B4CB9
// reference: 68CDFF32D9D4312B, 32003A0E5C4D29D5
// shader: 8B30, FBAD3A8C4DFB82D2
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));

vec4 diffuse_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec4 specular_sum = vec4(0.0, 0.0, 0.0, 1.0);
vec3 light_vector = vec3(0.0);
vec3 refl_value = vec3(0.0);
vec3 spot_dir = vec3(0.0);
vec3 half_vector = vec3(0.0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor0).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1.0, 0.0, 0.0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = quaternion_rotate(normalized_normquat, surface_normal);
vec3 tangent = quaternion_rotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1.0);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(view)), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) * (texcolor0.bbb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0A6DB2FB0493DC7B, FBAD3A8C4DFB82D2
// program: 1A2F11F6C34A4059, 3B3AE026C742C7D5, FBAD3A8C4DFB82D2
// shader: 8B30, 6D34102DCFE65578
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 2.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F1DE68B6EB924342, 6D34102DCFE65578
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, 6D34102DCFE65578
// shader: 8B30, D7E65BF17F6104BB
in vec4 primary_color;
in vec2 texcoord0;
in vec2 texcoord1;
in vec2 texcoord2;
in float texcoord0_w;
in vec4 normquat;
in vec3 view;

#ifndef CITRA_GLES
in vec4 gl_FragCoord;
#endif // CITRA_GLES
out vec4 color;

uniform sampler2D tex0;
uniform sampler2D tex1;
uniform sampler2D tex2;
uniform samplerCube tex_cube;
uniform samplerBuffer texture_buffer_lut_lf;
uniform samplerBuffer texture_buffer_lut_rg;
uniform samplerBuffer texture_buffer_lut_rgba;

#define NUM_TEV_STAGES 6
#define NUM_LIGHTS 8
#define NUM_LIGHTING_SAMPLERS 24

struct LightSrc {
    vec3 specular_0;
    vec3 specular_1;
    vec3 diffuse;
    vec3 ambient;
    vec3 position;
    vec3 spot_direction;
    float dist_atten_bias;
    float dist_atten_scale;
};

layout (std140) uniform shader_data {
    int alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    float proctex_bias;
    vec3 fog_color;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 const_color[NUM_TEV_STAGES];
    vec4 tev_combiner_buffer_color;
    vec4 clip_coef;
};

layout (std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[NUM_LIGHTING_SAMPLERS / 4];
    vec3 lighting_global_ambient;
    LightSrc light_src[NUM_LIGHTS];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

// Rotate the vector v by the quaternion q
vec3 quaternion_rotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(texture_buffer_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
    return entry.r + entry.g * delta;
}

float LookupLightingLUTUnsigned(int lut_index, float pos) {
    int index = clamp(int(pos * 256.0), 0, 255);
    float delta = pos * 256.0 - float(index);
    return LookupLightingLUT(lut_index, index, delta);
}

float LookupLightingLUTSigned(int lut_index, float pos) {
    int index = clamp(int(pos * 128.0), -128, 127);
    float delta = pos * 128.0 - float(index);
    if (index < 0) index += 256;
    return LookupLightingLUT(lut_index, index, delta);
}

float byteround(float x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec2 byteround(vec2 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec3 byteround(vec3 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

vec4 byteround(vec4 x) {
    return round(x * 255.0) * (1.0 / 255.0);
}

// PICA's LOD formula for 2D textures.
// This LOD formula is the same as the LOD lower limit defined in OpenGL.
// f(x, y) >= max{m_u, m_v, m_w}
// (See OpenGL 4.6 spec, 8.14.1 - Scale Factor and Level-of-Detail)
float getLod(vec2 coord) {
    vec2 d = max(abs(dFdx(coord)), abs(dFdy(coord)));
    return log2(max(d.x, d.y));
}

vec4 shadowTexture(vec2 uv, float w) {
    return vec4(1.0);
}

vec4 shadowTextureCube(vec2 uv, float w) {
    return vec4(1.0);
}

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0.0);
vec4 secondary_fragment_color = vec4(0.0);
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.g), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BECC626AE262294E, D7E65BF17F6104BB
// program: 4F9DAEA1DCDB2FC6, 6CF3F3B70E23AA85, D7E65BF17F6104BB
