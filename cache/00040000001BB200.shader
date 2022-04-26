// shader: 8B31, 3CBBD256D783A3F0

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_8_26();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_143_144();
bool sub_145_181();
bool sub_147_157();
bool sub_159_180();
bool sub_160_162();
bool sub_163_171();
bool sub_173_179();
bool sub_181_182();
bool sub_183_189();
bool sub_186_187();
bool sub_187_188();
bool sub_189_190();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_8_26() {
    // 8: add
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    // 9: dp3
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    // 10: rsq
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    // 11: mul
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    // 12: rcp
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    // 13: dp3
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    // 14: max
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    // 15: min
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    // 16: add
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    // 17: add
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    // 18: rcp
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    // 19: mul
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    // 20: min
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    // 21: max
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    // 22: add
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    // 23: mul
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    // 24: mul
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    // 25: mul
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 123: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 124: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 125: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 126: mov
    vs_out_attr3 = reg_tmp2;
    // 127: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 128: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 129: call
    {
        sub_0_8();
    }
    // 130: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 131: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 132: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 133: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 134: mov
    vs_out_attr4 = reg_tmp2;
    // 135: mov
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    // 136: call
    {
        sub_0_8();
    }
    // 137: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 138: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 139: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 140: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 141: mov
    vs_out_attr5 = reg_tmp2;
    // 142: ifu
    if (uniforms.b[7]) {
        sub_143_144();
    }
    // 144: ifu
    if (uniforms.b[11]) {
        sub_145_181();
    } else {
        sub_181_182();
    }
    // 182: ifu
    if (uniforms.b[7]) {
        sub_183_189();
    } else {
        sub_189_190();
    }
    // 190: mov
    vs_out_attr6 = reg_tmp11;
    // 191: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_143_144() {
    // 143: mov
    reg_tmp10 = uniforms.f[17].xxyx;
    return false;
}
bool sub_145_181() {
    // 145: mov
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    // 146: ifu
    if (uniforms.b[6]) {
        sub_147_157();
    }
    // 157: nop
    // 158: ifu
    if (uniforms.b[5]) {
        sub_159_180();
    }
    // 180: nop
    return false;
}
bool sub_147_157() {
    // 147: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    // 148: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    // 149: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 150: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 151: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    // 152: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    // 153: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    // 154: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 155: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 156: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_159_180() {
    // 159: ifu
    if (uniforms.b[7]) {
        sub_160_162();
    }
    // 162: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop162 = 0u; loop162 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop162) {
        sub_163_171();
    }
    // 171: nop
    // 172: ifu
    if (uniforms.b[7]) {
        sub_173_179();
    }
    // 179: nop
    return false;
}
bool sub_160_162() {
    // 160: max
    reg_tmp10.x = (max(reg_tmp11.xxxx, reg_tmp11.yyyy)).x;
    // 161: max
    reg_tmp10.x = (max(reg_tmp10.xxxx, reg_tmp11.zzzz)).x;
    return false;
}
bool sub_163_171() {
    // 163: mov
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    // 164: mov
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    // 165: mov
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    // 166: mov
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    // 167: call
    {
        sub_8_26();
    }
    // 168: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    // 169: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 170: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_173_179() {
    // 173: max
    reg_tmp10.y = (max(reg_tmp11.xxxx, reg_tmp11.yyyy)).y;
    // 174: max
    reg_tmp10.y = (max(reg_tmp10.yyyy, reg_tmp11.zzzz)).y;
    // 175: add
    reg_tmp10.z = (reg_tmp10.yyyy + -reg_tmp10.xxxx).z;
    // 176: max
    reg_tmp10.z = (max(uniforms.f[17].xxxx, reg_tmp10.zzzz)).z;
    // 177: min
    reg_tmp10.z = (min(uniforms.f[17].yyyy, reg_tmp10.zzzz)).z;
    // 178: add
    reg_tmp10.z = (uniforms.f[17].yyyy + -reg_tmp10.zzzz).z;
    return false;
}
bool sub_181_182() {
    // 181: mov
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
bool sub_183_189() {
    // 183: mov
    reg_tmp13.w = (uniforms.f[95].wwww).w;
    // 184: cmp
    conditional_code = lessThan(reg_tmp8.zz, reg_tmp13.ww);
    // 185: ifc
    if (conditional_code.x) {
        sub_186_187();
    } else {
        sub_187_188();
    }
    // 188: nop
    return false;
}
bool sub_186_187() {
    // 186: mov
    reg_tmp11.w = (reg_tmp10.zzzz).w;
    return false;
}
bool sub_187_188() {
    // 187: mov
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_189_190() {
    // 189: mov
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    return false;
}
// reference: 84E4A09237F27966, 3CBBD256D783A3F0
// shader: 8DD9, 1558286C47B9A4C4

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

    vec4 vtx_color = vec4(vtx.attributes[6].x, vtx.attributes[6].y, vtx.attributes[6].z, vtx.attributes[6].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);
    texcoord1 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);

    texcoord0_w = 0.0;
    view = vec3(vtx.attributes[2].x, vtx.attributes[2].y, vtx.attributes[2].z);

    texcoord2 = vec2(vtx.attributes[5].x, vtx.attributes[5].y);

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
// reference: B7A1AA8AEE71524D, 1558286C47B9A4C4
// shader: 8B30, 0C3173328C8E2801

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8DC4F5161F04A720, 0C3173328C8E2801
// program: 3CBBD256D783A3F0, 1558286C47B9A4C4, 0C3173328C8E2801
// reference: 5767676BEC4BD821, 3CBBD256D783A3F0
// shader: 8B31, C7012B058DBFF174

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_8_26();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_143_168();
bool sub_145_155();
bool sub_157_167();
bool sub_158_166();
bool sub_168_169();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_8_26() {
    // 8: add
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    // 9: dp3
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    // 10: rsq
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    // 11: mul
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    // 12: rcp
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    // 13: dp3
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    // 14: max
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    // 15: min
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    // 16: add
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    // 17: add
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    // 18: rcp
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    // 19: mul
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    // 20: min
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    // 21: max
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    // 22: add
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    // 23: mul
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    // 24: mul
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    // 25: mul
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 123: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 124: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 125: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 126: mov
    vs_out_attr3 = reg_tmp2;
    // 127: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 128: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 129: call
    {
        sub_0_8();
    }
    // 130: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 131: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 132: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 133: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 134: mov
    vs_out_attr4 = reg_tmp2;
    // 135: mov
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    // 136: call
    {
        sub_0_8();
    }
    // 137: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 138: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 139: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 140: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 141: mov
    vs_out_attr5 = reg_tmp2;
    // 142: ifu
    if (uniforms.b[11]) {
        sub_143_168();
    } else {
        sub_168_169();
    }
    // 169: mov
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    // 170: mov
    vs_out_attr6 = reg_tmp11;
    // 171: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_143_168() {
    // 143: mov
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    // 144: ifu
    if (uniforms.b[6]) {
        sub_145_155();
    }
    // 155: nop
    // 156: ifu
    if (uniforms.b[5]) {
        sub_157_167();
    }
    // 167: nop
    return false;
}
bool sub_145_155() {
    // 145: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    // 146: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    // 147: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 148: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 149: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    // 150: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    // 151: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    // 152: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 153: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 154: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_157_167() {
    // 157: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop157 = 0u; loop157 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop157) {
        sub_158_166();
    }
    // 166: nop
    return false;
}
bool sub_158_166() {
    // 158: mov
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    // 159: mov
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    // 160: mov
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    // 161: mov
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    // 162: call
    {
        sub_8_26();
    }
    // 163: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    // 164: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 165: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_168_169() {
    // 168: mov
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
// reference: 37CEF7D467574BDF, C7012B058DBFF174
// shader: 8B30, 17B4EDCF91680497

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97D23F052B69DDB3, 17B4EDCF91680497
// program: C7012B058DBFF174, 1558286C47B9A4C4, 17B4EDCF91680497
// shader: 8B31, F141D81C8BBB3C08

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_123_126();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: ifu
    if (uniforms.b[8]) {
        sub_123_126();
    }
    // 126: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 127: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 128: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 129: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 130: mov
    vs_out_attr3 = reg_tmp2;
    // 131: mov
    vs_out_attr4 = vs_in_reg6;
    // 132: mov
    vs_out_attr5 = vs_in_reg7;
    // 133: mov
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    // 134: mov
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    // 135: mov
    vs_out_attr6 = reg_tmp11;
    // 136: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_123_126() {
    // 123: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 124: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 125: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 153E12445D260123, F141D81C8BBB3C08
// shader: 8B30, 6A6FAB67EC30DF59

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1.0) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7297BC267C38FBF, 6A6FAB67EC30DF59
// program: F141D81C8BBB3C08, 1558286C47B9A4C4, 6A6FAB67EC30DF59
// shader: 8B30, 95FFD538EC92B26D

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C94679FBEC4EA26B, 95FFD538EC92B26D
// program: F141D81C8BBB3C08, 1558286C47B9A4C4, 95FFD538EC92B26D
// shader: 8B31, 948945C9E52B38B8

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_123_126();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: ifu
    if (uniforms.b[8]) {
        sub_123_126();
    }
    // 126: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 127: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 128: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 129: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 130: mov
    vs_out_attr3 = reg_tmp2;
    // 131: mov
    vs_out_attr4 = vs_in_reg6;
    // 132: mov
    vs_out_attr5 = vs_in_reg7;
    // 133: mov
    vs_out_attr6 = vs_in_reg8;
    // 134: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_123_126() {
    // 123: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 124: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 125: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 153E1244CB8EB4A8, 948945C9E52B38B8
// shader: 8B30, 33E878D269D742C6

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((vec3(1.0) - primary_fragment_color.aaa) * (vec3(1.0) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97A606328584E266, 33E878D269D742C6
// program: 948945C9E52B38B8, 1558286C47B9A4C4, 33E878D269D742C6
// shader: 8B31, 5C4AE0ED323B16A1

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_105_106();
bool sub_109_119();
bool sub_115_117();
bool sub_117_118();
bool sub_119_120();
bool sub_125_128();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: ifu
    if (uniforms.b[10]) {
        sub_105_106();
    }
    // 106: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 107: mov
    vs_out_attr0 = reg_tmp10;
    // 108: ifu
    if (uniforms.b[4]) {
        sub_109_119();
    } else {
        sub_119_120();
    }
    // 120: mov
    vs_out_attr2 = -reg_tmp12;
    // 121: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 122: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 123: call
    {
        sub_0_8();
    }
    // 124: ifu
    if (uniforms.b[8]) {
        sub_125_128();
    }
    // 128: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 129: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 130: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 131: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 132: mov
    vs_out_attr3 = reg_tmp2;
    // 133: mov
    vs_out_attr4 = vs_in_reg6;
    // 134: mov
    vs_out_attr5 = vs_in_reg7;
    // 135: mov
    vs_out_attr6 = vs_in_reg8;
    // 136: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_105_106() {
    // 105: add
    reg_tmp12.z = (-uniforms.f[80].wwww + reg_tmp12.zzzz).z;
    return false;
}
bool sub_109_119() {
    // 109: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 110: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 111: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 112: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 113: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 114: ifc
    if (!conditional_code.x) {
        sub_115_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_115_117() {
    // 115: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 116: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_119_120() {
    // 119: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_125_128() {
    // 125: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 126: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 127: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 883B045BB6AB962E, 5C4AE0ED323B16A1
// shader: 8B30, 5A9DBA5B31BC353E

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.aaa) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].ggg) + (last_tex_env_out.rrr) * (vec3(1.0) - (const_color[2].ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].g) + (last_tex_env_out.r) * (1.0 - (const_color[2].g)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2 * 4.0, alpha_output_2 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.r) * (combiner_buffer.b) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.b)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp(min((rounded_primary_color.a) + (const_color[4].a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(1.0)) * (combiner_buffer.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7DE2EB867678105E, 5A9DBA5B31BC353E
// program: 5C4AE0ED323B16A1, 1558286C47B9A4C4, 5A9DBA5B31BC353E
// reference: 3B98747BCB8EB4A8, 948945C9E52B38B8
// shader: 8B31, 955E4F6CC504D390

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    vs_out_attr3 = vs_in_reg5;
    // 120: mov
    vs_out_attr4 = vs_in_reg6;
    // 121: mov
    vs_out_attr5 = vs_in_reg7;
    // 122: mov
    vs_out_attr6 = vs_in_reg8;
    // 123: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
// reference: F08C8792A7F8A37E, 955E4F6CC504D390
// shader: 8B30, C8096B09A214E787

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5D3A7CB0744F75C5, C8096B09A214E787
// program: 955E4F6CC504D390, 1558286C47B9A4C4, C8096B09A214E787
// shader: 8B31, 8C95EA03C79EDDD2

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_105_106();
bool sub_109_119();
bool sub_115_117();
bool sub_117_118();
bool sub_119_120();
bool sub_125_128();
bool sub_137_140();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: ifu
    if (uniforms.b[10]) {
        sub_105_106();
    }
    // 106: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 107: mov
    vs_out_attr0 = reg_tmp10;
    // 108: ifu
    if (uniforms.b[4]) {
        sub_109_119();
    } else {
        sub_119_120();
    }
    // 120: mov
    vs_out_attr2 = -reg_tmp12;
    // 121: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 122: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 123: call
    {
        sub_0_8();
    }
    // 124: ifu
    if (uniforms.b[8]) {
        sub_125_128();
    }
    // 128: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 129: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 130: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 131: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 132: mov
    vs_out_attr3 = reg_tmp2;
    // 133: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 134: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 135: call
    {
        sub_0_8();
    }
    // 136: ifu
    if (uniforms.b[9]) {
        sub_137_140();
    }
    // 140: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 141: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 142: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 143: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 144: mov
    vs_out_attr4 = reg_tmp2;
    // 145: mov
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    // 146: call
    {
        sub_0_8();
    }
    // 147: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 148: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 149: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 150: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 151: mov
    vs_out_attr5 = reg_tmp2;
    // 152: mov
    vs_out_attr6 = vs_in_reg8;
    // 153: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_105_106() {
    // 105: add
    reg_tmp12.z = (-uniforms.f[80].wwww + reg_tmp12.zzzz).z;
    return false;
}
bool sub_109_119() {
    // 109: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 110: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 111: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 112: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 113: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 114: ifc
    if (!conditional_code.x) {
        sub_115_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_115_117() {
    // 115: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 116: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_119_120() {
    // 119: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_125_128() {
    // 125: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 126: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 127: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_137_140() {
    // 137: dp4
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    // 138: dp4
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    // 139: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: F724F0A401D5F722, 8C95EA03C79EDDD2
// shader: 8B30, 045DD07097152919

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1.0) - (const_color[3].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4 * 1.0, alpha_output_4 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E3C1FE180A99180E, 045DD07097152919
// program: 8C95EA03C79EDDD2, 1558286C47B9A4C4, 045DD07097152919
// shader: 8B31, 650D1363A7B584AA

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: mov
    reg_tmp0 = vs_in_reg0;
    // 1: mov
    reg_tmp0.zw = (uniforms.f[93].xyyy).zw;
    // 2: dp4
    reg_tmp1.x = dot_s(uniforms.f[25], reg_tmp0);
    // 3: dp4
    reg_tmp1.y = dot_s(uniforms.f[26], reg_tmp0);
    // 4: add
    reg_tmp2.y = (uniforms.f[93].yyyy + reg_tmp1.yyyy).y;
    // 5: mul
    reg_tmp2.y = (mul_s(uniforms.f[94].xxxx, reg_tmp2.yyyy)).y;
    // 6: add
    reg_tmp2.y = (uniforms.f[93].yyyy + -reg_tmp2.yyyy).y;
    // 7: mul
    reg_tmp2.y = (mul_s(uniforms.f[93].zzzz, reg_tmp2.yyyy)).y;
    // 8: add
    reg_tmp1.y = (-uniforms.f[93].yyyy + reg_tmp2.yyyy).y;
    // 9: mov
    vs_out_attr0.xy = (reg_tmp1.yxxx).xy;
    // 10: mov
    vs_out_attr0.z = (uniforms.f[93].xxxx).z;
    // 11: mov
    vs_out_attr0.w = (uniforms.f[93].yyyy).w;
    // 12: mov
    reg_tmp0 = uniforms.f[93].xxyx;
    // 13: add
    reg_tmp0.xy = (-uniforms.f[17].xyyy + vs_in_reg2.xyyy).xy;
    // 14: dp4
    reg_tmp3.x = dot_s(uniforms.f[13], reg_tmp0);
    // 15: dp4
    reg_tmp3.y = dot_s(uniforms.f[14], reg_tmp0);
    // 16: add
    reg_tmp0.xy = (uniforms.f[17].xyyy + reg_tmp3.xyyy).xy;
    // 17: add
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    // 18: mov
    vs_out_attr2 = reg_tmp0;
    // 19: mov
    reg_tmp0 = uniforms.f[93].xxyx;
    // 20: add
    reg_tmp0.xy = (-uniforms.f[18].xyyy + vs_in_reg2.xyyy).xy;
    // 21: dp4
    reg_tmp3.x = dot_s(uniforms.f[15], reg_tmp0);
    // 22: dp4
    reg_tmp3.y = dot_s(uniforms.f[16], reg_tmp0);
    // 23: add
    reg_tmp0.xy = (uniforms.f[18].xyyy + reg_tmp3.xyyy).xy;
    // 24: add
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    // 25: mov
    vs_out_attr3 = reg_tmp0;
    // 26: mov
    reg_tmp0 = uniforms.f[93].xxxx;
    // 27: mov
    reg_tmp0 = vs_in_reg2;
    // 28: add
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    // 29: mov
    vs_out_attr4 = reg_tmp0;
    // 30: mov
    vs_out_attr1 = vs_in_reg1;
    // 31: end
    return true;
}
// reference: 2BB13004BFE6E0DE, 650D1363A7B584AA
// shader: 8DD9, 5D764F9A6220D694

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
    return vec4(0.0, 0.0, 0.0, 0.0);
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);

    texcoord2 = vec2(vtx.attributes[4].x, vtx.attributes[4].y);

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
// reference: 5DAD5699F59B3586, 5D764F9A6220D694
// shader: 8B30, 7FDFB08AA001B9CC

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 2.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1.0)) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1.0) - (const_color[3].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8881965D9537D39A, 7FDFB08AA001B9CC
// program: 650D1363A7B584AA, 5D764F9A6220D694, 7FDFB08AA001B9CC
// shader: 8B31, 0EE2E818CD1370A1

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

 out vec4 vs_out_attr0;
 out vec4 vs_out_attr1;
 out vec4 vs_out_attr2;

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr2 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: mov
    reg_tmp0.xy = (vs_in_reg0.xyyy).xy;
    // 1: mov
    reg_tmp0.zw = (uniforms.f[93].xyyy).zw;
    // 2: dp4
    reg_tmp1.x = dot_s(uniforms.f[25], reg_tmp0);
    // 3: dp4
    reg_tmp1.y = dot_s(uniforms.f[26], reg_tmp0);
    // 4: add
    reg_tmp2.y = (uniforms.f[93].yyyy + reg_tmp1.yyyy).y;
    // 5: mul
    reg_tmp2.y = (mul_s(uniforms.f[94].xxxx, reg_tmp2.yyyy)).y;
    // 6: add
    reg_tmp2.y = (uniforms.f[93].yyyy + -reg_tmp2.yyyy).y;
    // 7: mul
    reg_tmp2.y = (mul_s(uniforms.f[93].zzzz, reg_tmp2.yyyy)).y;
    // 8: add
    reg_tmp1.y = (-uniforms.f[93].yyyy + reg_tmp2.yyyy).y;
    // 9: mov
    reg_tmp2.x = (uniforms.f[27].wwww).x;
    // 10: mul
    reg_tmp2.x = (mul_s(uniforms.f[29].xxxx, reg_tmp2.xxxx)).x;
    // 11: add
    reg_tmp1.y = (reg_tmp1.yyyy + reg_tmp2.xxxx).y;
    // 12: mov
    vs_out_attr0.xy = (-reg_tmp1.yxxx).xy;
    // 13: mov
    vs_out_attr0.z = (uniforms.f[93].xxxx).z;
    // 14: mov
    vs_out_attr0.w = (uniforms.f[93].yyyy).w;
    // 15: mov
    reg_tmp0 = vs_in_reg2;
    // 16: add
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    // 17: mov
    vs_out_attr2 = reg_tmp0;
    // 18: mov
    vs_out_attr1 = vs_in_reg1;
    // 19: end
    return true;
}
// reference: 51AC9CCDD221F2C6, 0EE2E818CD1370A1
// shader: 8DD9, 4E5317772B5CE683

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

struct Vertex {
    vec4 attributes[3];
};

vec4 GetVertexQuaternion(Vertex vtx) {
    return vec4(0.0, 0.0, 0.0, 0.0);
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

    vec4 vtx_color = vec4(vtx.attributes[1].x, vtx.attributes[1].y, vtx.attributes[1].z, vtx.attributes[1].w);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);
    texcoord1 = vec2(0.0, 0.0);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);

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
    prim_buffer[0].attributes = vec4[3](vs_out_attr0[0], vs_out_attr1[0], vs_out_attr2[0]);
    prim_buffer[1].attributes = vec4[3](vs_out_attr0[1], vs_out_attr1[1], vs_out_attr2[1]);
    prim_buffer[2].attributes = vec4[3](vs_out_attr0[2], vs_out_attr1[2], vs_out_attr2[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: 7B07DA3E334A19B0, 4E5317772B5CE683
// shader: 8B30, DAC934827CE046A0

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 4.0, alpha_output_0 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[4].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5880AA7C2968D6EB, DAC934827CE046A0
// program: 0EE2E818CD1370A1, 4E5317772B5CE683, DAC934827CE046A0
// reference: 822F5B345245A145, 0EE2E818CD1370A1
// shader: 8B30, 51FC9310092F95FF

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 4.0, alpha_output_0 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[4].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 67C182062968D6EB, 51FC9310092F95FF
// program: 0EE2E818CD1370A1, 4E5317772B5CE683, 51FC9310092F95FF
// shader: 8B30, F69EEFE820DC278C

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E8F96D340291B4F7, F69EEFE820DC278C
// program: 3CBBD256D783A3F0, 1558286C47B9A4C4, F69EEFE820DC278C
// shader: 8B30, FC64A66DC2079D25

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F2EFA72736FCCE64, FC64A66DC2079D25
// program: C7012B058DBFF174, 1558286C47B9A4C4, FC64A66DC2079D25
// reference: 153E1244869599F7, 955E4F6CC504D390
// reference: 0F43A0EB6D0640EF, 51FC9310092F95FF
// reference: 5767676BB7962AE5, 3CBBD256D783A3F0
// shader: 8B31, D8CF3A496BEB73F0

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
layout(location = 5) in vec4 vs_in_reg5;
layout(location = 6) in vec4 vs_in_reg6;
layout(location = 7) in vec4 vs_in_reg7;

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_8_26();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_145_170();
bool sub_147_157();
bool sub_159_169();
bool sub_160_168();
bool sub_170_171();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_8_26() {
    // 8: add
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    // 9: dp3
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    // 10: rsq
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    // 11: mul
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    // 12: rcp
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    // 13: dp3
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    // 14: max
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    // 15: min
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    // 16: add
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    // 17: add
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    // 18: rcp
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    // 19: mul
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    // 20: min
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    // 21: max
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    // 22: add
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    // 23: mul
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    // 24: mul
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    // 25: mul
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0 = reg_tmp3.xyzz;
    // 120: mul
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    // 121: add
    reg_tmp11.xy = (uniforms.f[18].xxxx + reg_tmp1.xyyy).xy;
    // 122: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 123: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 124: call
    {
        sub_0_8();
    }
    // 125: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 126: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 127: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 128: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 129: mov
    vs_out_attr3 = reg_tmp2;
    // 130: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 131: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 132: call
    {
        sub_0_8();
    }
    // 133: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 134: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 135: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 136: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 137: mov
    vs_out_attr4 = reg_tmp2;
    // 138: mov
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    // 139: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 140: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 141: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 142: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 143: mov
    vs_out_attr5 = reg_tmp2;
    // 144: ifu
    if (uniforms.b[11]) {
        sub_145_170();
    } else {
        sub_170_171();
    }
    // 171: mov
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    // 172: mov
    vs_out_attr6 = reg_tmp11;
    // 173: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_145_170() {
    // 145: mov
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    // 146: ifu
    if (uniforms.b[6]) {
        sub_147_157();
    }
    // 157: nop
    // 158: ifu
    if (uniforms.b[5]) {
        sub_159_169();
    }
    // 169: nop
    return false;
}
bool sub_147_157() {
    // 147: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    // 148: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    // 149: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 150: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 151: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    // 152: max
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    // 153: mul
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    // 154: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    // 155: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 156: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_159_169() {
    // 159: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop159 = 0u; loop159 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop159) {
        sub_160_168();
    }
    // 168: nop
    return false;
}
bool sub_160_168() {
    // 160: mov
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    // 161: mov
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    // 162: mov
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    // 163: mov
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    // 164: call
    {
        sub_8_26();
    }
    // 165: add
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    // 166: max
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    // 167: min
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_170_171() {
    // 170: mov
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
// reference: 4ED75380F77FD6E9, D8CF3A496BEB73F0
// shader: 8B30, 308C4602F0691E8D

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((primary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1.0)) * (texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07D6D3745C52E1F8, 308C4602F0691E8D
// program: D8CF3A496BEB73F0, 1558286C47B9A4C4, 308C4602F0691E8D
// reference: 4ED75380ACA2242D, D8CF3A496BEB73F0
// shader: 8B31, D7A4C45FDF2D269A

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_123_126();
bool sub_135_138();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: ifu
    if (uniforms.b[8]) {
        sub_123_126();
    }
    // 126: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 127: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 128: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 129: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 130: mov
    vs_out_attr3 = reg_tmp2;
    // 131: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 132: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 133: call
    {
        sub_0_8();
    }
    // 134: ifu
    if (uniforms.b[9]) {
        sub_135_138();
    }
    // 138: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 139: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 140: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 141: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 142: mov
    vs_out_attr4 = reg_tmp2;
    // 143: mov
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    // 144: call
    {
        sub_0_8();
    }
    // 145: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 146: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 147: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 148: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 149: mov
    vs_out_attr5 = reg_tmp2;
    // 150: mov
    vs_out_attr6 = vs_in_reg8;
    // 151: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_123_126() {
    // 123: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 124: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 125: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_135_138() {
    // 135: dp4
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    // 136: dp4
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    // 137: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 9893B29D6C09C906, D7A4C45FDF2D269A
// shader: 8B30, EBA7F9B9C89EF964

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (1.0 - texcolor1.r) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E4C8C4D7C016F6E7, EBA7F9B9C89EF964
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, EBA7F9B9C89EF964
// shader: 8B30, E26E9C9BEE44BF6A

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rrr), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1.0) - (const_color[3].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4 * 1.0, alpha_output_4 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B0406A090A99180E, E26E9C9BEE44BF6A
// program: 8C95EA03C79EDDD2, 1558286C47B9A4C4, E26E9C9BEE44BF6A
// reference: 8F93736D1086C272, 955E4F6CC504D390
// reference: 5306B3CDAEB5D58E, 650D1363A7B584AA
// reference: 291B1F04C372C796, 0EE2E818CD1370A1
// reference: FA98D8FD43169415, 0EE2E818CD1370A1
// shader: 8B30, 4A3C5DEE38AC2214

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8A58895C0291B4F7, 4A3C5DEE38AC2214
// program: 3CBBD256D783A3F0, 1558286C47B9A4C4, 4A3C5DEE38AC2214
// shader: 8B30, 96CF0C24C2A9CF63

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((primary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1.0)) * (texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6577371C5C52E1F8, 96CF0C24C2A9CF63
// program: D8CF3A496BEB73F0, 1558286C47B9A4C4, 96CF0C24C2A9CF63
// shader: 8B30, AB1C067D3041E1DB

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 904E434F36FCCE64, AB1C067D3041E1DB
// program: C7012B058DBFF174, 1558286C47B9A4C4, AB1C067D3041E1DB
// shader: 8B30, 7F0F45D71E888C2C

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1.0) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 95889FAA67C38FBF, 7F0F45D71E888C2C
// program: F141D81C8BBB3C08, 1558286C47B9A4C4, 7F0F45D71E888C2C
// reference: 866920BFC016F6E7, EBA7F9B9C89EF964
// reference: D2E18E610A99180E, E26E9C9BEE44BF6A
// reference: 3F9B98D8744F75C5, C8096B09A214E787
// reference: EA2072359537D39A, 7FDFB08AA001B9CC
// reference: 3A214E142968D6EB, DAC934827CE046A0
// reference: 0560666E2968D6EB, 51FC9310092F95FF
// reference: 2E17E7DD56815930, D7A4C45FDF2D269A
// reference: 2E17E7DD1B9A746F, 955E4F6CC504D390
// reference: F282277DA5A96393, 650D1363A7B584AA
// reference: 889F8BB4C86E718B, 0EE2E818CD1370A1
// reference: 5B1C4C4D480A2208, 0EE2E818CD1370A1
// reference: 9893B29D2112E459, 955E4F6CC504D390
// reference: 4406723D9F21F3A5, 650D1363A7B584AA
// reference: 3E1BDEF4F2E6E1BD, 0EE2E818CD1370A1
// reference: ED98190D7282B23E, 0EE2E818CD1370A1
// reference: 9893B29D6F16A7B6, 948945C9E52B38B8
// shader: 8B30, D0EEBCBBC4EA66E2

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((vec3(1.0) - primary_fragment_color.aaa) * (vec3(1.0) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F507E25A69330965, D0EEBCBBC4EA66E2
// program: 948945C9E52B38B8, 1558286C47B9A4C4, D0EEBCBBC4EA66E2
// reference: 9893B29D34CB5572, 948945C9E52B38B8
// reference: 153E124406FBF3E7, F141D81C8BBB3C08
// reference: 9893B29D37D43BC2, D7A4C45FDF2D269A
// reference: F724F0A45A0805E6, 8C95EA03C79EDDD2
// reference: 8F93736D4B5B30B6, 955E4F6CC504D390
// reference: 2E17E7DD0D5CABF4, D7A4C45FDF2D269A
// shader: 8B30, D6B3AB8C46ABE2FA

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) - (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0 * 1.0, alpha_output_0 * 2.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 121C4F42D769F006, D6B3AB8C46ABE2FA
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, D6B3AB8C46ABE2FA
// shader: 8B31, F24C1B4722D7246A

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 123: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 124: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 125: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 126: mov
    vs_out_attr3 = reg_tmp2;
    // 127: mov
    vs_out_attr4 = vs_in_reg6;
    // 128: mov
    vs_out_attr5 = vs_in_reg7;
    // 129: mov
    vs_out_attr6 = vs_in_reg8;
    // 130: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
// reference: 44878084F74A813B, F24C1B4722D7246A
// shader: 8B30, 3AAC5B12D5F9EEF7

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (const_color[2].aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[2].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) * (const_color[2].a) + (last_tex_env_out.a) * (1.0 - (const_color[2].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C0189A31418C1E09, 3AAC5B12D5F9EEF7
// program: F24C1B4722D7246A, 1558286C47B9A4C4, 3AAC5B12D5F9EEF7
// reference: 4487808412E998A9, 955E4F6CC504D390
// reference: 4487808449346A6D, 955E4F6CC504D390
// reference: 98124024F7077D91, 650D1363A7B584AA
// reference: E20FECED9AC06F89, 0EE2E818CD1370A1
// reference: 318C2B141AA43C0A, 0EE2E818CD1370A1
// reference: 2E17E7DDE08B4A1D, F24C1B4722D7246A
// shader: 8B30, D9F99AB5F099EE0B

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (const_color[2].aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[2].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) * (const_color[2].a) + (last_tex_env_out.a) * (1.0 - (const_color[2].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 93990E20418C1E09, D9F99AB5F099EE0B
// program: F24C1B4722D7246A, 1558286C47B9A4C4, D9F99AB5F099EE0B
// reference: 2E17E7DDBB56B8D9, F24C1B4722D7246A
// reference: 2E17E7DD5EF5A14B, 955E4F6CC504D390
// reference: 2E17E7DD0528538F, 955E4F6CC504D390
// reference: F282277DBB1B4473, 650D1363A7B584AA
// reference: 889F8BB4D6DC566B, 0EE2E818CD1370A1
// reference: 5B1C4C4D56B805E8, 0EE2E818CD1370A1
// shader: 8B30, 4C8F75F3E3847C5A

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5352E8CE7F181D99, 4C8F75F3E3847C5A
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, 4C8F75F3E3847C5A
// reference: FD9420248D38F877, D7A4C45FDF2D269A
// shader: 8B30, 74F2A7AF93760CF9

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3C0BC44095028EBA, 74F2A7AF93760CF9
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, 74F2A7AF93760CF9
// shader: 8B31, 6F68D3AA0A914E7A

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_129_132();
bool sub_140_143();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: rcp
    reg_tmp0.w = rcp_s(reg_tmp10.w);
    // 120: mul
    reg_tmp0.xy = (mul_s(reg_tmp10.xyyy, reg_tmp0.wwww)).xy;
    // 121: mul
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    // 122: add
    reg_tmp1.xy = (uniforms.f[18].xxxx + reg_tmp1.yxxx).xy;
    // 123: mov
    reg_tmp0.xy = (uniforms.f[17].yyyy).xy;
    // 124: add
    reg_tmp14.xy = (reg_tmp0.xxxx + -reg_tmp1.xyyy).xy;
    // 125: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 126: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 127: call
    {
        sub_0_8();
    }
    // 128: ifu
    if (uniforms.b[8]) {
        sub_129_132();
    }
    // 132: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 133: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 134: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 135: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 136: mov
    vs_out_attr3 = reg_tmp2;
    // 137: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 138: mov
    reg_tmp0.xy = (reg_tmp14.xyyy).xy;
    // 139: ifu
    if (uniforms.b[9]) {
        sub_140_143();
    }
    // 143: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 144: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 145: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 146: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 147: mov
    vs_out_attr4 = reg_tmp2;
    // 148: mov
    vs_out_attr5 = vs_in_reg7;
    // 149: mov
    vs_out_attr6 = vs_in_reg8;
    // 150: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_129_132() {
    // 129: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 130: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 131: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_140_143() {
    // 140: dp4
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    // 141: dp4
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    // 142: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: AE285E35B1A1A8AA, 6F68D3AA0A914E7A
// shader: 8B30, A12AA4F561CC688F

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3 * 1.0, alpha_output_3 * 4.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a) + (combiner_buffer.a) * (1.0 - (const_color[4].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9207AC2772F87C55, A12AA4F561CC688F
// program: 6F68D3AA0A914E7A, 1558286C47B9A4C4, A12AA4F561CC688F
// reference: E8933AF65A0805E6, 8C95EA03C79EDDD2
// reference: 31A02D8F0D5CABF4, D7A4C45FDF2D269A
// reference: 31A02D8F56815930, D7A4C45FDF2D269A
// reference: E8933AF601D5F722, 8C95EA03C79EDDD2
// reference: 5B304AD6F74A813B, F24C1B4722D7246A
// reference: A319718A44649886, F24C1B4722D7246A
// reference: A319718AA1C78114, 955E4F6CC504D390
// reference: A319718AFA1A73D0, 955E4F6CC504D390
// reference: 7F8CB12A4429642C, 650D1363A7B584AA
// reference: 05911DE329EE7634, 0EE2E818CD1370A1
// reference: D612DA1AA98A25B7, 0EE2E818CD1370A1
// reference: BC4211658D38F877, D7A4C45FDF2D269A
// reference: 6FC1D69C0D5CABF4, D7A4C45FDF2D269A
// reference: 6FC1D69C56815930, D7A4C45FDF2D269A
// reference: AE285E35017A7B85, 6F68D3AA0A914E7A
// reference: AE285E355AA78941, 6F68D3AA0A914E7A
// reference: AE285E353BF2EBB3, 6F68D3AA0A914E7A
// reference: AE285E35602F1977, 6F68D3AA0A914E7A
// reference: AE285E35FDEC4D38, 6F68D3AA0A914E7A
// reference: 6FC1D69C37D43BC2, D7A4C45FDF2D269A
// reference: 6FC1D69C6C09C906, D7A4C45FDF2D269A
// reference: 9893B29DDA03DA2B, F24C1B4722D7246A
// reference: 9893B29D81DE28EF, F24C1B4722D7246A
// reference: 9893B29D647D317D, 955E4F6CC504D390
// reference: 9893B29D3FA0C3B9, 955E4F6CC504D390
// reference: 4406723D8193D445, 650D1363A7B584AA
// reference: 3E1BDEF4EC54C65D, 0EE2E818CD1370A1
// reference: ED98190D6C3095DE, 0EE2E818CD1370A1
// reference: 153E12440F4CE5AC, F24C1B4722D7246A
// reference: 153E1244EAEFFC3E, 955E4F6CC504D390
// reference: 153E1244B1320EFA, 955E4F6CC504D390
// reference: C9ABD2E40F011906, 650D1363A7B584AA
// reference: B3B67E2D62C60B1E, 0EE2E818CD1370A1
// reference: 6035B9D4E2A2589D, 0EE2E818CD1370A1
// reference: 37CEF7D43C8AB91B, C7012B058DBFF174
// shader: 8B31, D738EB2792A7060F

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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0_8();
bool sub_2_3();
bool sub_3_7();
bool sub_4_5();
bool sub_5_6();
bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_131_134();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_0_8() {
    // 0: cmp
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    // 1: ifc
    if (all(not(conditional_code))) {
        sub_2_3();
    } else {
        sub_3_7();
    }
    // 7: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_2_3() {
    // 2: mov
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_3_7() {
    // 3: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_4_5();
    } else {
        sub_5_6();
    }
    // 6: nop
    return false;
}
bool sub_4_5() {
    // 4: mov
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_5_6() {
    // 5: mov
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 120: mov
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    // 121: call
    {
        sub_0_8();
    }
    // 122: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 123: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 124: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 125: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 126: mov
    vs_out_attr3 = reg_tmp2;
    // 127: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 128: mov
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    // 129: call
    {
        sub_0_8();
    }
    // 130: ifu
    if (uniforms.b[9]) {
        sub_131_134();
    }
    // 134: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 135: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 136: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 137: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 138: mov
    vs_out_attr4 = reg_tmp2;
    // 139: mov
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    // 140: call
    {
        sub_0_8();
    }
    // 141: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 142: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 143: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 144: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 145: mov
    vs_out_attr5 = reg_tmp2;
    // 146: mov
    vs_out_attr6 = vs_in_reg8;
    // 147: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_131_134() {
    // 131: dp4
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    // 132: dp4
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    // 133: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 9893B29DB0B09A1B, D738EB2792A7060F
// shader: 8B30, 9BAB9D29AEF36710

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (secondary_fragment_color.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (secondary_fragment_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((texcolor2.rgb) * (vec3(1.0) - secondary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor1.r) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (texcolor0.r) + (last_tex_env_out.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 361595F9CBE52A8F, 9BAB9D29AEF36710
// program: D738EB2792A7060F, 1558286C47B9A4C4, 9BAB9D29AEF36710
// reference: 9893B29DEB6D68DF, D738EB2792A7060F
// reference: 9893B29DBA60BC5F, F24C1B4722D7246A
// reference: 1ED8EA78418C1E09, D9F99AB5F099EE0B
// reference: 9893B29DE1BD4E9B, F24C1B4722D7246A
// reference: 4D597E69418C1E09, 3AAC5B12D5F9EEF7
// reference: 9893B29D041E5709, 955E4F6CC504D390
// reference: B2DA7C80744F75C5, C8096B09A214E787
// reference: 9893B29D5FC3A5CD, 955E4F6CC504D390
// reference: 4406723DE1F0B231, 650D1363A7B584AA
// reference: 6761966D9537D39A, 7FDFB08AA001B9CC
// reference: 3E1BDEF48C37A029, 0EE2E818CD1370A1
// reference: B760AA4C2968D6EB, DAC934827CE046A0
// reference: ED98190D0C53F3AA, 0EE2E818CD1370A1
// reference: 882182362968D6EB, 51FC9310092F95FF
// shader: 8B30, 15BE5516D2D8B298

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1 * 4.0, alpha_output_1 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07196D040291B4F7, 15BE5516D2D8B298
// program: 3CBBD256D783A3F0, 1558286C47B9A4C4, 15BE5516D2D8B298
// reference: 2E17E7DDF2A0BDD0, D738EB2792A7060F
// reference: 2E17E7DDA97D4F14, D738EB2792A7060F
// reference: 2E17E7DDF8709B94, F24C1B4722D7246A
// reference: 2E17E7DDA3AD6950, F24C1B4722D7246A
// reference: 2E17E7DD460E70C2, 955E4F6CC504D390
// reference: 2E17E7DD1DD38206, 955E4F6CC504D390
// reference: F282277DA3E095FA, 650D1363A7B584AA
// reference: 889F8BB4CE2787E2, 0EE2E818CD1370A1
// reference: 5B1C4C4D4E43D461, 0EE2E818CD1370A1
// reference: F850DC048DE76019, F141D81C8BBB3C08
// reference: 75FD7CDD16DBBD97, D738EB2792A7060F
// reference: 75FD7CDD4D064F53, D738EB2792A7060F
// reference: 75FD7CDD1C0B9BD3, F24C1B4722D7246A
// reference: 75FD7CDD47D66917, F24C1B4722D7246A
// reference: 75FD7CDDA2757085, 955E4F6CC504D390
// reference: 75FD7CDDF9A88241, 955E4F6CC504D390
// reference: A968BC7D479B95BD, 650D1363A7B584AA
// reference: D37510B42A5C87A5, 0EE2E818CD1370A1
// reference: 00F6D74DAA38D426, 0EE2E818CD1370A1
// reference: 75FD7CDD05ACB534, D7A4C45FDF2D269A
// reference: 75FD7CDD5E7147F0, D7A4C45FDF2D269A
// reference: 75FD7CDD610DFEC7, D738EB2792A7060F
// reference: 75FD7CDDD5A333D5, 955E4F6CC504D390
// reference: A968BC7D6B902429, 650D1363A7B584AA
// reference: D37510B406573631, 0EE2E818CD1370A1
// reference: 00F6D74D863365B2, 0EE2E818CD1370A1
// reference: 9893B29D53757031, D738EB2792A7060F
// reference: 9893B29DE7DBBD23, 955E4F6CC504D390
// reference: 4406723D59E8AADF, 650D1363A7B584AA
// reference: 3E1BDEF4342FB8C7, 0EE2E818CD1370A1
// reference: ED98190DB44BEB44, 0EE2E818CD1370A1
// reference: 9893B29D0278A4B1, F24C1B4722D7246A
// reference: 9893B29D59A55675, F24C1B4722D7246A
// reference: 9893B29DBC064FE7, 955E4F6CC504D390
// reference: DE130C967F181D99, 4C8F75F3E3847C5A
// reference: 5FA06A390A99180E, E26E9C9BEE44BF6A
// reference: 2E17E7DD322012C3, D738EB2792A7060F
// reference: 2E17E7DD69FDE007, D738EB2792A7060F
// reference: B14A201895028EBA, 74F2A7AF93760CF9
// reference: 883B045B5F0DA4E6, 5C4AE0ED323B16A1
// reference: 1F430FEE7678105E, 5A9DBA5B31BC353E
// reference: 883B045B04D05622, 5C4AE0ED323B16A1
// reference: 9893B29D08A882F5, D738EB2792A7060F
// reference: 883B045B1605F6B2, 5C4AE0ED323B16A1
// reference: 9202EBB67678105E, 5A9DBA5B31BC353E
// reference: 3B98747BBB47726F, F24C1B4722D7246A
// reference: 3B98747B5EE46BFD, 955E4F6CC504D390
// reference: 3B98747B05399939, 955E4F6CC504D390
// reference: E70DB4DBBB0A8EC5, 650D1363A7B584AA
// reference: 9D101812D6CD9CDD, 0EE2E818CD1370A1
// reference: 4E93DFEB56A9CF5E, 0EE2E818CD1370A1
// reference: 883B045BED7664EA, 5C4AE0ED323B16A1
// reference: 883B045BAE1DEE5C, 5C4AE0ED323B16A1
// reference: 3B98747B035F6A81, F24C1B4722D7246A
// reference: 3B98747BE6FC7313, 955E4F6CC504D390
// reference: 3B98747BBD2181D7, 955E4F6CC504D390
// reference: E70DB4DB0312962B, 650D1363A7B584AA
// reference: 9D1018126ED58433, 0EE2E818CD1370A1
// reference: 4E93DFEBEEB1D7B0, 0EE2E818CD1370A1
// reference: F724F0A40B3D95B3, 5C4AE0ED323B16A1
// reference: 3EBF511BEC0DC997, 5C4AE0ED323B16A1
// reference: 8D1C213B414F4D4A, F24C1B4722D7246A
// reference: 8D1C213BA4EC54D8, 955E4F6CC504D390
// reference: 8D1C213BFF31A61C, 955E4F6CC504D390
// reference: 5189E19B4102B1E0, 650D1363A7B584AA
// reference: 2B944D522CC5A3F8, 0EE2E818CD1370A1
// reference: F8178AABACA1F07B, 0EE2E818CD1370A1
// reference: 883B045BF5C01C98, 5C4AE0ED323B16A1
// reference: 6555CA1B666AF714, 5C4AE0ED323B16A1
// reference: 6555CA1B53AB3B14, 5C4AE0ED323B16A1
// reference: 6555CA1B0876C9D0, 5C4AE0ED323B16A1
// reference: D6F6BA3B4097549F, 955E4F6CC504D390
// reference: D6F6BA3B1B4AA65B, 955E4F6CC504D390
// reference: 0A637A9BA579B1A7, 650D1363A7B584AA
// reference: 707ED652C8BEA3BF, 0EE2E818CD1370A1
// reference: A3FD11AB48DAF03C, 0EE2E818CD1370A1
// reference: 7210E051599E0120, 8C95EA03C79EDDD2
// reference: AB23F728F136B916, D738EB2792A7060F
// reference: AB23F728AAEB4BD2, D738EB2792A7060F
// reference: AB23F728FBE69F52, F24C1B4722D7246A
// reference: AB23F728A03B6D96, F24C1B4722D7246A
// reference: AB23F72845987404, 955E4F6CC504D390
// reference: AB23F7281E4586C0, 955E4F6CC504D390
// reference: 77B63788A076913C, 650D1363A7B584AA
// reference: 0DAB9B41CDB18324, 0EE2E818CD1370A1
// reference: DE285CB84DD5D0A7, 0EE2E818CD1370A1
// reference: 7210E0510243F3E4, 8C95EA03C79EDDD2
// reference: AB23F7280ECAAF32, D7A4C45FDF2D269A
// shader: 8B30, 77367DAEBAD0D6B0

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1AAD98A13E08E26C, 77367DAEBAD0D6B0
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, 77367DAEBAD0D6B0
// reference: 75FD7CDD3AD00C03, D738EB2792A7060F
// shader: 8B30, 952B918DBAD0D6B0

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 49E652093E08E26C, 952B918DBAD0D6B0
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, 952B918DBAD0D6B0
// reference: 75FD7CDD5605BF8B, 955E4F6CC504D390
// reference: 75FD7CDD0DD84D4F, 955E4F6CC504D390
// reference: A968BC7DB3EB5AB3, 650D1363A7B584AA
// reference: D37510B4DE2C48AB, 0EE2E818CD1370A1
// reference: 00F6D74D5E481B28, 0EE2E818CD1370A1
// reference: AB23F72831B61605, D738EB2792A7060F
// reference: AB23F7286A6BE4C1, D738EB2792A7060F
// reference: AB23F72855175DF6, D7A4C45FDF2D269A
// reference: C1B39071117F9C6F, 955E4F6CC504D390
// reference: C1B390714AA26EAB, 955E4F6CC504D390
// reference: 1D2650D1F4917957, 650D1363A7B584AA
// reference: 673BFC1899566B4F, 0EE2E818CD1370A1
// reference: B4B83BE1193238CC, 0EE2E818CD1370A1
// reference: 97EC7CF93E08E26C, 77367DAEBAD0D6B0
// reference: ABE79D93EC4EA26B, 95FFD538EC92B26D
// reference: F850DC04D63A92DD, F141D81C8BBB3C08
// shader: 8B30, 310A956E3DE1AB8E

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B19979ED8DEEF25B, 310A956E3DE1AB8E
// program: D7A4C45FDF2D269A, 1558286C47B9A4C4, 310A956E3DE1AB8E
// reference: 3B98747B4C31CB6D, 955E4F6CC504D390
// reference: 3B98747B17EC39A9, 955E4F6CC504D390
// reference: E70DB4DBA9DF2E55, 650D1363A7B584AA
// reference: 9D101812C4183C4D, 0EE2E818CD1370A1
// reference: 4E93DFEB447C6FCE, 0EE2E818CD1370A1
// shader: 8B31, FF482857BFE5B258

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
layout(location = 3) in vec4 vs_in_reg3;
layout(location = 4) in vec4 vs_in_reg4;
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


#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_27_38();
bool sub_30_34();
bool sub_39_54();
bool sub_41_42();
bool sub_52_53();
bool sub_55_64();
bool sub_57_58();
bool sub_62_63();
bool sub_65_74();
bool sub_67_68();
bool sub_72_73();
bool sub_75_4096();
bool sub_78_79();
bool sub_87_90();
bool sub_95_101();
bool sub_107_117();
bool sub_113_115();
bool sub_115_116();
bool sub_117_118();
bool sub_125_128();
bool sub_136_139();

bool exec_shader() {
    sub_75_4096();
    return true;
}

bool sub_27_38() {
    // 27: mova
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    // 28: nop
    // 29: ifu
    if (uniforms.b[4]) {
        sub_30_34();
    }
    // 34: dp4
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    // 35: dp4
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    // 36: dp4
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    // 37: mad
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_30_34() {
    // 30: dp3
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    // 31: dp3
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    // 32: dp3
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    // 33: mad
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_39_54() {
    // 39: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 40: ifu
    if (uniforms.b[4]) {
        sub_41_42();
    }
    // 42: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 43: mov
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    // 44: call
    {
        sub_27_38();
    }
    // 45: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    // 46: mov
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    // 47: call
    {
        sub_27_38();
    }
    // 48: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    // 49: mov
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    // 50: call
    {
        sub_27_38();
    }
    // 51: ifu
    if (uniforms.b[4]) {
        sub_52_53();
    }
    // 53: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_41_42() {
    // 41: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_52_53() {
    // 52: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_55_64() {
    // 55: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 56: ifu
    if (uniforms.b[4]) {
        sub_57_58();
    }
    // 58: mul
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    // 59: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 60: call
    {
        sub_27_38();
    }
    // 61: ifu
    if (uniforms.b[4]) {
        sub_62_63();
    }
    // 63: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_57_58() {
    // 57: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_62_63() {
    // 62: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_65_74() {
    // 65: mov
    reg_tmp7 = uniforms.f[17].xxxy;
    // 66: ifu
    if (uniforms.b[4]) {
        sub_67_68();
    }
    // 68: mov
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    // 69: mov
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    // 70: call
    {
        sub_27_38();
    }
    // 71: ifu
    if (uniforms.b[4]) {
        sub_72_73();
    }
    // 73: mov
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_67_68() {
    // 67: mov
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_72_73() {
    // 72: mov
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_75_4096() {
    // 75: mov
    reg_tmp10 = uniforms.f[17].xxxy;
    // 76: mov
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    // 77: ifu
    if (uniforms.b[4]) {
        sub_78_79();
    }
    // 79: callu
    if (uniforms.b[3]) {
        sub_39_54();
    }
    // 80: callu
    if (uniforms.b[2]) {
        sub_55_64();
    }
    // 81: callu
    if (uniforms.b[1]) {
        sub_65_74();
    }
    // 82: dp4
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    // 83: dp4
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    // 84: dp4
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    // 85: mov
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    // 86: ifu
    if (uniforms.b[4]) {
        sub_87_90();
    }
    // 90: dp4
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    // 91: dp4
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    // 92: dp4
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    // 93: mov
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    // 94: ifu
    if (uniforms.b[4]) {
        sub_95_101();
    }
    // 101: dp4
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    // 102: dp4
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    // 103: dp4
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    // 104: dp4
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    // 105: mov
    vs_out_attr0 = reg_tmp10;
    // 106: ifu
    if (uniforms.b[4]) {
        sub_107_117();
    } else {
        sub_117_118();
    }
    // 118: mov
    vs_out_attr2 = -reg_tmp12;
    // 119: mov
    reg_tmp0 = reg_tmp3.xyzz;
    // 120: mul
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    // 121: add
    reg_tmp11.xy = (uniforms.f[18].xxxx + reg_tmp1.xyyy).xy;
    // 122: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 123: mov
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    // 124: ifu
    if (uniforms.b[8]) {
        sub_125_128();
    }
    // 128: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 129: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 130: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 131: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 132: mov
    vs_out_attr3 = reg_tmp2;
    // 133: mov
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    // 134: mov
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    // 135: ifu
    if (uniforms.b[9]) {
        sub_136_139();
    }
    // 139: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 140: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 141: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 142: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 143: mov
    vs_out_attr4 = reg_tmp2;
    // 144: mov
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    // 145: mov
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    // 146: mov
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    // 147: add
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    // 148: mov
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    // 149: mov
    vs_out_attr5 = reg_tmp2;
    // 150: mov
    vs_out_attr6 = vs_in_reg8;
    // 151: end
    return true;
}
bool sub_78_79() {
    // 78: mov
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_87_90() {
    // 87: dp3
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    // 88: dp3
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    // 89: dp3
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_95_101() {
    // 95: dp3
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    // 96: dp3
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    // 97: dp3
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    // 98: dp4
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    // 99: rsq
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    // 100: mul
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_107_117() {
    // 107: add
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    // 108: mul
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    // 109: cmp
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    // 110: rsq
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    // 111: mul
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    // 112: ifc
    if (!conditional_code.x) {
        sub_113_115();
    } else {
        sub_115_116();
    }
    // 116: mov
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_113_115() {
    // 113: rcp
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    // 114: mul
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_115_116() {
    // 115: mov
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_117_118() {
    // 117: mov
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_125_128() {
    // 125: dp4
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    // 126: dp4
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    // 127: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_136_139() {
    // 136: dp4
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    // 137: dp4
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    // 138: mov
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 9E95E2364328DD22, FF482857BFE5B258
// shader: 8B30, 51084D92732AF849

precision highp int;
precision highp float;
precision highp samplerBuffer;
precision highp usampler2D;
precision highp uimage2D;
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
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
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
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16,clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor0.aaa) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) + (texcolor1.r), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1.0)) * (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((texcolor0.aaa) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2815826E3AF816D5, 51084D92732AF849
// program: FF482857BFE5B258, 1558286C47B9A4C4, 51084D92732AF849
// reference: 9E95E23618F52FE6, FF482857BFE5B258
// reference: 3EBF511B761B0F05, 5C4AE0ED323B16A1
// reference: 8D1C213B3EFA924A, 955E4F6CC504D390
// reference: 8D1C213B6527608E, 955E4F6CC504D390
// reference: 5189E19BDB147772, 650D1363A7B584AA
// reference: 2B944D52B6D3656A, 0EE2E818CD1370A1
// reference: F8178AAB36B736E9, 0EE2E818CD1370A1
// reference: 2811B77679A04D14, FF482857BFE5B258
// reference: 2811B776227DBFD0, FF482857BFE5B258
// reference: 883B045B4C939F33, 5C4AE0ED323B16A1
// reference: 3B98747B0472027C, 955E4F6CC504D390
// reference: 3B98747B5FAFF0B8, 955E4F6CC504D390
// reference: E70DB4DBE19CE744, 650D1363A7B584AA
// reference: 9D1018128C5BF55C, 0EE2E818CD1370A1
// reference: 4E93DFEB0C3FA6DF, 0EE2E818CD1370A1
