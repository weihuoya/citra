// shader: 8B31, 11E23C46714F2847

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

// accurate_mul: off
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
// reference: 84E4A09237F27966, 11E23C46714F2847
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
// shader: 8B30, 40F8EFB49BDAF61F
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
// reference: 8DC4F5161F04A720, 40F8EFB49BDAF61F
// program: 11E23C46714F2847, 1558286C47B9A4C4, 40F8EFB49BDAF61F
// reference: 0F04773DEC4BD821, 11E23C46714F2847
// shader: 8B31, 62AC3A573F5BC1DA

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

// accurate_mul: off
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
// reference: 6FADE78267574BDF, 62AC3A573F5BC1DA
// shader: 8B30, 473E83AAACA26D23
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
// reference: 97D23F052B69DDB3, 473E83AAACA26D23
// program: 62AC3A573F5BC1DA, 1558286C47B9A4C4, 473E83AAACA26D23
// shader: 8B31, C5CB5AE762DDBE00

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

// accurate_mul: off
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
// reference: 4D5D02125D260123, C5CB5AE762DDBE00
// shader: 8B30, 38CED9AB9E619327
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
// reference: F7297BC267C38FBF, 38CED9AB9E619327
// program: C5CB5AE762DDBE00, 1558286C47B9A4C4, 38CED9AB9E619327
// shader: 8B30, 78758B230A73C535
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
// reference: C94679FBEC4EA26B, 78758B230A73C535
// program: C5CB5AE762DDBE00, 1558286C47B9A4C4, 78758B230A73C535
// shader: 8B31, 3AD8D2DAECDF7CB7

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

// accurate_mul: off
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
// reference: 4D5D0212CB8EB4A8, 3AD8D2DAECDF7CB7
// shader: 8B30, BB6719D4609B27EB
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
// reference: 97A606328584E266, BB6719D4609B27EB
// program: 3AD8D2DAECDF7CB7, 1558286C47B9A4C4, BB6719D4609B27EB
// shader: 8B31, A980391F77320435

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

// accurate_mul: off
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
// reference: D058140DB6AB962E, A980391F77320435
// shader: 8B30, C3A3333B2605F9E8
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
// reference: 7DE2EB867678105E, C3A3333B2605F9E8
// program: A980391F77320435, 1558286C47B9A4C4, C3A3333B2605F9E8
// reference: 63FB642DCB8EB4A8, 3AD8D2DAECDF7CB7
// shader: 8B31, 1FEF377927E41FEB

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

// accurate_mul: off
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
// reference: A8EF97C4A7F8A37E, 1FEF377927E41FEB
// shader: 8B30, DA18EB77535D4A6B
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
// reference: 5D3A7CB0744F75C5, DA18EB77535D4A6B
// program: 1FEF377927E41FEB, 1558286C47B9A4C4, DA18EB77535D4A6B
// shader: 8B31, FF79F51CFF07B40E

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

// accurate_mul: off
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
// reference: AF47E0F201D5F722, FF79F51CFF07B40E
// shader: 8B30, 6A6EBBD14B0B402F
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
// reference: E3C1FE180A99180E, 6A6EBBD14B0B402F
// program: FF79F51CFF07B40E, 1558286C47B9A4C4, 6A6EBBD14B0B402F
// shader: 8B31, 601D0DED55163701

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

// accurate_mul: off
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
// reference: 2BB13004BFE6E0DE, 601D0DED55163701
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
// shader: 8B30, D3C92BE999E5576B
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
// reference: 8881965D9537D39A, D3C92BE999E5576B
// program: 601D0DED55163701, 5D764F9A6220D694, D3C92BE999E5576B
// shader: 8B31, E2719FB2D51789A8

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

// accurate_mul: off
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
// reference: 51AC9CCDD221F2C6, E2719FB2D51789A8
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
// shader: 8B30, B0994BBEEF35477B
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
// reference: 5880AA7C2968D6EB, B0994BBEEF35477B
// program: E2719FB2D51789A8, 4E5317772B5CE683, B0994BBEEF35477B
// reference: DA4C4B625245A145, E2719FB2D51789A8
// shader: 8B30, A744B7F491DEFF28
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
// reference: 67C182062968D6EB, A744B7F491DEFF28
// program: E2719FB2D51789A8, 4E5317772B5CE683, A744B7F491DEFF28
// shader: 8B30, 25BDE373387E0A9D
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
// reference: E8F96D340291B4F7, 25BDE373387E0A9D
// program: 11E23C46714F2847, 1558286C47B9A4C4, 25BDE373387E0A9D
// shader: 8B30, 59A1B2CEC38A81F6
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
// reference: F2EFA72736FCCE64, 59A1B2CEC38A81F6
// program: 62AC3A573F5BC1DA, 1558286C47B9A4C4, 59A1B2CEC38A81F6
// reference: 4D5D0212869599F7, 1FEF377927E41FEB
// reference: 0F43A0EB6D0640EF, A744B7F491DEFF28
// reference: 0F04773DB7962AE5, 11E23C46714F2847
// shader: 8B31, 296606112F4BB331

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

// accurate_mul: off
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
// reference: 16B443D6F77FD6E9, 296606112F4BB331
// shader: 8B30, F8F7C2E9694AE96C
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
// reference: 07D6D3745C52E1F8, F8F7C2E9694AE96C
// program: 296606112F4BB331, 1558286C47B9A4C4, F8F7C2E9694AE96C
// reference: 16B443D6ACA2242D, 296606112F4BB331
// shader: 8B31, CC418327A26829B6

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

// accurate_mul: off
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
// reference: C0F0A2CB6C09C906, CC418327A26829B6
// shader: 8B30, 707C29E761BB927A
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
// reference: E4C8C4D7C016F6E7, 707C29E761BB927A
// program: CC418327A26829B6, 1558286C47B9A4C4, 707C29E761BB927A
// shader: 8B30, 40B85CA9AABDE064
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
// reference: B0406A090A99180E, 40B85CA9AABDE064
// program: FF79F51CFF07B40E, 1558286C47B9A4C4, 40B85CA9AABDE064
// reference: D7F0633B1086C272, 1FEF377927E41FEB
// reference: 5306B3CDAEB5D58E, 601D0DED55163701
// reference: 291B1F04C372C796, E2719FB2D51789A8
// reference: A2FBC8AB43169415, E2719FB2D51789A8
// shader: 8B30, D095B23F8F249D25
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
// reference: 8A58895C0291B4F7, D095B23F8F249D25
// program: 11E23C46714F2847, 1558286C47B9A4C4, D095B23F8F249D25
// shader: 8B30, 102AAF2736C4D6BC
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
// reference: 6577371C5C52E1F8, 102AAF2736C4D6BC
// program: 296606112F4BB331, 1558286C47B9A4C4, 102AAF2736C4D6BC
// shader: 8B30, 4C6576ED3A916D69
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
// reference: 904E434F36FCCE64, 4C6576ED3A916D69
// program: 62AC3A573F5BC1DA, 1558286C47B9A4C4, 4C6576ED3A916D69
// shader: 8B30, CAC5F47EE27F0882
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
// reference: 95889FAA67C38FBF, CAC5F47EE27F0882
// program: C5CB5AE762DDBE00, 1558286C47B9A4C4, CAC5F47EE27F0882
// reference: 866920BFC016F6E7, 707C29E761BB927A
// reference: D2E18E610A99180E, 40B85CA9AABDE064
// reference: 3F9B98D8744F75C5, DA18EB77535D4A6B
// reference: EA2072359537D39A, D3C92BE999E5576B
// reference: 3A214E142968D6EB, B0994BBEEF35477B
// reference: 0560666E2968D6EB, A744B7F491DEFF28
// reference: 7674F78B56815930, CC418327A26829B6
// reference: 7674F78B1B9A746F, 1FEF377927E41FEB
// reference: F282277DA5A96393, 601D0DED55163701
// reference: 889F8BB4C86E718B, E2719FB2D51789A8
// reference: 037F5C1B480A2208, E2719FB2D51789A8
// reference: C0F0A2CB2112E459, 1FEF377927E41FEB
// reference: 4406723D9F21F3A5, 601D0DED55163701
// reference: 3E1BDEF4F2E6E1BD, E2719FB2D51789A8
// reference: B5FB095B7282B23E, E2719FB2D51789A8
// reference: C0F0A2CB6F16A7B6, 3AD8D2DAECDF7CB7
// shader: 8B30, 24274D0DEBBF80FB
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
// reference: F507E25A69330965, 24274D0DEBBF80FB
// program: 3AD8D2DAECDF7CB7, 1558286C47B9A4C4, 24274D0DEBBF80FB
// reference: C0F0A2CB34CB5572, 3AD8D2DAECDF7CB7
// reference: 4D5D021206FBF3E7, C5CB5AE762DDBE00
// reference: C0F0A2CB37D43BC2, CC418327A26829B6
// reference: AF47E0F25A0805E6, FF79F51CFF07B40E
// reference: D7F0633B4B5B30B6, 1FEF377927E41FEB
// reference: 7674F78B0D5CABF4, CC418327A26829B6
// shader: 8B30, 731C705D041CE04B
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
// reference: 121C4F42D769F006, 731C705D041CE04B
// program: CC418327A26829B6, 1558286C47B9A4C4, 731C705D041CE04B
// shader: 8B31, 5DE2B29E60B51CEE

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

// accurate_mul: off
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
// reference: 1CE490D2F74A813B, 5DE2B29E60B51CEE
// shader: 8B30, F61A4642FFC556DF
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
// reference: C0189A31418C1E09, F61A4642FFC556DF
// program: 5DE2B29E60B51CEE, 1558286C47B9A4C4, F61A4642FFC556DF
// reference: 1CE490D212E998A9, 1FEF377927E41FEB
// reference: 1CE490D249346A6D, 1FEF377927E41FEB
// reference: 98124024F7077D91, 601D0DED55163701
// reference: E20FECED9AC06F89, E2719FB2D51789A8
// reference: 69EF3B421AA43C0A, E2719FB2D51789A8
// reference: 7674F78BE08B4A1D, 5DE2B29E60B51CEE
// shader: 8B30, E9B5F5169F5288ED
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
// reference: 93990E20418C1E09, E9B5F5169F5288ED
// program: 5DE2B29E60B51CEE, 1558286C47B9A4C4, E9B5F5169F5288ED
// reference: 7674F78BBB56B8D9, 5DE2B29E60B51CEE
// reference: 7674F78B5EF5A14B, 1FEF377927E41FEB
// reference: 7674F78B0528538F, 1FEF377927E41FEB
// reference: F282277DBB1B4473, 601D0DED55163701
// reference: 889F8BB4D6DC566B, E2719FB2D51789A8
// reference: 037F5C1B56B805E8, E2719FB2D51789A8
// shader: 8B30, 6B798FABF5442B14
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
// reference: 5352E8CE7F181D99, 6B798FABF5442B14
// program: CC418327A26829B6, 1558286C47B9A4C4, 6B798FABF5442B14
// reference: FD9420248D38F877, CC418327A26829B6
// shader: 8B30, E11545490C6ED519
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
// reference: 3C0BC44095028EBA, E11545490C6ED519
// program: CC418327A26829B6, 1558286C47B9A4C4, E11545490C6ED519
// shader: 8B31, FF98BDB3D0B326BD

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

// accurate_mul: off
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
// reference: F64B4E63B1A1A8AA, FF98BDB3D0B326BD
// shader: 8B30, 2AC0F01FB624A193
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
// reference: 9207AC2772F87C55, 2AC0F01FB624A193
// program: FF98BDB3D0B326BD, 1558286C47B9A4C4, 2AC0F01FB624A193
// reference: 37A2C6CA0D5CABF4, CC418327A26829B6
// reference: 37A2C6CA56815930, CC418327A26829B6
// reference: B0F02AA001D5F722, FF79F51CFF07B40E
// reference: B0F02AA05A0805E6, FF79F51CFF07B40E
// reference: 69C33DD90D5CABF4, CC418327A26829B6
// reference: 03535A80F74A813B, 5DE2B29E60B51CEE
// reference: FB7A61DC44649886, 5DE2B29E60B51CEE
// reference: FB7A61DCA1C78114, 1FEF377927E41FEB
// reference: FB7A61DCFA1A73D0, 1FEF377927E41FEB
// reference: 7F8CB12A4429642C, 601D0DED55163701
// reference: 05911DE329EE7634, E2719FB2D51789A8
// reference: 8E71CA4CA98A25B7, E2719FB2D51789A8
// reference: BC4211658D38F877, CC418327A26829B6
// reference: C0F0A2CBDA03DA2B, 5DE2B29E60B51CEE
// reference: C0F0A2CB81DE28EF, 5DE2B29E60B51CEE
// reference: C0F0A2CB647D317D, 1FEF377927E41FEB
// reference: C0F0A2CB3FA0C3B9, 1FEF377927E41FEB
// reference: 4406723D8193D445, 601D0DED55163701
// reference: 3E1BDEF4EC54C65D, E2719FB2D51789A8
// reference: B5FB095B6C3095DE, E2719FB2D51789A8
// reference: 4D5D02120F4CE5AC, 5DE2B29E60B51CEE
// reference: 4D5D0212EAEFFC3E, 1FEF377927E41FEB
// reference: 4D5D0212B1320EFA, 1FEF377927E41FEB
// reference: C9ABD2E40F011906, 601D0DED55163701
// reference: B3B67E2D62C60B1E, E2719FB2D51789A8
// reference: 3856A982E2A2589D, E2719FB2D51789A8
// reference: 6FADE7823C8AB91B, 62AC3A573F5BC1DA
// shader: 8B31, B7F8F6B7FE98CBF3

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

// accurate_mul: off
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
// reference: C0F0A2CBB0B09A1B, B7F8F6B7FE98CBF3
// shader: 8B30, 06F087482AD90BE7
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
// reference: 361595F9CBE52A8F, 06F087482AD90BE7
// program: B7F8F6B7FE98CBF3, 1558286C47B9A4C4, 06F087482AD90BE7
// reference: C0F0A2CBEB6D68DF, B7F8F6B7FE98CBF3
// reference: C0F0A2CBBA60BC5F, 5DE2B29E60B51CEE
// reference: 1ED8EA78418C1E09, E9B5F5169F5288ED
// reference: C0F0A2CBE1BD4E9B, 5DE2B29E60B51CEE
// reference: 4D597E69418C1E09, F61A4642FFC556DF
// reference: C0F0A2CB041E5709, 1FEF377927E41FEB
// reference: B2DA7C80744F75C5, DA18EB77535D4A6B
// reference: C0F0A2CB5FC3A5CD, 1FEF377927E41FEB
// reference: 4406723DE1F0B231, 601D0DED55163701
// reference: 6761966D9537D39A, D3C92BE999E5576B
// reference: 3E1BDEF48C37A029, E2719FB2D51789A8
// reference: B760AA4C2968D6EB, B0994BBEEF35477B
// reference: B5FB095B0C53F3AA, E2719FB2D51789A8
// reference: 882182362968D6EB, A744B7F491DEFF28
// shader: 8B30, EEF39F91F2E53207
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
// reference: 07196D040291B4F7, EEF39F91F2E53207
// program: 11E23C46714F2847, 1558286C47B9A4C4, EEF39F91F2E53207
// reference: 7674F78BF2A0BDD0, B7F8F6B7FE98CBF3
// reference: 7674F78BA97D4F14, B7F8F6B7FE98CBF3
// reference: 7674F78BF8709B94, 5DE2B29E60B51CEE
// reference: 7674F78BA3AD6950, 5DE2B29E60B51CEE
// reference: 7674F78B460E70C2, 1FEF377927E41FEB
// reference: 7674F78B1DD38206, 1FEF377927E41FEB
// reference: F282277DA3E095FA, 601D0DED55163701
// reference: 889F8BB4CE2787E2, E2719FB2D51789A8
// reference: 037F5C1B4E43D461, E2719FB2D51789A8
// reference: A033CC528DE76019, C5CB5AE762DDBE00
// reference: 2D9E6C8B16DBBD97, B7F8F6B7FE98CBF3
// reference: 2D9E6C8B4D064F53, B7F8F6B7FE98CBF3
// reference: 2D9E6C8BF9A88241, 1FEF377927E41FEB
// reference: A968BC7D479B95BD, 601D0DED55163701
// reference: D37510B42A5C87A5, E2719FB2D51789A8
// reference: 5895C71BAA38D426, E2719FB2D51789A8
// reference: 2D9E6C8B05ACB534, CC418327A26829B6
// reference: 2D9E6C8B5E7147F0, CC418327A26829B6
// reference: 2D9E6C8B610DFEC7, B7F8F6B7FE98CBF3
// reference: 2D9E6C8BD5A333D5, 1FEF377927E41FEB
// reference: A968BC7D6B902429, 601D0DED55163701
// reference: D37510B406573631, E2719FB2D51789A8
// reference: 5895C71B863365B2, E2719FB2D51789A8
// reference: 2D9E6C8B30002A47, 5DE2B29E60B51CEE
// reference: 2D9E6C8B6BDDD883, 5DE2B29E60B51CEE
// reference: 2D9E6C8B8E7EC111, 1FEF377927E41FEB
// reference: DE130C967F181D99, 6B798FABF5442B14
// reference: 2A73F007599E0120, FF79F51CFF07B40E
// reference: 5FA06A390A99180E, 40B85CA9AABDE064
// reference: F340E77EF136B916, B7F8F6B7FE98CBF3
// reference: F340E77EAAEB4BD2, B7F8F6B7FE98CBF3
// reference: F340E77EFBE69F52, 5DE2B29E60B51CEE
// reference: F340E77EA03B6D96, 5DE2B29E60B51CEE
// reference: F340E77E45987404, 1FEF377927E41FEB
// reference: F340E77E1E4586C0, 1FEF377927E41FEB
// reference: 77B63788A076913C, 601D0DED55163701
// reference: 0DAB9B41CDB18324, E2719FB2D51789A8
// reference: 864B4CEE4DD5D0A7, E2719FB2D51789A8
// reference: 2A73F0070243F3E4, FF79F51CFF07B40E
// reference: F340E77E0ECAAF32, CC418327A26829B6
// reference: 99D08027F4DC85FD, 5DE2B29E60B51CEE
// reference: 99D08027117F9C6F, 1FEF377927E41FEB
// reference: 99D080274AA26EAB, 1FEF377927E41FEB
// reference: 1D2650D1F4917957, 601D0DED55163701
// reference: 673BFC1899566B4F, E2719FB2D51789A8
// reference: ECDB2BB7193238CC, E2719FB2D51789A8
// reference: C0F0A2CB53757031, B7F8F6B7FE98CBF3
// reference: D058140DED7664EA, A980391F77320435
// reference: 1F430FEE7678105E, C3A3333B2605F9E8
// reference: 66DC414D658534D0, A980391F77320435
// reference: 66DC414D3E58C614, A980391F77320435
// reference: D57F316D76B95B5B, 1FEF377927E41FEB
// reference: D57F316D2D64A99F, 1FEF377927E41FEB
// reference: 5189E19B9357BE63, 601D0DED55163701
// reference: 2B944D52FE90AC7B, E2719FB2D51789A8
// reference: A0749AFD7EF4FFF8, E2719FB2D51789A8
