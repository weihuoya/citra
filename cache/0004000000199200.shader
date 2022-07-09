// shader: 8B31, D9C174A4A54356AC

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

bool sub_0_97();
bool sub_5_50();
bool sub_12_13();
bool sub_20_21();
bool sub_22_24();
bool sub_31_32();
bool sub_33_34();
bool sub_40_41();
bool sub_42_43();
bool sub_48_49();
bool sub_52_69();
bool sub_57_60();
bool sub_60_68();
bool sub_62_63();
bool sub_63_64();
bool sub_65_66();
bool sub_66_67();
bool sub_69_96();
bool sub_70_93();
bool sub_74_79();
bool sub_79_92();
bool sub_83_87();
bool sub_84_85();
bool sub_85_86();
bool sub_87_91();
bool sub_88_89();
bool sub_89_90();
bool sub_93_95();
bool sub_97_109();
bool sub_109_4096();
bool sub_125_127();
bool sub_127_149();
bool sub_137_141();
bool sub_158_160();
bool sub_161_162();
bool sub_171_173();
bool sub_174_175();
bool sub_184_186();
bool sub_187_188();
bool sub_198_207();
bool sub_215_223();
bool sub_223_227();

bool exec_shader() {
    sub_109_4096();
    return true;
}

bool sub_0_97() {
    reg_tmp13 = floor(reg_tmp0.xxxx);
    reg_tmp13 = reg_tmp0.xxxx + -reg_tmp13;
    address_registers.y = (ivec2(reg_tmp11.zz)).y;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xx);
    if (conditional_code.x) {
        sub_5_50();
    }
    conditional_code = lessThanEqual(uniforms.f[5].yy, reg_tmp11.xy);
    if (!conditional_code.y) {
        sub_52_69();
    } else {
        sub_69_96();
    }
    return false;
}
bool sub_5_50() {
    reg_tmp12.xy = (uniforms.f[5].xyyy + vs_in_reg0.zwww).xy;
    reg_tmp14.xy = (uniforms.f[6].wzzz).xy;
    reg_tmp13.xy = (mul_s(uniforms.f[5].zzzz, reg_tmp0.xxxx)).xy;
    reg_tmp13.y = (floor(reg_tmp13)).y;
    reg_tmp13.x = (reg_tmp13.xxxx + -reg_tmp13.yyyy).x;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xx);
    if (conditional_code.x) {
        sub_12_13();
    }
    reg_tmp14.xy = (mul_s(reg_tmp14, reg_tmp2)).xy;
    reg_tmp13.x = (mul_s(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    reg_tmp13 = mul_s(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xy);
    if (conditional_code.y) {
        sub_20_21();
    }
    if (conditional_code.x) {
        sub_22_24();
    }
    reg_tmp14.xy = (uniforms.f[5].yyyy + -reg_tmp14.xyyy).xy;
    reg_tmp13.x = (mul_s(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    reg_tmp13 = mul_s(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xy);
    if (conditional_code.y) {
        sub_31_32();
    }
    if (conditional_code.x) {
        sub_33_34();
    }
    reg_tmp13.x = (mul_s(uniforms.f[5].zzzz, reg_tmp13.xxxx)).x;
    reg_tmp13 = mul_s(uniforms.f[5].zyzy, reg_tmp13.xxxx);
    reg_tmp13.zw = (floor(reg_tmp13)).zw;
    reg_tmp13.xy = (reg_tmp13.xyyy + -reg_tmp13.zwww).xy;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xy);
    if (conditional_code.y) {
        sub_40_41();
    }
    if (conditional_code.x) {
        sub_42_43();
    }
    reg_tmp13.xy = (mul_s(uniforms.f[5].zzzz, reg_tmp0.xxxx)).xy;
    reg_tmp13.y = (floor(reg_tmp13)).y;
    reg_tmp13.x = (reg_tmp13.xxxx + -reg_tmp13.yyyy).x;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xx);
    if (conditional_code.x) {
        sub_48_49();
    }
    reg_tmp12.y = (uniforms.f[5].yyyy + -reg_tmp12.yyyy).y;
    return false;
}
bool sub_12_13() {
    reg_tmp14.xy = (reg_tmp14.yxxx).xy;
    return false;
}
bool sub_20_21() {
    reg_tmp12.x = (mul_s(reg_tmp12.xxxx, reg_tmp14.xxxx)).x;
    return false;
}
bool sub_22_24() {
    reg_tmp12.y = (fma_s(reg_tmp12.yyyy, reg_tmp14.yyyy, uniforms.f[5].yyyy)).y;
    reg_tmp12.y = (reg_tmp12.yyyy + -reg_tmp14.yyyy).y;
    return false;
}
bool sub_31_32() {
    reg_tmp12.x = (reg_tmp12.xxxx + reg_tmp14.xxxx).x;
    return false;
}
bool sub_33_34() {
    reg_tmp12.y = (reg_tmp12.yyyy + -reg_tmp14.yyyy).y;
    return false;
}
bool sub_40_41() {
    reg_tmp12.x = (uniforms.f[5].yyyy + -reg_tmp12.xxxx).x;
    return false;
}
bool sub_42_43() {
    reg_tmp12.y = (uniforms.f[5].yyyy + -reg_tmp12.yyyy).y;
    return false;
}
bool sub_48_49() {
    reg_tmp12.xy = (uniforms.f[5].yyyy + -reg_tmp12.yxxx).xy;
    return false;
}
bool sub_52_69() {
    reg_tmp13.x = (floor(reg_tmp0.xxxx)).x;
    reg_tmp13.x = (reg_tmp0.xxxx + -reg_tmp13).x;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xx);
    reg_tmp13 = uniforms.f[32 + address_registers.y].wzyx;
    if (conditional_code.x) {
        sub_57_60();
    } else {
        sub_60_68();
    }
    reg_tmp11.z = (uniforms.f[5].yyyy + reg_tmp11.zzzz).z;
    return false;
}
bool sub_57_60() {
    reg_tmp11.xy = (fma_s(reg_tmp12.xyyy, reg_tmp13.xyyy, reg_tmp13.zwww)).xy;
    reg_tmp11.xy = (mul_s(reg_tmp11.xyyy, reg_tmp14.zwww)).xy;
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_60_68() {
    conditional_code = notEqual(uniforms.f[5].xx, vs_in_reg0.zw);
    if (!conditional_code.x) {
        sub_62_63();
    } else {
        sub_63_64();
    }
    if (!conditional_code.y) {
        sub_65_66();
    } else {
        sub_66_67();
    }
    return false;
}
bool sub_62_63() {
    reg_tmp11.x = (reg_tmp13.xxxx).x;
    return false;
}
bool sub_63_64() {
    reg_tmp11.x = (reg_tmp13.zzzz).x;
    return false;
}
bool sub_65_66() {
    reg_tmp11.y = (reg_tmp13.yyyy).y;
    return false;
}
bool sub_66_67() {
    reg_tmp11.y = (reg_tmp13.wwww).y;
    return false;
}
bool sub_69_96() {
    if (!conditional_code.x) {
        sub_70_93();
    } else {
        sub_93_95();
    }
    reg_tmp11.z = (uniforms.f[5].zzzz + reg_tmp11.zzzz).z;
    return false;
}
bool sub_70_93() {
    reg_tmp13.x = (floor(reg_tmp0.xxxx)).x;
    reg_tmp13.x = (reg_tmp0.xxxx + -reg_tmp13).x;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp13.xx);
    if (conditional_code.x) {
        sub_74_79();
    } else {
        sub_79_92();
    }
    return false;
}
bool sub_74_79() {
    reg_tmp12.zw = (uniforms.f[5].xxxy).zw;
    reg_tmp11.x = dot_s(uniforms.f[32 + address_registers.y].wzyx, reg_tmp12);
    reg_tmp11.y = dot_s(uniforms.f[33 + address_registers.y].wzyx, reg_tmp12);
    reg_tmp11.xy = (mul_s(reg_tmp11.xyyy, reg_tmp14.zwww)).xy;
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_79_92() {
    reg_tmp14 = uniforms.f[32 + address_registers.y].wzyx;
    reg_tmp13 = uniforms.f[33 + address_registers.y].wzyx;
    conditional_code = notEqual(uniforms.f[5].xx, vs_in_reg0.zw);
    if (!conditional_code.y) {
        sub_83_87();
    } else {
        sub_87_91();
    }
    return false;
}
bool sub_83_87() {
    if (!conditional_code.x) {
        sub_84_85();
    } else {
        sub_85_86();
    }
    return false;
}
bool sub_84_85() {
    reg_tmp11.xy = (reg_tmp14.xyyy).xy;
    return false;
}
bool sub_85_86() {
    reg_tmp11.xy = (reg_tmp13.zwww).xy;
    return false;
}
bool sub_87_91() {
    if (!conditional_code.x) {
        sub_88_89();
    } else {
        sub_89_90();
    }
    return false;
}
bool sub_88_89() {
    reg_tmp11.xy = (reg_tmp13.xyyy).xy;
    return false;
}
bool sub_89_90() {
    reg_tmp11.xy = (reg_tmp14.zwww).xy;
    return false;
}
bool sub_93_95() {
    reg_tmp11.x = dot_s(uniforms.f[32 + address_registers.y].wzyx, reg_tmp1);
    reg_tmp11.y = dot_s(uniforms.f[33 + address_registers.y].wzyx, reg_tmp1);
    return false;
}
bool sub_97_109() {
    uint jmp_to = 97u;
    while (true) {
        switch (jmp_to) {
        case 97u: {
            reg_tmp3.x = dot_s(uniforms.f[32 + address_registers.x].wzyx, reg_tmp1);
            reg_tmp3.y = dot_s(uniforms.f[33 + address_registers.x].wzyx, reg_tmp1);
            reg_tmp3.z = dot_s(uniforms.f[34 + address_registers.x].wzyx, reg_tmp1);
            reg_tmp3.w = (reg_tmp1.wwww).w;
            reg_tmp11 = uniforms.f[4].wzyx;
            reg_tmp11.z = (-uniforms.f[34 + address_registers.x].xxxx + reg_tmp11.zzzz).z;
            conditional_code.x = uniforms.f[5].xxxx.x != reg_tmp11.xzzz.x;
            conditional_code.y = uniforms.f[5].xxxx.y < reg_tmp11.xzzz.y;
            if (any(not(conditional_code))) {
                { jmp_to = 108u; break; }
            }
            reg_tmp11.z = rcp_s(reg_tmp11.z);
            reg_tmp3.x = (reg_tmp3.xxxx + reg_tmp11.xxxx).x;
            reg_tmp3.x = (fma_s(-reg_tmp11.yyyy, reg_tmp11.zzzz, reg_tmp3.xxxx)).x;
        }
        case 108u: {
        }
        default: return false;
        }
    }
    return false;
}
bool sub_109_4096() {
    uint jmp_to = 109u;
    while (true) {
        switch (jmp_to) {
        case 109u: {
            address_registers.x = (ivec2(vs_in_reg0.xx)).x;
            reg_tmp0 = uniforms.f[9 + address_registers.x].wzyx;
            reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
            reg_tmp1.zw = (uniforms.f[5].xyxy).zw;
            address_registers.xy = ivec2(reg_tmp0.xy);
            reg_tmp2 = uniforms.f[32 + address_registers.y].wzyx;
            if (uniforms.b[0]) {
                { jmp_to = 191u; break; }
            }
            reg_tmp4 = uniforms.f[31 + address_registers.x].wzyx;
            reg_tmp1.xy = (fma_s(reg_tmp1.xyyy, reg_tmp2.xyyy, reg_tmp2.zwww)).xy;
            {
                sub_97_109();
            }
            vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp3);
            vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp3);
            vs_out_attr0.z = dot_s(uniforms.f[2].wzyx, reg_tmp3);
            vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp3);
            conditional_code = greaterThanEqual(uniforms.f[5].yy, reg_tmp0.ww);
            if (all(conditional_code)) {
                sub_125_127();
            } else {
                sub_127_149();
            }
            reg_tmp11.z = (reg_tmp0.zzzz).z;
            reg_tmp9 = mul_s(uniforms.f[5].zyzy, reg_tmp0.zzzz);
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            reg_tmp9 = mul_s(uniforms.f[5].zzzz, reg_tmp9);
            reg_tmp14 = uniforms.f[6].wzyx;
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            {
                sub_0_97();
            }
            if (uniforms.b[1]) {
                sub_158_160();
            }
            if (uniforms.b[2]) {
                sub_161_162();
            }
            vs_out_attr2 = reg_tmp11.xyyy;
            reg_tmp9 = mul_s(uniforms.f[5].zyzy, reg_tmp9.xxxx);
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            reg_tmp9 = mul_s(uniforms.f[5].zzzz, reg_tmp9);
            reg_tmp14 = uniforms.f[7].wzyx;
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            {
                sub_0_97();
            }
            if (uniforms.b[3]) {
                sub_171_173();
            }
            if (uniforms.b[4]) {
                sub_174_175();
            }
            vs_out_attr3 = reg_tmp11.xyyy;
            reg_tmp9 = mul_s(uniforms.f[5].zyzy, reg_tmp9.xxxx);
            reg_tmp9.xy = (floor(reg_tmp9)).xy;
            reg_tmp9.xy = (reg_tmp9.zwww + -reg_tmp9.xyyy).xy;
            reg_tmp9 = mul_s(uniforms.f[5].zzzz, reg_tmp9);
            reg_tmp14 = uniforms.f[8].wzyx;
            reg_tmp11.xy = (reg_tmp9.xyyy).xy;
            {
                sub_0_97();
            }
            if (uniforms.b[5]) {
                sub_184_186();
            }
            if (uniforms.b[6]) {
                sub_187_188();
            }
            vs_out_attr4 = reg_tmp11.xyyy;
            return true;
        }
        case 191u: {
            reg_tmp2.w = (reg_tmp2.wwww + reg_tmp2.yyyy).w;
            reg_tmp1.y = (-uniforms.f[5].yyyy + -reg_tmp1.yyyy).y;
            reg_tmp13.xy = (mul_s(uniforms.f[36 + address_registers.x].wzzz, reg_tmp2.xyyy)).xy;
            reg_tmp11.x = (mul_s(uniforms.f[35 + address_registers.x].wwww, -reg_tmp1.yyyy)).x;
            reg_tmp1.xy = (mul_s(reg_tmp1.xyyy, reg_tmp13.xyyy)).xy;
            reg_tmp1.x = (reg_tmp1.xxxx + reg_tmp11.xxxx).x;
            if (uniforms.b[1]) {
                sub_198_207();
            }
            reg_tmp1.xy = (reg_tmp1.xyyy + reg_tmp2.zwww).xy;
            reg_tmp1.xy = (uniforms.f[36 + address_registers.x].yxxx + reg_tmp1.xyyy).xy;
            {
                sub_97_109();
            }
            vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp3);
            vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp3);
            vs_out_attr0.z = dot_s(uniforms.f[2].wzyx, reg_tmp3);
            vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp3);
            if (uniforms.b[2]) {
                sub_215_223();
            } else {
                sub_223_227();
            }
            reg_tmp8 = reg_tmp8 + -reg_tmp7;
            vs_out_attr1 = fma_s(reg_tmp8, reg_tmp11.yyyy, reg_tmp7);
            reg_tmp9.xy = (mul_s(uniforms.f[32 + address_registers.x].yxxx, reg_tmp11)).xy;
            reg_tmp11.zw = (vec4(lessThan(reg_tmp11, uniforms.f[5].yyyy))).zw;
            reg_tmp9.xy = (fma_s(reg_tmp11.zwww, uniforms.f[32 + address_registers.x].wzzz, reg_tmp9.xyyy)).xy;
            reg_tmp9.y = (uniforms.f[5].yyyy + -reg_tmp9.yyyy).y;
            vs_out_attr2 = reg_tmp9;
            vs_out_attr3 = reg_tmp9;
            vs_out_attr4 = reg_tmp9;
            return true;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_125_127() {
    vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
    vs_out_attr1.w = (reg_tmp0.wwww).w;
    return false;
}
bool sub_127_149() {
    address_registers.y = (ivec2(reg_tmp0.ww)).y;
    reg_tmp7 = uniforms.f[32 + address_registers.y].wzyx;
    reg_tmp8 = uniforms.f[33 + address_registers.y].wzyx;
    reg_tmp9 = uniforms.f[34 + address_registers.y].wzyx;
    reg_tmp10 = uniforms.f[35 + address_registers.y].wzyx;
    reg_tmp11.xy = (vs_in_reg0.zwww).xy;
    reg_tmp14.x = (floor(reg_tmp0.yyyy)).x;
    reg_tmp14.x = (reg_tmp0.yyyy + -reg_tmp14.xxxx).x;
    conditional_code = lessThanEqual(uniforms.f[5].ww, reg_tmp14.xx);
    if (conditional_code.x) {
        sub_137_141();
    }
    reg_tmp11.xy = (abs(reg_tmp11.xyyy)).xy;
    reg_tmp8 = reg_tmp8 + -reg_tmp7;
    reg_tmp8 = fma_s(reg_tmp8, reg_tmp11.xxxx, reg_tmp7);
    reg_tmp10 = reg_tmp10 + -reg_tmp9;
    reg_tmp10 = fma_s(reg_tmp10, reg_tmp11.xxxx, reg_tmp9);
    reg_tmp10 = reg_tmp10 + -reg_tmp8;
    reg_tmp10 = fma_s(reg_tmp10, reg_tmp11.yyyy, reg_tmp8);
    vs_out_attr1 = reg_tmp10;
    return false;
}
bool sub_137_141() {
    reg_tmp11.z = rcp_s(reg_tmp4.x);
    reg_tmp11.w = rcp_s(reg_tmp4.y);
    reg_tmp11.xy = (reg_tmp1.xyyy + -reg_tmp4.zwww).xy;
    reg_tmp11.xy = (mul_s(reg_tmp11.xyyy, reg_tmp11.zwww)).xy;
    return false;
}
bool sub_158_160() {
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_161_162() {
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_171_173() {
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_174_175() {
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_184_186() {
    reg_tmp11.xy = (reg_tmp11.yxxx).xy;
    reg_tmp11.y = (uniforms.f[5].yyyy + -reg_tmp11.yyyy).y;
    return false;
}
bool sub_187_188() {
    reg_tmp11.xy = (uniforms.f[5].yyyy + -reg_tmp11.yxxx).xy;
    return false;
}
bool sub_198_207() {
    reg_tmp13.xzw = (mul_s(uniforms.f[5].wxxx, reg_tmp13.xxxx)).xzw;
    reg_tmp13.y = (mul_s(uniforms.f[34 + address_registers.y].yyyy, reg_tmp13.yyyy)).y;
    reg_tmp11 = fma_s(reg_tmp1, uniforms.f[5].yyxx, -reg_tmp13);
    reg_tmp14 = uniforms.f[33 + address_registers.y].wzyx;
    reg_tmp1.x = dot_3(reg_tmp11.xyz, reg_tmp14.xyy);
    reg_tmp1.y = dot_3(reg_tmp11.xyz, reg_tmp14.zww);
    reg_tmp14 = uniforms.f[34 + address_registers.y].wzyx;
    reg_tmp1.z = dot_s(vec4(reg_tmp11.xyz, 1.0), reg_tmp14);
    reg_tmp1.xy = (reg_tmp1.xyyy + reg_tmp13.xyyy).xy;
    return false;
}
bool sub_215_223() {
    reg_tmp11 = abs(vs_in_reg0.zwzw);
    reg_tmp14.w = (floor(reg_tmp0.zzzz)).w;
    reg_tmp14.w = (reg_tmp0.zzzz + -reg_tmp14).w;
    address_registers.xy = ivec2(reg_tmp0.zx);
    reg_tmp14.w = (mul_s(uniforms.f[5].zzzz, reg_tmp14.wwww)).w;
    reg_tmp14.xyz = (uniforms.f[5].yyyy).xyz;
    reg_tmp7 = mul_s(uniforms.f[37 + address_registers.y].wzyx, reg_tmp14);
    reg_tmp8 = mul_s(uniforms.f[38 + address_registers.y].wzyx, reg_tmp14);
    return false;
}
bool sub_223_227() {
    reg_tmp11 = abs(vs_in_reg0.zwzw);
    address_registers.xy = ivec2(reg_tmp0.zw);
    reg_tmp7 = uniforms.f[32 + address_registers.y].wzyx;
    reg_tmp8 = uniforms.f[33 + address_registers.y].wzyx;
    return false;
}
// reference: 26B611F73A6918E6, D9C174A4A54356AC
// shader: 8DD9, 42937135801BAA7E

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
// reference: 5DAD5699F59B3586, 42937135801BAA7E
// shader: 8B30, CC1023E585D9119C

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
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (const_color[3].rgb);
float alpha_output_3 = (const_color[3].a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (combiner_buffer.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (combiner_buffer.a) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1F78A607A93F09BE, CC1023E585D9119C
// program: D9C174A4A54356AC, 42937135801BAA7E, CC1023E585D9119C
// reference: 68E2EEB7B99C7E94, D9C174A4A54356AC
// shader: 8B31, FE89E80F22BCBB5D

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

bool sub_27_4096();

bool exec_shader() {
    sub_27_4096();
    return true;
}

bool sub_27_4096() {
    vs_out_attr0.xy = (vs_in_reg0.xyyy).xy;
    vs_out_attr0.z = (uniforms.f[93].xxxx).z;
    vs_out_attr0.w = (uniforms.f[93].yyyy).w;
    vs_out_attr1 = vs_in_reg1.xyxy;
    reg_tmp0 = uniforms.f[8];
    reg_tmp1 = uniforms.f[9];
    vs_out_attr2.xy = (fma_s(vs_in_reg1.xyxy, reg_tmp0.xyxy, reg_tmp0.zwzw)).xy;
    vs_out_attr2.zw = (fma_s(vs_in_reg1.xyxy, reg_tmp1.xyxy, reg_tmp1.zwzw)).zw;
    return true;
}
// reference: 7361F5A13461E066, FE89E80F22BCBB5D
// shader: 8DD9, 23ACCF022546CCEE

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

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = min(abs(vtx_color), vec4(1.0));

    texcoord0 = vec2(vtx.attributes[1].x, vtx.attributes[1].y);
    texcoord1 = vec2(vtx.attributes[2].x, vtx.attributes[2].y);

    texcoord0_w = 0.0;
    view = vec3(0.0, 0.0, 0.0);
    texcoord2 = vec2(vtx.attributes[2].z, vtx.attributes[2].w);

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
// reference: 052B39C2C3B53B3E, 23ACCF022546CCEE
// shader: 8B30, FC9E10BE42F07103

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
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF94AFA08A2, FC9E10BE42F07103
// program: FE89E80F22BCBB5D, 23ACCF022546CCEE, FC9E10BE42F07103
// reference: 3D350AE1B7948614, FE89E80F22BCBB5D
// shader: 8B30, E452B501B9CF9987

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
if (!(gl_FragCoord.x >= float(scissor_x1) && gl_FragCoord.y >= float(scissor_y1) && gl_FragCoord.x < float(scissor_x2) && gl_FragCoord.y < float(scissor_y2))) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D4F4BEF9E876F49A, E452B501B9CF9987
// program: FE89E80F22BCBB5D, 23ACCF022546CCEE, E452B501B9CF9987
