// shader: 8B31, D48CB49BFF2FE17F

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_30();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_31();
bool sub_32();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_36();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_30() {
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    if (uniforms.b[7]) {
        sub_24();
    }
    if (uniforms.b[11]) {
        sub_25();
    } else {
        sub_32();
    }
    if (uniforms.b[7]) {
        sub_33();
    } else {
        sub_36();
    }
    vs_out_attr6 = reg_tmp11;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp10 = uniforms.f[17].xxyx;
    return false;
}
bool sub_25() {
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    if (uniforms.b[6]) {
        sub_26();
    }
    if (uniforms.b[5]) {
        sub_27();
    }
    return false;
}
bool sub_26() {
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_27() {
    if (uniforms.b[7]) {
        sub_28();
    }
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop162 = 0u; loop162 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop162) {
        sub_29();
    }
    if (uniforms.b[7]) {
        sub_31();
    }
    return false;
}
bool sub_28() {
    reg_tmp10.x = (max(reg_tmp11.xxxx, reg_tmp11.yyyy)).x;
    reg_tmp10.x = (max(reg_tmp10.xxxx, reg_tmp11.zzzz)).x;
    return false;
}
bool sub_29() {
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    {
        sub_30();
    }
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_31() {
    reg_tmp10.y = (max(reg_tmp11.xxxx, reg_tmp11.yyyy)).y;
    reg_tmp10.y = (max(reg_tmp10.yyyy, reg_tmp11.zzzz)).y;
    reg_tmp10.z = (reg_tmp10.yyyy + -reg_tmp10.xxxx).z;
    reg_tmp10.z = (max(uniforms.f[17].xxxx, reg_tmp10.zzzz)).z;
    reg_tmp10.z = (min(uniforms.f[17].yyyy, reg_tmp10.zzzz)).z;
    reg_tmp10.z = (uniforms.f[17].yyyy + -reg_tmp10.zzzz).z;
    return false;
}
bool sub_32() {
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
bool sub_33() {
    reg_tmp13.w = (uniforms.f[95].wwww).w;
    conditional_code = lessThan(reg_tmp8.zz, reg_tmp13.ww);
    if (conditional_code.x) {
        sub_34();
    } else {
        sub_35();
    }
    return false;
}
bool sub_34() {
    reg_tmp11.w = (reg_tmp10.zzzz).w;
    return false;
}
bool sub_35() {
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_36() {
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    return false;
}
// reference: 12EF0F1092054503, D48CB49BFF2FE17F
// shader: 8DD9, 277EE8E572ED8EC1

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
// reference: B7A1AA8AEE71524D, 277EE8E572ED8EC1
// shader: 8B30, C8F2C0F24BE75B0C
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8DC4F5161F04A720, C8F2C0F24BE75B0C
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, C8F2C0F24BE75B0C
// reference: 5CBBF05011F02371, D48CB49BFF2FE17F
// shader: 8B31, 5CAA7F1C6EA2B781

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_28();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_29();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_28() {
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    if (uniforms.b[11]) {
        sub_24();
    } else {
        sub_29();
    }
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    vs_out_attr6 = reg_tmp11;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    if (uniforms.b[6]) {
        sub_25();
    }
    if (uniforms.b[5]) {
        sub_26();
    }
    return false;
}
bool sub_25() {
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_26() {
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop157 = 0u; loop157 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop157) {
        sub_27();
    }
    return false;
}
bool sub_27() {
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    {
        sub_28();
    }
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_29() {
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
// reference: 46AC0102322FA69B, 5CAA7F1C6EA2B781
// shader: 8B30, FC8C06E25BADC34E
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97D23F052B69DDB3, FC8C06E25BADC34E
// program: 5CAA7F1C6EA2B781, 277EE8E572ED8EC1, FC8C06E25BADC34E
// shader: 8B31, 2AA18A53EC1683ED

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    if (uniforms.b[8]) {
        sub_24();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    vs_out_attr4 = vs_in_reg6;
    vs_out_attr5 = vs_in_reg7;
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    vs_out_attr6 = reg_tmp11;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 430655DB3DF7C784, 2AA18A53EC1683ED
// shader: 8B30, A7C410C37E048115
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1.0) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7297BC267C38FBF, A7C410C37E048115
// program: 2AA18A53EC1683ED, 277EE8E572ED8EC1, A7C410C37E048115
// shader: 8B30, 8EAB6CCD16EB3E10
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
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C94679FBEC4EA26B, 8EAB6CCD16EB3E10
// program: 2AA18A53EC1683ED, 277EE8E572ED8EC1, 8EAB6CCD16EB3E10
// shader: 8B31, 219C98951262F510

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    if (uniforms.b[8]) {
        sub_24();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    vs_out_attr4 = vs_in_reg6;
    vs_out_attr5 = vs_in_reg7;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: F5CC0005F5588DE2, 219C98951262F510
// shader: 8B30, 7BFBB49C7BA8FDBB
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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((vec3(1.0) - primary_fragment_color.aaa) * (vec3(1.0) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 97A606328584E266, 7BFBB49C7BA8FDBB
// program: 219C98951262F510, 277EE8E572ED8EC1, 7BFBB49C7BA8FDBB
// shader: 8B31, 42D649B2E555DA56

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_25();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_20() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_21();
    } else {
        sub_22();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_21() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_22() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_23();
    } else {
        sub_24();
    }
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_24() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    if (uniforms.b[10]) {
        sub_15();
    }
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_16();
    } else {
        sub_19();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_20();
    }
    if (uniforms.b[8]) {
        sub_25();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    vs_out_attr4 = vs_in_reg6;
    vs_out_attr5 = vs_in_reg7;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp12.z = (-uniforms.f[80].wwww + reg_tmp12.zzzz).z;
    return false;
}
bool sub_16() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_17();
    } else {
        sub_18();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_17() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_18() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_19() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_25() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: BC4551D76C7198CF, 42D649B2E555DA56
// shader: 8B30, 4CF1B84491FE3775
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
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.aaa) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].ggg) + (last_tex_env_out.rrr) * (vec3(1.0) - (const_color[2].ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].g) + (last_tex_env_out.r) * (1.0 - (const_color[2].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (const_color[3].aaa);
float alpha_output_3 = byteround(clamp((last_tex_env_out.r) * (combiner_buffer.b) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp(min((rounded_primary_color.a) + (const_color[4].a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(1.0)) * (combiner_buffer.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7DE2EB867678105E, 4CF1B84491FE3775
// program: 42D649B2E555DA56, 277EE8E572ED8EC1, 4CF1B84491FE3775
// shader: 8B31, 508D3D16E6E14912

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    vs_out_attr3 = vs_in_reg5;
    vs_out_attr4 = vs_in_reg6;
    vs_out_attr5 = vs_in_reg7;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
// reference: 73B7BA5DD35023CA, 508D3D16E6E14912
// shader: 8B30, 6BB790355B32C198
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
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5D3A7CB0744F75C5, 6BB790355B32C198
// program: 508D3D16E6E14912, 277EE8E572ED8EC1, 6BB790355B32C198
// shader: 8B31, A6208E117BF75FB2

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_24();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_25();
bool sub_26();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_20() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_21();
    } else {
        sub_22();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_21() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_22() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_23();
    } else {
        sub_24();
    }
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_24() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    if (uniforms.b[10]) {
        sub_15();
    }
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_16();
    } else {
        sub_19();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_20();
    }
    if (uniforms.b[8]) {
        sub_25();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_20();
    }
    if (uniforms.b[9]) {
        sub_26();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_20();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp12.z = (-uniforms.f[80].wwww + reg_tmp12.zzzz).z;
    return false;
}
bool sub_16() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_17();
    } else {
        sub_18();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_17() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_18() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_19() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_25() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_26() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 4645A55148FA3DA7, A6208E117BF75FB2
// shader: 8B30, C57AE00DD924292B
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1.0) - (const_color[3].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E3C1FE180A99180E, C57AE00DD924292B
// program: A6208E117BF75FB2, 277EE8E572ED8EC1, C57AE00DD924292B
// shader: 8B31, 366C7259709C8208

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp0 = vs_in_reg0;
    reg_tmp0.zw = (uniforms.f[93].xyyy).zw;
    reg_tmp1.x = dot_s(uniforms.f[25], reg_tmp0);
    reg_tmp1.y = dot_s(uniforms.f[26], reg_tmp0);
    reg_tmp2.y = (uniforms.f[93].yyyy + reg_tmp1.yyyy).y;
    reg_tmp2.y = (mul_s(uniforms.f[94].xxxx, reg_tmp2.yyyy)).y;
    reg_tmp2.y = (uniforms.f[93].yyyy + -reg_tmp2.yyyy).y;
    reg_tmp2.y = (mul_s(uniforms.f[93].zzzz, reg_tmp2.yyyy)).y;
    reg_tmp1.y = (-uniforms.f[93].yyyy + reg_tmp2.yyyy).y;
    vs_out_attr0.xy = (reg_tmp1.yxxx).xy;
    vs_out_attr0.z = (uniforms.f[93].xxxx).z;
    vs_out_attr0.w = (uniforms.f[93].yyyy).w;
    reg_tmp0 = uniforms.f[93].xxyx;
    reg_tmp0.xy = (-uniforms.f[17].xyyy + vs_in_reg2.xyyy).xy;
    reg_tmp3.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp3.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (uniforms.f[17].xyyy + reg_tmp3.xyyy).xy;
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    vs_out_attr2 = reg_tmp0;
    reg_tmp0 = uniforms.f[93].xxyx;
    reg_tmp0.xy = (-uniforms.f[18].xyyy + vs_in_reg2.xyyy).xy;
    reg_tmp3.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp3.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (uniforms.f[18].xyyy + reg_tmp3.xyyy).xy;
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    vs_out_attr3 = reg_tmp0;
    reg_tmp0 = uniforms.f[93].xxxx;
    reg_tmp0 = vs_in_reg2;
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    vs_out_attr4 = reg_tmp0;
    vs_out_attr1 = vs_in_reg1;
    return true;
}
// reference: B6841DC6574D5C81, 366C7259709C8208
// shader: 8DD9, 1C4CBC8096EA16CD

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
// reference: 5DAD5699F59B3586, 1C4CBC8096EA16CD
// shader: 8B30, 5723A2ED01752525
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
float alpha_output_0 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1.0)) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1.0) - (const_color[3].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8881965D9537D39A, 5723A2ED01752525
// program: 366C7259709C8208, 1C4CBC8096EA16CD, 5723A2ED01752525
// shader: 8B31, 0512FCDBF144BBEE

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_0();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_0() {
    reg_tmp0.xy = (vs_in_reg0.xyyy).xy;
    reg_tmp0.zw = (uniforms.f[93].xyyy).zw;
    reg_tmp1.x = dot_s(uniforms.f[25], reg_tmp0);
    reg_tmp1.y = dot_s(uniforms.f[26], reg_tmp0);
    reg_tmp2.y = (uniforms.f[93].yyyy + reg_tmp1.yyyy).y;
    reg_tmp2.y = (mul_s(uniforms.f[94].xxxx, reg_tmp2.yyyy)).y;
    reg_tmp2.y = (uniforms.f[93].yyyy + -reg_tmp2.yyyy).y;
    reg_tmp2.y = (mul_s(uniforms.f[93].zzzz, reg_tmp2.yyyy)).y;
    reg_tmp1.y = (-uniforms.f[93].yyyy + reg_tmp2.yyyy).y;
    reg_tmp2.x = (uniforms.f[27].wwww).x;
    reg_tmp2.x = (mul_s(uniforms.f[29].xxxx, reg_tmp2.xxxx)).x;
    reg_tmp1.y = (reg_tmp1.yyyy + reg_tmp2.xxxx).y;
    vs_out_attr0.xy = (-reg_tmp1.yxxx).xy;
    vs_out_attr0.z = (uniforms.f[93].xxxx).z;
    vs_out_attr0.w = (uniforms.f[93].yyyy).w;
    reg_tmp0 = vs_in_reg2;
    reg_tmp0.y = (uniforms.f[93].yyyy + -reg_tmp0.yyyy).y;
    vs_out_attr2 = reg_tmp0;
    vs_out_attr1 = vs_in_reg1;
    return true;
}
// reference: 131E5DCD180ED306, 0512FCDBF144BBEE
// shader: 8DD9, 219384019281D7FD

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
// reference: 7B07DA3E334A19B0, 219384019281D7FD
// shader: 8B30, FACB7D1C9D9B5DCA
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
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[4].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5880AA7C2968D6EB, FACB7D1C9D9B5DCA
// program: 0512FCDBF144BBEE, 219384019281D7FD, FACB7D1C9D9B5DCA
// reference: 5D4AA28D9BFBB574, 0512FCDBF144BBEE
// shader: 8B30, 4E960E90A6059085
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
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[4].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 67C182062968D6EB, 4E960E90A6059085
// program: 0512FCDBF144BBEE, 219384019281D7FD, 4E960E90A6059085
// shader: 8B30, B9C1C0E161537F1D
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E8F96D340291B4F7, B9C1C0E161537F1D
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, B9C1C0E161537F1D
// shader: 8B30, DAE87710AE7C96E3
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F2EFA72736FCCE64, DAE87710AE7C96E3
// program: 5CAA7F1C6EA2B781, 277EE8E572ED8EC1, DAE87710AE7C96E3
// reference: 0F43A0EB6D0640EF, 4E960E90A6059085
// shader: 8B31, 4EC243B85840F9EC

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_28();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_29();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_28() {
    reg_tmp0.xyz = (reg_tmp2.xyzz + -reg_tmp8.xyzz).xyz;
    reg_tmp0.w = dot_3(reg_tmp0.xyz, reg_tmp0.xyz);
    reg_tmp6.xyz = vec3(rsq_s(reg_tmp0.w));
    reg_tmp0.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp6.xxxx)).xyz;
    reg_tmp0.w = rcp_s(reg_tmp6.x);
    reg_tmp1.x = dot_3(reg_tmp4.xyz, reg_tmp0.xyz);
    reg_tmp6.x = (max(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
    reg_tmp1.x = (min(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
    reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
    reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
    reg_tmp6.y = rcp_s(reg_tmp6.z);
    reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
    reg_tmp6.x = (min(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
    reg_tmp6.y = (max(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
    reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
    reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
    reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
    reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0 = reg_tmp3.xyzz;
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    reg_tmp11.xy = (uniforms.f[18].xxxx + reg_tmp1.xyyy).xy;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    if (uniforms.b[11]) {
        sub_24();
    } else {
        sub_29();
    }
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    vs_out_attr6 = reg_tmp11;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp11.xyz = (uniforms.f[17].xxxy).xyz;
    if (uniforms.b[6]) {
        sub_25();
    }
    if (uniforms.b[5]) {
        sub_26();
    }
    return false;
}
bool sub_25() {
    reg_tmp13.x = (max(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    reg_tmp13.x = (max(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
    reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_26() {
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop159 = 0u; loop159 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop159) {
        sub_27();
    }
    return false;
}
bool sub_27() {
    reg_tmp14.xyz = (uniforms.f[81 + address_registers.z].xyzz).xyz;
    reg_tmp3.x = (uniforms.f[81 + address_registers.z].wwww).x;
    reg_tmp2.xyz = (uniforms.f[87 + address_registers.z].xyzz).xyz;
    reg_tmp3.y = (uniforms.f[87 + address_registers.z].wwww).y;
    {
        sub_28();
    }
    reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
    reg_tmp13 = max(uniforms.f[17].xxxy, reg_tmp11.xyzz);
    reg_tmp11.xyz = (min(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
    return false;
}
bool sub_29() {
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    return false;
}
// reference: 715F81062EC082E7, 4EC243B85840F9EC
// shader: 8B30, 729B60B846062D54
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (primary_fragment_color.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1.0)) * (texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07D6D3745C52E1F8, 729B60B846062D54
// program: 4EC243B85840F9EC, 277EE8E572ED8EC1, 729B60B846062D54
// shader: 8B31, E46BE7090D7DD32C

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();
bool sub_25();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    if (uniforms.b[8]) {
        sub_24();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    if (uniforms.b[9]) {
        sub_25();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_25() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 887AE868BEA4B5B5, E46BE7090D7DD32C
// shader: 8B30, 695D8F09D24720D4
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
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (1.0 - texcolor1.r) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E4C8C4D7C016F6E7, 695D8F09D24720D4
// program: E46BE7090D7DD32C, 277EE8E572ED8EC1, 695D8F09D24720D4
// shader: 8B30, 9D7CC0C6741E5D67
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1.0) - (const_color[3].rrr)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B0406A090A99180E, 9D7CC0C6741E5D67
// program: A6208E117BF75FB2, 277EE8E572ED8EC1, 9D7CC0C6741E5D67
// shader: 8B30, 2CEFE9FB6AA0E7C3
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8A58895C0291B4F7, 2CEFE9FB6AA0E7C3
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, 2CEFE9FB6AA0E7C3
// shader: 8B30, FDED0259375B5EED
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (primary_fragment_color.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1.0)) * (texcolor1.rrr), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1.0)) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_4 = byteround(clamp((vec3(1.0) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1.0) - (texcolor1.ggg)), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6577371C5C52E1F8, FDED0259375B5EED
// program: 4EC243B85840F9EC, 277EE8E572ED8EC1, FDED0259375B5EED
// shader: 8B30, 91EA336412FF3B15
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 904E434F36FCCE64, 91EA336412FF3B15
// program: 5CAA7F1C6EA2B781, 277EE8E572ED8EC1, 91EA336412FF3B15
// shader: 8B30, 488AE02ADE5A7D41
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1.0) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 95889FAA67C38FBF, 488AE02ADE5A7D41
// program: 2AA18A53EC1683ED, 277EE8E572ED8EC1, 488AE02ADE5A7D41
// reference: 866920BFC016F6E7, 695D8F09D24720D4
// reference: D2E18E610A99180E, 9D7CC0C6741E5D67
// reference: 3F9B98D8744F75C5, 6BB790355B32C198
// reference: EA2072359537D39A, 5723A2ED01752525
// reference: 3A214E142968D6EB, FACB7D1C9D9B5DCA
// reference: 0560666E2968D6EB, 4E960E90A6059085
// shader: 8B30, 05BDEDA1A4C5CDAE
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((vec3(1.0) - primary_fragment_color.aaa) * (vec3(1.0) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F507E25A69330965, 05BDEDA1A4C5CDAE
// program: 219C98951262F510, 277EE8E572ED8EC1, 05BDEDA1A4C5CDAE
// shader: 8B31, 96764EFAA32D0C86

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    if (uniforms.b[9]) {
        sub_24();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 887AE86851F0D84C, 96764EFAA32D0C86
// shader: 8B30, D8AE2A297A60549D
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (secondary_fragment_color.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (secondary_fragment_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((texcolor2.rgb) * (vec3(1.0) - secondary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor1.r) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (texcolor0.r) + (last_tex_env_out.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (last_tex_env_out.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (combiner_buffer.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 361595F9CBE52A8F, D8AE2A297A60549D
// program: 96764EFAA32D0C86, 277EE8E572ED8EC1, D8AE2A297A60549D
// reference: B2DA7C80744F75C5, 6BB790355B32C198
// reference: 6761966D9537D39A, 5723A2ED01752525
// reference: B760AA4C2968D6EB, FACB7D1C9D9B5DCA
// reference: 882182362968D6EB, 4E960E90A6059085
// shader: 8B30, D9C7022487F10E7C
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07196D040291B4F7, D9C7022487F10E7C
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, D9C7022487F10E7C
// reference: 5FA06A390A99180E, 9D7CC0C6741E5D67
// shader: 8B30, 966D6F6CB310FD3E
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.a) - (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 1.0, alpha_output_0 * 2.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (texcolor0.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9F5DAB1AD769F006, 966D6F6CB310FD3E
// program: E46BE7090D7DD32C, 277EE8E572ED8EC1, 966D6F6CB310FD3E
// reference: 121C4F42D769F006, 966D6F6CB310FD3E
// shader: 8B31, 409EAE6E00CECA59

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    vs_out_attr4 = vs_in_reg6;
    vs_out_attr5 = vs_in_reg7;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
// reference: F5CC0005B14CF29D, 409EAE6E00CECA59
// shader: 8B30, 13D98E90635F5264
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (const_color[2].aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[2].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) * (const_color[2].a) + (last_tex_env_out.a) * (1.0 - (const_color[2].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C0189A31418C1E09, 13D98E90635F5264
// program: 409EAE6E00CECA59, 277EE8E572ED8EC1, 13D98E90635F5264
// shader: 8B31, 66E949C89E763455

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_24();
bool sub_25();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.w = rcp_s(reg_tmp10.w);
    reg_tmp0.xy = (mul_s(reg_tmp10.xyyy, reg_tmp0.wwww)).xy;
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    reg_tmp1.xy = (uniforms.f[18].xxxx + reg_tmp1.yxxx).xy;
    reg_tmp0.xy = (uniforms.f[17].yyyy).xy;
    reg_tmp14.xy = (reg_tmp0.xxxx + -reg_tmp1.xyyy).xy;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    if (uniforms.b[8]) {
        sub_24();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (reg_tmp14.xyyy).xy;
    if (uniforms.b[9]) {
        sub_25();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    vs_out_attr5 = vs_in_reg7;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_24() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_25() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 81EF2B9083F29D92, 66E949C89E763455
// shader: 8B30, 59F083790A3F5B56
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3 * 1.0, alpha_output_3 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a) + (combiner_buffer.a) * (1.0 - (const_color[4].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9207AC2772F87C55, 59F083790A3F5B56
// program: 66E949C89E763455, 277EE8E572ED8EC1, 59F083790A3F5B56
// shader: 8B30, 4F5851C81C09C758
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
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5352E8CE7F181D99, 4F5851C81C09C758
// program: E46BE7090D7DD32C, 277EE8E572ED8EC1, 4F5851C81C09C758
// shader: 8B30, 1C2EDDB40775FB2E
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb) + (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((combiner_buffer.rgb) * (const_color[2].aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (const_color[2].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) * (const_color[2].a) + (last_tex_env_out.a) * (1.0 - (const_color[2].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 93990E20418C1E09, 1C2EDDB40775FB2E
// program: 409EAE6E00CECA59, 277EE8E572ED8EC1, 1C2EDDB40775FB2E
// reference: C62E17283D51D3C7, E46BE7090D7DD32C
// shader: 8B30, D0E90C5AFE68276C
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3C0BC44095028EBA, D0E90C5AFE68276C
// program: E46BE7090D7DD32C, 277EE8E572ED8EC1, D0E90C5AFE68276C
// reference: 1ED8EA78418C1E09, 1C2EDDB40775FB2E
// reference: 4D597E69418C1E09, 13D98E90635F5264
// reference: DE130C967F181D99, 4F5851C81C09C758
// reference: ABE79D93EC4EA26B, 8EAB6CCD16EB3E10
// shader: 8B30, 762C8D7213ABD100
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
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B19979ED8DEEF25B, 762C8D7213ABD100
// program: E46BE7090D7DD32C, 277EE8E572ED8EC1, 762C8D7213ABD100
// reference: 1F430FEE7678105E, 4CF1B84491FE3775
// shader: 8B31, C8C4CDE6D205253F

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_20();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0 = reg_tmp3.xyzz;
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    reg_tmp11.xy = (uniforms.f[18].xxxx + reg_tmp1.xyyy).xy;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    if (uniforms.b[8]) {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    if (uniforms.b[9]) {
        sub_20();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_19() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_20() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 8A590533309E1A24, C8C4CDE6D205253F
// shader: 8B30, 3B636F9C69DE1A13
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (primary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;

vec3 color_output_1 = byteround(clamp((texcolor0.aaa) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((texcolor1.a) + (texcolor1.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1.0)) * (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((texcolor0.aaa) * (const_color[3].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2815826E3AF816D5, 3B636F9C69DE1A13
// program: C8C4CDE6D205253F, 277EE8E572ED8EC1, 3B636F9C69DE1A13
// reference: 3CD89DB58DEEF25B, 762C8D7213ABD100
// reference: B14A201895028EBA, D0E90C5AFE68276C
// shader: 8B30, 6F85C8C330668614
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
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (1.0 - const_color[0].r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp(min((const_color[1].a) + (const_color[1].r), 1.0) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 2.0, alpha_output_4 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FBD811446DFBEBE7, 6F85C8C330668614
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, 6F85C8C330668614
// shader: 8B31, C3BCED9942E6DF10

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_20();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0 = reg_tmp3.xyzz;
    reg_tmp1.xy = (mul_s(uniforms.f[18].xxxx, reg_tmp0.xyyy)).xy;
    reg_tmp11.xy = (uniforms.f[18].xxxx + reg_tmp1.xyyy).xy;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    if (uniforms.b[8]) {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    if (uniforms.b[9]) {
        sub_20();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (reg_tmp11.xyyy).xy;
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
    reg_tmp11.w = (uniforms.f[17].xxxx).w;
    vs_out_attr6 = reg_tmp11;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
bool sub_19() {
    reg_tmp2.x = dot_s(uniforms.f[13], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[14], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
bool sub_20() {
    reg_tmp2.x = dot_s(uniforms.f[15], reg_tmp0);
    reg_tmp2.y = dot_s(uniforms.f[16], reg_tmp0);
    reg_tmp0.xy = (reg_tmp2.xyyy).xy;
    return false;
}
// reference: 12D3BD8F02BD00FA, C3BCED9942E6DF10
// shader: 8B30, F1E293470B6B167B
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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (texcolor1.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp(min((texcolor0.r) + (texcolor1.r), 1.0) * (secondary_fragment_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = (last_tex_env_out.aaa);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].aaa) + (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7A38E07A8A2487C, F1E293470B6B167B
// program: C3BCED9942E6DF10, 277EE8E572ED8EC1, F1E293470B6B167B
// shader: 8B30, D94A79CE07D79D17
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1D0FA71736FCCE64, D94A79CE07D79D17
// program: 5CAA7F1C6EA2B781, 277EE8E572ED8EC1, D94A79CE07D79D17
// shader: 8B30, 35825BE1E4F6A406
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
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1.0)) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = (1.0 - const_color[0].r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp(min((const_color[1].a) + (const_color[1].r), 1.0) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1.0) - (rounded_primary_color.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (combiner_buffer.aaa) + (last_tex_env_out.rgb) * (vec3(1.0) - (combiner_buffer.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 2.0, alpha_output_4 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7699F51C6DFBEBE7, 35825BE1E4F6A406
// program: D48CB49BFF2FE17F, 277EE8E572ED8EC1, 35825BE1E4F6A406
// shader: 8B31, 4CCA7B29096FE042

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

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

bool sub_19();
bool sub_20();
bool sub_21();
bool sub_22();
bool sub_23();
bool sub_4();
bool sub_5();
bool sub_2();
bool sub_3();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
bool sub_0();
bool sub_1();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();

bool exec_shader() {
    sub_0();
    return true;
}

bool sub_19() {
    conditional_code = equal(uniforms.f[17].yz, reg_tmp0.xy);
    if (all(not(conditional_code))) {
        sub_20();
    } else {
        sub_21();
    }
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    return false;
}
bool sub_20() {
    reg_tmp0.xy = (vs_in_reg5.xyyy).xy;
    return false;
}
bool sub_21() {
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_22();
    } else {
        sub_23();
    }
    return false;
}
bool sub_22() {
    reg_tmp0.xy = (vs_in_reg6.xyyy).xy;
    return false;
}
bool sub_23() {
    reg_tmp0.xy = (vs_in_reg7.xyyy).xy;
    return false;
}
bool sub_4() {
    address_registers.x = (ivec2(reg_tmp14.xx)).x;
    if (uniforms.b[4]) {
        sub_5();
    }
    reg_tmp0.x = dot_s(uniforms.f[19 + address_registers.x], reg_tmp10);
    reg_tmp0.y = dot_s(uniforms.f[20 + address_registers.x], reg_tmp10);
    reg_tmp0.z = dot_s(uniforms.f[21 + address_registers.x], reg_tmp10);
    reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
    return false;
}
bool sub_5() {
    reg_tmp0.x = dot_3(uniforms.f[19 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.y = dot_3(uniforms.f[20 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp0.z = dot_3(uniforms.f[21 + address_registers.x].xyz, reg_tmp5.xyz);
    reg_tmp6.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp6.xyzz)).xyz;
    return false;
}
bool sub_2() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_3();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (vs_in_reg4.xxxx).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
    reg_tmp14.w = (vs_in_reg4.yyyy).w;
    {
        sub_4();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
    reg_tmp14.w = (vs_in_reg4.zzzz).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_6();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_3() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_6() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_7() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_8();
    }
    reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.xxxx)).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_9();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_8() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_9() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_10() {
    reg_tmp7 = uniforms.f[17].xxxy;
    if (uniforms.b[4]) {
        sub_11();
    }
    reg_tmp14.x = (uniforms.f[17].xxxx).x;
    reg_tmp14.w = (uniforms.f[17].yyyy).w;
    {
        sub_4();
    }
    if (uniforms.b[4]) {
        sub_12();
    }
    reg_tmp10 = reg_tmp7;
    return false;
}
bool sub_11() {
    reg_tmp6 = uniforms.f[17].xxxy;
    return false;
}
bool sub_12() {
    reg_tmp5 = reg_tmp6;
    return false;
}
bool sub_0() {
    reg_tmp10 = uniforms.f[17].xxxy;
    reg_tmp10.xyz = (vs_in_reg0).xyz;
    if (uniforms.b[4]) {
        sub_1();
    }
    if (uniforms.b[3]) {
        sub_2();
    }
    if (uniforms.b[2]) {
        sub_7();
    }
    if (uniforms.b[1]) {
        sub_10();
    }
    reg_tmp8.x = dot_s(uniforms.f[8], reg_tmp10);
    reg_tmp8.y = dot_s(uniforms.f[9], reg_tmp10);
    reg_tmp8.z = dot_s(uniforms.f[10], reg_tmp10);
    reg_tmp8.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_13();
    }
    reg_tmp12.x = dot_s(uniforms.f[4], reg_tmp8);
    reg_tmp12.y = dot_s(uniforms.f[5], reg_tmp8);
    reg_tmp12.z = dot_s(uniforms.f[6], reg_tmp8);
    reg_tmp12.w = (uniforms.f[17].yyyy).w;
    if (uniforms.b[4]) {
        sub_14();
    }
    reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp12);
    reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp12);
    reg_tmp10.w = dot_s(uniforms.f[3], reg_tmp12);
    reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp12);
    vs_out_attr0 = reg_tmp10;
    if (uniforms.b[4]) {
        sub_15();
    } else {
        sub_18();
    }
    vs_out_attr2 = -reg_tmp12;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].xxxx).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr3 = reg_tmp2;
    reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
    reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr4 = reg_tmp2;
    reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
    {
        sub_19();
    }
    reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
    reg_tmp2.y = (uniforms.f[17].yyyy).y;
    reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
    reg_tmp2.x = (reg_tmp0.xxxx).x;
    vs_out_attr5 = reg_tmp2;
    vs_out_attr6 = vs_in_reg8;
    return true;
}
bool sub_1() {
    reg_tmp5.xyz = (vs_in_reg1).xyz;
    return false;
}
bool sub_13() {
    reg_tmp4.x = dot_3(uniforms.f[8].xyz, reg_tmp5.xyz);
    reg_tmp4.y = dot_3(uniforms.f[9].xyz, reg_tmp5.xyz);
    reg_tmp4.z = dot_3(uniforms.f[10].xyz, reg_tmp5.xyz);
    return false;
}
bool sub_14() {
    reg_tmp0.x = dot_3(uniforms.f[4].xyz, reg_tmp4.xyz);
    reg_tmp0.y = dot_3(uniforms.f[5].xyz, reg_tmp4.xyz);
    reg_tmp0.z = dot_3(uniforms.f[6].xyz, reg_tmp4.xyz);
    reg_tmp3.x = dot_s(reg_tmp0.xyzz, reg_tmp0.xyzz);
    reg_tmp3.x = rsq_s(reg_tmp3.x);
    reg_tmp3.xyz = (mul_s(reg_tmp0.xyzz, reg_tmp3.xxxx)).xyz;
    return false;
}
bool sub_15() {
    reg_tmp0 = uniforms.f[17].yyyy + reg_tmp3.zzzz;
    reg_tmp0 = mul_s(uniforms.f[18].xxxx, reg_tmp0);
    conditional_code = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
    reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
    reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
    if (!conditional_code.x) {
        sub_16();
    } else {
        sub_17();
    }
    vs_out_attr1.w = (uniforms.f[17].xxxx).w;
    return false;
}
bool sub_16() {
    vs_out_attr1.z = rcp_s(reg_tmp0.x);
    vs_out_attr1.xy = (mul_s(reg_tmp1, reg_tmp0)).xy;
    return false;
}
bool sub_17() {
    vs_out_attr1.xyz = (uniforms.f[17].yxxx).xyz;
    return false;
}
bool sub_18() {
    vs_out_attr1 = uniforms.f[17].yxxx;
    return false;
}
// reference: 887AE8688613D182, 4CCA7B29096FE042
// shader: 8B30, 0991213C1D186EF5
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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-view - light_src[0].position) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((const_color[1].aaa) - (last_tex_env_out.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3 * 2.0, alpha_output_3 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
float fog_index = depth * 128.0;
int fog_i = int(fog_index);
float fog_f = fract(fog_index);
vec2 fog_lut_entry = texelFetch(texture_buffer_lut_lf, fog_i + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 08D2E79249153AD8, 0991213C1D186EF5
// program: 4CCA7B29096FE042, 277EE8E572ED8EC1, 0991213C1D186EF5
