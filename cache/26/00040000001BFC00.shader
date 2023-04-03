// shader: 8DD9, C8EFC3EA7CDDD5A3

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
layout(location=5) in vec4 vs_out_attr5[];
layout(location=6) in vec4 vs_out_attr6[];
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
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

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
// reference: B7A1AA8AEE71524D, C8EFC3EA7CDDD5A3
// shader: 8B31, F55B22C1EDEDB856

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
reg_tmp6.x = (max_s(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
reg_tmp1.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
reg_tmp6.y = rcp_s(reg_tmp6.z);
reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
reg_tmp6.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
reg_tmp6.y = (max_s(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr3 = reg_tmp2;
reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
sub_19();
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr4 = reg_tmp2;
reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_27() {
if (uniforms.b[7]) {
sub_28();
}
addr_regs.z = int(uniforms.i[0].y);
for (uint i = 0u; i <= uniforms.i[0].x; addr_regs.z += int(uniforms.i[0].z), ++i) {
sub_29();
}
if (uniforms.b[7]) {
sub_31();
}
return false;
}
bool sub_28() {
reg_tmp10.x = (max_s(reg_tmp11.xxxx, reg_tmp11.yyyy)).x;
reg_tmp10.x = (max_s(reg_tmp10.xxxx, reg_tmp11.zzzz)).x;
return false;
}
bool sub_29() {
reg_tmp14.xyz = (uniforms.f[81 + addr_regs.z].xyzz).xyz;
reg_tmp3.x = (uniforms.f[81 + addr_regs.z].wwww).x;
reg_tmp2.xyz = (uniforms.f[87 + addr_regs.z].xyzz).xyz;
reg_tmp3.y = (uniforms.f[87 + addr_regs.z].wwww).y;
sub_30();
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_31() {
reg_tmp10.y = (max_s(reg_tmp11.xxxx, reg_tmp11.yyyy)).y;
reg_tmp10.y = (max_s(reg_tmp10.yyyy, reg_tmp11.zzzz)).y;
reg_tmp10.z = (reg_tmp10.yyyy + -reg_tmp10.xxxx).z;
reg_tmp10.z = (max_s(uniforms.f[17].xxxx, reg_tmp10.zzzz)).z;
reg_tmp10.z = (min_s(uniforms.f[17].yyyy, reg_tmp10.zzzz)).z;
reg_tmp10.z = (uniforms.f[17].yyyy + -reg_tmp10.zzzz).z;
return false;
}
bool sub_32() {
reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
return false;
}
bool sub_33() {
reg_tmp13.w = (uniforms.f[95].wwww).w;
bool_regs = lessThan(reg_tmp8.zz, reg_tmp13.ww);
if (bool_regs.x) {
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
// reference: AA53687511F02371, F55B22C1EDEDB856
// shader: 8B30, 8E949040DA64FABE
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1) - (rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 94FA6DDD603D2C69, 8E949040DA64FABE
// program: F55B22C1EDEDB856, C8EFC3EA7CDDD5A3, 8E949040DA64FABE
// shader: 8B31, 626F97B8DBDF0948

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
reg_tmp6.x = (max_s(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
reg_tmp1.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
reg_tmp6.y = rcp_s(reg_tmp6.z);
reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
reg_tmp6.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
reg_tmp6.y = (max_s(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr3 = reg_tmp2;
reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
sub_19();
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr4 = reg_tmp2;
reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_26() {
addr_regs.z = int(uniforms.i[0].y);
for (uint i = 0u; i <= uniforms.i[0].x; addr_regs.z += int(uniforms.i[0].z), ++i) {
sub_27();
}
return false;
}
bool sub_27() {
reg_tmp14.xyz = (uniforms.f[81 + addr_regs.z].xyzz).xyz;
reg_tmp3.x = (uniforms.f[81 + addr_regs.z].wwww).x;
reg_tmp2.xyz = (uniforms.f[87 + addr_regs.z].xyzz).xyz;
reg_tmp3.y = (uniforms.f[87 + addr_regs.z].wwww).y;
sub_28();
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_29() {
reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
return false;
}
// reference: B0449927322FA69B, 626F97B8DBDF0948
// shader: 8B30, 3277E661AF02295B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2D747E35AB39EBAB, 3277E661AF02295B
// program: 626F97B8DBDF0948, C8EFC3EA7CDDD5A3, 3277E661AF02295B
// shader: 8B31, A52020F0BCE0EE09

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: B5EECDFE3DF7C784, A52020F0BCE0EE09
// shader: 8B30, EA695AABCCC23CB2
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 22BE8F692E229BA9, EA695AABCCC23CB2
// program: A52020F0BCE0EE09, C8EFC3EA7CDDD5A3, EA695AABCCC23CB2
// shader: 8B30, 23DD2C5B53A1A8DF
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.r);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].aaa), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D01BC0F6A66FA886, 23DD2C5B53A1A8DF
// program: A52020F0BCE0EE09, C8EFC3EA7CDDD5A3, 23DD2C5B53A1A8DF
// shader: 8B31, 9871BDE695FC9AF2

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: 03249820F5588DE2, 9871BDE695FC9AF2
// shader: 8B30, 547EB0DA06101512
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.aaa) * (vec3(1) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 01C4610D4BBD7409, 547EB0DA06101512
// program: 9871BDE695FC9AF2, C8EFC3EA7CDDD5A3, 547EB0DA06101512
// shader: 8B31, EECAD833A277ABD2

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_20();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: 4AADC9F26C7198CF, EECAD833A277ABD2
// shader: 8B30, A3A8AFAB07B607AD
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.aaa) * (const_color[1].aaa), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) - (1.0 - const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.aaa) * (const_color[2].ggg) + (last_tex_env_out.rrr) * (vec3(1) - (const_color[2].ggg)), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].g) + (last_tex_env_out.r) * (1.0 - (const_color[2].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2 * 4.0, alpha_output_2 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (const_color[3].aaa);
float alpha_output_3 = byteround(clamp((last_tex_env_out.r) * (combiner_buffer.b) + (last_tex_env_out.a) * (1.0 - (combiner_buffer.b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp(min((rounded_primary_color.a) + (const_color[4].a), 1.0) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp(min((last_tex_env_out.rgb) + (const_color[5].rgb), vec3(1)) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5844162E7F907C99, A3A8AFAB07B607AD
// program: EECAD833A277ABD2, C8EFC3EA7CDDD5A3, A3A8AFAB07B607AD
// shader: 8B31, 42D71AA55F115F52

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: 855F2278D35023CA, 42D71AA55F115F52
// shader: 8B30, 3374FE3C2F1DB00C
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4F16F65A1D6A5DCA, 3374FE3C2F1DB00C
// program: 42D71AA55F115F52, C8EFC3EA7CDDD5A3, 3374FE3C2F1DB00C
// shader: 8B31, C6AD1F9C8D20C2A1

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_20();
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
sub_20();
if (uniforms.b[9]) {
sub_26();
}
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr4 = reg_tmp2;
reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
sub_20();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: B0AD3D7448FA3DA7, C6AD1F9C8D20C2A1
// shader: 8B30, F6CBE578AA44F595
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1) - (const_color[3].rrr)), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2362AED51A540572, F6CBE578AA44F595
// program: C6AD1F9C8D20C2A1, C8EFC3EA7CDDD5A3, F6CBE578AA44F595
// shader: 8DD9, B6B95AFD9466EC70

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
layout(location=3) in vec4 vs_out_attr3[];
layout(location=4) in vec4 vs_out_attr4[];
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
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

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
// reference: 5DAD5699F59B3586, B6B95AFD9466EC70
// shader: 8B31, 05E9ED95B740E0DD

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
// reference: 0E387AA3D4B83AF3, 05E9ED95B740E0DD
// shader: 8B30, F19417D717A57884
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 2.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.rgb), vec3(1)) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa) + (const_color[3].rgb) * (vec3(1) - (const_color[3].aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 732682A282E696EC, F19417D717A57884
// program: 05E9ED95B740E0DD, B6B95AFD9466EC70, F19417D717A57884
// shader: 8DD9, 8F66AB8FEF098E16

layout(triangles) in;
layout(triangle_strip, max_vertices=3) out;

layout(location=1) out vec4 primary_color;
layout(location=2) out vec2 texcoord0;
layout(location=3) out vec2 texcoord1;
layout(location=4) out vec2 texcoord2;
layout(location=5) out float texcoord0_w;
layout(location=6) out vec4 normquat;
layout(location=7) out vec3 view;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

layout(location=0) in vec4 vs_out_attr0[];
layout(location=1) in vec4 vs_out_attr1[];
layout(location=2) in vec4 vs_out_attr2[];
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
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

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
// reference: 7B07DA3E334A19B0, 8F66AB8FEF098E16
// shader: 8B31, 96B1976A0DE96173

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=2) in vec4 vs_in_reg2;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
// reference: 131E5DCD9BFBB574, 96B1976A0DE96173
// shader: 8B30, 718ACC30D6F6098C
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3D77A45DB0EFDE91, 718ACC30D6F6098C
// program: 96B1976A0DE96173, 8F66AB8FEF098E16, 718ACC30D6F6098C
// shader: 8B30, C4329D35E3F32E44
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3D77A45D89538ECD, C4329D35E3F32E44
// program: 96B1976A0DE96173, 8F66AB8FEF098E16, C4329D35E3F32E44
// shader: 8B30, 63881C613B297187
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1) - (rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 94FA6DDDA7FA83F0, 63881C613B297187
// program: F55B22C1EDEDB856, C8EFC3EA7CDDD5A3, 63881C613B297187
// shader: 8B30, B503B299344D8A09
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2D747E356CFE4432, B503B299344D8A09
// program: 626F97B8DBDF0948, C8EFC3EA7CDDD5A3, B503B299344D8A09
// shader: 8B30, 84C68CB65EF0719B
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

float byteround(float x) {
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0 * 4.0, alpha_output_0 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (const_color[1].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((combiner_buffer.rgb) * (const_color[4].rrr) + (last_tex_env_out.rgb) * (vec3(1) - (const_color[4].rrr)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((combiner_buffer.a) * (const_color[4].r) + (last_tex_env_out.a) * (1.0 - (const_color[4].r)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B70CC691BCA7260D, 84C68CB65EF0719B
// program: 96B1976A0DE96173, 8F66AB8FEF098E16, 84C68CB65EF0719B
// shader: 8B31, 461AD8F9213BCBA8

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
reg_tmp6.x = (max_s(uniforms.f[17].xxxy, reg_tmp1.xxxx)).x;
reg_tmp1.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.xxxx)).x;
reg_tmp6.x = (reg_tmp0.wwww + -reg_tmp3.xxxx).x;
reg_tmp6.z = (reg_tmp3.yyyy + -reg_tmp3.xxxx).z;
reg_tmp6.y = rcp_s(reg_tmp6.z);
reg_tmp6.z = (mul_s(reg_tmp6.xxxx, reg_tmp6.yyyy)).z;
reg_tmp6.x = (min_s(uniforms.f[17].yyyy, reg_tmp6.zzzz)).x;
reg_tmp6.y = (max_s(uniforms.f[17].xxxx, reg_tmp6.xxxx)).y;
reg_tmp6.x = (uniforms.f[17].yyyy + -reg_tmp6.yyyy).x;
reg_tmp1.x = (mul_s(reg_tmp1.xxxx, reg_tmp6.xxxx)).x;
reg_tmp6.xyz = (mul_s(uniforms.f[80].xyzz, reg_tmp14.xyzz)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp6.xyzz, reg_tmp1.xxxx)).xyz;
return false;
}
bool sub_4() {
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr3 = reg_tmp2;
reg_tmp0.zw = (uniforms.f[17].yyyy).zw;
reg_tmp0.xy = (uniforms.f[95].yyyy).xy;
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[93].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
reg_tmp13.x = (max_s(uniforms.f[17].xxxx, -reg_tmp4.yyyy)).x;
reg_tmp0.xyz = (mul_s(uniforms.f[94].xyzz, reg_tmp13.xxxx)).xyz;
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp0.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_26() {
addr_regs.z = int(uniforms.i[0].y);
for (uint i = 0u; i <= uniforms.i[0].x; addr_regs.z += int(uniforms.i[0].z), ++i) {
sub_27();
}
return false;
}
bool sub_27() {
reg_tmp14.xyz = (uniforms.f[81 + addr_regs.z].xyzz).xyz;
reg_tmp3.x = (uniforms.f[81 + addr_regs.z].wwww).x;
reg_tmp2.xyz = (uniforms.f[87 + addr_regs.z].xyzz).xyz;
reg_tmp3.y = (uniforms.f[87 + addr_regs.z].wwww).y;
sub_28();
reg_tmp11.xyz = (reg_tmp11.xyzz + reg_tmp5.xyzz).xyz;
reg_tmp13 = max_s(uniforms.f[17].xxxy, reg_tmp11.xyzz);
reg_tmp11.xyz = (min_s(uniforms.f[17].yyyy, reg_tmp13.xyzz)).xyz;
return false;
}
bool sub_29() {
reg_tmp11.xyz = (uniforms.f[17].yyyy).xyz;
return false;
}
// reference: 87B719232EC082E7, 461AD8F9213BCBA8
// shader: 8B30, A7A6AFBA973E1D97
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
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
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (primary_fragment_color.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1)) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1)) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((vec3(1) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor1.ggg)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 27D1E017232E74C3, A7A6AFBA973E1D97
// program: 461AD8F9213BCBA8, C8EFC3EA7CDDD5A3, A7A6AFBA973E1D97
// shader: 8B31, FBB49A89710CEC73

#define mul_s(x, y) (x * y)
#define fma_s(x, y, z) fma(x, y, z)
#define dot_s(x, y) dot(x, y)
#define dot_3(x, y) dot(x, y)

#define rcp_s(x) (1.0 / x)
#define rsq_s(x) inversesqrt(x)

#define min_s(x, y) min(x, y)
#define max_s(x, y) max(x, y)

struct pica_uniforms {
    bool b[16];
    uvec4 i[4];
    vec4 f[96];
};

bool exec_shader();

#define uniforms vs_uniforms
layout(binding=2, std140) uniform vs_config {
    pica_uniforms uniforms;
};
layout(location=0) in vec4 vs_in_reg0;
layout(location=1) in vec4 vs_in_reg1;
layout(location=3) in vec4 vs_in_reg3;
layout(location=4) in vec4 vs_in_reg4;
layout(location=5) in vec4 vs_in_reg5;
layout(location=6) in vec4 vs_in_reg6;
layout(location=7) in vec4 vs_in_reg7;
layout(location=8) in vec4 vs_in_reg8;

layout(location=0) out vec4 vs_out_attr0;
layout(location=1) out vec4 vs_out_attr1;
layout(location=2) out vec4 vs_out_attr2;
layout(location=3) out vec4 vs_out_attr3;
layout(location=4) out vec4 vs_out_attr4;
layout(location=5) out vec4 vs_out_attr5;
layout(location=6) out vec4 vs_out_attr6;

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
vs_out_attr4 = vec4(0, 0, 0, 1);
vs_out_attr5 = vec4(0, 0, 0, 1);
vs_out_attr6 = vec4(0, 0, 0, 1);
exec_shader();
}
bvec2 bool_regs = bvec2(false);
ivec3 addr_regs = ivec3(0);
vec4 reg_tmp0 = vec4(0, 0, 0, 1);
vec4 reg_tmp1 = vec4(0, 0, 0, 1);
vec4 reg_tmp2 = vec4(0, 0, 0, 1);
vec4 reg_tmp3 = vec4(0, 0, 0, 1);
vec4 reg_tmp4 = vec4(0, 0, 0, 1);
vec4 reg_tmp5 = vec4(0, 0, 0, 1);
vec4 reg_tmp6 = vec4(0, 0, 0, 1);
vec4 reg_tmp7 = vec4(0, 0, 0, 1);
vec4 reg_tmp8 = vec4(0, 0, 0, 1);
vec4 reg_tmp9 = vec4(0, 0, 0, 1);
vec4 reg_tmp10 = vec4(0, 0, 0, 1);
vec4 reg_tmp11 = vec4(0, 0, 0, 1);
vec4 reg_tmp12 = vec4(0, 0, 0, 1);
vec4 reg_tmp13 = vec4(0, 0, 0, 1);
vec4 reg_tmp14 = vec4(0, 0, 0, 1);
vec4 reg_tmp15 = vec4(0, 0, 0, 1);

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
bool_regs = equal(uniforms.f[17].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
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
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
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
addr_regs.x = (ivec2(reg_tmp14.xx)).x;
if (uniforms.b[4]) {
sub_5();
}
reg_tmp0.x = dot_s(uniforms.f[19 + addr_regs.x], reg_tmp10);
reg_tmp0.y = dot_s(uniforms.f[20 + addr_regs.x], reg_tmp10);
reg_tmp0.z = dot_s(uniforms.f[21 + addr_regs.x], reg_tmp10);
reg_tmp7.xyz = (fma_s(reg_tmp14.wwww, reg_tmp0.xyzz, reg_tmp7.xyzz)).xyz;
return false;
}
bool sub_5() {
reg_tmp0.x = dot_3(uniforms.f[19 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.y = dot_3(uniforms.f[20 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp0.z = dot_3(uniforms.f[21 + addr_regs.x].xyz, reg_tmp5.xyz);
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
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.yyyy)).x;
reg_tmp14.w = (vs_in_reg4.yyyy).w;
sub_4();
reg_tmp14.x = (mul_s(uniforms.f[17].wwww, vs_in_reg3.zzzz)).x;
reg_tmp14.w = (vs_in_reg4.zzzz).w;
sub_4();
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
sub_4();
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
sub_4();
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
sub_19();
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
sub_19();
if (uniforms.b[9]) {
sub_25();
}
reg_tmp2.zw = (uniforms.f[17].xxxx).zw;
reg_tmp2.y = (uniforms.f[17].yyyy).y;
reg_tmp2.y = (reg_tmp2.yyyy + -reg_tmp0.yyyy).y;
reg_tmp2.x = (reg_tmp0.xxxx).x;
vs_out_attr4 = reg_tmp2;
reg_tmp0.xy = (uniforms.f[95].zzzz).xy;
sub_19();
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
bool_regs = greaterThanEqual(uniforms.f[17].xx, reg_tmp0.xx);
reg_tmp0 = vec4(rsq_s(reg_tmp0.x));
reg_tmp1 = mul_s(uniforms.f[18].xxxx, reg_tmp3.xyzz);
if (!bool_regs.x) {
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
// reference: 7E92704DBEA4B5B5, FBB49A89710CEC73
// shader: 8B30, AD8D2FE73E4E2A03
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((texcolor0.r) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (last_tex_env_out.rgb);
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) + (1.0 - texcolor1.r) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = (last_tex_env_out.rgb);
float alpha_output_3 = byteround(clamp((rounded_primary_color.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_4 = (last_tex_env_out.rgb);
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (combiner_buffer.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (const_color[5].aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (1.0 - texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CCE4CDEB4CF5EF29, AD8D2FE73E4E2A03
// program: FBB49A89710CEC73, C8EFC3EA7CDDD5A3, AD8D2FE73E4E2A03
// shader: 8B30, 595440CC92DBF658
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (const_color[0].rrr), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) * (const_color[0].a) + (texcolor0.a) * (1.0 - (const_color[0].a)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((const_color[1].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((combiner_buffer.a) - (1.0 - rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].rrr) + (combiner_buffer.rgb) * (vec3(1) - (const_color[3].rrr)), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].g) + (combiner_buffer.a) * (1.0 - (const_color[3].g)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4 * 1.0, alpha_output_4 * 4.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (last_tex_env_out.rgb);
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (const_color[5].b) + (combiner_buffer.a) * (1.0 - (const_color[5].b)), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2362AED5FDFBB8B6, 595440CC92DBF658
// program: C6AD1F9C8D20C2A1, C8EFC3EA7CDDD5A3, 595440CC92DBF658
// shader: 8B30, 3680A7777FBA4551
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor0.r) + (const_color[0].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.aaa) + (combiner_buffer.rgb) * (vec3(1) - (rounded_primary_color.aaa)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].aaa), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F65B89B5A7FA83F0, 3680A7777FBA4551
// program: F55B22C1EDEDB856, C8EFC3EA7CDDD5A3, 3680A7777FBA4551
// shader: 8B30, FBAC6878EC0DAF46
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((texcolor1.r) * (texcolor2.r), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (primary_fragment_color.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp(min((last_tex_env_out.rgb) + (combiner_buffer.aaa), vec3(1)) * (texcolor1.rrr), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((primary_fragment_color.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp(min((const_color[3].rgb) + (last_tex_env_out.aaa), vec3(1)) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((combiner_buffer.a) + (const_color[3].a) - 0.5, 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((vec3(1) - last_tex_env_out.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((const_color[5].rgb) * (texcolor1.ggg) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor1.ggg)), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4570047F232E74C3, FBAC6878EC0DAF46
// program: 461AD8F9213BCBA8, C8EFC3EA7CDDD5A3, FBAC6878EC0DAF46
// shader: 8B30, E313932159A84A37
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(half_vector)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_0) + ((lut_scale_d1 * LookupLightingLUTUnsigned(1, max(dot(light_vector, normal), 0.0))) * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp(min((primary_fragment_color.rgb) + (secondary_fragment_color.rgb), vec3(1)) * (texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1 * 4.0, alpha_output_1 * 1.0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (const_color[2].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4FD59A5D6CFE4432, E313932159A84A37
// program: 626F97B8DBDF0948, C8EFC3EA7CDDD5A3, E313932159A84A37
// shader: 8B30, 98494674CE353E55
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) + (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) + (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((primary_fragment_color.aaa) * (vec3(1) - const_color[2].aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (const_color[3].aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 401F6B012E229BA9, 98494674CE353E55
// program: A52020F0BCE0EE09, C8EFC3EA7CDDD5A3, 98494674CE353E55
// reference: AE4529834CF5EF29, AD8D2FE73E4E2A03
// reference: 41C34ABDFDFBB8B6, 595440CC92DBF658
// reference: 2DB712321D6A5DCA, 3374FE3C2F1DB00C
// reference: 118766CA82E696EC, F19417D717A57884
// reference: 5FD64035B0EFDE91, 718ACC30D6F6098C
// reference: 5FD6403589538ECD, C4329D35E3F32E44
// shader: 8B30, 1C8337CBA6940863
layout(location=1) in vec4 primary_color;
layout(location=2) in vec2 texcoord0;
layout(location=3) in vec2 texcoord1;
layout(location=4) in vec2 texcoord2;
layout(location=5) in float texcoord0_w;
layout(location=6) in vec4 normquat;
layout(location=7) in vec3 view;
out vec4 color;

layout(binding=0) uniform sampler2D tex0;
layout(binding=1) uniform sampler2D tex1;
layout(binding=2) uniform sampler2D tex2;
layout(binding=3) uniform samplerBuffer tex_lut_lf;
layout(binding=4) uniform samplerBuffer tex_lut_rg;
layout(binding=5) uniform samplerBuffer tex_lut_rgba;
layout(binding=6) uniform samplerCube tex_cube;

layout(binding=0, std140) uniform shader_data {
    int scissor_x1;
    int scissor_y1;
    int scissor_x2;
    int scissor_y2;
    int fog_lut_offset;
    int proctex_lut_offset;
    int proctex_diff_lut_offset;
    int proctex_noise_lut_offset;
    int proctex_color_map_offset;
    int proctex_alpha_map_offset;
    float alphatest_ref;
    float depth_scale;
    float depth_offset;
    float shadow_bias_constant;
    float shadow_bias_linear;
    float proctex_bias;
    vec3 fog_color;
    vec3 tex_lod_bias;
    vec2 proctex_noise_f;
    vec2 proctex_noise_a;
    vec2 proctex_noise_p;
    vec4 clip_coef;
    vec4 tev_combiner_buffer_color;
    vec4 const_color[6];
};

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
layout(binding=1, std140) uniform shader_light_data {
    ivec4 lighting_lut_offset[6];
    vec3 lighting_global_ambient;
    LightSrc light_src[8];
    float lut_scale_d0;
    float lut_scale_d1;
    float lut_scale_sp;
    float lut_scale_fr;
    float lut_scale_rb;
    float lut_scale_rg;
    float lut_scale_rr;
    int shadow_texture_bias;
};

vec3 QuaternionRotate(vec4 q, vec3 v) {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}
float LookupLightingLUT(int lut_index, int index, float delta) {
    vec2 entry = texelFetch(tex_lut_lf, lighting_lut_offset[lut_index >> 2][lut_index & 3] + index).rg;
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

void main() {
vec4 rounded_primary_color = byteround(primary_color);
vec4 primary_fragment_color = vec4(0);
vec4 secondary_fragment_color = vec4(0);
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
depth /= gl_FragCoord.w;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = vec3(0, 0, 1);
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position + view);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
clamp_highlights = sign(dot_product);
refl_value.r = (lut_scale_rr * LookupLightingLUTUnsigned(6, max(dot(normal, normalize(half_vector)), 0.0)));
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (refl_value * light_src[0].specular_1)) * clamp_highlights * LookupLightingLUTUnsigned(16, clamp(light_src[0].dist_atten_scale * length(-light_src[0].position - view) + light_src[0].dist_atten_bias, 0.0, 1.0)) * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.aaa) * (vec3(1) - const_color[1].aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (const_color[2].aaa), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
float fog_index = depth * 128.0;
float fog_i = clamp(floor(fog_index), 0.0, 127.0);
float fog_f = fog_index - fog_i;
vec2 fog_lut_entry = texelFetch(tex_lut_lf, int(fog_i) + fog_lut_offset).rg;
float fog_factor = fog_lut_entry.r + fog_lut_entry.g * fog_f;
fog_factor = clamp(fog_factor, 0.0, 1.0);
last_tex_env_out.rgb = mix(fog_color.rgb, last_tex_env_out.rgb, fog_factor);
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 54F660444BBD7409, 1C8337CBA6940863
// program: 9871BDE695FC9AF2, C8EFC3EA7CDDD5A3, 1C8337CBA6940863
