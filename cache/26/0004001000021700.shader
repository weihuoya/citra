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
// shader: 8B31, 3AE9AAE877A9E01A

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
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_6();
bool sub_7();
bool sub_8();
bool sub_9();
bool sub_10();
bool sub_11();
bool sub_12();
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

bool sub_0() {
addr_regs.x = (ivec2(vs_in_reg0.xy)).x;
reg_tmp0 = uniforms.f[6 + addr_regs.x].wzyx;
reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
reg_tmp1.zw = (uniforms.f[5].xyxy).zw;
addr_regs.xy = ivec2(reg_tmp0.xy);
reg_tmp2.xw = (uniforms.f[64 + addr_regs.y].wwyy).xw;
reg_tmp2.yz = (uniforms.f[5].xxxx).yz;
reg_tmp4.x = dot_s(reg_tmp1, reg_tmp2);
reg_tmp2.yw = (uniforms.f[64 + addr_regs.y].zzxx).yw;
reg_tmp2.xz = (uniforms.f[5].xxxx).xz;
reg_tmp4.y = dot_s(reg_tmp1, reg_tmp2);
reg_tmp4.zw = (reg_tmp1.zwzw).zw;
reg_tmp3.x = dot_s(uniforms.f[32 + addr_regs.x].wzyx, reg_tmp4);
reg_tmp3.y = dot_s(uniforms.f[33 + addr_regs.x].wzyx, reg_tmp4);
reg_tmp3.z = dot_s(uniforms.f[34 + addr_regs.x].wzyx, reg_tmp4);
reg_tmp3.w = (reg_tmp1.wwww).w;
reg_tmp4.z = (uniforms.f[34 + addr_regs.x].xxxx).z;
reg_tmp4.z = (abs(reg_tmp4.zzzz)).z;
reg_tmp4.z = (uniforms.f[4].yyyy + reg_tmp4.zzzz).z;
reg_tmp4.x = (uniforms.f[4].wwww).x;
bool_regs = notEqual(uniforms.f[5].xx, reg_tmp4.xz);
if (all(bool_regs)) {
sub_1();
}
vs_out_attr0.x = dot_s(uniforms.f[0].wzyx, reg_tmp3);
vs_out_attr0.y = dot_s(uniforms.f[1].wzyx, reg_tmp3);
vs_out_attr0.z = dot_s(uniforms.f[2].wzyx, reg_tmp3);
vs_out_attr0.w = dot_s(uniforms.f[3].wzyx, reg_tmp3);
bool_regs = greaterThanEqual(uniforms.f[5].yy, reg_tmp0.ww);
if (all(bool_regs)) {
sub_2();
} else {
sub_3();
}
bool_regs = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
if (all(not(bool_regs))) {
sub_8();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_9();
}
if (all(bvec2(!bool_regs.x, bool_regs.y))) {
sub_10();
}
if (all(bool_regs)) {
sub_11();
}
reg_tmp8 = uniforms.f[5].xxxx;
addr_regs.z = int(uniforms.i[0].y);
for (uint i = 0u; i <= uniforms.i[0].x; addr_regs.z += int(uniforms.i[0].z), ++i) {
sub_12();
}
vs_out_attr2 = reg_tmp5;
vs_out_attr3 = reg_tmp6;
vs_out_attr4 = reg_tmp7;
return true;
}
bool sub_1() {
reg_tmp4.x = (uniforms.f[4].wwww).x;
reg_tmp4.y = (-uniforms.f[4].zzzz + reg_tmp4.zzzz).y;
reg_tmp4.z = rcp_s(reg_tmp4.z);
reg_tmp4.z = (mul_s(reg_tmp4.yyyy, reg_tmp4.zzzz)).z;
reg_tmp3.x = (fma_s(reg_tmp4.xxxx, reg_tmp4.zzzz, reg_tmp3.xxxx)).x;
return false;
}
bool sub_2() {
vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
vs_out_attr1.w = (reg_tmp0.wwww).w;
return false;
}
bool sub_3() {
addr_regs.y = (ivec2(reg_tmp0.ww)).y;
bool_regs = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
if (all(not(bool_regs))) {
sub_4();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_5();
}
if (all(bvec2(!bool_regs.x, bool_regs.y))) {
sub_6();
}
if (all(bool_regs)) {
sub_7();
}
return false;
}
bool sub_4() {
vs_out_attr1 = uniforms.f[32 + addr_regs.y].wzyx;
return false;
}
bool sub_5() {
vs_out_attr1 = uniforms.f[33 + addr_regs.y].wzyx;
return false;
}
bool sub_6() {
vs_out_attr1 = uniforms.f[34 + addr_regs.y].wzyx;
return false;
}
bool sub_7() {
vs_out_attr1 = uniforms.f[35 + addr_regs.y].wzyx;
return false;
}
bool sub_8() {
reg_tmp5 = uniforms.f[5].xyyy;
reg_tmp6 = uniforms.f[5].xyyy;
reg_tmp7 = uniforms.f[5].xyyy;
return false;
}
bool sub_9() {
reg_tmp5 = uniforms.f[5].yyyy;
reg_tmp6 = uniforms.f[5].yyyy;
reg_tmp7 = uniforms.f[5].yyyy;
return false;
}
bool sub_10() {
reg_tmp5 = uniforms.f[5].xxyy;
reg_tmp6 = uniforms.f[5].xxyy;
reg_tmp7 = uniforms.f[5].xxyy;
return false;
}
bool sub_11() {
reg_tmp5 = uniforms.f[5].yxyy;
reg_tmp6 = uniforms.f[5].yxyy;
reg_tmp7 = uniforms.f[5].yxyy;
return false;
}
bool sub_12() {
bool_regs = equal(uniforms.f[5].yy, reg_tmp8.xy);
if (all(bool_regs)) {
sub_13();
}
bool_regs = lessThan(uniforms.f[5].ww, reg_tmp8.xy);
if (all(bool_regs)) {
sub_18();
}
reg_tmp8 = uniforms.f[5].yyyy + reg_tmp8;
return false;
}
bool sub_13() {
addr_regs.y = (ivec2(reg_tmp0.zz)).y;
bool_regs = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
if (all(not(bool_regs))) {
sub_14();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_15();
}
if (all(bvec2(!bool_regs.x, bool_regs.y))) {
sub_16();
}
if (all(bool_regs)) {
sub_17();
}
return false;
}
bool sub_14() {
reg_tmp5.xy = (uniforms.f[64 + addr_regs.y].wzzz).xy;
reg_tmp6.xy = (uniforms.f[65 + addr_regs.y].wzzz).xy;
reg_tmp7.xy = (uniforms.f[66 + addr_regs.y].wzzz).xy;
return false;
}
bool sub_15() {
reg_tmp5.xy = (uniforms.f[64 + addr_regs.y].yzzz).xy;
reg_tmp6.xy = (uniforms.f[65 + addr_regs.y].yzzz).xy;
reg_tmp7.xy = (uniforms.f[66 + addr_regs.y].yzzz).xy;
return false;
}
bool sub_16() {
reg_tmp5.xy = (uniforms.f[64 + addr_regs.y].wxxx).xy;
reg_tmp6.xy = (uniforms.f[65 + addr_regs.y].wxxx).xy;
reg_tmp7.xy = (uniforms.f[66 + addr_regs.y].wxxx).xy;
return false;
}
bool sub_17() {
reg_tmp5.xy = (uniforms.f[64 + addr_regs.y].yxxx).xy;
reg_tmp6.xy = (uniforms.f[65 + addr_regs.y].yxxx).xy;
reg_tmp7.xy = (uniforms.f[66 + addr_regs.y].yxxx).xy;
return false;
}
bool sub_18() {
bool_regs = notEqual(uniforms.f[5].xx, reg_tmp1.xy);
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_19();
}
if (all(bvec2(!bool_regs.x, bool_regs.y))) {
sub_20();
}
return false;
}
bool sub_19() {
reg_tmp5.xy = (uniforms.f[67 + addr_regs.y].yxxx).xy;
reg_tmp6.xy = (uniforms.f[68 + addr_regs.y].yxxx).xy;
reg_tmp7.xy = (uniforms.f[69 + addr_regs.y].yxxx).xy;
return false;
}
bool sub_20() {
reg_tmp5.xy = (uniforms.f[67 + addr_regs.y].wzzz).xy;
reg_tmp6.xy = (uniforms.f[68 + addr_regs.y].wzzz).xy;
reg_tmp7.xy = (uniforms.f[69 + addr_regs.y].wzzz).xy;
return false;
}
// reference: 456A7DE6F31AC733, 3AE9AAE877A9E01A
// shader: 8B30, D72CD26488CB85F8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].aaa) * (last_tex_env_out.rgb) + (const_color[3].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (rounded_primary_color.rgb) + (const_color[4].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A3697530A02F5D62, D72CD26488CB85F8
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, D72CD26488CB85F8
// shader: 8B30, 60B8B9F729FD438A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5AC9C18CA5589F9D, 60B8B9F729FD438A
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 60B8B9F729FD438A
// shader: 8B30, 4F126FA0395F9805
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (1.0 - last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (last_tex_env_out.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9F9A1EE07D377C28, 4F126FA0395F9805
// program: 0000000000000000, 0000000000000000, 4F126FA0395F9805
// shader: 8B30, 69256B94345D4996
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3660884E053E3E85, 69256B94345D4996
// program: 0000000000000000, 0000000000000000, 69256B94345D4996
// shader: 8B30, 23D00B8A236C474C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((texcolor0.ggg) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.bbb) * (const_color[2].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AC9DA544288FDD43, 23D00B8A236C474C
// program: 0000000000000000, 0000000000000000, 23D00B8A236C474C
// shader: 8B30, F9E9AB04A162E8FA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 01F36D6FF3689984, F9E9AB04A162E8FA
// program: 0000000000000000, 0000000000000000, F9E9AB04A162E8FA
// shader: 8B30, 458CD380C42AA773
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3660884E5497CFD9, 458CD380C42AA773
// program: 0000000000000000, 0000000000000000, 458CD380C42AA773
// shader: 8B30, 5CB79E9B1E8774FA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 01F36D6F7D377C28, 5CB79E9B1E8774FA
// program: 0000000000000000, 0000000000000000, 5CB79E9B1E8774FA
// shader: 8B30, 5D91A8F499053494
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 77CC947209D9BD65, 5D91A8F499053494
// program: 0000000000000000, 0000000000000000, 5D91A8F499053494
// shader: 8DD9, EADA2116091C5E01

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

    vec4 vtx_color = vec4(vtx.attributes[3].x, vtx.attributes[3].y, vtx.attributes[3].z, vtx.attributes[3].w);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

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
// reference: FC74FA4ACA1C8C74, EADA2116091C5E01
// shader: 8B31, 52440BFACA1196F5

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

bool sub_0();
bool sub_23();
bool sub_6();
bool sub_11();
bool sub_1();
bool sub_2();
bool sub_3();
bool sub_4();
bool sub_5();
bool sub_7();
bool sub_9();
bool sub_10();
bool sub_12();
bool sub_22();
bool sub_24();
bool sub_25();
bool sub_26();
bool sub_27();
bool sub_28();
bool sub_29();
bool sub_30();
bool sub_31();
bool sub_32();
bool sub_8();
bool sub_13();
bool sub_14();
bool sub_15();
bool sub_16();
bool sub_17();
bool sub_18();
bool sub_19();
bool sub_20();
bool sub_21();
bool sub_33();
bool sub_34();
bool sub_35();
bool sub_47();
bool sub_36();
bool sub_37();
bool sub_38();
bool sub_39();
bool sub_40();
bool sub_41();
bool sub_42();
bool sub_43();
bool sub_44();
bool sub_45();
bool sub_46();
bool sub_48();
bool sub_49();
bool sub_55();
bool sub_56();
bool sub_57();
bool sub_58();
bool sub_60();
bool sub_50();
bool sub_51();
bool sub_52();
bool sub_53();
bool sub_54();
bool sub_59();
bool sub_61();
bool sub_62();
bool sub_63();
bool sub_64();
bool sub_65();
bool sub_66();
bool sub_67();
bool sub_68();
bool sub_69();
bool sub_70();
bool sub_71();
bool sub_72();
bool sub_73();

bool exec_shader() {
sub_0();
return true;
}

bool sub_0() {
sub_1();
sub_33();
sub_48();
sub_62();
sub_69();
return true;
}
bool sub_23() {
addr_regs.x = (ivec2(reg_tmp1.xx)).x;
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
return false;
}
bool sub_6() {
reg_tmp1.y = (reg_tmp1.yyyy + reg_tmp2.yyyy).y;
addr_regs.xy = ivec2(reg_tmp1.xy);
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
return false;
}
bool sub_11() {
reg_tmp1.y = (reg_tmp1.yyyy + reg_tmp2.yyyy).y;
addr_regs.xy = ivec2(reg_tmp1.xy);
reg_tmp3.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp3.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp3.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp4.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp4.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp4.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp5.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp5.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp5.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp7 = fma_s(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
reg_tmp12 = fma_s(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
reg_tmp11 = fma_s(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
return false;
}
bool sub_1() {
reg_tmp15.xyz = (mul_s(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
reg_tmp14.xyz = (mul_s(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
reg_tmp13.xyz = (mul_s(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
reg_tmp15.w = (uniforms.f[93].yyyy).w;
if (uniforms.b[1]) {
sub_2();
} else {
sub_25();
}
return false;
}
bool sub_2() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
reg_tmp7 = uniforms.f[93].xxxx;
reg_tmp12 = uniforms.f[93].xxxx;
reg_tmp11 = uniforms.f[93].xxxx;
reg_tmp2.x = (uniforms.f[9].xxxx).x;
if (uniforms.b[4]) {
sub_3();
} else {
sub_4();
}
reg_tmp2.xy = (mul_s(uniforms.f[93].wwww, reg_tmp2.xyyy)).xy;
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_5();
} else {
sub_9();
}
vs_out_attr2 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_3() {
reg_tmp2.y = (uniforms.f[9].zzzz).y;
return false;
}
bool sub_4() {
reg_tmp2.y = (uniforms.f[93].xxxx).y;
return false;
}
bool sub_5() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (mul_s(vs_in_reg7.xxxx, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_6();
reg_tmp1.xy = (mul_s(vs_in_reg7.yyyy, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_6();
reg_tmp1.xy = (mul_s(vs_in_reg7.zzzz, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_6();
}
if (uniforms.b[8]) {
sub_7();
}
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_8();
return false;
}
bool sub_7() {
reg_tmp1.xy = (mul_s(vs_in_reg7.wwww, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_6();
}
return false;
}
bool sub_9() {
if (all(bool_regs)) {
sub_10();
} else {
sub_22();
}
return false;
}
bool sub_10() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (mul_s(vs_in_reg7.xxxx, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_11();
reg_tmp1.xy = (mul_s(vs_in_reg7.yyyy, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_11();
reg_tmp1.xy = (mul_s(vs_in_reg7.zzzz, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_11();
}
if (uniforms.b[8]) {
sub_12();
}
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
sub_13();
return false;
}
bool sub_12() {
reg_tmp1.xy = (mul_s(vs_in_reg7.wwww, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_11();
}
return false;
}
bool sub_22() {
bool_regs = notEqual(uniforms.f[93].xx, vs_in_reg8.zw);
reg_tmp1.xy = (mul_s(vs_in_reg7.xxxx, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
sub_23();
reg_tmp1.xy = (mul_s(vs_in_reg7.yyyy, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
sub_23();
reg_tmp1.xy = (mul_s(vs_in_reg7.zzzz, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
if (bool_regs.x) {
sub_23();
}
if (uniforms.b[8]) {
sub_24();
}
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_24() {
reg_tmp1.xy = (mul_s(vs_in_reg7.wwww, reg_tmp2.xxxx)).xy;
reg_tmp1.w = (mul_s(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
if (bool_regs.y) {
sub_23();
}
return false;
}
bool sub_25() {
reg_tmp0 = uniforms.f[7];
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.yz);
if (uniforms.b[2]) {
sub_26();
} else {
sub_28();
}
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_29();
} else {
sub_30();
}
vs_out_attr2 = -reg_tmp15;
vs_out_attr0.x = dot_s(uniforms.f[86], reg_tmp15);
vs_out_attr0.y = dot_s(uniforms.f[87], reg_tmp15);
vs_out_attr0.z = dot_s(uniforms.f[88], reg_tmp15);
vs_out_attr0.w = dot_s(uniforms.f[89], reg_tmp15);
return false;
}
bool sub_26() {
reg_tmp1.xy = (mul_s(uniforms.f[9].xxxx, vs_in_reg7.xxxx)).xy;
if (uniforms.b[4]) {
sub_27();
}
reg_tmp1.xy = (mul_s(uniforms.f[93].wwww, reg_tmp1.xyyy)).xy;
addr_regs.xy = ivec2(reg_tmp1.xy);
reg_tmp7.x = dot_s(uniforms.f[25 + addr_regs.x], reg_tmp15);
reg_tmp7.y = dot_s(uniforms.f[26 + addr_regs.x], reg_tmp15);
reg_tmp7.z = dot_s(uniforms.f[27 + addr_regs.x], reg_tmp15);
reg_tmp7.w = (uniforms.f[93].yyyy).w;
reg_tmp10.x = dot_s(uniforms.f[0], reg_tmp7);
reg_tmp10.y = dot_s(uniforms.f[1], reg_tmp7);
reg_tmp10.z = dot_s(uniforms.f[2], reg_tmp7);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_27() {
reg_tmp1.y = (uniforms.f[9].zzzz + reg_tmp1.yyyy).y;
return false;
}
bool sub_28() {
addr_regs.xy = ivec2(uniforms.f[93].xx);
reg_tmp10.x = dot_s(uniforms.f[25], reg_tmp15);
reg_tmp10.y = dot_s(uniforms.f[26], reg_tmp15);
reg_tmp10.z = dot_s(uniforms.f[27], reg_tmp15);
reg_tmp10.w = (uniforms.f[93].yyyy).w;
return false;
}
bool sub_29() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
sub_8();
return false;
}
bool sub_30() {
if (all(bool_regs)) {
sub_31();
} else {
sub_32();
}
return false;
}
bool sub_31() {
reg_tmp12.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp12.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp12.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp14.xyz);
reg_tmp11.x = dot_3(uniforms.f[25 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp11.y = dot_3(uniforms.f[26 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp11.z = dot_3(uniforms.f[27 + addr_regs.y].xyz, reg_tmp13.xyz);
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
reg_tmp14.x = dot_3(uniforms.f[3].xyz, reg_tmp12.xyz);
reg_tmp14.y = dot_3(uniforms.f[4].xyz, reg_tmp12.xyz);
reg_tmp14.z = dot_3(uniforms.f[5].xyz, reg_tmp12.xyz);
reg_tmp13.x = dot_3(uniforms.f[3].xyz, reg_tmp11.xyz);
reg_tmp13.y = dot_3(uniforms.f[4].xyz, reg_tmp11.xyz);
reg_tmp13.z = dot_3(uniforms.f[5].xyz, reg_tmp11.xyz);
sub_13();
return false;
}
bool sub_32() {
reg_tmp15.x = dot_s(uniforms.f[90], reg_tmp10);
reg_tmp15.y = dot_s(uniforms.f[91], reg_tmp10);
reg_tmp15.z = dot_s(uniforms.f[92], reg_tmp10);
reg_tmp15.w = (uniforms.f[93].yyyy).w;
vs_out_attr1 = uniforms.f[93].xxxx;
return false;
}
bool sub_8() {
uint jmp_to = 211u;
while (true) {
switch (jmp_to) {
case 211u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[15]) {
jmp_to = 227u; break;
}
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (bool_regs.x) {
jmp_to = 227u; break;
}
reg_tmp0.z = rcp_s(reg_tmp4.x);
reg_tmp0.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
case 227u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_13() {
uint jmp_to = 228u;
while (true) {
switch (jmp_to) {
case 228u:
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp7.x = dot_3(reg_tmp12.xyz, reg_tmp12.xyz);
reg_tmp6.x = rsq_s(reg_tmp6.x);
reg_tmp7.x = rsq_s(reg_tmp7.x);
reg_tmp14.xyz = (mul_s(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp12.xyz = (mul_s(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp0 = uniforms.f[93].yxxx;
if (!uniforms.b[15]) {
jmp_to = 303u; break;
}
reg_tmp13.xyz = (mul_s(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
reg_tmp11.xyz = (mul_s(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
reg_tmp5 = mul_s(reg_tmp14.yzxx, reg_tmp13.zxyy);
reg_tmp5 = fma_s(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
reg_tmp5.w = dot_3(reg_tmp5.xyz, reg_tmp5.xyz);
reg_tmp5.w = rsq_s(reg_tmp5.w);
reg_tmp5 = mul_s(reg_tmp5, reg_tmp5.wwww);
reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
reg_tmp13 = mul_s(reg_tmp5.yzxx, reg_tmp14.zxyy);
reg_tmp13 = fma_s(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
reg_tmp13.w = (reg_tmp5.zzzz).w;
reg_tmp5.z = (reg_tmp13.xxxx).z;
reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
reg_tmp14.w = (reg_tmp5.xxxx).w;
reg_tmp5.x = (reg_tmp14.zzzz).x;
bool_regs = lessThan(uniforms.f[94].yy, reg_tmp6.ww);
reg_tmp6.x = (uniforms.f[93].yyyy).x;
reg_tmp6.y = (-uniforms.f[93].yyyy).y;
if (!bool_regs.x) {
jmp_to = 265u; break;
}
reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
reg_tmp7.w = (reg_tmp6).w;
reg_tmp6 = vec4(dot_s(reg_tmp7, reg_tmp7));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp7, reg_tmp6);
if (uniforms.b[0]) {
jmp_to = 303u; break;
}
case 265u:
bool_regs = greaterThan(reg_tmp5.zy, reg_tmp5.yx);
if (bool_regs.x) {
sub_14();
} else {
sub_19();
}
reg_tmp6 = vec4(dot_s(reg_tmp8, reg_tmp8));
reg_tmp6 = vec4(rsq_s(reg_tmp6.x));
reg_tmp0 = mul_s(reg_tmp8, reg_tmp6);
case 303u:
vs_out_attr1 = reg_tmp0;
default: return false;
}
}
return false;
}
bool sub_14() {
if (bool_regs.y) {
sub_15();
} else {
sub_16();
}
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_15() {
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_16() {
bool_regs = greaterThan(reg_tmp5.zz, reg_tmp5.xx);
reg_tmp8 = mul_s(reg_tmp13.yyzw, reg_tmp6.xxxy);
reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
if (bool_regs.x) {
sub_17();
} else {
sub_18();
}
return false;
}
bool sub_17() {
reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
return false;
}
bool sub_18() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
return false;
}
bool sub_19() {
if (bool_regs.y) {
sub_20();
} else {
sub_21();
}
return false;
}
bool sub_20() {
reg_tmp8 = mul_s(reg_tmp13.yywz, reg_tmp6.xxxy);
reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
return false;
}
bool sub_21() {
reg_tmp8 = mul_s(reg_tmp13.zwwy, reg_tmp6.xxxy);
reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
reg_tmp8.w = (-reg_tmp8).w;
return false;
}
bool sub_33() {
reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
reg_tmp0.y = (uniforms.f[7].wwww).y;
bool_regs = notEqual(uniforms.f[93].xx, reg_tmp0.xy);
reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
reg_tmp9.w = (uniforms.f[21].wwww).w;
if (bool_regs.y) {
sub_34();
}
if (uniforms.b[12]) {
sub_36();
}
if (uniforms.b[5]) {
sub_45();
}
bool_regs = equal(uniforms.f[93].xx, reg_tmp8.xy);
if (all(bool_regs)) {
sub_47();
}
vs_out_attr3 = max_s(uniforms.f[93].xxxx, reg_tmp9);
return false;
}
bool sub_34() {
reg_tmp0 = mul_s(uniforms.f[7].wwww, vs_in_reg3);
if (uniforms.b[7]) {
sub_35();
}
reg_tmp9.xyz = (mul_s(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_35() {
reg_tmp9.w = (mul_s(reg_tmp9.wwww, reg_tmp0.wwww)).w;
return false;
}
bool sub_47() {
reg_tmp9 = uniforms.f[21];
return false;
}
bool sub_36() {
reg_tmp1 = uniforms.f[20];
reg_tmp2 = uniforms.f[21];
reg_tmp3 = uniforms.f[93].xxxx;
addr_regs.z = int(uniforms.i[0].y);
for (uint i = 0u; i <= uniforms.i[0].x; addr_regs.z += int(uniforms.i[0].z), ++i) {
sub_37();
}
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_37() {
addr_regs.x = (ivec2(reg_tmp3.xy)).x;
reg_tmp4.x = (uniforms.f[81 + addr_regs.x].wwww).x;
reg_tmp4.y = (uniforms.f[83 + addr_regs.x].wwww).y;
bool_regs = equal(uniforms.f[93].xy, reg_tmp4.xy);
if (bool_regs.x) {
sub_38();
} else {
sub_39();
}
bool_regs.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
bool_regs.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
if (bool_regs.y) {
sub_44();
}
reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
return false;
}
bool sub_38() {
reg_tmp6.x = dot_3(uniforms.f[81 + addr_regs.x].xyz, reg_tmp14.xyz);
reg_tmp6.y = (uniforms.f[93].yyyy).y;
return false;
}
bool sub_39() {
reg_tmp4 = uniforms.f[81 + addr_regs.x] + -reg_tmp15;
reg_tmp6.y = (uniforms.f[93].yyyy).y;
if (bool_regs.y) {
sub_40();
}
reg_tmp5 = uniforms.f[82 + addr_regs.x];
bool_regs = equal(uniforms.f[93].yy, reg_tmp5.ww);
reg_tmp4.w = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
reg_tmp4.w = rsq_s(reg_tmp4.w);
reg_tmp4 = mul_s(reg_tmp4, reg_tmp4.wwww);
if (bool_regs.x) {
sub_41();
}
reg_tmp6.x = dot_3(reg_tmp14.xyz, reg_tmp4.xyz);
return false;
}
bool sub_40() {
reg_tmp5.x = (uniforms.f[93].yyyy).x;
reg_tmp5.z = dot_3(reg_tmp4.xyz, reg_tmp4.xyz);
reg_tmp5.y = (mul_s(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
reg_tmp6.y = dot_3(uniforms.f[83 + addr_regs.x].xyz, reg_tmp5.xyz);
reg_tmp6.y = rcp_s(reg_tmp6.y);
return false;
}
bool sub_41() {
reg_tmp5.x = dot_3(uniforms.f[82 + addr_regs.x].xyz, -reg_tmp4.xyz);
reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx, uniforms.f[84 + addr_regs.x].yyyy))).y;
bool_regs = equal(uniforms.f[93].yy, reg_tmp5.xy);
if (bool_regs.y) {
sub_42();
} else {
sub_43();
}
reg_tmp6.y = (mul_s(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
return false;
}
bool sub_42() {
reg_tmp5.x = (uniforms.f[93].xxxx).x;
return false;
}
bool sub_43() {
reg_tmp5.y = log2(reg_tmp5.x);
reg_tmp5.y = (mul_s(uniforms.f[84 + addr_regs.x].xxxx, reg_tmp5.yyyy)).y;
reg_tmp5.x = exp2(reg_tmp5.y);
return false;
}
bool sub_44() {
reg_tmp6.x = (max_s(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
reg_tmp9.xyz = (fma_s(reg_tmp1.xyzz, uniforms.f[79 + addr_regs.x].xyzz, reg_tmp9.xyzz)).xyz;
reg_tmp4 = mul_s(uniforms.f[80 + addr_regs.x], reg_tmp2);
reg_tmp5.xyz = (mul_s(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
reg_tmp5.xyz = (mul_s(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
return false;
}
bool sub_45() {
reg_tmp1 = vec4(dot_3(uniforms.f[24].xyz, reg_tmp14.xyz));
reg_tmp2 = uniforms.f[24].wwww;
reg_tmp1 = fma_s(reg_tmp1, reg_tmp2, reg_tmp2);
reg_tmp3 = uniforms.f[22];
reg_tmp2 = uniforms.f[23] + -reg_tmp3;
reg_tmp4 = fma_s(reg_tmp2, reg_tmp1, reg_tmp3);
if (uniforms.b[6]) {
sub_46();
}
reg_tmp9.xyz = (fma_s(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
reg_tmp8.x = (uniforms.f[93].yyyy).x;
return false;
}
bool sub_46() {
reg_tmp4 = mul_s(reg_tmp4, reg_tmp9.wwww);
return false;
}
bool sub_48() {
reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
if (uniforms.b[9]) {
sub_49();
} else {
sub_55();
}
return false;
}
bool sub_49() {
sub_50();
reg_tmp3.x = dot_s(uniforms.f[11].xywz, reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12].xywz, reg_tmp6);
reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_55() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
if (all(not(bool_regs))) {
sub_56();
} else {
sub_57();
}
vs_out_attr4 = reg_tmp3;
return false;
}
bool sub_56() {
reg_tmp6 = reg_tmp10;
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
reg_tmp3.z = dot_s(uniforms.f[13], reg_tmp6);
reg_tmp0.xy = (mul_s(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
return false;
}
bool sub_57() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_58();
} else {
sub_60();
}
return false;
}
bool sub_58() {
sub_59();
reg_tmp3.x = dot_3(uniforms.f[11].xyz, reg_tmp6.xyz);
reg_tmp3.y = dot_3(uniforms.f[12].xyz, reg_tmp6.xyz);
reg_tmp3.z = dot_3(uniforms.f[13].xyz, reg_tmp6.xyz);
return false;
}
bool sub_60() {
sub_61();
reg_tmp3.x = dot_s(uniforms.f[11], reg_tmp6);
reg_tmp3.y = dot_s(uniforms.f[12], reg_tmp6);
return false;
}
bool sub_50() {
bool_regs = equal(uniforms.f[93].yz, reg_tmp0.xy);
if (all(not(bool_regs))) {
sub_51();
} else {
sub_52();
}
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_51() {
reg_tmp6.xy = (mul_s(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
return false;
}
bool sub_52() {
if (all(bvec2(bool_regs.x, !bool_regs.y))) {
sub_53();
} else {
sub_54();
}
return false;
}
bool sub_53() {
reg_tmp6.xy = (mul_s(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
return false;
}
bool sub_54() {
reg_tmp6.xy = (mul_s(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
return false;
}
bool sub_59() {
reg_tmp2 = -reg_tmp15;
reg_tmp2.w = dot_3(reg_tmp2.xyz, reg_tmp2.xyz);
reg_tmp2.w = rsq_s(reg_tmp2.w);
reg_tmp2 = mul_s(reg_tmp2, reg_tmp2.wwww);
reg_tmp1 = vec4(dot_3(reg_tmp2.xyz, reg_tmp14.xyz));
reg_tmp1 = reg_tmp1 + reg_tmp1;
reg_tmp6 = fma_s(reg_tmp1, reg_tmp14, -reg_tmp2);
return false;
}
bool sub_61() {
reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
reg_tmp6 = fma_s(reg_tmp14, reg_tmp1, reg_tmp1);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
return false;
}
bool sub_62() {
reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
if (uniforms.b[10]) {
sub_63();
} else {
sub_64();
}
return false;
}
bool sub_63() {
sub_50();
reg_tmp4.x = dot_s(uniforms.f[14].xywz, reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15].xywz, reg_tmp6);
reg_tmp4.zw = (reg_tmp6.zwww).zw;
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_64() {
if (uniforms.b[13]) {
sub_65();
} else {
sub_68();
}
return false;
}
bool sub_65() {
bool_regs = equal(uniforms.f[95].xy, reg_tmp0.xy);
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp4.zw = (reg_tmp6.zwww).zw;
if (all(not(bool_regs))) {
sub_66();
} else {
sub_67();
}
vs_out_attr5 = reg_tmp4;
return false;
}
bool sub_66() {
reg_tmp6 = reg_tmp10;
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
reg_tmp4.z = dot_s(uniforms.f[16], reg_tmp6);
reg_tmp6.w = rcp_s(reg_tmp4.z);
reg_tmp4.xy = (mul_s(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
return false;
}
bool sub_67() {
sub_61();
reg_tmp4.x = dot_s(uniforms.f[14], reg_tmp6);
reg_tmp4.y = dot_s(uniforms.f[15], reg_tmp6);
return false;
}
bool sub_68() {
vs_out_attr5 = uniforms.f[93].xxxx;
return false;
}
bool sub_69() {
reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
if (uniforms.b[11]) {
sub_70();
} else {
sub_71();
}
return false;
}
bool sub_70() {
sub_50();
reg_tmp5.x = dot_s(uniforms.f[17].xywz, reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18].xywz, reg_tmp6);
reg_tmp5.zw = (reg_tmp6.zwww).zw;
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_71() {
if (uniforms.b[14]) {
sub_72();
} else {
sub_73();
}
return false;
}
bool sub_72() {
reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
reg_tmp5.zw = (reg_tmp6.zwww).zw;
sub_61();
reg_tmp5.x = dot_s(uniforms.f[17], reg_tmp6);
reg_tmp5.y = dot_s(uniforms.f[18], reg_tmp6);
vs_out_attr6 = reg_tmp5;
return false;
}
bool sub_73() {
vs_out_attr6 = uniforms.f[93].xxxx;
return false;
}
// reference: A8406484DB2D8FA7, 52440BFACA1196F5
// shader: 8B30, 9F6EC010718A231A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (rounded_primary_color.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F55886C750C981AC, 9F6EC010718A231A
// program: 52440BFACA1196F5, EADA2116091C5E01, 9F6EC010718A231A
// shader: 8DD9, 31CD851CDF77D5BE

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

    vec4 vtx_color = vec4(0.0, 0.0, 0.0, 0.0);
    primary_color = clamp(vtx_color, vec4(0), vec4(1));

    texcoord0 = vec2(vtx.attributes[3].x, vtx.attributes[3].y);
    texcoord1 = vec2(vtx.attributes[3].z, vtx.attributes[3].w);

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
// reference: 1EA377C1D7B39071, 31CD851CDF77D5BE
// shader: 8B31, 8829BDC547BF937D

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

void main() {
vs_out_attr0 = vec4(0, 0, 0, 1);
vs_out_attr1 = vec4(0, 0, 0, 1);
vs_out_attr2 = vec4(0, 0, 0, 1);
vs_out_attr3 = vec4(0, 0, 0, 1);
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

bool sub_1();
bool sub_2();
bool sub_3();
bool sub_0();

bool exec_shader() {
sub_0();
return true;
}

bool sub_1() {
reg_tmp11.x = dot_s(uniforms.f[7], vs_in_reg0);
reg_tmp11.y = dot_s(uniforms.f[8], vs_in_reg0);
reg_tmp11.z = dot_s(uniforms.f[9], vs_in_reg0);
reg_tmp11.w = (uniforms.f[93].yyyy).w;
reg_tmp13.x = dot_s(uniforms.f[4], reg_tmp11);
reg_tmp13.y = dot_s(uniforms.f[5], reg_tmp11);
reg_tmp13.z = dot_s(uniforms.f[6], reg_tmp11);
reg_tmp13.w = (uniforms.f[93].yyyy).w;
vs_out_attr2 = -reg_tmp13;
reg_tmp14.x = dot_3(uniforms.f[10].xyz, vs_in_reg1.xyz);
reg_tmp14.y = dot_3(uniforms.f[11].xyz, vs_in_reg1.xyz);
reg_tmp14.z = dot_3(uniforms.f[12].xyz, vs_in_reg1.xyz);
vs_out_attr0.x = dot_s(uniforms.f[0], reg_tmp13);
vs_out_attr0.y = dot_s(uniforms.f[1], reg_tmp13);
vs_out_attr0.z = dot_s(uniforms.f[2], reg_tmp13);
vs_out_attr0.w = dot_s(uniforms.f[3], reg_tmp13);
reg_tmp3.x = dot_3(reg_tmp14.xyz, reg_tmp14.xyz);
reg_tmp3.x = rsq_s(reg_tmp3.x);
reg_tmp14 = mul_s(reg_tmp14, reg_tmp3.xxxx);
reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
reg_tmp4 = mul_s(uniforms.f[94].zzzz, reg_tmp4);
bool_regs = greaterThanEqual(uniforms.f[93].xx, reg_tmp4.xx);
vs_out_attr1.w = (uniforms.f[93].xxxx).w;
reg_tmp4 = vec4(rsq_s(reg_tmp4.x));
reg_tmp5 = mul_s(uniforms.f[94].zzzz, reg_tmp14);
if (!bool_regs.x) {
sub_2();
} else {
sub_3();
}
return false;
}
bool sub_2() {
vs_out_attr1.z = rcp_s(reg_tmp4.x);
vs_out_attr1.xy = (mul_s(reg_tmp5, reg_tmp4)).xy;
return false;
}
bool sub_3() {
vs_out_attr1.xyz = (uniforms.f[93].yxxx).xyz;
return false;
}
bool sub_0() {
sub_1();
reg_tmp15.x = dot_s(uniforms.f[13], vs_in_reg2);
reg_tmp15.y = dot_s(uniforms.f[14], vs_in_reg2);
vs_out_attr3 = reg_tmp15.xyxy;
return true;
}
// reference: BFBC0358DC4A4A4E, 8829BDC547BF937D
// shader: 8B30, 2CD18AA564EF347C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 419D1B64072C6B48, 2CD18AA564EF347C
// program: 8829BDC547BF937D, 31CD851CDF77D5BE, 2CD18AA564EF347C
// shader: 8B30, B747557A3AFBFB36
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;

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
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 419D1B64A0D33D15, B747557A3AFBFB36
// program: 8829BDC547BF937D, 31CD851CDF77D5BE, B747557A3AFBFB36
// shader: 8B30, F006E277F366183E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E06C19B2ADC6A082, F006E277F366183E
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, F006E277F366183E
// shader: 8B30, 3EB24E39396E08FC
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C5A0DE99A91E7099, 3EB24E39396E08FC
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 3EB24E39396E08FC
// shader: 8B30, ED21357AF1EDA466
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].aaa) * (last_tex_env_out.rgb) + (const_color[3].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (rounded_primary_color.rgb) + (const_color[4].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 822C02BBA95438F9, ED21357AF1EDA466
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, ED21357AF1EDA466
// shader: 8B30, BEE153178DF520CB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7B8CB607AC23FA06, BEE153178DF520CB
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, BEE153178DF520CB
// shader: 8B30, D63F00C62294FF13
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5E40712CA8FB2A1D, D63F00C62294FF13
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, D63F00C62294FF13
// shader: 8B30, 8364F71B23C00351
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor0.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (last_tex_env_out.rgb) + (const_color[4].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 73F8AB156AE6C923, 8364F71B23C00351
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 8364F71B23C00351
// shader: 8B30, 65820E2E46864C85
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 7B8CB60782C71407, 65820E2E46864C85
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 65820E2E46864C85
// shader: 8B30, F42D82E6FE2CA58C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (const_color[4].aaa) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A7B5D49C45B2ED3F, F42D82E6FE2CA58C
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, F42D82E6FE2CA58C
// shader: 8B30, 24E6EA03C6D935BD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EF5B24FDCF51392A, 24E6EA03C6D935BD
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 24E6EA03C6D935BD
// shader: 8B30, 4C38B9D269B8EA65
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CA97E3D6CB89E931, 4C38B9D269B8EA65
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 4C38B9D269B8EA65
// shader: 8B30, 3AFDBD40A63C9F88
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (const_color[4].aaa) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A7B5D49CF151A2E2, 3AFDBD40A63C9F88
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 3AFDBD40A63C9F88
// shader: 8B30, 895F4B859ABD6DEB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (const_color[3].rgb);
float alpha_output_3 = byteround(clamp((texcolor0.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (texcolor1.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 768CFD18D140E11C, 895F4B859ABD6DEB
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 895F4B859ABD6DEB
// shader: 8B30, 1AB62049FD46252E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EF5B24FD6E2A44D6, 1AB62049FD46252E
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 1AB62049FD46252E
// shader: 8B30, 0F23B89D7FBF3485
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CA97E3D66AF294CD, 0F23B89D7FBF3485
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 0F23B89D7FBF3485
// shader: 8B30, 745D50FF937552AD
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((texcolor0.aaa) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.aaa) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].aaa), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 07D9AABD0515133A, 745D50FF937552AD
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 745D50FF937552AD
// shader: 8B30, A88F5F0D7EE5D501
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 761686820645DAD1, A88F5F0D7EE5D501
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, A88F5F0D7EE5D501
// shader: 8B30, 5CB791AD4554E255
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 53DA41A9029D0ACA, 5CB791AD4554E255
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 5CB791AD4554E255
// shader: 8B30, 156516620C800BD9
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (const_color[4].aaa) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5BB65DD3047A0A2C, 156516620C800BD9
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 156516620C800BD9
// shader: 8B30, ED74BA02A2244293
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1358ADB2A975923F, ED74BA02A2244293
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, ED74BA02A2244293
// shader: 8B30, 194C74A2999575C7
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 36946A99ADAD4224, 194C74A2999575C7
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 194C74A2999575C7
// shader: 8B30, 1146DD78C9AAF65A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].aaa) * (last_tex_env_out.rgb) + (const_color[3].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (rounded_primary_color.rgb) + (const_color[4].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0196344D4396329D, 1146DD78C9AAF65A
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 1146DD78C9AAF65A
// shader: 8B30, BF54A667FF01792F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = byteround(clamp((texcolor0.a) * (rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F83680F146E1F062, BF54A667FF01792F
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, BF54A667FF01792F
// shader: 8B30, 95188FC40F1ADC98
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((secondary_fragment_color.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 43C29FDDAED393AB, 95188FC40F1ADC98
// program: 52440BFACA1196F5, EADA2116091C5E01, 95188FC40F1ADC98
// shader: 8B30, 377BC4AFF8F93784
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 4EE11CF25D6CE9C7, 377BC4AFF8F93784
// program: 52440BFACA1196F5, EADA2116091C5E01, 377BC4AFF8F93784
// reference: 405F715365C8CAC7, 2CD18AA564EF347C
// reference: 405F7153C2379C9A, B747557A3AFBFB36
// shader: 8B30, 87A3924880102DB5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor0.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (last_tex_env_out.rgb) + (const_color[4].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E81804A01FED52EA, 87A3924880102DB5
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 87A3924880102DB5
// shader: 8B30, 9CACA2A2853BE56A
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E06C19B2F7CC8FCE, 9CACA2A2853BE56A
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 9CACA2A2853BE56A
// shader: 8B30, ADF48CD4DFC591C7
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor0.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (last_tex_env_out.rgb) + (const_color[4].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3EE0778B91A28521, ADF48CD4DFC591C7
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, ADF48CD4DFC591C7
// shader: 8B30, A7031AB4EDA1ABDF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 36946A9979835805, A7031AB4EDA1ABDF
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, A7031AB4EDA1ABDF
// shader: 8B30, EB80AED10C800BD9
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (const_color[4].aaa) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5BB65DD3EB2B2EDB, EB80AED10C800BD9
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, EB80AED10C800BD9
// shader: 8B30, E8ACF1FFA2244293
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1358ADB261C8FACE, E8ACF1FFA2244293
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, E8ACF1FFA2244293
// shader: 8B30, 1C943F5F999575C7
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((const_color[2].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 36946A9965102AD5, 1C943F5F999575C7
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 1C943F5F999575C7
// shader: 8B30, 6DAE12BCF7649C38
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((rounded_primary_color.rgb) * (const_color[4].aaa) + (const_color[4].rgb), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (texcolor1.rgb) + (combiner_buffer.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 5BB65DD35FC86106, 6DAE12BCF7649C38
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 6DAE12BCF7649C38
// shader: 8B30, F9A57211B4D857A8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = (const_color[3].rgb);
float alpha_output_3 = byteround(clamp((texcolor0.a) * (texcolor2.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (texcolor1.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (const_color[4].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) + (const_color[5].aaa), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((last_tex_env_out.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a == alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8A8F74577FD922F8, F9A57211B4D857A8
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, F9A57211B4D857A8
// shader: 8B30, D8E603F31E154C4C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 1358ADB2C0B38732, D8E603F31E154C4C
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, D8E603F31E154C4C
// shader: 8B30, 8E9A706BA10CFCE8
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))) + tex_lod_bias[2]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (rounded_primary_color.rgb);
float alpha_output_0 = byteround(clamp(min((texcolor2.a) + (texcolor0.a), 1.0) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = byteround(clamp((texcolor2.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[2].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 36946A99C46B5729, 8E9A706BA10CFCE8
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 8E9A706BA10CFCE8
// shader: 8B30, 5B47095775F5DC1B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp(min((const_color[0].a) + (const_color[0].a), 1.0) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (1.0 - last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (last_tex_env_out.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E0B667D809D9BD65, 5B47095775F5DC1B
// program: 0000000000000000, 0000000000000000, 5B47095775F5DC1B
// shader: 8B30, E6F172AA0216CCC4
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 39BB2F11B589F62F, E6F172AA0216CCC4
// program: 0000000000000000, 0000000000000000, E6F172AA0216CCC4
// shader: 8B30, 10DD52F8A62E8324
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0E28CA30CD80B482, 10DD52F8A62E8324
// program: 0000000000000000, 0000000000000000, 10DD52F8A62E8324
// shader: 8B30, 5402CA805591DAB5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.ggg) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.bbb) * (const_color[2].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 94D5E73AE0315744, 5402CA805591DAB5
// program: 0000000000000000, 0000000000000000, 5402CA805591DAB5
// shader: 8B30, CDA943BC9C114C05
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 39BB2F113BD61383, CDA943BC9C114C05
// program: 0000000000000000, 0000000000000000, CDA943BC9C114C05
// shader: 8B30, 16AB2837E8E48C57
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0E28CA309C2945DE, 16AB2837E8E48C57
// program: 0000000000000000, 0000000000000000, 16AB2837E8E48C57
// shader: 8B30, FBE526AFCA27A27B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 8B156357A4D18B54, FBE526AFCA27A27B
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, FBE526AFCA27A27B
// shader: 8B30, 44FEA035B19A2228
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E06C19B2AE244730, 44FEA035B19A2228
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 44FEA035B19A2228
// shader: 8B30, 31CC6442837C3CF5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C5A0DE99F3145FD5, 31CC6442837C3CF5
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 31CC6442837C3CF5
// shader: 8B30, 04105FA405030B40
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[3].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((texcolor1.rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: BA04AC880963612A, 04105FA405030B40
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 04105FA405030B40
// shader: 8B30, EC5A2B7D3857BAD3
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A1B38FD13855B37A, EC5A2B7D3857BAD3
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, EC5A2B7D3857BAD3
// shader: 8B30, D56B5462A2B11E54
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A1B38FD13BB754C8, D56B5462A2B11E54
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, D56B5462A2B11E54
// shader: 8B30, 1862E5DD03E68D87
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 847F48FA3C8D6361, 1862E5DD03E68D87
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 1862E5DD03E68D87
// shader: 8B30, 4EE8E959DB99620C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (texcolor0.rgb) + (combiner_buffer.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((texcolor1.rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 81565990C78E5E01, 4EE8E959DB99620C
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 4EE8E959DB99620C
// shader: 8B30, 4827A0ADCE6CCB09
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 847F48FAFCCBB40D, 4827A0ADCE6CCB09
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 4827A0ADCE6CCB09
// shader: 8B30, 4B641AA5F5C73375
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A1B38FD1F8136416, 4B641AA5F5C73375
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 4B641AA5F5C73375
// shader: 8B30, 443C52090992F092
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[1].rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A1B38FD1FBF183A4, 443C52090992F092
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 443C52090992F092
// shader: 8B30, F0F940ED00EBA113
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);

vec4 diffuse_sum = vec4(0, 0, 0, 1);
vec4 specular_sum = vec4(0, 0, 0, 1);
vec3 light_vector = vec3(0);
vec3 refl_value = vec3(0);
vec3 spot_dir = vec3(0);
vec3 half_vector = vec3(0);
float dot_product = 0.0;
float clamp_highlights = 1.0;
float geo_factor = 1.0;
vec3 surface_normal = 2.0 * (texcolor1).rgb - 1.0;
surface_normal.z = sqrt(max((1.0 - (surface_normal.x*surface_normal.x + surface_normal.y*surface_normal.y)), 0.0));
vec3 surface_tangent = vec3(1, 0, 0);
vec4 normalized_normquat = normalize(normquat);
vec3 normal = QuaternionRotate(normalized_normquat, surface_normal);
vec3 tangent = QuaternionRotate(normalized_normquat, surface_tangent);
vec4 shadow = vec4(1);
light_vector = normalize(light_src[0].position);
spot_dir = light_src[0].spot_direction;
half_vector = normalize(view) + light_vector;
dot_product = max(dot(light_vector, normal), 0.0);
refl_value.r = 1.0;
refl_value.g = refl_value.r;
refl_value.b = refl_value.r;
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0), vec4(1));
secondary_fragment_color = clamp(specular_sum, vec4(0), vec4(1));
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[1].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((vec3(1) - primary_fragment_color.rgb) * (const_color[2].rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((vec3(1) - last_tex_env_out.rgb) * (combiner_buffer.rgb) + (secondary_fragment_color.rgb), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[4].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 85F0FDF3F05E428D, F0F940ED00EBA113
// program: 52440BFACA1196F5, EADA2116091C5E01, F0F940ED00EBA113
// shader: 8B30, B3363ED47A050735
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (vec3(1) - rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_1 = (const_color[1].rgb);
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (combiner_buffer.rgb) + (const_color[2].rgb) * (vec3(1) - (combiner_buffer.rgb)), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (const_color[3].rgb) * (vec3(1) - (rounded_primary_color.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 16D5DDDFB4A9A8B7, B3363ED47A050735
// program: 52440BFACA1196F5, EADA2116091C5E01, B3363ED47A050735
// shader: 8B30, 30562984F927BE1B
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EB98A8DCE305E6C0, 30562984F927BE1B
// program: 52440BFACA1196F5, EADA2116091C5E01, 30562984F927BE1B
// shader: 8B30, C3449630C5D64261
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: DC0B4DFDE305E6C0, C3449630C5D64261
// program: 52440BFACA1196F5, EADA2116091C5E01, C3449630C5D64261
// shader: 8B30, C29B7BCC6897DA8F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C16D5E74D9E83188, C29B7BCC6897DA8F
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, C29B7BCC6897DA8F
// shader: 8B30, 777CA2E80FC38378
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C16D5E74DA0AD63A, 777CA2E80FC38378
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 777CA2E80FC38378
// shader: 8B30, 42D4CFE8930F66F2
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E4A1995FDD30E193, 42D4CFE8930F66F2
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 42D4CFE8930F66F2
// shader: 8B30, D03B9170DB68ED0E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor0.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (last_tex_env_out.rgb) + (const_color[4].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C9194366D79388AA, D03B9170DB68ED0E
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, D03B9170DB68ED0E
// shader: 8B30, E56A5910169BE1DB
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C16D5E743FB2558E, E56A5910169BE1DB
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, E56A5910169BE1DB
// shader: 8B30, 6B525FD63E757FDA
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E4A1995F3B6A8595, 6B525FD63E757FDA
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 6B525FD63E757FDA
// shader: 8B30, BB87DBD1DF1E2796
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.rgb) + (const_color[3].rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((texcolor1.rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F5F54D916D463A38, BB87DBD1DF1E2796
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, BB87DBD1DF1E2796
// shader: 8B30, 08205604ED2179C5
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE426EC85C70E868, 08205604ED2179C5
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 08205604ED2179C5
// shader: 8B30, 60B4656910A3AF91
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE426EC85F920FDA, 60B4656910A3AF91
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 60B4656910A3AF91
// shader: 8B30, 02A8639059CF3B22
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (const_color[2].rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CB8EA9E358A83873, 02A8639059CF3B22
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 02A8639059CF3B22
// shader: 8B30, BA738E186676612D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (texcolor0.rgb) + (combiner_buffer.rgb) * (vec3(1) - (texcolor0.rgb)), vec3(0), vec3(1)));
float alpha_output_3 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (texcolor0.aaa), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((texcolor1.rgb) * (const_color[5].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CEA7B889A3AB0513, BA738E186676612D
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, BA738E186676612D
// shader: 8B30, BD89AF7CFED2C64F
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CB8EA9E398EEEF1F, BD89AF7CFED2C64F
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, BD89AF7CFED2C64F
// shader: 8B30, 3DC61B58054A7A32
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE426EC89C363F04, 3DC61B58054A7A32
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 3DC61B58054A7A32
// shader: 8B30, AC8ECEB40F87D56D
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE426EC89FD4D8B6, AC8ECEB40F87D56D
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, AC8ECEB40F87D56D
// shader: 8B30, 0E0C2F054377CD32
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CB8EA9E39B0C08AD, 0E0C2F054377CD32
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 0E0C2F054377CD32
// shader: 8B30, FD53C3DB01C101AE
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((texcolor0.rgb) + (rounded_primary_color.aaa), vec3(0), vec3(1)));
float alpha_output_3 = (texcolor0.a);
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].aaa) * (last_tex_env_out.rgb) + (const_color[4].rgb) * (vec3(1) - (last_tex_env_out.rgb)), vec3(0), vec3(1)));
float alpha_output_4 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0), vec3(1)));
float alpha_output_5 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C3FAB4F19695563D, FD53C3DB01C101AE
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, FD53C3DB01C101AE
// shader: 8B30, E2549797DFFA5047
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1) - texcolor0.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: CB8EA9E37EB48B19, E2549797DFFA5047
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, E2549797DFFA5047
// shader: 8B30, 6C6C9151F714CE46
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))) + tex_lod_bias[1]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.a = last_tex_env_out.a;
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (texcolor0.rgb);
float alpha_output_2 = byteround(clamp((texcolor0.a) + (texcolor1.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((vec3(1) - texcolor0.rgb) * (const_color[3].rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
next_combiner_buffer.rgb = last_tex_env_out.rgb;
vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EE426EC87A6C5B02, 6C6C9151F714CE46
// program: 3AE9AAE877A9E01A, B6B95AFD9466EC70, 6C6C9151F714CE46
// shader: 8B30, 97B1ECD7C578213E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = byteround(clamp((const_color[0].a) * (const_color[0].a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_2 = (last_tex_env_out.rgb);
float alpha_output_2 = byteround(clamp((last_tex_env_out.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) * (vec3(1) - last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a) * (1.0 - last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_3, alpha_output_3);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (last_tex_env_out.rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a) * (last_tex_env_out.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = vec4(color_output_4, alpha_output_4);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0EDD768FB589F62F, 97B1ECD7C578213E
// program: 0000000000000000, 0000000000000000, 97B1ECD7C578213E
// shader: 8B30, 2309FC9374EE2A0C
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3660884ECD80B482, 2309FC9374EE2A0C
// program: 0000000000000000, 0000000000000000, 2309FC9374EE2A0C
// shader: 8B30, 1D4B033B050A9965
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = byteround(clamp((texcolor0.rrr) * (const_color[0].rgb), vec3(0), vec3(1)));
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_1 = byteround(clamp((texcolor0.ggg) * (const_color[1].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_1 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_1, alpha_output_1);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
vec3 color_output_2 = byteround(clamp((texcolor0.bbb) * (const_color[2].rgb) + (last_tex_env_out.rgb), vec3(0), vec3(1)));
float alpha_output_2 = (last_tex_env_out.a);
last_tex_env_out = vec4(color_output_2, alpha_output_2);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AC9DA544E0315744, 1D4B033B050A9965
// program: 0000000000000000, 0000000000000000, 1D4B033B050A9965
// shader: 8B30, 70CF1DF725C8A199
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 01F36D6F3BD61383, 70CF1DF725C8A199
// program: 0000000000000000, 0000000000000000, 70CF1DF725C8A199
// shader: 8B30, 90FD149F4E7B1C2E
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 texcolor0 = textureLod(tex0, texcoord0, getLod(texcoord0 * vec2(textureSize(tex0, 0))) + tex_lod_bias[0]);
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (texcolor0.rgb);
float alpha_output_0 = (texcolor0.a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
if (last_tex_env_out.a <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3660884E9C2945DE, 90FD149F4E7B1C2E
// program: 0000000000000000, 0000000000000000, 90FD149F4E7B1C2E
// shader: 8B30, BD193A6F0A0BABFF
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
if (gl_FragCoord.x < float(scissor_x1) || gl_FragCoord.y < float(scissor_y1) || gl_FragCoord.x >= float(scissor_x2) || gl_FragCoord.y >= float(scissor_y2)) discard;
float z_over_w = 2.0 * gl_FragCoord.z - 1.0;
float depth = z_over_w * depth_scale + depth_offset;
vec4 combiner_buffer = vec4(0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0);
vec3 color_output_0 = (const_color[0].rgb);
float alpha_output_0 = (const_color[0].a);
last_tex_env_out = vec4(color_output_0, alpha_output_0);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
combiner_buffer = next_combiner_buffer;
vec3 color_output_5 = (const_color[5].rgb);
float alpha_output_5 = (const_color[5].a);
last_tex_env_out = vec4(color_output_5, alpha_output_5);
last_tex_env_out = clamp(last_tex_env_out, vec4(0), vec4(1));
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 01F36D6FB589F62F, BD193A6F0A0BABFF
// program: 0000000000000000, 0000000000000000, BD193A6F0A0BABFF
