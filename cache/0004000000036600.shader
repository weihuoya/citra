// shader: 8B31, 8F6D6A21B1F5C895

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

#define fma_safe(x, y, z) fma(x, y, z)
#define mul_safe(x, y) (x * y)
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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
bool sub_22_27();
bool sub_33_35();
bool sub_35_46();
bool sub_38_39();
bool sub_40_41();
bool sub_42_43();
bool sub_44_45();
bool sub_48_51();
bool sub_52_55();
bool sub_56_59();
bool sub_60_63();
bool sub_65_99();
bool sub_67_86();
bool sub_70_73();
bool sub_74_77();
bool sub_78_81();
bool sub_82_85();
bool sub_88_98();
bool sub_90_93();
bool sub_94_97();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: mova
    address_registers.x = (ivec2(vs_in_reg0)).x;
    // 1: mov
    reg_tmp0 = uniforms.f[6 + address_registers.x].wzyx;
    // 2: mov
    reg_tmp1.xy = (vs_in_reg0.zwzw).xy;
    // 3: mov
    reg_tmp1.zw = (uniforms.f[5].xyxy).zw;
    // 4: mova
    address_registers.xy = ivec2(reg_tmp0.xyyy);
    // 5: mov
    reg_tmp2.xw = (uniforms.f[64 + address_registers.y].wwyy).xw;
    // 6: mov
    reg_tmp2.yz = (uniforms.f[5].xxxx).yz;
    // 7: dp4
    reg_tmp4.x = dot(reg_tmp1, reg_tmp2);
    // 8: mov
    reg_tmp2.yw = (uniforms.f[64 + address_registers.y].zzxx).yw;
    // 9: mov
    reg_tmp2.xz = (uniforms.f[5].xxxx).xz;
    // 10: dp4
    reg_tmp4.y = dot(reg_tmp1, reg_tmp2);
    // 11: mov
    reg_tmp4.zw = (reg_tmp1.zwzw).zw;
    // 12: dp4
    reg_tmp3.x = dot(uniforms.f[32 + address_registers.x].wzyx, reg_tmp4);
    // 13: dp4
    reg_tmp3.y = dot(uniforms.f[33 + address_registers.x].wzyx, reg_tmp4);
    // 14: dp4
    reg_tmp3.z = dot(uniforms.f[34 + address_registers.x].wzyx, reg_tmp4);
    // 15: mov
    reg_tmp3.w = (reg_tmp1.wwww).w;
    // 16: mov
    reg_tmp4.z = (uniforms.f[34 + address_registers.x].xxxx).z;
    // 17: max
    reg_tmp4.z = (max(reg_tmp4.zzzz, -reg_tmp4.zzzz)).z;
    // 18: add
    reg_tmp4.z = (uniforms.f[4].yyyy + reg_tmp4.zzzz).z;
    // 19: mov
    reg_tmp4.x = (uniforms.f[4].wwww).x;
    // 20: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(reg_tmp4.xzzz));
    // 21: ifc
    if (all(conditional_code)) {
        sub_22_27();
    }
    // 27: dp4
    vs_out_attr0.x = dot(uniforms.f[0].wzyx, reg_tmp3);
    // 28: dp4
    vs_out_attr0.y = dot(uniforms.f[1].wzyx, reg_tmp3);
    // 29: dp4
    vs_out_attr0.z = dot(uniforms.f[2].wzyx, reg_tmp3);
    // 30: dp4
    vs_out_attr0.w = dot(uniforms.f[3].wzyx, reg_tmp3);
    // 31: cmp
    conditional_code = greaterThanEqual(vec2(uniforms.f[5].yyyy), vec2(reg_tmp0.wwww));
    // 32: ifc
    if (all(conditional_code)) {
        sub_33_35();
    } else {
        sub_35_46();
    }
    // 46: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(reg_tmp1.xyyy));
    // 47: ifc
    if (all(not(conditional_code))) {
        sub_48_51();
    }
    // 51: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_52_55();
    }
    // 55: ifc
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_56_59();
    }
    // 59: ifc
    if (all(conditional_code)) {
        sub_60_63();
    }
    // 63: mov
    reg_tmp8 = uniforms.f[5].xxxx;
    // 64: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop64 = 0u; loop64 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop64) {
        sub_65_99();
    }
    // 99: mov
    vs_out_attr2 = reg_tmp5;
    // 100: mov
    vs_out_attr3 = reg_tmp6;
    // 101: mov
    vs_out_attr4 = reg_tmp7;
    // 102: end
    return true;
}
bool sub_22_27() {
    // 22: mov
    reg_tmp4.x = (uniforms.f[4].wwww).x;
    // 23: add
    reg_tmp4.y = (-uniforms.f[4].zzzz + reg_tmp4.zzzz).y;
    // 24: rcp
    reg_tmp4.z = rcp_safe(reg_tmp4.zzzz.x);
    // 25: mul
    reg_tmp4.z = (mul_safe(reg_tmp4.yyyy, reg_tmp4.zzzz)).z;
    // 26: mad
    reg_tmp3.x = (fma_safe(reg_tmp4.xxxx, reg_tmp4.zzzz, reg_tmp3.xxxx)).x;
    return false;
}
bool sub_33_35() {
    // 33: mov
    vs_out_attr1.xyz = (uniforms.f[5].yyyy).xyz;
    // 34: mov
    vs_out_attr1.w = (reg_tmp0.wwww).w;
    return false;
}
bool sub_35_46() {
    // 35: mova
    address_registers.y = (ivec2(reg_tmp0.wwww)).y;
    // 36: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(reg_tmp1.xyyy));
    // 37: ifc
    if (all(not(conditional_code))) {
        sub_38_39();
    }
    // 39: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_40_41();
    }
    // 41: ifc
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_42_43();
    }
    // 43: ifc
    if (all(conditional_code)) {
        sub_44_45();
    }
    // 45: nop
    return false;
}
bool sub_38_39() {
    // 38: mov
    vs_out_attr1 = uniforms.f[32 + address_registers.y].wzyx;
    return false;
}
bool sub_40_41() {
    // 40: mov
    vs_out_attr1 = uniforms.f[33 + address_registers.y].wzyx;
    return false;
}
bool sub_42_43() {
    // 42: mov
    vs_out_attr1 = uniforms.f[34 + address_registers.y].wzyx;
    return false;
}
bool sub_44_45() {
    // 44: mov
    vs_out_attr1 = uniforms.f[35 + address_registers.y].wzyx;
    return false;
}
bool sub_48_51() {
    // 48: mov
    reg_tmp5 = uniforms.f[5].xyyy;
    // 49: mov
    reg_tmp6 = uniforms.f[5].xyyy;
    // 50: mov
    reg_tmp7 = uniforms.f[5].xyyy;
    return false;
}
bool sub_52_55() {
    // 52: mov
    reg_tmp5 = uniforms.f[5].yyyy;
    // 53: mov
    reg_tmp6 = uniforms.f[5].yyyy;
    // 54: mov
    reg_tmp7 = uniforms.f[5].yyyy;
    return false;
}
bool sub_56_59() {
    // 56: mov
    reg_tmp5 = uniforms.f[5].xxyy;
    // 57: mov
    reg_tmp6 = uniforms.f[5].xxyy;
    // 58: mov
    reg_tmp7 = uniforms.f[5].xxyy;
    return false;
}
bool sub_60_63() {
    // 60: mov
    reg_tmp5 = uniforms.f[5].yxyy;
    // 61: mov
    reg_tmp6 = uniforms.f[5].yxyy;
    // 62: mov
    reg_tmp7 = uniforms.f[5].yxyy;
    return false;
}
bool sub_65_99() {
    // 65: cmp
    conditional_code = equal(vec2(uniforms.f[5].yyyy), vec2(reg_tmp8));
    // 66: ifc
    if (all(conditional_code)) {
        sub_67_86();
    }
    // 86: cmp
    conditional_code = lessThan(vec2(uniforms.f[5].wwww), vec2(reg_tmp8));
    // 87: ifc
    if (all(conditional_code)) {
        sub_88_98();
    }
    // 98: add
    reg_tmp8 = uniforms.f[5].yyyy + reg_tmp8;
    return false;
}
bool sub_67_86() {
    // 67: mova
    address_registers.y = (ivec2(reg_tmp0.zzzz)).y;
    // 68: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(reg_tmp1.xyyy));
    // 69: ifc
    if (all(not(conditional_code))) {
        sub_70_73();
    }
    // 73: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_74_77();
    }
    // 77: ifc
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_78_81();
    }
    // 81: ifc
    if (all(conditional_code)) {
        sub_82_85();
    }
    // 85: nop
    return false;
}
bool sub_70_73() {
    // 70: mov
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wzzz).xy;
    // 71: mov
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wzzz).xy;
    // 72: mov
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wzzz).xy;
    return false;
}
bool sub_74_77() {
    // 74: mov
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yzzz).xy;
    // 75: mov
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yzzz).xy;
    // 76: mov
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yzzz).xy;
    return false;
}
bool sub_78_81() {
    // 78: mov
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].wxxx).xy;
    // 79: mov
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].wxxx).xy;
    // 80: mov
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].wxxx).xy;
    return false;
}
bool sub_82_85() {
    // 82: mov
    reg_tmp5.xy = (uniforms.f[64 + address_registers.y].yxxx).xy;
    // 83: mov
    reg_tmp6.xy = (uniforms.f[65 + address_registers.y].yxxx).xy;
    // 84: mov
    reg_tmp7.xy = (uniforms.f[66 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_88_98() {
    // 88: cmp
    conditional_code = notEqual(vec2(uniforms.f[5].xxxx), vec2(reg_tmp1.xyyy));
    // 89: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_90_93();
    }
    // 93: ifc
    if (all(bvec2(!conditional_code.x, conditional_code.y))) {
        sub_94_97();
    }
    // 97: nop
    return false;
}
bool sub_90_93() {
    // 90: mov
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].yxxx).xy;
    // 91: mov
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].yxxx).xy;
    // 92: mov
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].yxxx).xy;
    return false;
}
bool sub_94_97() {
    // 94: mov
    reg_tmp5.xy = (uniforms.f[67 + address_registers.y].wzzz).xy;
    // 95: mov
    reg_tmp6.xy = (uniforms.f[68 + address_registers.y].wzzz).xy;
    // 96: mov
    reg_tmp7.xy = (uniforms.f[69 + address_registers.y].wzzz).xy;
    return false;
}
// reference: F2DBBDF0B2B33B49, 8F6D6A21B1F5C895
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
// shader: 8B30, CD4515CCFFE8D17C

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2EA60C87BBA2AA7E, CD4515CCFFE8D17C
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, CD4515CCFFE8D17C
// reference: 6B8EC5B1B2B33B49, 8F6D6A21B1F5C895
// shader: 8B30, A5031CF3136200AA

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 74081FC0DB8E6A89, A5031CF3136200AA
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A5031CF3136200AA
// shader: 8B30, C11AFEE305B9C79D

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E13EE8783A5DBCC4, C11AFEE305B9C79D
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, C11AFEE305B9C79D
// shader: 8B30, A1C568C5933B2AE7

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2EA60C87F55A2364, A1C568C5933B2AE7
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A1C568C5933B2AE7
// shader: 8B30, 757BBA2E095000A6

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor1.rgb) + (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((texcolor1.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((const_color[5].a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: ACB0198B24A06AD5, 757BBA2E095000A6
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 757BBA2E095000A6
// shader: 8B30, A5031CF324845C13

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) * (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 437E085EDB8E6A89, A5031CF324845C13
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A5031CF324845C13
// shader: 8B30, 931FD46CC3EE4FFE

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((texcolor0.rgb) * (const_color[3].aaa) + (texcolor1.rgb) * (vec3(1.0) - (const_color[3].aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a) + (texcolor1.a) * (1.0 - (const_color[3].a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (last_tex_env_out.rgb) + (combiner_buffer.rgb) * (vec3(1.0) - (last_tex_env_out.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (last_tex_env_out.a) + (combiner_buffer.a) * (1.0 - (last_tex_env_out.a)), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C2474A94CBCD3FBE, 931FD46CC3EE4FFE
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 931FD46CC3EE4FFE
// shader: 8B31, DE3624946DADEE51

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

#define fma_safe(x, y, z) fma(x, y, z)
#define mul_safe(x, y) (x * y)
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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
bool sub_34_202();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_201();
bool sub_143_153();
bool sub_153_158();
bool sub_159_171();
bool sub_171_196();
bool sub_172_190();
bool sub_190_195();
bool sub_202_219();
bool sub_219_295();
bool sub_258_278();
bool sub_259_264();
bool sub_264_277();
bool sub_268_271();
bool sub_271_276();
bool sub_278_291();
bool sub_279_284();
bool sub_284_290();
bool sub_295_312();
bool sub_301_306();
bool sub_303_304();
bool sub_310_311();
bool sub_312_358();
bool sub_316_357();
bool sub_321_323();
bool sub_323_347();
bool sub_326_331();
bool sub_337_346();
bool sub_341_342();
bool sub_342_345();
bool sub_349_356();
bool sub_358_368();
bool sub_365_366();
bool sub_368_395();
bool sub_370_375();
bool sub_375_394();
bool sub_378_384();
bool sub_384_393();
bool sub_385_389();
bool sub_389_392();
bool sub_395_403();
bool sub_397_398();
bool sub_398_402();
bool sub_399_400();
bool sub_400_401();
bool sub_403_410();
bool sub_410_414();
bool sub_414_440();
bool sub_416_421();
bool sub_421_439();
bool sub_422_437();
bool sub_426_433();
bool sub_433_436();
bool sub_437_438();
bool sub_440_457();
bool sub_442_447();
bool sub_447_456();
bool sub_448_454();
bool sub_454_455();

bool exec_shader() {
    sub_0_4096();
    return true;
}

bool sub_0_4096() {
    // 0: call
    {
        sub_34_202();
    }
    // 1: call
    {
        sub_295_312();
    }
    // 2: call
    {
        sub_368_395();
    }
    // 3: call
    {
        sub_414_440();
    }
    // 4: call
    {
        sub_440_457();
    }
    // 5: end
    return true;
}
bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 17: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 18: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 26: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 27: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 28: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 29: dp3
    reg_tmp5.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 30: dp3
    reg_tmp5.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_202() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_201();
    }
    // 201: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 72: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 73: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 74: call
    {
        sub_202_219();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
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
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 98: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 99: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 100: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 101: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 102: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 103: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_219_295();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_201() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 142: ifu
    if (uniforms.b[2]) {
        sub_143_153();
    } else {
        sub_153_158();
    }
    // 158: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_159_171();
    } else {
        sub_171_196();
    }
    // 196: mov
    vs_out_attr2 = -reg_tmp15;
    // 197: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 198: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 199: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 200: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_143_153() {
    // 143: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 144: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 145: dp4
    reg_tmp7.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 146: dp4
    reg_tmp7.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 147: dp4
    reg_tmp7.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 148: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 149: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 150: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 151: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 152: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_153_158() {
    // 153: mova
    address_registers.x = (ivec2(uniforms.f[93].xxxx)).x;
    // 154: dp4
    reg_tmp10.x = dot(uniforms.f[25], reg_tmp15);
    // 155: dp4
    reg_tmp10.y = dot(uniforms.f[26], reg_tmp15);
    // 156: dp4
    reg_tmp10.z = dot(uniforms.f[27], reg_tmp15);
    // 157: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_159_171() {
    // 159: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 160: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 161: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 162: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 163: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 164: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 165: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 166: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 167: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 168: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 169: call
    {
        sub_202_219();
    }
    // 170: nop
    return false;
}
bool sub_171_196() {
    // 171: ifc
    if (all(conditional_code)) {
        sub_172_190();
    } else {
        sub_190_195();
    }
    // 195: nop
    return false;
}
bool sub_172_190() {
    // 172: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 173: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 174: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 175: dp3
    reg_tmp11.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 176: dp3
    reg_tmp11.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 177: dp3
    reg_tmp11.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 178: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 179: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 180: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 181: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 182: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 183: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 184: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 185: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 186: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 187: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 188: call
    {
        sub_219_295();
    }
    // 189: nop
    return false;
}
bool sub_190_195() {
    // 190: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 191: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 192: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 193: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 194: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_202_219() {
    uint jmp_to = 202u;
    while (true) {
        switch (jmp_to) {
        case 202u: {
            // 202: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 203: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 204: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 205: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 206: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 207: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 208: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 209: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 218u; break; }
            }
            // 210: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 211: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 212: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 213: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 214: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 215: jmpc
            if (conditional_code.x) {
                { jmp_to = 218u; break; }
            }
            // 216: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 217: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 218u: {
            // 218: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_219_295() {
    uint jmp_to = 219u;
    while (true) {
        switch (jmp_to) {
        case 219u: {
            // 219: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 220: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 221: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 222: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 223: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 224: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 225: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 226: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 227: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 228: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 294u; break; }
            }
            // 229: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 230: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 231: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 232: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 233: dp3
            reg_tmp5.w = dot(vec3(reg_tmp5), vec3(reg_tmp5));
            // 234: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 235: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 236: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 237: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 238: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 239: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 240: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 241: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 242: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 243: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 244: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 245: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 246: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 247: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 248: jmpc
            if (!conditional_code.x) {
                { jmp_to = 256u; break; }
            }
            // 249: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 250: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 251: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 252: dp4
            reg_tmp6 = vec4(dot(reg_tmp7, reg_tmp7));
            // 253: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 254: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 255: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 294u; break; }
            }
        }
        case 256u: {
            // 256: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 257: ifc
            if (conditional_code.x) {
                sub_258_278();
            } else {
                sub_278_291();
            }
            // 291: dp4
            reg_tmp6 = vec4(dot(reg_tmp8, reg_tmp8));
            // 292: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 293: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 294u: {
            // 294: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_258_278() {
    // 258: ifc
    if (conditional_code.y) {
        sub_259_264();
    } else {
        sub_264_277();
    }
    // 277: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_259_264() {
    // 259: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 260: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 261: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 262: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 263: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_264_277() {
    // 264: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 265: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 266: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 267: ifc
    if (conditional_code.x) {
        sub_268_271();
    } else {
        sub_271_276();
    }
    // 276: nop
    return false;
}
bool sub_268_271() {
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_276() {
    // 271: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 272: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 273: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 274: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 275: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_278_291() {
    // 278: ifc
    if (conditional_code.y) {
        sub_279_284();
    } else {
        sub_284_290();
    }
    // 290: nop
    return false;
}
bool sub_279_284() {
    // 279: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 280: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 281: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 282: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 283: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_284_290() {
    // 284: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 285: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 286: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 287: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 288: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 289: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_295_312() {
    // 295: mov
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    // 296: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 297: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 298: mov
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    // 299: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 300: ifc
    if (conditional_code.y) {
        sub_301_306();
    }
    // 306: callu
    if (uniforms.b[12]) {
        sub_312_358();
    }
    // 307: callu
    if (uniforms.b[5]) {
        sub_358_368();
    }
    // 308: cmp
    conditional_code = equal(vec2(uniforms.f[93].xxxx), vec2(reg_tmp8.xyyy));
    // 309: ifc
    if (all(conditional_code)) {
        sub_310_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_301_306() {
    // 301: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 302: ifu
    if (uniforms.b[7]) {
        sub_303_304();
    }
    // 304: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 305: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_303_304() {
    // 303: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_310_311() {
    // 310: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_312_358() {
    // 312: mov
    reg_tmp1 = uniforms.f[20];
    // 313: mov
    reg_tmp2 = uniforms.f[21];
    // 314: mov
    reg_tmp3 = uniforms.f[93].xxxx;
    // 315: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop315 = 0u; loop315 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop315) {
        sub_316_357();
    }
    // 357: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_316_357() {
    // 316: mova
    address_registers.x = (ivec2(reg_tmp3)).x;
    // 317: mov
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    // 318: mov
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[93].xyyy), vec2(reg_tmp4.xyyy));
    // 320: ifc
    if (conditional_code.x) {
        sub_321_323();
    } else {
        sub_323_347();
    }
    // 347: cmp
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    // 348: ifc
    if (conditional_code.y) {
        sub_349_356();
    }
    // 356: add
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_321_323() {
    // 321: dp3
    reg_tmp6.x = dot(vec3(uniforms.f[81 + address_registers.x]), vec3(reg_tmp14));
    // 322: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_323_347() {
    // 323: add
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    // 324: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    // 325: ifc
    if (conditional_code.y) {
        sub_326_331();
    }
    // 331: mov
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    // 332: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.wwww));
    // 333: dp3
    reg_tmp4.w = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 334: rsq
    reg_tmp4.w = rsq_safe(reg_tmp4.wwww.x);
    // 335: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp4.wwww);
    // 336: ifc
    if (conditional_code.x) {
        sub_337_346();
    }
    // 346: dp3
    reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp4));
    return false;
}
bool sub_326_331() {
    // 326: mov
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    // 327: dp3
    reg_tmp5.z = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 328: mul
    reg_tmp5.y = (mul_safe(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    // 329: dp3
    reg_tmp6.y = dot(vec3(uniforms.f[83 + address_registers.x]), vec3(reg_tmp5));
    // 330: rcp
    reg_tmp6.y = rcp_safe(reg_tmp6.yyyy.x);
    return false;
}
bool sub_337_346() {
    // 337: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[82 + address_registers.x]), vec3(-reg_tmp4));
    // 338: slti
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx,uniforms.f[84 + address_registers.x].yyyy))).y;
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.xyyy));
    // 340: ifc
    if (conditional_code.y) {
        sub_341_342();
    } else {
        sub_342_345();
    }
    // 345: mul
    reg_tmp6.y = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_341_342() {
    // 341: mov
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_342_345() {
    // 342: log
    reg_tmp5.y = log2(reg_tmp5.xxxx.x);
    // 343: mul
    reg_tmp5.y = (mul_safe(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    // 344: exp
    reg_tmp5.x = exp2(reg_tmp5.yyyy.x);
    return false;
}
bool sub_349_356() {
    // 349: max
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    // 350: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    // 351: mul
    reg_tmp4 = mul_safe(uniforms.f[80 + address_registers.x], reg_tmp2);
    // 352: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    // 353: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    // 354: add
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    // 355: add
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_358_368() {
    // 358: dp3
    reg_tmp1 = vec4(dot(vec3(uniforms.f[24]), vec3(reg_tmp14)));
    // 359: mov
    reg_tmp2 = uniforms.f[24].wwww;
    // 360: mad
    reg_tmp1 = fma_safe(reg_tmp1, reg_tmp2, reg_tmp2);
    // 361: mov
    reg_tmp3 = uniforms.f[22];
    // 362: add
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    // 363: mad
    reg_tmp4 = fma_safe(reg_tmp2, reg_tmp1, reg_tmp3);
    // 364: ifu
    if (uniforms.b[6]) {
        sub_365_366();
    }
    // 366: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    // 367: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_365_366() {
    // 365: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_368_395() {
    // 368: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 369: ifu
    if (uniforms.b[9]) {
        sub_370_375();
    } else {
        sub_375_394();
    }
    // 394: nop
    return false;
}
bool sub_370_375() {
    // 370: call
    {
        sub_395_403();
    }
    // 371: dp4
    reg_tmp3.x = dot(uniforms.f[11].xywz, reg_tmp6);
    // 372: dp4
    reg_tmp3.y = dot(uniforms.f[12].xywz, reg_tmp6);
    // 373: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 374: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_375_394() {
    // 375: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 376: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 377: ifc
    if (all(not(conditional_code))) {
        sub_378_384();
    } else {
        sub_384_393();
    }
    // 393: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_378_384() {
    // 378: mov
    reg_tmp6 = reg_tmp10;
    // 379: dp4
    reg_tmp3.x = dot(uniforms.f[11], reg_tmp6);
    // 380: dp4
    reg_tmp3.y = dot(uniforms.f[12], reg_tmp6);
    // 381: dp4
    reg_tmp3.z = dot(uniforms.f[13], reg_tmp6);
    // 382: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 383: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_384_393() {
    // 384: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_385_389();
    } else {
        sub_389_392();
    }
    // 392: nop
    return false;
}
bool sub_385_389() {
    // 385: call
    {
        sub_403_410();
    }
    // 386: dp3
    reg_tmp3.x = dot(vec3(uniforms.f[11]), vec3(reg_tmp6));
    // 387: dp3
    reg_tmp3.y = dot(vec3(uniforms.f[12]), vec3(reg_tmp6));
    // 388: dp3
    reg_tmp3.z = dot(vec3(uniforms.f[13]), vec3(reg_tmp6));
    return false;
}
bool sub_389_392() {
    // 389: call
    {
        sub_410_414();
    }
    // 390: dp4
    reg_tmp3.x = dot(uniforms.f[11], reg_tmp6);
    // 391: dp4
    reg_tmp3.y = dot(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_395_403() {
    // 395: cmp
    conditional_code = equal(vec2(uniforms.f[93].yzzz), vec2(reg_tmp0.xyyy));
    // 396: ifc
    if (all(not(conditional_code))) {
        sub_397_398();
    } else {
        sub_398_402();
    }
    // 402: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_397_398() {
    // 397: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_398_402() {
    // 398: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_399_400();
    } else {
        sub_400_401();
    }
    // 401: nop
    return false;
}
bool sub_399_400() {
    // 399: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_400_401() {
    // 400: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_403_410() {
    // 403: mov
    reg_tmp2 = -reg_tmp15;
    // 404: dp3
    reg_tmp2.w = dot(vec3(reg_tmp2), vec3(reg_tmp2));
    // 405: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 406: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 407: dp3
    reg_tmp1 = vec4(dot(vec3(reg_tmp2), vec3(reg_tmp14)));
    // 408: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 409: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_410_414() {
    // 410: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 411: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 412: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 413: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_414_440() {
    // 414: mov
    reg_tmp0.xy = (uniforms.f[10].yyyy).xy;
    // 415: ifu
    if (uniforms.b[10]) {
        sub_416_421();
    } else {
        sub_421_439();
    }
    // 439: nop
    return false;
}
bool sub_416_421() {
    // 416: call
    {
        sub_395_403();
    }
    // 417: dp4
    reg_tmp4.x = dot(uniforms.f[14].xywz, reg_tmp6);
    // 418: dp4
    reg_tmp4.y = dot(uniforms.f[15].xywz, reg_tmp6);
    // 419: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 420: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_421_439() {
    // 421: ifu
    if (uniforms.b[13]) {
        sub_422_437();
    } else {
        sub_437_438();
    }
    // 438: nop
    return false;
}
bool sub_422_437() {
    // 422: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 423: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 424: mov
    reg_tmp4.zw = (reg_tmp6.zwww).zw;
    // 425: ifc
    if (all(not(conditional_code))) {
        sub_426_433();
    } else {
        sub_433_436();
    }
    // 436: mov
    vs_out_attr5 = reg_tmp4;
    return false;
}
bool sub_426_433() {
    // 426: mov
    reg_tmp6 = reg_tmp10;
    // 427: dp4
    reg_tmp4.x = dot(uniforms.f[14], reg_tmp6);
    // 428: dp4
    reg_tmp4.y = dot(uniforms.f[15], reg_tmp6);
    // 429: dp4
    reg_tmp4.z = dot(uniforms.f[16], reg_tmp6);
    // 430: rcp
    reg_tmp6.w = rcp_safe(reg_tmp4.zzzz.x);
    // 431: mul
    reg_tmp4.xy = (mul_safe(reg_tmp4.xyyy, reg_tmp6.wwww)).xy;
    // 432: add
    reg_tmp4.xy = (uniforms.f[19].zwww + reg_tmp4.xyyy).xy;
    return false;
}
bool sub_433_436() {
    // 433: call
    {
        sub_410_414();
    }
    // 434: dp4
    reg_tmp4.x = dot(uniforms.f[14], reg_tmp6);
    // 435: dp4
    reg_tmp4.y = dot(uniforms.f[15], reg_tmp6);
    return false;
}
bool sub_437_438() {
    // 437: mov
    vs_out_attr5 = uniforms.f[93].xxxx;
    return false;
}
bool sub_440_457() {
    // 440: mov
    reg_tmp0.xy = (uniforms.f[10].zzzz).xy;
    // 441: ifu
    if (uniforms.b[11]) {
        sub_442_447();
    } else {
        sub_447_456();
    }
    // 456: nop
    return false;
}
bool sub_442_447() {
    // 442: call
    {
        sub_395_403();
    }
    // 443: dp4
    reg_tmp5.x = dot(uniforms.f[17].xywz, reg_tmp6);
    // 444: dp4
    reg_tmp5.y = dot(uniforms.f[18].xywz, reg_tmp6);
    // 445: mov
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    // 446: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_447_456() {
    // 447: ifu
    if (uniforms.b[14]) {
        sub_448_454();
    } else {
        sub_454_455();
    }
    // 455: nop
    return false;
}
bool sub_448_454() {
    // 448: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 449: mov
    reg_tmp5.zw = (reg_tmp6.zwww).zw;
    // 450: call
    {
        sub_410_414();
    }
    // 451: dp4
    reg_tmp5.x = dot(uniforms.f[17], reg_tmp6);
    // 452: dp4
    reg_tmp5.y = dot(uniforms.f[18], reg_tmp6);
    // 453: mov
    vs_out_attr6 = reg_tmp5;
    return false;
}
bool sub_454_455() {
    // 454: mov
    vs_out_attr6 = uniforms.f[93].xxxx;
    return false;
}
// reference: 23974C6497464DA4, DE3624946DADEE51
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
// shader: 8B30, ECC4377586C6AC61

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 9C384505A3235F1C, ECC4377586C6AC61
// program: DE3624946DADEE51, AD7922A63ED54CA7, ECC4377586C6AC61
// reference: BAC2342597464DA4, DE3624946DADEE51
// shader: 8B30, B335C72627F48034

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
// reference: 42891F1ADB530FE4, B335C72627F48034
// program: DE3624946DADEE51, AD7922A63ED54CA7, B335C72627F48034
// shader: 8B31, D6D39274467B8B59

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

#define fma_safe(x, y, z) fma(x, y, z)
#define mul_safe(x, y) (x * y)
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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
bool sub_34_202();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_201();
bool sub_143_153();
bool sub_153_158();
bool sub_159_171();
bool sub_171_196();
bool sub_172_190();
bool sub_190_195();
bool sub_202_219();
bool sub_219_295();
bool sub_258_278();
bool sub_259_264();
bool sub_264_277();
bool sub_268_271();
bool sub_271_276();
bool sub_278_291();
bool sub_279_284();
bool sub_284_290();
bool sub_295_312();
bool sub_301_306();
bool sub_303_304();
bool sub_310_311();
bool sub_312_358();
bool sub_316_357();
bool sub_321_323();
bool sub_323_347();
bool sub_326_331();
bool sub_337_346();
bool sub_341_342();
bool sub_342_345();
bool sub_349_356();
bool sub_358_368();
bool sub_365_366();
bool sub_368_395();
bool sub_370_375();
bool sub_375_394();
bool sub_378_384();
bool sub_384_393();
bool sub_385_389();
bool sub_389_392();
bool sub_395_403();
bool sub_397_398();
bool sub_398_402();
bool sub_399_400();
bool sub_400_401();
bool sub_403_410();
bool sub_410_414();
bool sub_463_4096();

bool exec_shader() {
    sub_463_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 17: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 18: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 26: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 27: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 28: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 29: dp3
    reg_tmp5.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 30: dp3
    reg_tmp5.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_202() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_201();
    }
    // 201: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 72: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 73: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 74: call
    {
        sub_202_219();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
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
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 98: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 99: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 100: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 101: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 102: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 103: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_219_295();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_201() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 142: ifu
    if (uniforms.b[2]) {
        sub_143_153();
    } else {
        sub_153_158();
    }
    // 158: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_159_171();
    } else {
        sub_171_196();
    }
    // 196: mov
    vs_out_attr2 = -reg_tmp15;
    // 197: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 198: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 199: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 200: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_143_153() {
    // 143: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 144: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 145: dp4
    reg_tmp7.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 146: dp4
    reg_tmp7.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 147: dp4
    reg_tmp7.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 148: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 149: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 150: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 151: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 152: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_153_158() {
    // 153: mova
    address_registers.x = (ivec2(uniforms.f[93].xxxx)).x;
    // 154: dp4
    reg_tmp10.x = dot(uniforms.f[25], reg_tmp15);
    // 155: dp4
    reg_tmp10.y = dot(uniforms.f[26], reg_tmp15);
    // 156: dp4
    reg_tmp10.z = dot(uniforms.f[27], reg_tmp15);
    // 157: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_159_171() {
    // 159: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 160: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 161: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 162: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 163: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 164: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 165: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 166: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 167: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 168: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 169: call
    {
        sub_202_219();
    }
    // 170: nop
    return false;
}
bool sub_171_196() {
    // 171: ifc
    if (all(conditional_code)) {
        sub_172_190();
    } else {
        sub_190_195();
    }
    // 195: nop
    return false;
}
bool sub_172_190() {
    // 172: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 173: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 174: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 175: dp3
    reg_tmp11.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 176: dp3
    reg_tmp11.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 177: dp3
    reg_tmp11.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 178: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 179: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 180: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 181: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 182: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 183: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 184: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 185: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 186: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 187: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 188: call
    {
        sub_219_295();
    }
    // 189: nop
    return false;
}
bool sub_190_195() {
    // 190: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 191: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 192: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 193: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 194: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_202_219() {
    uint jmp_to = 202u;
    while (true) {
        switch (jmp_to) {
        case 202u: {
            // 202: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 203: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 204: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 205: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 206: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 207: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 208: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 209: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 218u; break; }
            }
            // 210: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 211: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 212: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 213: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 214: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 215: jmpc
            if (conditional_code.x) {
                { jmp_to = 218u; break; }
            }
            // 216: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 217: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 218u: {
            // 218: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_219_295() {
    uint jmp_to = 219u;
    while (true) {
        switch (jmp_to) {
        case 219u: {
            // 219: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 220: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 221: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 222: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 223: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 224: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 225: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 226: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 227: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 228: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 294u; break; }
            }
            // 229: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 230: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 231: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 232: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 233: dp3
            reg_tmp5.w = dot(vec3(reg_tmp5), vec3(reg_tmp5));
            // 234: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 235: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 236: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 237: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 238: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 239: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 240: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 241: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 242: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 243: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 244: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 245: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 246: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 247: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 248: jmpc
            if (!conditional_code.x) {
                { jmp_to = 256u; break; }
            }
            // 249: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 250: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 251: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 252: dp4
            reg_tmp6 = vec4(dot(reg_tmp7, reg_tmp7));
            // 253: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 254: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 255: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 294u; break; }
            }
        }
        case 256u: {
            // 256: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 257: ifc
            if (conditional_code.x) {
                sub_258_278();
            } else {
                sub_278_291();
            }
            // 291: dp4
            reg_tmp6 = vec4(dot(reg_tmp8, reg_tmp8));
            // 292: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 293: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 294u: {
            // 294: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_258_278() {
    // 258: ifc
    if (conditional_code.y) {
        sub_259_264();
    } else {
        sub_264_277();
    }
    // 277: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_259_264() {
    // 259: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 260: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 261: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 262: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 263: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_264_277() {
    // 264: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 265: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 266: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 267: ifc
    if (conditional_code.x) {
        sub_268_271();
    } else {
        sub_271_276();
    }
    // 276: nop
    return false;
}
bool sub_268_271() {
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_276() {
    // 271: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 272: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 273: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 274: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 275: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_278_291() {
    // 278: ifc
    if (conditional_code.y) {
        sub_279_284();
    } else {
        sub_284_290();
    }
    // 290: nop
    return false;
}
bool sub_279_284() {
    // 279: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 280: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 281: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 282: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 283: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_284_290() {
    // 284: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 285: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 286: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 287: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 288: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 289: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_295_312() {
    // 295: mov
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    // 296: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 297: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 298: mov
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    // 299: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 300: ifc
    if (conditional_code.y) {
        sub_301_306();
    }
    // 306: callu
    if (uniforms.b[12]) {
        sub_312_358();
    }
    // 307: callu
    if (uniforms.b[5]) {
        sub_358_368();
    }
    // 308: cmp
    conditional_code = equal(vec2(uniforms.f[93].xxxx), vec2(reg_tmp8.xyyy));
    // 309: ifc
    if (all(conditional_code)) {
        sub_310_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_301_306() {
    // 301: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 302: ifu
    if (uniforms.b[7]) {
        sub_303_304();
    }
    // 304: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 305: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_303_304() {
    // 303: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_310_311() {
    // 310: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_312_358() {
    // 312: mov
    reg_tmp1 = uniforms.f[20];
    // 313: mov
    reg_tmp2 = uniforms.f[21];
    // 314: mov
    reg_tmp3 = uniforms.f[93].xxxx;
    // 315: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop315 = 0u; loop315 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop315) {
        sub_316_357();
    }
    // 357: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_316_357() {
    // 316: mova
    address_registers.x = (ivec2(reg_tmp3)).x;
    // 317: mov
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    // 318: mov
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[93].xyyy), vec2(reg_tmp4.xyyy));
    // 320: ifc
    if (conditional_code.x) {
        sub_321_323();
    } else {
        sub_323_347();
    }
    // 347: cmp
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    // 348: ifc
    if (conditional_code.y) {
        sub_349_356();
    }
    // 356: add
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_321_323() {
    // 321: dp3
    reg_tmp6.x = dot(vec3(uniforms.f[81 + address_registers.x]), vec3(reg_tmp14));
    // 322: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_323_347() {
    // 323: add
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    // 324: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    // 325: ifc
    if (conditional_code.y) {
        sub_326_331();
    }
    // 331: mov
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    // 332: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.wwww));
    // 333: dp3
    reg_tmp4.w = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 334: rsq
    reg_tmp4.w = rsq_safe(reg_tmp4.wwww.x);
    // 335: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp4.wwww);
    // 336: ifc
    if (conditional_code.x) {
        sub_337_346();
    }
    // 346: dp3
    reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp4));
    return false;
}
bool sub_326_331() {
    // 326: mov
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    // 327: dp3
    reg_tmp5.z = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 328: mul
    reg_tmp5.y = (mul_safe(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    // 329: dp3
    reg_tmp6.y = dot(vec3(uniforms.f[83 + address_registers.x]), vec3(reg_tmp5));
    // 330: rcp
    reg_tmp6.y = rcp_safe(reg_tmp6.yyyy.x);
    return false;
}
bool sub_337_346() {
    // 337: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[82 + address_registers.x]), vec3(-reg_tmp4));
    // 338: slti
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx,uniforms.f[84 + address_registers.x].yyyy))).y;
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.xyyy));
    // 340: ifc
    if (conditional_code.y) {
        sub_341_342();
    } else {
        sub_342_345();
    }
    // 345: mul
    reg_tmp6.y = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_341_342() {
    // 341: mov
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_342_345() {
    // 342: log
    reg_tmp5.y = log2(reg_tmp5.xxxx.x);
    // 343: mul
    reg_tmp5.y = (mul_safe(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    // 344: exp
    reg_tmp5.x = exp2(reg_tmp5.yyyy.x);
    return false;
}
bool sub_349_356() {
    // 349: max
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    // 350: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    // 351: mul
    reg_tmp4 = mul_safe(uniforms.f[80 + address_registers.x], reg_tmp2);
    // 352: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    // 353: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    // 354: add
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    // 355: add
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_358_368() {
    // 358: dp3
    reg_tmp1 = vec4(dot(vec3(uniforms.f[24]), vec3(reg_tmp14)));
    // 359: mov
    reg_tmp2 = uniforms.f[24].wwww;
    // 360: mad
    reg_tmp1 = fma_safe(reg_tmp1, reg_tmp2, reg_tmp2);
    // 361: mov
    reg_tmp3 = uniforms.f[22];
    // 362: add
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    // 363: mad
    reg_tmp4 = fma_safe(reg_tmp2, reg_tmp1, reg_tmp3);
    // 364: ifu
    if (uniforms.b[6]) {
        sub_365_366();
    }
    // 366: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    // 367: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_365_366() {
    // 365: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_368_395() {
    // 368: mov
    reg_tmp0.xy = (uniforms.f[10].xxxx).xy;
    // 369: ifu
    if (uniforms.b[9]) {
        sub_370_375();
    } else {
        sub_375_394();
    }
    // 394: nop
    return false;
}
bool sub_370_375() {
    // 370: call
    {
        sub_395_403();
    }
    // 371: dp4
    reg_tmp3.x = dot(uniforms.f[11].xywz, reg_tmp6);
    // 372: dp4
    reg_tmp3.y = dot(uniforms.f[12].xywz, reg_tmp6);
    // 373: mov
    reg_tmp3.zw = (uniforms.f[93].xxxx).zw;
    // 374: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_375_394() {
    // 375: cmp
    conditional_code = equal(vec2(uniforms.f[95].xyyy), vec2(reg_tmp0.xyyy));
    // 376: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    // 377: ifc
    if (all(not(conditional_code))) {
        sub_378_384();
    } else {
        sub_384_393();
    }
    // 393: mov
    vs_out_attr4 = reg_tmp3;
    return false;
}
bool sub_378_384() {
    // 378: mov
    reg_tmp6 = reg_tmp10;
    // 379: dp4
    reg_tmp3.x = dot(uniforms.f[11], reg_tmp6);
    // 380: dp4
    reg_tmp3.y = dot(uniforms.f[12], reg_tmp6);
    // 381: dp4
    reg_tmp3.z = dot(uniforms.f[13], reg_tmp6);
    // 382: mul
    reg_tmp0.xy = (mul_safe(uniforms.f[19].xyyy, reg_tmp3.zzzz)).xy;
    // 383: add
    reg_tmp3.xy = (reg_tmp3.xyyy + reg_tmp0.xyyy).xy;
    return false;
}
bool sub_384_393() {
    // 384: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_385_389();
    } else {
        sub_389_392();
    }
    // 392: nop
    return false;
}
bool sub_385_389() {
    // 385: call
    {
        sub_403_410();
    }
    // 386: dp3
    reg_tmp3.x = dot(vec3(uniforms.f[11]), vec3(reg_tmp6));
    // 387: dp3
    reg_tmp3.y = dot(vec3(uniforms.f[12]), vec3(reg_tmp6));
    // 388: dp3
    reg_tmp3.z = dot(vec3(uniforms.f[13]), vec3(reg_tmp6));
    return false;
}
bool sub_389_392() {
    // 389: call
    {
        sub_410_414();
    }
    // 390: dp4
    reg_tmp3.x = dot(uniforms.f[11], reg_tmp6);
    // 391: dp4
    reg_tmp3.y = dot(uniforms.f[12], reg_tmp6);
    return false;
}
bool sub_395_403() {
    // 395: cmp
    conditional_code = equal(vec2(uniforms.f[93].yzzz), vec2(reg_tmp0.xyyy));
    // 396: ifc
    if (all(not(conditional_code))) {
        sub_397_398();
    } else {
        sub_398_402();
    }
    // 402: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_397_398() {
    // 397: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].xxxx, vs_in_reg4.xyyy)).xy;
    return false;
}
bool sub_398_402() {
    // 398: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_399_400();
    } else {
        sub_400_401();
    }
    // 401: nop
    return false;
}
bool sub_399_400() {
    // 399: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].yyyy, vs_in_reg5.xyyy)).xy;
    return false;
}
bool sub_400_401() {
    // 400: mul
    reg_tmp6.xy = (mul_safe(uniforms.f[8].zzzz, vs_in_reg6.xyyy)).xy;
    return false;
}
bool sub_403_410() {
    // 403: mov
    reg_tmp2 = -reg_tmp15;
    // 404: dp3
    reg_tmp2.w = dot(vec3(reg_tmp2), vec3(reg_tmp2));
    // 405: rsq
    reg_tmp2.w = rsq_safe(reg_tmp2.wwww.x);
    // 406: mul
    reg_tmp2 = mul_safe(reg_tmp2, reg_tmp2.wwww);
    // 407: dp3
    reg_tmp1 = vec4(dot(vec3(reg_tmp2), vec3(reg_tmp14)));
    // 408: add
    reg_tmp1 = reg_tmp1 + reg_tmp1;
    // 409: mad
    reg_tmp6 = fma_safe(reg_tmp1, reg_tmp14, -reg_tmp2);
    return false;
}
bool sub_410_414() {
    // 410: mov
    reg_tmp1.xy = (uniforms.f[94].zzzz).xy;
    // 411: mov
    reg_tmp1.zw = (uniforms.f[93].xxxx).zw;
    // 412: mad
    reg_tmp6 = fma_safe(reg_tmp14, reg_tmp1, reg_tmp1);
    // 413: mov
    reg_tmp6.zw = (uniforms.f[93].xxyy).zw;
    return false;
}
bool sub_463_4096() {
    // 463: call
    {
        sub_34_202();
    }
    // 464: call
    {
        sub_295_312();
    }
    // 465: call
    {
        sub_368_395();
    }
    // 466: end
    return true;
}
// reference: 3F3A16EA05AE839E, D6D39274467B8B59
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
// shader: 8B30, FB4E9E925D4DB26E

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((secondary_fragment_color.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F7F99D96B6F0D597, FB4E9E925D4DB26E
// program: D6D39274467B8B59, 6CF3F3B70E23AA85, FB4E9E925D4DB26E
// shader: 8B30, 9CFDF657447B9FC7

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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (primary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 75D4372129438D7A, 9CFDF657447B9FC7
// program: DE3624946DADEE51, AD7922A63ED54CA7, 9CFDF657447B9FC7
// shader: 8B30, 26E3A1642C0501CC

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
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
// reference: 584C8F8BC2F04615, 26E3A1642C0501CC
// program: DE3624946DADEE51, AD7922A63ED54CA7, 26E3A1642C0501CC
// reference: 9C38450570BAA706, ECC4377586C6AC61
// reference: 42891F1A08CAF7FE, B335C72627F48034
// shader: 8B30, 38F2FB67AD774145

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: 42891F1A22F363DC, 38F2FB67AD774145
// program: DE3624946DADEE51, AD7922A63ED54CA7, 38F2FB67AD774145
// shader: 8B30, D0E6A641340115E3

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 42891F1AE47D1CFD, D0E6A641340115E3
// program: DE3624946DADEE51, AD7922A63ED54CA7, D0E6A641340115E3
// shader: 8B30, 6CB806B51A77C8C7

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (rounded_primary_color.rgb) + (texcolor1.rgb) * (vec3(1.0) - (rounded_primary_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: A085755742132E3B, 6CB806B51A77C8C7
// program: DE3624946DADEE51, AD7922A63ED54CA7, 6CB806B51A77C8C7
// shader: 8B30, 25953BF3E0597251

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: 9C38450560538B98, 25953BF3E0597251
// program: DE3624946DADEE51, AD7922A63ED54CA7, 25953BF3E0597251
// shader: 8B30, A314B701187AE6F0

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb) + (texcolor1.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: D2E728577729FA4A, A314B701187AE6F0
// program: DE3624946DADEE51, AD7922A63ED54CA7, A314B701187AE6F0
// shader: 8B30, 723911D6E1C81075

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor1.rgb) * (texcolor1.aaa) + (texcolor0.rgb) * (vec3(1.0) - (texcolor1.aaa)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AD65D7B852023BEA, 723911D6E1C81075
// program: DE3624946DADEE51, AD7922A63ED54CA7, 723911D6E1C81075
// shader: 8B30, 4505944EC8415D62

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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (primary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

if (int(last_tex_env_out.a * 255.0) <= alphatest_ref) discard;
gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 75D43721C5F46679, 4505944EC8415D62
// program: DE3624946DADEE51, AD7922A63ED54CA7, 4505944EC8415D62
// shader: 8B30, A4F526C2F219139E

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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (primary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: 13F7A6D06C7E3C4C, A4F526C2F219139E
// program: DE3624946DADEE51, AD7922A63ED54CA7, A4F526C2F219139E
// shader: 8B30, 8D9356AD2387DEC1

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 texcolor2 = textureLod(tex2, texcoord2, getLod(texcoord2 * vec2(textureSize(tex2, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((const_color[0].rgb) * (texcolor0.rgb) + (rounded_primary_color.rgb) * (vec3(1.0) - (texcolor0.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((texcolor1.rgb) * (rounded_primary_color.rgb) + (last_tex_env_out.rgb) * (vec3(1.0) - (rounded_primary_color.rgb)), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((last_tex_env_out.rgb) * (texcolor2.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((last_tex_env_out.rgb) + (texcolor1.rgb) - vec3(0.5), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3 * 2.0, alpha_output_3 * 1.0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((last_tex_env_out.rgb) * (const_color[4].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 683529447A3EA7F3, 8D9356AD2387DEC1
// program: DE3624946DADEE51, AD7922A63ED54CA7, 8D9356AD2387DEC1
// reference: BAC234257A0D7C10, DE3624946DADEE51
// shader: 8B30, 50BEB7920B12C81A

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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (primary_fragment_color.aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B1A0FDAF29438D7A, 50BEB7920B12C81A
// program: DE3624946DADEE51, AD7922A63ED54CA7, 50BEB7920B12C81A
// shader: 8B30, 992EDE3BB73FAF76

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
specular_sum.a = (lut_scale_fr * LookupLightingLUTUnsigned(3, max(dot(normal, normalize(view)), 0.0)));
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += (((lut_scale_d0 * LookupLightingLUTUnsigned(0, max(dot(normal, normalize(half_vector)), 0.0))) * light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb) + (const_color[0].aaa), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;
next_combiner_buffer.rgb = last_tex_env_out.rgb;
next_combiner_buffer.a = last_tex_env_out.a;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (rounded_primary_color.rgb) + (secondary_fragment_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B1A0FDAF69510614, 992EDE3BB73FAF76
// program: DE3624946DADEE51, AD7922A63ED54CA7, 992EDE3BB73FAF76
// reference: FB35E2F4801C7747, 25953BF3E0597251
// shader: 8B31, 7DBDA125486056E4

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

#define fma_safe(x, y, z) fma(x, y, z)
#define mul_safe(x, y) (x * y)
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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
bool sub_34_202();
bool sub_40_140();
bool sub_47_76();
bool sub_58_62();
bool sub_76_135();
bool sub_77_109();
bool sub_88_92();
bool sub_109_134();
bool sub_120_124();
bool sub_140_201();
bool sub_143_153();
bool sub_153_158();
bool sub_159_171();
bool sub_171_196();
bool sub_172_190();
bool sub_190_195();
bool sub_202_219();
bool sub_219_295();
bool sub_258_278();
bool sub_259_264();
bool sub_264_277();
bool sub_268_271();
bool sub_271_276();
bool sub_278_291();
bool sub_279_284();
bool sub_284_290();
bool sub_295_312();
bool sub_301_306();
bool sub_303_304();
bool sub_310_311();
bool sub_312_358();
bool sub_316_357();
bool sub_321_323();
bool sub_323_347();
bool sub_326_331();
bool sub_337_346();
bool sub_341_342();
bool sub_342_345();
bool sub_349_356();
bool sub_358_368();
bool sub_365_366();
bool sub_468_4096();

bool exec_shader() {
    sub_468_4096();
    return true;
}

bool sub_7_12() {
    // 7: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 8: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 9: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 10: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 11: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    return false;
}
bool sub_12_21() {
    // 12: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 13: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 14: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 15: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 16: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 17: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 18: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 19: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 20: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    return false;
}
bool sub_21_34() {
    // 21: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 22: dp4
    reg_tmp3.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 23: dp4
    reg_tmp3.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 24: dp4
    reg_tmp3.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 25: dp3
    reg_tmp4.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 26: dp3
    reg_tmp4.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 27: dp3
    reg_tmp4.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 28: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 29: dp3
    reg_tmp5.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 30: dp3
    reg_tmp5.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 31: mad
    reg_tmp7 = fma_safe(reg_tmp1.wwww, reg_tmp3, reg_tmp7);
    // 32: mad
    reg_tmp12 = fma_safe(reg_tmp1.wwww, reg_tmp4, reg_tmp12);
    // 33: mad
    reg_tmp11 = fma_safe(reg_tmp1.wwww, reg_tmp5, reg_tmp11);
    return false;
}
bool sub_34_202() {
    // 34: mul
    reg_tmp15.xyz = (mul_safe(uniforms.f[7].xxxx, vs_in_reg0)).xyz;
    // 35: mul
    reg_tmp14.xyz = (mul_safe(uniforms.f[7].yyyy, vs_in_reg1)).xyz;
    // 36: mul
    reg_tmp13.xyz = (mul_safe(uniforms.f[7].zzzz, vs_in_reg2)).xyz;
    // 37: add
    reg_tmp15.xyz = (uniforms.f[6] + reg_tmp15).xyz;
    // 38: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 39: ifu
    if (uniforms.b[1]) {
        sub_40_140();
    } else {
        sub_140_201();
    }
    // 201: nop
    return false;
}
bool sub_40_140() {
    // 40: mov
    reg_tmp0 = uniforms.f[7];
    // 41: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 42: mov
    reg_tmp7 = uniforms.f[93].xxxx;
    // 43: mov
    reg_tmp12 = uniforms.f[93].xxxx;
    // 44: mov
    reg_tmp11 = uniforms.f[93].xxxx;
    // 45: mul
    reg_tmp2 = mul_safe(uniforms.f[93].wwww, vs_in_reg7);
    // 46: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_47_76();
    } else {
        sub_76_135();
    }
    // 135: mov
    vs_out_attr2 = -reg_tmp15;
    // 136: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 137: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 138: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 139: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_47_76() {
    // 47: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 48: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 49: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 50: call
    {
        sub_12_21();
    }
    // 51: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 52: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 53: call
    {
        sub_12_21();
    }
    // 54: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 55: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 56: callc
    if (conditional_code.x) {
        sub_12_21();
    }
    // 57: ifu
    if (uniforms.b[8]) {
        sub_58_62();
    }
    // 62: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 63: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 64: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 65: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 66: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 67: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 68: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 69: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 70: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 71: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 72: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 73: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 74: call
    {
        sub_202_219();
    }
    // 75: nop
    return false;
}
bool sub_58_62() {
    // 58: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 59: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
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
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 78: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 79: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 80: call
    {
        sub_21_34();
    }
    // 81: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 82: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 83: call
    {
        sub_21_34();
    }
    // 84: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 85: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 86: callc
    if (conditional_code.x) {
        sub_21_34();
    }
    // 87: ifu
    if (uniforms.b[8]) {
        sub_88_92();
    }
    // 92: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 93: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 94: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 95: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 96: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 97: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 98: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 99: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 100: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 101: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 102: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 103: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 104: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 105: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 106: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 107: call
    {
        sub_219_295();
    }
    // 108: nop
    return false;
}
bool sub_88_92() {
    // 88: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 89: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 90: callc
    if (conditional_code.y) {
        sub_21_34();
    }
    // 91: nop
    return false;
}
bool sub_109_134() {
    // 109: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(vs_in_reg8.zwww));
    // 110: mov
    reg_tmp1.xy = (reg_tmp2.xxxx).xy;
    // 111: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.xxxx)).w;
    // 112: call
    {
        sub_7_12();
    }
    // 113: mov
    reg_tmp1.xy = (reg_tmp2.yyyy).xy;
    // 114: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.yyyy)).w;
    // 115: call
    {
        sub_7_12();
    }
    // 116: mov
    reg_tmp1.xy = (reg_tmp2.zzzz).xy;
    // 117: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.zzzz)).w;
    // 118: callc
    if (conditional_code.x) {
        sub_7_12();
    }
    // 119: ifu
    if (uniforms.b[8]) {
        sub_120_124();
    }
    // 124: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 125: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 126: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 127: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 128: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    // 129: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 130: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 131: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 132: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 133: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_120_124() {
    // 120: mov
    reg_tmp1.xy = (reg_tmp2.wwww).xy;
    // 121: mul
    reg_tmp1.w = (mul_safe(uniforms.f[8].wwww, vs_in_reg8.wwww)).w;
    // 122: callc
    if (conditional_code.y) {
        sub_7_12();
    }
    // 123: nop
    return false;
}
bool sub_140_201() {
    // 140: mov
    reg_tmp0 = uniforms.f[7];
    // 141: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.yzzz));
    // 142: ifu
    if (uniforms.b[2]) {
        sub_143_153();
    } else {
        sub_153_158();
    }
    // 158: ifc
    if (all(bvec2(conditional_code.x, !conditional_code.y))) {
        sub_159_171();
    } else {
        sub_171_196();
    }
    // 196: mov
    vs_out_attr2 = -reg_tmp15;
    // 197: dp4
    vs_out_attr0.x = dot(uniforms.f[86], reg_tmp15);
    // 198: dp4
    vs_out_attr0.y = dot(uniforms.f[87], reg_tmp15);
    // 199: dp4
    vs_out_attr0.z = dot(uniforms.f[88], reg_tmp15);
    // 200: dp4
    vs_out_attr0.w = dot(uniforms.f[89], reg_tmp15);
    return false;
}
bool sub_143_153() {
    // 143: mul
    reg_tmp1.x = (mul_safe(uniforms.f[93].wwww, vs_in_reg7.xxxx)).x;
    // 144: mova
    address_registers.x = (ivec2(reg_tmp1.xxxx)).x;
    // 145: dp4
    reg_tmp7.x = dot(uniforms.f[25 + address_registers.x], reg_tmp15);
    // 146: dp4
    reg_tmp7.y = dot(uniforms.f[26 + address_registers.x], reg_tmp15);
    // 147: dp4
    reg_tmp7.z = dot(uniforms.f[27 + address_registers.x], reg_tmp15);
    // 148: mov
    reg_tmp7.w = (uniforms.f[93].yyyy).w;
    // 149: dp4
    reg_tmp10.x = dot(uniforms.f[0], reg_tmp7);
    // 150: dp4
    reg_tmp10.y = dot(uniforms.f[1], reg_tmp7);
    // 151: dp4
    reg_tmp10.z = dot(uniforms.f[2], reg_tmp7);
    // 152: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_153_158() {
    // 153: mova
    address_registers.x = (ivec2(uniforms.f[93].xxxx)).x;
    // 154: dp4
    reg_tmp10.x = dot(uniforms.f[25], reg_tmp15);
    // 155: dp4
    reg_tmp10.y = dot(uniforms.f[26], reg_tmp15);
    // 156: dp4
    reg_tmp10.z = dot(uniforms.f[27], reg_tmp15);
    // 157: mov
    reg_tmp10.w = (uniforms.f[93].yyyy).w;
    return false;
}
bool sub_159_171() {
    // 159: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 160: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 161: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 162: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 163: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 164: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 165: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 166: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 167: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 168: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 169: call
    {
        sub_202_219();
    }
    // 170: nop
    return false;
}
bool sub_171_196() {
    // 171: ifc
    if (all(conditional_code)) {
        sub_172_190();
    } else {
        sub_190_195();
    }
    // 195: nop
    return false;
}
bool sub_172_190() {
    // 172: dp3
    reg_tmp12.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp14));
    // 173: dp3
    reg_tmp12.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp14));
    // 174: dp3
    reg_tmp12.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp14));
    // 175: dp3
    reg_tmp11.x = dot(vec3(uniforms.f[25 + address_registers.x]), vec3(reg_tmp13));
    // 176: dp3
    reg_tmp11.y = dot(vec3(uniforms.f[26 + address_registers.x]), vec3(reg_tmp13));
    // 177: dp3
    reg_tmp11.z = dot(vec3(uniforms.f[27 + address_registers.x]), vec3(reg_tmp13));
    // 178: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 179: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 180: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 181: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 182: dp3
    reg_tmp14.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp12));
    // 183: dp3
    reg_tmp14.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp12));
    // 184: dp3
    reg_tmp14.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp12));
    // 185: dp3
    reg_tmp13.x = dot(vec3(uniforms.f[3]), vec3(reg_tmp11));
    // 186: dp3
    reg_tmp13.y = dot(vec3(uniforms.f[4]), vec3(reg_tmp11));
    // 187: dp3
    reg_tmp13.z = dot(vec3(uniforms.f[5]), vec3(reg_tmp11));
    // 188: call
    {
        sub_219_295();
    }
    // 189: nop
    return false;
}
bool sub_190_195() {
    // 190: dp4
    reg_tmp15.x = dot(uniforms.f[90], reg_tmp10);
    // 191: dp4
    reg_tmp15.y = dot(uniforms.f[91], reg_tmp10);
    // 192: dp4
    reg_tmp15.z = dot(uniforms.f[92], reg_tmp10);
    // 193: mov
    reg_tmp15.w = (uniforms.f[93].yyyy).w;
    // 194: mov
    vs_out_attr1 = uniforms.f[93].xxxx;
    return false;
}
bool sub_202_219() {
    uint jmp_to = 202u;
    while (true) {
        switch (jmp_to) {
        case 202u: {
            // 202: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 203: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 204: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 205: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 206: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 207: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 208: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 209: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 218u; break; }
            }
            // 210: add
            reg_tmp4 = uniforms.f[93].yyyy + reg_tmp14.zzzz;
            // 211: mul
            reg_tmp4 = mul_safe(uniforms.f[94].zzzz, reg_tmp4);
            // 212: cmp
            conditional_code = greaterThanEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp4.xxxx));
            // 213: rsq
            reg_tmp4 = vec4(rsq_safe(reg_tmp4.xxxx.x));
            // 214: mul
            reg_tmp5 = mul_safe(uniforms.f[94].zzzz, reg_tmp14);
            // 215: jmpc
            if (conditional_code.x) {
                { jmp_to = 218u; break; }
            }
            // 216: rcp
            reg_tmp0.z = rcp_safe(reg_tmp4.xxxx.x);
            // 217: mul
            reg_tmp0.xy = (mul_safe(reg_tmp5, reg_tmp4)).xy;
        }
        case 218u: {
            // 218: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_219_295() {
    uint jmp_to = 219u;
    while (true) {
        switch (jmp_to) {
        case 219u: {
            // 219: dp3
            reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp14));
            // 220: dp3
            reg_tmp7.x = dot(vec3(reg_tmp12), vec3(reg_tmp12));
            // 221: rsq
            reg_tmp6.x = rsq_safe(reg_tmp6.xxxx.x);
            // 222: rsq
            reg_tmp7.x = rsq_safe(reg_tmp7.xxxx.x);
            // 223: mul
            reg_tmp14.xyz = (mul_safe(reg_tmp14.xyzz, reg_tmp6.xxxx)).xyz;
            // 224: mul
            reg_tmp12.xyz = (mul_safe(reg_tmp12.xyzz, reg_tmp7.xxxx)).xyz;
            // 225: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 226: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 227: mov
            reg_tmp0 = uniforms.f[93].yxxx;
            // 228: jmpu
            if (!uniforms.b[15]) {
                { jmp_to = 294u; break; }
            }
            // 229: mul
            reg_tmp13.xyz = (mul_safe(reg_tmp13.xyzz, reg_tmp6.xxxx)).xyz;
            // 230: mul
            reg_tmp11.xyz = (mul_safe(reg_tmp11.xyzz, reg_tmp7.xxxx)).xyz;
            // 231: mul
            reg_tmp5 = mul_safe(reg_tmp14.yzxx, reg_tmp13.zxyy);
            // 232: mad
            reg_tmp5 = fma_safe(-reg_tmp13.yzxx, reg_tmp14.zxyy, reg_tmp5);
            // 233: dp3
            reg_tmp5.w = dot(vec3(reg_tmp5), vec3(reg_tmp5));
            // 234: rsq
            reg_tmp5.w = rsq_safe(reg_tmp5.wwww.x);
            // 235: mul
            reg_tmp5 = mul_safe(reg_tmp5, reg_tmp5.wwww);
            // 236: add
            reg_tmp6.w = (reg_tmp14.zzzz + reg_tmp5.yyyy).w;
            // 237: mul
            reg_tmp13 = mul_safe(reg_tmp5.yzxx, reg_tmp14.zxyy);
            // 238: mad
            reg_tmp13 = fma_safe(-reg_tmp14.yzxx, reg_tmp5.zxyy, reg_tmp13);
            // 239: add
            reg_tmp6.w = (reg_tmp13.xxxx + reg_tmp6).w;
            // 240: mov
            reg_tmp13.w = (reg_tmp5.zzzz).w;
            // 241: mov
            reg_tmp5.z = (reg_tmp13.xxxx).z;
            // 242: add
            reg_tmp6.w = (uniforms.f[93].yyyy + reg_tmp6).w;
            // 243: mov
            reg_tmp14.w = (reg_tmp5.xxxx).w;
            // 244: mov
            reg_tmp5.x = (reg_tmp14.zzzz).x;
            // 245: cmp
            conditional_code = lessThan(vec2(uniforms.f[94].yyyy), vec2(reg_tmp6.wwww));
            // 246: mov
            reg_tmp6.x = (uniforms.f[93].yyyy).x;
            // 247: mov
            reg_tmp6.y = (-uniforms.f[93].yyyy).y;
            // 248: jmpc
            if (!conditional_code.x) {
                { jmp_to = 256u; break; }
            }
            // 249: add
            reg_tmp7.xz = (reg_tmp13.wwyy + -reg_tmp14.yyww).xz;
            // 250: add
            reg_tmp7.y = (reg_tmp14.xxxx + -reg_tmp13.zzzz).y;
            // 251: mov
            reg_tmp7.w = (reg_tmp6).w;
            // 252: dp4
            reg_tmp6 = vec4(dot(reg_tmp7, reg_tmp7));
            // 253: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 254: mul
            reg_tmp0 = mul_safe(reg_tmp7, reg_tmp6);
            // 255: jmpu
            if (uniforms.b[0]) {
                { jmp_to = 294u; break; }
            }
        }
        case 256u: {
            // 256: cmp
            conditional_code = greaterThan(vec2(reg_tmp5.zyyy), vec2(reg_tmp5.yxxx));
            // 257: ifc
            if (conditional_code.x) {
                sub_258_278();
            } else {
                sub_278_291();
            }
            // 291: dp4
            reg_tmp6 = vec4(dot(reg_tmp8, reg_tmp8));
            // 292: rsq
            reg_tmp6 = vec4(rsq_safe(reg_tmp6.xxxx.x));
            // 293: mul
            reg_tmp0 = mul_safe(reg_tmp8, reg_tmp6);
        }
        case 294u: {
            // 294: mov
            vs_out_attr1 = reg_tmp0;
        }
        default: return false;
        }
    }
    return false;
}
bool sub_258_278() {
    // 258: ifc
    if (conditional_code.y) {
        sub_259_264();
    } else {
        sub_264_277();
    }
    // 277: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_259_264() {
    // 259: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 260: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 261: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 262: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 263: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_264_277() {
    // 264: cmp
    conditional_code = greaterThan(vec2(reg_tmp5.zzzz), vec2(reg_tmp5.xxxx));
    // 265: mul
    reg_tmp8 = mul_safe(reg_tmp13.yyzw, reg_tmp6.xxxy);
    // 266: add
    reg_tmp8.x = (uniforms.f[93].yyyy + -reg_tmp5.yyyy).x;
    // 267: ifc
    if (conditional_code.x) {
        sub_268_271();
    } else {
        sub_271_276();
    }
    // 276: nop
    return false;
}
bool sub_268_271() {
    // 268: add
    reg_tmp9 = reg_tmp5.zzzz + -reg_tmp5.xxxx;
    // 269: add
    reg_tmp8.yzw = (reg_tmp8 + reg_tmp14.wwxy).yzw;
    // 270: add
    reg_tmp8.x = (reg_tmp9 + reg_tmp8).x;
    return false;
}
bool sub_271_276() {
    // 271: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 272: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 273: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 274: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 275: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    return false;
}
bool sub_278_291() {
    // 278: ifc
    if (conditional_code.y) {
        sub_279_284();
    } else {
        sub_284_290();
    }
    // 290: nop
    return false;
}
bool sub_279_284() {
    // 279: mul
    reg_tmp8 = mul_safe(reg_tmp13.yywz, reg_tmp6.xxxy);
    // 280: add
    reg_tmp8.y = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).y;
    // 281: add
    reg_tmp9 = reg_tmp5.yyyy + -reg_tmp5.xxxx;
    // 282: add
    reg_tmp8.xzw = (reg_tmp8 + reg_tmp14.wwyx).xzw;
    // 283: add
    reg_tmp8.y = (reg_tmp9 + reg_tmp8).y;
    return false;
}
bool sub_284_290() {
    // 284: mul
    reg_tmp8 = mul_safe(reg_tmp13.zwwy, reg_tmp6.xxxy);
    // 285: add
    reg_tmp8.z = (uniforms.f[93].yyyy + -reg_tmp5.zzzz).z;
    // 286: add
    reg_tmp9 = reg_tmp5.xxxx + -reg_tmp5.yyyy;
    // 287: add
    reg_tmp8.xyw = (reg_tmp8 + reg_tmp14.xyyw).xyw;
    // 288: add
    reg_tmp8.z = (reg_tmp9 + reg_tmp8).z;
    // 289: mov
    reg_tmp8.w = (-reg_tmp8).w;
    return false;
}
bool sub_295_312() {
    // 295: mov
    reg_tmp8.xy = (uniforms.f[93].xxxx).xy;
    // 296: mov
    reg_tmp0.y = (uniforms.f[7].wwww).y;
    // 297: cmp
    conditional_code = notEqual(vec2(uniforms.f[93].xxxx), vec2(reg_tmp0.xyyy));
    // 298: mov
    reg_tmp9.xyz = (uniforms.f[93].xxxx).xyz;
    // 299: mov
    reg_tmp9.w = (uniforms.f[21].wwww).w;
    // 300: ifc
    if (conditional_code.y) {
        sub_301_306();
    }
    // 306: callu
    if (uniforms.b[12]) {
        sub_312_358();
    }
    // 307: callu
    if (uniforms.b[5]) {
        sub_358_368();
    }
    // 308: cmp
    conditional_code = equal(vec2(uniforms.f[93].xxxx), vec2(reg_tmp8.xyyy));
    // 309: ifc
    if (all(conditional_code)) {
        sub_310_311();
    }
    // 311: max
    vs_out_attr3 = max(uniforms.f[93].xxxx, reg_tmp9);
    return false;
}
bool sub_301_306() {
    // 301: mul
    reg_tmp0 = mul_safe(uniforms.f[7].wwww, vs_in_reg3);
    // 302: ifu
    if (uniforms.b[7]) {
        sub_303_304();
    }
    // 304: mul
    reg_tmp9.xyz = (mul_safe(uniforms.f[20].wwww, reg_tmp0.xyzz)).xyz;
    // 305: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_303_304() {
    // 303: mul
    reg_tmp9.w = (mul_safe(reg_tmp9.wwww, reg_tmp0.wwww)).w;
    return false;
}
bool sub_310_311() {
    // 310: mov
    reg_tmp9 = uniforms.f[21];
    return false;
}
bool sub_312_358() {
    // 312: mov
    reg_tmp1 = uniforms.f[20];
    // 313: mov
    reg_tmp2 = uniforms.f[21];
    // 314: mov
    reg_tmp3 = uniforms.f[93].xxxx;
    // 315: loop
    address_registers.z = int(uniforms.i[0].y);
    for (uint loop315 = 0u; loop315 <= uniforms.i[0].x; address_registers.z += int(uniforms.i[0].z), ++loop315) {
        sub_316_357();
    }
    // 357: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_316_357() {
    // 316: mova
    address_registers.x = (ivec2(reg_tmp3)).x;
    // 317: mov
    reg_tmp4.x = (uniforms.f[81 + address_registers.x].wwww).x;
    // 318: mov
    reg_tmp4.y = (uniforms.f[83 + address_registers.x].wwww).y;
    // 319: cmp
    conditional_code = equal(vec2(uniforms.f[93].xyyy), vec2(reg_tmp4.xyyy));
    // 320: ifc
    if (conditional_code.x) {
        sub_321_323();
    } else {
        sub_323_347();
    }
    // 347: cmp
    conditional_code.x = uniforms.f[93].xxxx.x == reg_tmp6.xyyy.x;
    conditional_code.y = uniforms.f[93].xxxx.y < reg_tmp6.xyyy.y;
    // 348: ifc
    if (conditional_code.y) {
        sub_349_356();
    }
    // 356: add
    reg_tmp3 = -uniforms.f[95].wwww + reg_tmp3;
    return false;
}
bool sub_321_323() {
    // 321: dp3
    reg_tmp6.x = dot(vec3(uniforms.f[81 + address_registers.x]), vec3(reg_tmp14));
    // 322: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    return false;
}
bool sub_323_347() {
    // 323: add
    reg_tmp4 = uniforms.f[81 + address_registers.x] + -reg_tmp15;
    // 324: mov
    reg_tmp6.y = (uniforms.f[93].yyyy).y;
    // 325: ifc
    if (conditional_code.y) {
        sub_326_331();
    }
    // 331: mov
    reg_tmp5 = uniforms.f[82 + address_registers.x];
    // 332: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.wwww));
    // 333: dp3
    reg_tmp4.w = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 334: rsq
    reg_tmp4.w = rsq_safe(reg_tmp4.wwww.x);
    // 335: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp4.wwww);
    // 336: ifc
    if (conditional_code.x) {
        sub_337_346();
    }
    // 346: dp3
    reg_tmp6.x = dot(vec3(reg_tmp14), vec3(reg_tmp4));
    return false;
}
bool sub_326_331() {
    // 326: mov
    reg_tmp5.x = (uniforms.f[93].yyyy).x;
    // 327: dp3
    reg_tmp5.z = dot(vec3(reg_tmp4), vec3(reg_tmp4));
    // 328: mul
    reg_tmp5.y = (mul_safe(reg_tmp5.zzzz, reg_tmp5.zzzz)).y;
    // 329: dp3
    reg_tmp6.y = dot(vec3(uniforms.f[83 + address_registers.x]), vec3(reg_tmp5));
    // 330: rcp
    reg_tmp6.y = rcp_safe(reg_tmp6.yyyy.x);
    return false;
}
bool sub_337_346() {
    // 337: dp3
    reg_tmp5.x = dot(vec3(uniforms.f[82 + address_registers.x]), vec3(-reg_tmp4));
    // 338: slti
    reg_tmp5.y = (vec4(lessThan(reg_tmp5.xxxx,uniforms.f[84 + address_registers.x].yyyy))).y;
    // 339: cmp
    conditional_code = equal(vec2(uniforms.f[93].yyyy), vec2(reg_tmp5.xyyy));
    // 340: ifc
    if (conditional_code.y) {
        sub_341_342();
    } else {
        sub_342_345();
    }
    // 345: mul
    reg_tmp6.y = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xxxx)).y;
    return false;
}
bool sub_341_342() {
    // 341: mov
    reg_tmp5.x = (uniforms.f[93].xxxx).x;
    return false;
}
bool sub_342_345() {
    // 342: log
    reg_tmp5.y = log2(reg_tmp5.xxxx.x);
    // 343: mul
    reg_tmp5.y = (mul_safe(uniforms.f[84 + address_registers.x].xxxx, reg_tmp5.yyyy)).y;
    // 344: exp
    reg_tmp5.x = exp2(reg_tmp5.yyyy.x);
    return false;
}
bool sub_349_356() {
    // 349: max
    reg_tmp6.x = (max(uniforms.f[93].xxxx, reg_tmp6.xxxx)).x;
    // 350: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp1.xyzz, uniforms.f[79 + address_registers.x].xyzz, reg_tmp9.xyzz)).xyz;
    // 351: mul
    reg_tmp4 = mul_safe(uniforms.f[80 + address_registers.x], reg_tmp2);
    // 352: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.xxxx, reg_tmp4.xyzz)).xyz;
    // 353: mul
    reg_tmp5.xyz = (mul_safe(reg_tmp6.yyyy, reg_tmp5.xyzz)).xyz;
    // 354: add
    reg_tmp9.xyz = (reg_tmp9.xyzz + reg_tmp5.xyzz).xyz;
    // 355: add
    reg_tmp9.w = (reg_tmp9.wwww + reg_tmp4.wwww).w;
    return false;
}
bool sub_358_368() {
    // 358: dp3
    reg_tmp1 = vec4(dot(vec3(uniforms.f[24]), vec3(reg_tmp14)));
    // 359: mov
    reg_tmp2 = uniforms.f[24].wwww;
    // 360: mad
    reg_tmp1 = fma_safe(reg_tmp1, reg_tmp2, reg_tmp2);
    // 361: mov
    reg_tmp3 = uniforms.f[22];
    // 362: add
    reg_tmp2 = uniforms.f[23] + -reg_tmp3;
    // 363: mad
    reg_tmp4 = fma_safe(reg_tmp2, reg_tmp1, reg_tmp3);
    // 364: ifu
    if (uniforms.b[6]) {
        sub_365_366();
    }
    // 366: mad
    reg_tmp9.xyz = (fma_safe(reg_tmp4, uniforms.f[21], reg_tmp9)).xyz;
    // 367: mov
    reg_tmp8.x = (uniforms.f[93].yyyy).x;
    return false;
}
bool sub_365_366() {
    // 365: mul
    reg_tmp4 = mul_safe(reg_tmp4, reg_tmp9.wwww);
    return false;
}
bool sub_468_4096() {
    // 468: call
    {
        sub_34_202();
    }
    // 469: call
    {
        sub_295_312();
    }
    // 470: end
    return true;
}
// reference: 8DF6A1094CCD80A3, 7DBDA125486056E4
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
// shader: 8B30, DFCAA9C638C51D0E

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
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
// reference: 86FDD594079DD283, DFCAA9C638C51D0E
// program: 7DBDA125486056E4, 4BD70AD09292A3DA, DFCAA9C638C51D0E
// shader: 8B30, FFC2DE1EB53A000B

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: 2584B8EBD1B6582A, FFC2DE1EB53A000B
// program: DE3624946DADEE51, AD7922A63ED54CA7, FFC2DE1EB53A000B
// shader: 8B30, 2DD027877371AD3E

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
diffuse_sum.rgb += ((light_src[0].diffuse * dot_product) + light_src[0].ambient) * 1.0;
specular_sum.rgb += ((light_src[0].specular_0) + (light_src[0].specular_1)) * clamp_highlights * 1.0;
diffuse_sum.rgb += lighting_global_ambient;
primary_fragment_color = clamp(diffuse_sum, vec4(0.0), vec4(1.0));
secondary_fragment_color = clamp(specular_sum, vec4(0.0), vec4(1.0));

vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
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
// reference: 42891F1ACDA76368, 2DD027877371AD3E
// program: DE3624946DADEE51, AD7922A63ED54CA7, 2DD027877371AD3E
// shader: 8B30, DE83EDB96BAC75E2

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
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
// reference: FB35E2F4FDC17A5B, DE83EDB96BAC75E2
// program: DE3624946DADEE51, AD7922A63ED54CA7, DE83EDB96BAC75E2
// shader: 8B30, 46025CD4E2A1C021

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor1.a), 0.0, 1.0));
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
// reference: FB35E2F44D2C8B9D, 46025CD4E2A1C021
// program: DE3624946DADEE51, AD7922A63ED54CA7, 46025CD4E2A1C021
// reference: 34EB998A2FAD8FA1, 8F6D6A21B1F5C895
// shader: 8B30, A15CD1ED4D71039A

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 34639C16EE7DF859, A15CD1ED4D71039A
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A15CD1ED4D71039A
// reference: 9C384505A2E1352B, ECC4377586C6AC61
// reference: 42891F1ADA9165D3, B335C72627F48034
// reference: 3F3A16EAE8E5B22A, D6D39274467B8B59
// reference: F7F99D96B732BFA0, FB4E9E925D4DB26E
// reference: 75D437212881E74D, 9CFDF657447B9FC7
// reference: 584C8F8BC3322C22, 26E3A1642C0501CC
// reference: 9C3845057178CD31, ECC4377586C6AC61
// reference: 42891F1A09089DC9, B335C72627F48034
// reference: 42891F1A233109EB, 38F2FB67AD774145
// reference: 42891F1AE5BF76CA, D0E6A641340115E3
// reference: A085755743D1440C, 6CB806B51A77C8C7
// reference: 9C3845056191E1AF, 25953BF3E0597251
// reference: D2E7285776EB907D, A314B701187AE6F0
// reference: AD65D7B853C051DD, 723911D6E1C81075
// reference: 75D43721C4360C4E, 4505944EC8415D62
// reference: 13F7A6D06DBC567B, A4F526C2F219139E
// reference: 683529447BFCCDC4, 8D9356AD2387DEC1
// reference: B1A0FDAF2881E74D, 50BEB7920B12C81A
// reference: B1A0FDAF68936C23, 992EDE3BB73FAF76
// reference: FB35E2F481DE1D70, 25953BF3E0597251
// reference: 86FDD594065FB8B4, DFCAA9C638C51D0E
// reference: 2584B8EBD074321D, FFC2DE1EB53A000B
// reference: 42891F1ACC65095F, 2DD027877371AD3E
// reference: FB35E2F4FC03106C, DE83EDB96BAC75E2
// reference: FB35E2F44CEEE1AA, 46025CD4E2A1C021
// reference: ADBEE1CBC2E6BE15, 8F6D6A21B1F5C895
// shader: 8B30, AD033AC2B720157B

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FBFB78E96F82EEE3, AD033AC2B720157B
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, AD033AC2B720157B
// shader: 8B30, BE94A49123C032F2

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 34639C16A0857143, BE94A49123C032F2
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, BE94A49123C032F2
// shader: 8B30, EE1118B103A41F40

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6ECD8F518E5138AE, EE1118B103A41F40
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, EE1118B103A41F40
// reference: F7F99D9664AB47BA, FB4E9E925D4DB26E
// shader: 8B30, 96FCBE32EB816134

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F0175698E12ADD24, 96FCBE32EB816134
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 96FCBE32EB816134
// shader: 8B30, A12E4864E2F67B00

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F8FB26760D5CB9E, A12E4864E2F67B00
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A12E4864E2F67B00
// shader: 8B30, 7BF1B73424507E70

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: F0175698AFD2543E, 7BF1B73424507E70
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 7BF1B73424507E70
// shader: 8B30, 107C6C926AC98E68

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: AAB945DF81061DD3, 107C6C926AC98E68
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 107C6C926AC98E68
// reference: A66F6EAB05AE839E, D6D39274467B8B59
// shader: 8B30, 09356E551429A032

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 34639C162B106CCF, 09356E551429A032
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 09356E551429A032
// shader: 8B30, A2B6A02D520E9933

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: FBFB78E9AAEF7A75, A2B6A02D520E9933
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, A2B6A02D520E9933
// shader: 8B30, B806600E9BEE02AC

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
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 34639C1665E8E5D5, B806600E9BEE02AC
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, B806600E9BEE02AC
// shader: 8B30, C0585D0E9AA1AEF9

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((primary_fragment_color.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 6ECD8F514B3CAC38, C0585D0E9AA1AEF9
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, C0585D0E9AA1AEF9
// shader: 8B30, FBAD4993D41D503B

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EAD2C6097ABA43A2, FBAD4993D41D503B
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, FBAD4993D41D503B
// shader: 8B30, 1E60D6AE2138DF38

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 254A22F6FB455518, 1E60D6AE2138DF38
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 1E60D6AE2138DF38
// shader: 8B30, 1715DCF5E7582AC9

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EAD2C6093442CAB8, 1715DCF5E7582AC9
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 1715DCF5E7582AC9
// shader: 8B30, ADCD8AF7344CAC9C

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B07CD54E1A968355, ADCD8AF7344CAC9C
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, ADCD8AF7344CAC9C
// program: D6D39274467B8B59, 6CF3F3B70E23AA85, 25953BF3E0597251
// shader: 8B30, F8A111CDE6E19828

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
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((rounded_primary_color.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((rounded_primary_color.a) * (const_color[0].a), 0.0, 1.0));
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
// reference: FB35E2F4251483DC, F8A111CDE6E19828
// program: DE3624946DADEE51, AD7922A63ED54CA7, F8A111CDE6E19828
// shader: 8B30, D8746F93BF7431DD

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
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
// reference: FB35E2F4EEA4437B, D8746F93BF7431DD
// program: DE3624946DADEE51, AD7922A63ED54CA7, D8746F93BF7431DD
// shader: 8B30, B529DE061616215F

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EAD2C60915C01DA9, B529DE061616215F
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, B529DE061616215F
// shader: 8B30, 7D2E7E9C3E24B14C

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 254A22F6943F0B13, 7D2E7E9C3E24B14C
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 7D2E7E9C3E24B14C
// shader: 8B30, 6E25442F69B03994

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (const_color[5].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (const_color[5].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: EAD2C6095B3894B3, 6E25442F69B03994
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 6E25442F69B03994
// shader: 8B30, 7571221E25042AAF

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
vec4 texcolor1 = textureLod(tex1, texcoord1, getLod(texcoord1 * vec2(textureSize(tex1, 0))));
vec4 combiner_buffer = vec4(0.0);
vec4 next_combiner_buffer = tev_combiner_buffer_color;
vec4 last_tex_env_out = vec4(0.0);
vec3 color_output_0 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a) * (texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((texcolor0.a) - (texcolor1.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: B07CD54E75ECDD5E, 7571221E25042AAF
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 7571221E25042AAF
// shader: 8B30, CAC84D3EF66E29FC

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: D51EEC476CC3152B, CAC84D3EF66E29FC
// program: 0000000000000000, 0000000000000000, CAC84D3EF66E29FC
// reference: 33D810CB5A4824B9, 8F6D6A21B1F5C895
// shader: 8B30, 91E773DA07125DC4

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: C4F9C8BA97A74BF9, 91E773DA07125DC4
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 91E773DA07125DC4
// shader: 8B30, 8F7B7EB624685AA0

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
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_1 = byteround(clamp((last_tex_env_out.rgb) * (const_color[1].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_1 = byteround(clamp((last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_1, alpha_output_1), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 0B612C4516585D43, 8F7B7EB624685AA0
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 8F7B7EB624685AA0
// reference: AA8D688AB703150D, 8F6D6A21B1F5C895
// shader: 8B31, A497E7506F957EAB

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

void main() {
    vs_out_attr0 = vec4(0.0, 0.0, 0.0, 1.0);
    vs_out_attr1 = vec4(0.0, 0.0, 0.0, 1.0);

    exec_shader();
}

#define fma_safe(x, y, z) fma(x, y, z)
#define mul_safe(x, y) (x * y)
#define rcp_safe(x) (1.0f / x)
#define rsq_safe(x) inversesqrt(x)
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
bool sub_9_4096();

bool exec_shader() {
    sub_9_4096();
    return true;
}

bool sub_0_8() {
    // 0: dp4
    reg_tmp15.x = dot(uniforms.f[4], vs_in_reg0);
    // 1: dp4
    reg_tmp15.y = dot(uniforms.f[5], vs_in_reg0);
    // 2: dp4
    reg_tmp15.z = dot(uniforms.f[6], vs_in_reg0);
    // 3: dp4
    reg_tmp15.w = dot(uniforms.f[7], vs_in_reg0);
    // 4: dp4
    vs_out_attr0.x = dot(uniforms.f[0], reg_tmp15);
    // 5: dp4
    vs_out_attr0.y = dot(uniforms.f[1], reg_tmp15);
    // 6: dp4
    vs_out_attr0.z = dot(uniforms.f[2], reg_tmp15);
    // 7: dp4
    vs_out_attr0.w = dot(uniforms.f[3], reg_tmp15);
    return false;
}
bool sub_9_4096() {
    // 9: call
    {
        sub_0_8();
    }
    // 10: mov
    vs_out_attr1 = vs_in_reg1;
    // 11: end
    return true;
}
// reference: E9DC0036DBAD7029, A497E7506F957EAB
// shader: 8DD9, 7CD8A32812674D84

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

struct Vertex {
    vec4 attributes[2];
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
    prim_buffer[0].attributes = vec4[2](vs_out_attr0[0], vs_out_attr1[0]);
    prim_buffer[1].attributes = vec4[2](vs_out_attr0[1], vs_out_attr1[1]);
    prim_buffer[2].attributes = vec4[2](vs_out_attr0[2], vs_out_attr1[2]);
    EmitPrim(prim_buffer[0], prim_buffer[1], prim_buffer[2]);
}
// reference: E0A3E62E72D05152, 7CD8A32812674D84
// shader: 8B30, 518973204ECCEE76

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 3F41287AC5C2DAE1, 518973204ECCEE76
// program: A497E7506F957EAB, 7CD8A32812674D84, 518973204ECCEE76
// shader: 8B30, CFCAACDDBCC65F75

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((const_color[3].rgb) * (vec3(1.0) - texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((const_color[3].a) * (1.0 - texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((const_color[4].rgb) * (texcolor0.rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((const_color[4].a) * (texcolor0.a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: 2EA60C876C349F8B, CFCAACDDBCC65F75
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, CFCAACDDBCC65F75
// shader: 8B30, 7857E02E4F997FDC

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
vec3 color_output_0 = byteround(clamp((const_color[0].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_0 = byteround(clamp((const_color[0].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_0, alpha_output_0), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

combiner_buffer = next_combiner_buffer;

vec3 color_output_2 = byteround(clamp((texcolor0.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_2 = byteround(clamp((texcolor0.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_2, alpha_output_2), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_3 = byteround(clamp((vec3(1.0) - texcolor0.rgb) * (const_color[3].rgb), vec3(0.0), vec3(1.0)));
float alpha_output_3 = byteround(clamp((texcolor0.a) * (const_color[3].a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_3, alpha_output_3), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_4 = byteround(clamp((texcolor0.rgb) * (const_color[4].rgb) + (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_4 = byteround(clamp((1.0 - texcolor0.a) * (const_color[4].a) + (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_4, alpha_output_4), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

vec3 color_output_5 = byteround(clamp((rounded_primary_color.rgb) * (last_tex_env_out.rgb), vec3(0.0), vec3(1.0)));
float alpha_output_5 = byteround(clamp((rounded_primary_color.a) * (last_tex_env_out.a), 0.0, 1.0));
last_tex_env_out = clamp(vec4(color_output_5, alpha_output_5), vec4(0.0), vec4(1.0));
combiner_buffer = next_combiner_buffer;

gl_FragDepth = depth;
color = byteround(last_tex_env_out);
}
// reference: E13EE878EDCB8931, 7857E02E4F997FDC
// program: 8F6D6A21B1F5C895, 5D764F9A6220D694, 7857E02E4F997FDC
// reference: 2584B8EBA1852B99, B335C72627F48034
// program: 0000000000000000, 0000000000000000, B335C72627F48034
// reference: B2475BDC0F0BC456, A497E7506F957EAB
// shader: 8B30, FBA83801FD85F40C

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
vec3 color_output_0 = byteround(clamp((texcolor0.rgb) * (texcolor0.rgb), vec3(0.0), vec3(1.0)));
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
// reference: 2584B8EB7820B7A2, FBA83801FD85F40C
// program: 0000000000000000, 0000000000000000, FBA83801FD85F40C
